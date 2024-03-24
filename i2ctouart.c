/****************************************************************************************************************
MSP-EXP430F5529LP I2C to UART bridge, customized for MLX90640 starting with TI examples available in CCS Resource Explorer:
msp430F55xx_usci_i2c_standard_master.c
msp430F55xx_usci_i2c_standard_transceiver.c

Description: Receives commands from a UART to read or write I2C device registers at a 16-bit address.
In the case of reading, will send the requested number of bytes starting at a specified I2C register address back
over UART.
In the case of writing, will write one 16-bit register with one 16-bit value per command.

The USCIB0 I2C peripheral is controlled by an interrupt driven state machine. The USCIA1 UART peripheral uses
interrupts for buffering received characters, and busy-waits for sending.

The UART will communicate over the backchannel USB-UART interface on the MSP-EXP430F5529LP Launchpad with the RXD TXD jumpers
installed, this is tested stable at 115,200bps. Since the same emulation device is used for debugging, there can be a delay
in the serial comms if debugging is active, slowing the framerate from the sensor to the PC.

ACLK = NA, MCLK = SMCLK = DCO 16MHz.

In the connection diagram below notice the two pullup resistors!


                 MSP-EXP430F5529LP
                                                    MLX90640
                 -----------------                                     ----------------
            /|\ |              VCC|---+---+------ +3.3V --------------|PIN 2-VDD       |
             |  |                 |   |  2.2k                         |                |
             ---|RST              |  2.2k |                           |                |
                |                 |   |   |                           |                |
 (UCA1TXD)------|P4.4         P3.1|---|---+- I2C Clock (UCB0SCL) -----|PIN 4-SCL       |
                |                 |   |                               |                |
 (UCA1RXD)------|P4.5         P3.0|---+----- I2C Data (UCB0SDA) ------|PIN 1-SDA       |
                |                 |                                   |                |
                |                 |                                   |                |
                |                 |                                   |                |
                |              GND|-------------- 0VDC ---------------|PIN 3-GND       |
                 -----------------                                     ----------------

   UBC PHAS E-Lab Written by Mark Carlson
   Nov 2022
   Built with CCS V12.1
******************************************************************************************************************/

#include <msp430.h>
#include <stdint.h>
#include <stdbool.h>

#define DEFAULT_SLAVE_ADDR  0x33 //Default I2C slave address, 0x33 for MLX90620
#define TX_BUFFER_SIZE 30 //Size of the I2C TX and UART RX buffer
#define RX_BUFFER_SIZE 64 //Size of the I2C RX buffer

#define I2CTIMEOUTCOUNT 10000000

//I2C State Machine Modes
typedef enum I2C_ModeEnum{
    IDLE_MODE,
    NACK_MODE,
    TX_REG_ADDRESS_MODE,
    RX_REG_ADDRESS_MODE,
    TX_DATA_MODE,
    RX_DATA_MODE,
    SWITCH_TO_RX_MODE,
    SWITHC_TO_TX_MODE,
    TIMEOUT_MODE
} I2C_Mode;



//Globals accessed by the I2C state machine and interrupts
uint8_t g_ReceiveBuffer[RX_BUFFER_SIZE + 10]; //Buffer used to receive data in the I2C ISR
uint16_t g_RXByteCtr; //Number of bytes left to receive
uint16_t g_ReceiveIndex; //The index of the next byte to be received in g_ReceiveBuffer
uint8_t g_TransmitBuffer[TX_BUFFER_SIZE + 10]; //Buffer used to transmit data in the I2C ISR
uint16_t g_TXByteCtr; //Number of bytes left to transfer
uint16_t g_TransmitIndex; //The index of the next byte to be transmitted in g_TransmitBuffer
uint16_t g_TransmitRegAddr; //2 byte I2C register address to access
uint8_t g_RegByteCtr; //Index for sending two byte register address
I2C_Mode MasterMode = IDLE_MODE; //Used to track the state of the I2C state machine

//Globals accessed in the UART receive interrupt
uint8_t g_UartReceiveBuffer[TX_BUFFER_SIZE + 10]; //Buffer used to receive data in the UART ISR
uint8_t g_UartReceiveIndex; //The index of the next byte to be received in g_UartReceiveIndex


//Function prototypes
I2C_Mode I2C_Master_ReadReg(uint8_t dev_addr, uint16_t reg_addr, uint16_t count);
uint8_t ProcessUart(uint8_t *SlaveAddress, uint16_t *RegAddress, uint16_t *ValueorBytes);
void CopyArray(uint8_t *source, uint8_t *dest, uint16_t count);
void initClockTo16MHz();
void initGPIO();
void initI2C();
void initUART();
void SetVcoreUp(unsigned int level);
I2C_Mode I2C_Master_WriteReg(uint8_t dev_addr, uint16_t reg_addr, uint8_t *reg_data, uint16_t count);

#define LED1_INIT()    (P1DIR |= BIT0)
#define LED1_ON()      (P1OUT |= BIT0)
#define LED1_OFF()     (P1OUT &= ~BIT0)

#define LED2_INIT() P4DIR |= BIT7; // Set P4.7 as output
#define LED2_ON()      (P4OUT |= BIT7)
#define LED2_OFF()     (P4OUT &= ~BIT7)

#define MOTOR_A_FWD BIT5 // P1.4
#define MOTOR_A_REV BIT4 // P1.5
#define MOTOR_B_FWD BIT5 // P2.4
#define MOTOR_B_REV BIT4 // P2.5

uint8_t direction = 0;
uint8_t magnitude = 0;


void initMotor(void) {

    // Initialize Motor A direction control pins
    P1DIR |= MOTOR_A_FWD + MOTOR_A_REV; // Set pins as output

    // Initialize Motor B direction control pins
    P2DIR |= MOTOR_B_FWD + MOTOR_B_REV; // Set pins as output
}
void motorStop(void) {
    // Set all motor control pins to low
    P1OUT &= ~(MOTOR_A_FWD + MOTOR_A_REV);
    P2OUT &= ~(MOTOR_B_FWD + MOTOR_B_REV);
}

void delayVariable(unsigned int cycles) {
    while(cycles > 0) {
        __delay_cycles(50); // This assumes a small delay (1 cycle) per iteration
        cycles--;
    }
}



void motorControl(uint8_t direction, uint8_t magnitude) {
    unsigned int dutyCycle = magnitude * 100; // Convert magnitude to PWM duty cycle


    switch (direction) {
        case 0: // Turn Left (Motor A - Reverse, Motor B - Forward)
            // Motor A - Reverse
            LED2_ON();
            LED1_OFF();
            P1OUT &= ~MOTOR_A_FWD; // Disable PWM forward by setting pin low
            P1OUT |= MOTOR_A_REV;  // Set reverse direction pin high

            // Motor B - Forward
            P2OUT |= MOTOR_B_FWD;  // Enable PWM forward by setting pin high
            P2OUT &= ~MOTOR_B_REV; // Ensure reverse direction pin is low
            delayVariable(1000*dutyCycle);
            motorStop();
            break;

        case 1: // Turn Right (Motor A - Forward, Motor B - Reverse)
            // Motor A - Forward
            LED2_OFF();
            LED1_ON();
            P1OUT |= MOTOR_A_FWD;  // Enable PWM forward by setting pin high
            P1OUT &= ~MOTOR_A_REV; // Ensure reverse direction pin is low

            // Motor B - Reverse
            P2OUT &= ~MOTOR_B_FWD; // Disable PWM forward by setting pin low
            P2OUT |= MOTOR_B_REV;  // Set reverse direction pin high
            delayVariable(1000*dutyCycle);
            motorStop();
            break;

        default:
            // Stop both motors
            P1OUT |= (MOTOR_A_FWD | MOTOR_A_REV);
            P2OUT |= (MOTOR_B_FWD | MOTOR_B_REV);
            break;
    }
}

/*
Main:
Initialize all peripherals, and loop checking for commands from UART, and execute the command if necessary
*/

int main(void)
{
    WDTCTL = WDTPW | WDTHOLD; //Stop watchdog timer
    //Core voltage must be increased, step-by-step, for stable operation at higher clk frequencies
    SetVcoreUp (0x01);
    SetVcoreUp (0x02);
    SetVcoreUp (0x03);

    LED1_INIT();
    LED2_INIT()

    LED1_ON();
    LED2_ON();


    initMotor();

    initClockTo16MHz(); //Set main clock to 16MHz
    initGPIO(); //Configure GPIO for UART and I2C operation
    initI2C(); //Configure the I2C hardware peripheral
    initUART(); //Configure UART hardware peripheral
    SFRIE1 |= VMAIE;
    uint8_t Command = 0; //Selected command returned from ProcessUart()
    uint16_t UartSendIndex = 0; //Local index for comparing to I2C receive index
    uint8_t SlaveAddress = DEFAULT_SLAVE_ADDR; //Slave address returned from ProcessUart()
    uint16_t NumBytesValue = 400; //Number of bytes remaining to be read, or value to write, returned from ProcessUart()
    uint16_t RegAddress = 0x2000; //2-byte register address returned from ProcessUart()
    uint16_t NumBytesRequested = 0; // If buffer size is smaller than total number of bytes to be read, split read command into smaller sections
    uint8_t WriteValue[2] = {0}; //2-byte array to pass write value to I2C_Master_WriteReg
    uint32_t Timeout = 0;
    __enable_interrupt();


    while(1)
    {
        Command = ProcessUart(&SlaveAddress, &RegAddress, &NumBytesValue); //Check if a command was received over UART
        if(Command == 1) //Received a read command
        {
            NumBytesRequested = 0;

            while(NumBytesValue > 0) //Keep looping until all bytes are sent
            {
                if(NumBytesValue > RX_BUFFER_SIZE)//If necessary, split remaining requests to fit the buffer size
                {
                    NumBytesRequested = RX_BUFFER_SIZE;
                }
                else
                {
                    NumBytesRequested = NumBytesValue;
                }

                I2C_Master_ReadReg(SlaveAddress, RegAddress, NumBytesRequested);//Start the I2C read request
                Timeout = 0;
                UartSendIndex = 0;

                while(((g_RXByteCtr > 0) || (UartSendIndex < g_ReceiveIndex)) && (Timeout < I2CTIMEOUTCOUNT))//Loop until I2C read request complete and all data sent over UART and didn't timeout
                {
                    if(UartSendIndex < g_ReceiveIndex)
                    {
                        //while (!(IFG2&UCA0TXIFG)); //Wait for uart to be ready for another byte
                        while(!(UCTXIFG==(UCTXIFG & UCA1IFG))&&((UCA1STAT & UCBUSY)==UCBUSY));  // Wait for uart to be ready for another byte
                        UCA1TXBUF = g_ReceiveBuffer[UartSendIndex]; //Write byte to UART send register
                        UartSendIndex++;
                    }
                    Timeout++;
                }
                if(Timeout < I2CTIMEOUTCOUNT)
                {
                    NumBytesValue -= NumBytesRequested;
                    RegAddress += (NumBytesRequested / 2);
                }

            }
            //Send a 'DONE' message at the end of the read data
            //while (!(IFG2&UCA0TXIFG)); // Wait for TX Buffer
            while(!(UCTXIFG==(UCTXIFG & UCA1IFG))&&((UCA1STAT & UCBUSY)==UCBUSY));
            UCA1TXBUF = 'D'; //Send a character
            //while (!(IFG2&UCA0TXIFG));
            while(!(UCTXIFG==(UCTXIFG & UCA1IFG))&&((UCA1STAT & UCBUSY)==UCBUSY));
            UCA1TXBUF = 'O';
            //while (!(IFG2&UCA0TXIFG));
            while(!(UCTXIFG==(UCTXIFG & UCA1IFG))&&((UCA1STAT & UCBUSY)==UCBUSY));
            UCA1TXBUF = 'N';
            //while (!(IFG2&UCA0TXIFG));
            while(!(UCTXIFG==(UCTXIFG & UCA1IFG))&&((UCA1STAT & UCBUSY)==UCBUSY));
            UCA1TXBUF = 'E';
        }
        else if(Command == 2)//Received a write command
        {
            WriteValue[0] = (NumBytesValue >> 8) & 0xFF; //Convert 16-bit value to 2-byte array
            WriteValue[1] = NumBytesValue & 0xFF;
            I2C_Master_WriteReg(SlaveAddress, RegAddress, WriteValue, 2); //Write I2C register

        }else if (Command == 3) {

            motorControl(direction, magnitude);



        }
    }

}


void SetVcoreUp (unsigned int level)
{
  // Open PMM registers for write
  PMMCTL0_H = PMMPW_H;
  // Set SVS/SVM high side new level
  SVSMHCTL = SVSHE + SVSHRVL0 * level + SVMHE + SVSMHRRL0 * level;
  // Set SVM low side to new level
  SVSMLCTL = SVSLE + SVMLE + SVSMLRRL0 * level;
  // Wait till SVM is settled
  while ((PMMIFG & SVSMLDLYIFG) == 0);
  // Clear already set flags
  PMMIFG &= ~(SVMLVLRIFG + SVMLIFG);
  // Set VCore to new level
  PMMCTL0_L = PMMCOREV0 * level;
  // Wait till new level reached
  if ((PMMIFG & SVMLIFG))
    while ((PMMIFG & SVMLVLRIFG) == 0);
  // Set SVS/SVM low side to new level
  SVSMLCTL = SVSLE + SVSLRVL0 * level + SVMLE + SVSMLRRL0 * level;
  // Lock PMM registers for write access
  PMMCTL0_H = 0x00;
}


/*
ProcessUart:
Checks for valid commands coming in on the UART buffer, two different commands are supported:
Read command: 8 bytes in the format
[1-byte I2C slave address][2-byte I2C register address to start reading][2-byte Number of bytes to read][3-char escape sequence 'rdg']
Write command: 8 bytes in the format
[1-byte I2C slave address][2-byte I2C register address to write to][2-byte value to write to address][3-char escape sequence 'wrt']

Input Parameters:
None

Returns:
UartCommand:
    0: If no command detected
    1: If read command detected
    2: If write command detected
SlaveAddress: I2C slave to read or write
RegAddress: 2-byte I2C register to read or write
ValueorBytes: If read command, number of bytes to read, if write command, value to be written

*/

uint8_t ProcessUart(uint8_t *SlaveAddress, uint16_t *RegAddress, uint16_t *ValueorBytes)
{
    uint8_t UartCommand;
    static unsigned int LocalIndex = 0;
    static uint32_t readcount = 0;
    UartCommand = 0;
    while(LocalIndex < g_UartReceiveIndex)//Step through UART buffer and check for command
    {
        if(LocalIndex >= 7)//If we received at least 8 characters
        {



            if(g_UartReceiveBuffer[LocalIndex] == 'g' && g_UartReceiveBuffer[LocalIndex -1] == 'd' && g_UartReceiveBuffer[LocalIndex - 2] == 'r')//Received a read command
            {
                *SlaveAddress = g_UartReceiveBuffer[LocalIndex - 7];
                *RegAddress = (g_UartReceiveBuffer[LocalIndex - 6] << 8) | g_UartReceiveBuffer[LocalIndex - 5];
                *ValueorBytes = (g_UartReceiveBuffer[LocalIndex - 4] << 8) | g_UartReceiveBuffer[LocalIndex - 3];
                g_UartReceiveIndex = 0;
                LocalIndex = 0;
                UartCommand = 1;
                readcount++;
            }
            else if(g_UartReceiveBuffer[LocalIndex] == 't' && g_UartReceiveBuffer[LocalIndex -1] == 'r' && g_UartReceiveBuffer[LocalIndex - 2] == 'w')//received a write command
            {
                *SlaveAddress = g_UartReceiveBuffer[LocalIndex - 7];
                *RegAddress = (g_UartReceiveBuffer[LocalIndex - 6] << 8) | g_UartReceiveBuffer[LocalIndex - 5];
                *ValueorBytes = (g_UartReceiveBuffer[LocalIndex - 4] << 8) | g_UartReceiveBuffer[LocalIndex - 3];
                g_UartReceiveIndex = 0;
                LocalIndex = 0;
                UartCommand = 2;

            }

            if(g_UartReceiveBuffer[LocalIndex] == 'n' &&
                 g_UartReceiveBuffer[LocalIndex - 1] == 'r' &&
                 g_UartReceiveBuffer[LocalIndex - 2] == 't')
                 {
                  direction = g_UartReceiveBuffer[LocalIndex - 3]; // Get the direction byte
                  magnitude = g_UartReceiveBuffer[LocalIndex - 4];
                  UartCommand = 3; // Indicate that a turn command was received
                  }



        }
        LocalIndex++;

        if((LocalIndex >= TX_BUFFER_SIZE) || (LocalIndex > g_UartReceiveIndex))
        {
            LocalIndex = 0;
        }

    }

    return UartCommand;

}



/*
I2C_Master_ReadReg
For slave device with dev_addr, read the data specified in slaves reg_addr. Set up for 16 bit register addresses
The received data is available in ReceiveBuffer[]

Parameters:
dev_addr: The slave device address.
reg_addr: The register or command to send to the slave.
count: The length of data to read

Returns:
MasterMode: The mode of the I2C master state machine
*/
I2C_Mode I2C_Master_ReadReg(uint8_t dev_addr, uint16_t reg_addr, uint16_t count)
{
    //Initialize state machine
    MasterMode = TX_REG_ADDRESS_MODE;
    g_TransmitRegAddr = reg_addr;

    //Initialize counters
    g_RegByteCtr = 0;
    g_RXByteCtr = count;
    g_TXByteCtr = 0;
    g_ReceiveIndex = 0;
    g_TransmitIndex = 0;

    //Initialize slave address and interrupts
    UCB0I2CSA = dev_addr;

    UCB0IFG &= ~(UCTXIFG + UCRXIFG);       // Clear any pending interrupts
    UCB0IE &= ~UCRXIE;                       // Disable RX interrupt
    UCB0IE |= UCTXIE;                        // Enable TX interrupt

    UCB0CTL1 |= UCTR + UCTXSTT; //I2C TX, start condition

    //__enable_interrupt(); //Unmask interrupts

    return MasterMode;

}


/*
I2C_Master_WriteReg
For slave device with dev_addr, writes the data specified in *reg_data, it is set up for 16 bit register addresses

Parameters:
dev_addr: The slave device address.
reg_addr: The 16-bit register or command to send to the slave.
*reg_data: The buffer to write
count: The length of *reg_data

Returns:
Mastermode: The current mode of the I2C state machine
 */

I2C_Mode I2C_Master_WriteReg(uint8_t dev_addr, uint16_t reg_addr, uint8_t *reg_data, uint16_t count)
{
    //Initialize state machine
    MasterMode = TX_REG_ADDRESS_MODE;
    g_TransmitRegAddr = reg_addr;

    //Copy register data to TransmitBuffer
    CopyArray(reg_data, g_TransmitBuffer, count);

    //Initialize counters
    g_RegByteCtr = 0;
    g_TXByteCtr = count;
    g_RXByteCtr = 0;
    g_ReceiveIndex = 0;
    g_TransmitIndex = 0;

    //Initialize slave address and interrupts
    UCB0I2CSA = dev_addr;

    UCB0IFG &= ~(UCTXIFG + UCRXIFG);       // Clear any pending interrupts
    UCB0IE &= ~UCRXIE;                       // Disable RX interrupt
    UCB0IE |= UCTXIE;                        // Enable TX interrupt

    UCB0CTL1 |= UCTR + UCTXSTT; //I2C TX, start condition

    return MasterMode;
}


/*
CopyArray
Copy number number of bytes specified by count, from array *source to array *dest

Parameters:
*source: Pointer to array to be copied from
*dest: Pointer to array to be copied to
count: Number of bytes to copy

Returns:
None
 */

void CopyArray(uint8_t *source, uint8_t *dest, uint16_t count)
{
    uint16_t copyIndex = 0;
    for (copyIndex = 0; copyIndex < count; copyIndex++)
    {
        dest[copyIndex] = source[copyIndex];
    }
}

//******************************************************************************
// Device Initialization *******************************************************
//******************************************************************************

void initClockTo16MHz()
{
    UCSCTL3 |= SELREF_2;                      // Set DCO FLL reference = REFO
    UCSCTL4 |= SELA_2;                        // Set ACLK = REFO
    __bis_SR_register(SCG0);                  // Disable the FLL control loop
    UCSCTL0 = 0x0000;                         // Set lowest possible DCOx, MODx
    UCSCTL1 = DCORSEL_5;                      // Select DCO range 16MHz operation
    UCSCTL2 = FLLD_0 + 487;                   // Set DCO Multiplier for 16MHz
                                              // (N + 1) * FLLRef = Fdco
                                              // (487 + 1) * 32768 = 16MHz
                                              // Set FLL Div = fDCOCLK
    __bic_SR_register(SCG0);                  // Enable the FLL control loop

    // Worst-case settling time for the DCO when the DCO range bits have been
    // changed is n x 32 x 32 x f_MCLK / f_FLL_reference. See UCS chapter in 5xx
    // UG for optimization.
    // 32 x 32 x 16 MHz / 32,768 Hz = 500000 = MCLK cycles for DCO to settle
    __delay_cycles(500000);//
    // Loop until XT1,XT2 & DCO fault flag is cleared
    do
    {
        UCSCTL7 &= ~(XT2OFFG + XT1LFOFFG + DCOFFG); // Clear XT2,XT1,DCO fault flags
        SFRIFG1 &= ~OFIFG;                          // Clear fault flags
    }while (SFRIFG1&OFIFG);                         // Test oscillator fault flag
}

void initGPIO()
{
    P3SEL |= BIT0 | BIT1 | BIT3 | BIT4; // P3.0,3.1 = UCB0SDA, UCB0SCL
    P4SEL |= BIT4 | BIT5; //P4.4,4.5 UCA1TXD/RXD

}

void initI2C()
{
    UCB0CTL1 |= UCSWRST; //SW reset while configuring peripheral
    UCB0CTL0 = UCMST + UCMODE_3 + UCSYNC; //I2C Master, synchronous mode
    UCB0CTL1 = UCSSEL_2 + UCSWRST; //Select SMCLK as I2C clock source, keep SW reset
    //UCB0BR0 = 160; //fSCL = SMCLK/160 = ~100kHz
    UCB0BR0 = 80; //fSCL = SMCLK/40 = ~200kHz
    //UCB0BR0 = 40; //fSCL = SMCLK/40 = ~400kHz
    UCB0BR1 = 0;
    UCB0I2CSA = DEFAULT_SLAVE_ADDR; //Set default slave address
    UCB0CTL1 &= ~UCSWRST; // Clear SW reset, resume operation

}

void initUART()
{
    UCA1CTL1 |= UCSWRST;        // Put the USCI state machine in reset
    UCA1CTL1 |= UCSSEL__SMCLK;  // Use SMCLK as the bit clock

//    UCA1BR0 = 130; //16MHz 9600
//    UCA1BR1 = 6; //16MHz 9600
//    UCA1MCTL = UCBRS_6; //Modulation UCBRSx = 6

    UCA1BR0 = 138; //16MHz 115200
    UCA1BR1 = 0; //16MHz 115200
    UCA1MCTL = UCBRS_7; //Modulation UCBRSx = 7

    //UCA1BR0 = 21; //16MHz 57600
    //UCA1BR1 = 1; //16MHz 57600
    //UCA1MCTL = UCBRS_7; //Modulation UCBRSx = 7

    //Other untested baud rates
    //UCA1BR0 = 34; //16MHz 460800
    //UCA1BR1 = 0; //16MHz 460800
    //UCA1MCTL = UCBRS_6; //Modulation UCBRSx = 7

    //UCA1BR0 = 69; //16MHz 230400
    //UCA1BR1 = 0; //16MHz 230400
    //UCA1MCTL = UCBRS_4; //Modulation UCBRSx = 4


    UCA1CTL1 &= ~UCSWRST; //Clear SW reset, resume operation
    UCA1IE |= UCRXIE; // Enable USCI_A1 RX interrupt
}

#pragma vector=SYSNMI_VECTOR
__interrupt void SYSNMI_ISR(void)
{
    uint8_t blah = 1;
    blah++;
}

//******************************************************************************
//UART Interrupt for Receive
//******************************************************************************

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCI_A1_VECTOR
__interrupt void USCI_A1_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCI_A1_VECTOR))) USCI_A1_ISR (void)
#else
#error Compiler not supported!
#endif
{
  switch(__even_in_range(UCA1IV,4))
  {
  case 0:break;                             // Vector 0 - no interrupt
  case 2:                                   // Vector 2 - RXIFG
      if(g_UartReceiveIndex >= TX_BUFFER_SIZE)
      {
          g_UartReceiveIndex = 0;
      }
      g_UartReceiveBuffer[g_UartReceiveIndex] = UCA1RXBUF; //Write received byte to buffer, also clears interrupt flag
      g_UartReceiveIndex++;

    break;
  case 4:

      break;                             // Vector 4 - TXIFG
  default: break;
  }
}



//******************************************************************************
// I2C Interrupt For Received and Transmitted Data******************************
//******************************************************************************

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCI_B0_VECTOR
__interrupt void USCI_B0_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCI_B0_VECTOR))) USCI_B0_ISR (void)
#else
#error Compiler not supported!
#endif
{
    uint8_t rx_val = 0;

    switch(__even_in_range(UCB0IV,0xC))
    {
        case USCI_NONE:break;                             // Vector 0 - no interrupt
        case USCI_I2C_UCALIFG:break;                      // Interrupt Vector: I2C Mode: UCALIFG
        case USCI_I2C_UCNACKIFG:break;                    // Interrupt Vector: I2C Mode: UCNACKIFG
        case USCI_I2C_UCSTTIFG:break;                     // Interrupt Vector: I2C Mode: UCSTTIFG
        case USCI_I2C_UCSTPIFG:break;                     // Interrupt Vector: I2C Mode: UCSTPIFG
        case USCI_I2C_UCRXIFG:
        //Receive Data Interrupt
        //Must read from UCB0RXBUF to clear interrupt flag
        rx_val = UCB0RXBUF;

        if (g_RXByteCtr > 0) //Still some bytes left to receive for this transaction
        {
            g_ReceiveBuffer[g_ReceiveIndex++] = rx_val; //Write received byte to buffer
            g_RXByteCtr--;
        }

        if (g_RXByteCtr == 1) //Last byte to receive, send stop condition
        {
            UCB0CTL1 |= UCTXSTP;
        }
        else if (g_RXByteCtr == 0) //No more bytes left
        {
            UCB0IE &= ~UCRXIE; //Disable receive interrupt
            //IE2 &= ~UCB0RXIE; //Disable receive interrupt
            MasterMode = IDLE_MODE; //Place state machine in idle
        }
        break;

        case USCI_I2C_UCTXIFG: //Transmit Data Interrupt

        switch (MasterMode) //State machine determines which section of the I2C transaction we are in
        {
          case TX_REG_ADDRESS_MODE: //Transmitting register address
              if(g_RegByteCtr == 0) //We have to transmit the MSB of the address
              {
                  UCB0TXBUF = (uint8_t)((g_TransmitRegAddr >> 8) & 0xff);

                  g_RegByteCtr++;
              }
              else //We have to transmit the LSB of the address
              {
                  UCB0TXBUF = (uint8_t)(g_TransmitRegAddr & 0xff);
                  if (g_RXByteCtr)
                      MasterMode = SWITCH_TO_RX_MODE; //Need to start receiving now
                  else
                      MasterMode = TX_DATA_MODE; //Continue to transmission of data in Transmit Buffer
              }
          break;

          case SWITCH_TO_RX_MODE: //Transition to receive mode
              UCB0IE |= UCRXIE;              // Enable RX interrupt
              UCB0IE &= ~UCTXIE;             // Disable TX interrupt
              UCB0CTL1 &= ~UCTR; //Switch to receiver
              MasterMode = RX_DATA_MODE; //Change to receive data state
              UCB0CTL1 |= UCTXSTT; //Send repeated start condition (very important for MLX90640!)
              if (g_RXByteCtr == 1) //If this is already the last byte
              {
                  while((UCB0CTL1 & UCTXSTT)); //Wait for last byte to get sent
                  UCB0CTL1 |= UCTXSTP; //Send stop condition
              }
              break;

          case TX_DATA_MODE:
              if (g_TXByteCtr > 0) //Still some bytes in buffer to send
              {
                  UCB0TXBUF = g_TransmitBuffer[g_TransmitIndex++];
                  g_TXByteCtr--;
              }
              else
              {
                  //Done with transmission
                  UCB0CTL1 |= UCTXSTP; //Send stop condition
                  MasterMode = IDLE_MODE;
                  UCB0IE &= ~UCTXIE;  // disable TX interrupt

                  //IE2 &= ~UCB0TXIE; //disable TX interrupt
              }
              break;

          default:
              __no_operation();
              break;
      }
  }
}
