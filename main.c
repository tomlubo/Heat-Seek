//#include <msp430.h>
//#include <stdint.h>
//#include <stdbool.h>
//
//#define MOTOR_A_FWD BIT5 // P1.4
//#define MOTOR_A_REV BIT4 // P1.5
//#define MOTOR_B_FWD BIT5 // P2.4
//#define MOTOR_B_REV BIT4 // P2.5
//
//void initMotor(void) {
//
//    // Initialize Motor A direction control pins
//    P1DIR |= MOTOR_A_FWD + MOTOR_A_REV; // Set pins as output
//
//    // Initialize Motor B direction control pins
//    P2DIR |= MOTOR_B_FWD + MOTOR_B_REV; // Set pins as output
//}
//
//void motorStop(void) {
//    // Set all motor control pins to low
//    P1OUT &= ~(MOTOR_A_FWD + MOTOR_A_REV);
//    P2OUT &= ~(MOTOR_B_FWD + MOTOR_B_REV);
//}
//
//void delayVariable(unsigned int cycles) {
//    while(cycles > 0) {
//        __delay_cycles(100); // This assumes a small delay (1 cycle) per iteration
//        cycles--;
//    }
//}
//
//void motorControl(uint8_t direction, uint8_t magnitude) {
//    unsigned int dutyCycle = magnitude * 100; // Convert magnitude to PWM duty cycle
//
//
//    switch (direction) {
//        case 0: // Turn Left (Motor A - Reverse, Motor B - Forward)
//            // Motor A - Reverse
//            P1OUT &= ~MOTOR_A_FWD; // Disable PWM forward by setting pin low
//            P1OUT |= MOTOR_A_REV;  // Set reverse direction pin high
//
//            // Motor B - Forward
//            P2OUT |= MOTOR_B_FWD;  // Enable PWM forward by setting pin high
//            P2OUT &= ~MOTOR_B_REV; // Ensure reverse direction pin is low
//            delayVariable(1000*dutyCycle);
//            motorStop();
//            break;
//
//        case 1: // Turn Right (Motor A - Forward, Motor B - Reverse)
//            // Motor A - Forward
//            P1OUT |= MOTOR_A_FWD;  // Enable PWM forward by setting pin high
//            P1OUT &= ~MOTOR_A_REV; // Ensure reverse direction pin is low
//
//            // Motor B - Reverse
//            P2OUT &= ~MOTOR_B_FWD; // Disable PWM forward by setting pin low
//            P2OUT |= MOTOR_B_REV;  // Set reverse direction pin high
//            delayVariable(1000*dutyCycle);
//            motorStop();
//            break;
//
//        default:
//            // Stop both motors
//            P1OUT |= (MOTOR_A_FWD | MOTOR_A_REV);
//            P2OUT |= (MOTOR_B_FWD | MOTOR_B_REV);
//            break;
//    }
//}
//
//
//int main(void) {
//    WDTCTL = WDTPW | WDTHOLD;   // Stop watchdog timer
//
//    initMotor();                  // Initialize PWM
//
//    while(1) {
//        // Example usage
//        motorControl(1, 4); // Turn right with mid-range speed
//        __delay_cycles(1600000); // Simple delay
//        motorControl(0, 8); // Turn left with max speed
//        __delay_cycles(1600000); // Simple delay
//        // Add your logic for receiving direction and magnitude commands
//        //motorControl(0, 0); // Turn left with max speed
//        //__delay_cycles(10000000);
//    }
//}
