#Driver for MLX90640 to communicate with MSP430 I2CtoUART program using PySerial
#Modified from https://github.com/RevKarl/MLX90640-python
#For more details on operation of MLX90640 and calculating temperatures, refer to datasheet
#UBC PHAS E-lab, Nov 2022

import math, serial, struct

class MLX90640:
    
    def __init__(self, port='COM5', baud=9600, address=0x33, framerate=3): #default COM5, 9600 baud, I2C address 0x33
        self.addr = address        
        self.bus = serial.Serial(port, baud, timeout = 4, write_timeout = 4)  # open serial port
        while(self.bus.inWaiting() > 0): #Make sure serial port receive buffer is empty
            self.bus.read(self.bus.inWaiting())
            time.sleep(0.2)        
        print("Attempting to download MLX ROM...\n")
        self.ROM = self.getROM()
        print("Complete, downloading RAM...\n")
        self.updateRAM()
        self.setFramerate(framerate) #MLX Framerate values 0-7 are 0.5-64Hz
        self.gain = self.getGain() #Restore GAIN coefficient
        
        self.VDD0 = 3.3 #Calculate VDD
        self.DV = self.getVDD()
        self.VDD = self.DV+self.VDD0
        self.Ta0 = 25
        self.Ta = self.getTa() #Calculate ambient temp
        self.emissivity = 1 #Emissivity constant
        self.TGC = self.getTGC() #Restore TGC coefficient
        self.chessNotIL = 1 #Default display update mode is chess, not interleaved
        self.KsTa = self.getKsTa() #Restore KsTa coefficient
        self.KsTo1, self.KsTo2, self.KsTo3, self.KsTo4 = self.getKsTo() #Restore KsTo coefficient
        self.step, self.CT3, self.CT4 = self.getCorners() #Restore corner temperatures
        self.CT1 = 40
        self.CT2 = 0        
        self.alphaCorrR1 = 1/float(1+ self.KsTo1*(0-(-40))) #Restore sensitivity corrections
        self.alphaCorrR2 = 1
        self.alphaCorrR3 = 1 + self.KsTo2*(self.CT3-0)
        self.alphaCorrR4 = self.alphaCorrR3*(1+self.KsTo3*(self.CT4-self.CT3))
            
    def close(self):
        self.bus.close()

    def setFramerate(self, rate):
        #MLX Framerate values 0-7 are 0.5-64Hz
        value = (rate & 0x07)<<7
        controlReg1 = self.getRegf(0x800D)
        value |= (controlReg1 & 0xFC7F)
        self.setReg(0x800D,value)        

    def getRegs(self,reg,num): #Reading multiple registers 
        
        #construct the read command to send to MSP430
        packet = bytearray()
        packet.append(self.addr) #Slave address
        packet.append((reg >> 8) & 0xff ) #Register address
        packet.append(reg & 0xff)
        packet.append((num >> 8) & 0xff )#Number of bytes to read
        packet.append(num & 0xff)
        packet.extend(map(ord, 'rdg'))#Escape sequence at the end
        #print(packet)
        written = self.bus.write(packet)
        #print(written)
        s = bytearray(self.bus.read(num + 4)) #4 Extra characters for 'DONE'
        #print(s)
        if(len(s) != (num + 4)):
            raise Exception('Read Failed, len = ' + str(len(s)))
        count = int(len(s)/2)
        integers = struct.unpack('>'+'H'*count, s) #Unpack bytes as big-endian, signed integers
        return integers       
        
    def getRegf(self,reg): #Read single register
        result = self.getRegs(reg, 2)        
        return result[0]
    
    def turn(self, direction, magnitude):
        """
        Sends a turn command to the MSP430.
    
        Parameters:
            direction (int): The direction to turn (0 for left, 1 for right).
        """
        # Ensure the direction is either 0 or 1
        if direction not in [0, 1]:
            #print("Invalid direction. Must be 0 (left) or 1 (right).")
            return
        
        # Construct the packet with the 'trn' command
        packet = bytearray()
        
        
        packet.append(magnitude)
        packet.append(direction)
        
        packet.extend(map(ord, 'trn'))
        while len(packet) < 8:
            packet.insert(0, 0xFF)
    
        # Send the packet over the serial bus
        self.bus.write(packet)
        
        #print(f"Turn command sent. Packet: {packet}")
    
    def setReg(self,reg,value): #Write single register

        #Construct the write command to send to MSP430
        packet = bytearray()
        packet.append(self.addr) #Slave Address
        packet.append((reg >> 8) & 0xff ) #Register address
        packet.append(reg & 0xff)
        packet.append((value >> 8) & 0xff )#Value to write to register
        packet.append(value & 0xff)
        packet.extend(map(ord, 'wrt')) #Escape sequence
        
        self.bus.write(packet)     
        print(packet)
        return 0

    def getROM(self): #Read MLX ROM registers, any calculations should take into account 0x2400 address offset
        s = self.getRegs(0x2400, 1668)
        return list(s)

    def updateRAM(self): #Read MLX RAM registers, any calculations should take into account 0x0400 address offset
        self.RAM = self.getRegs(0x0400, 1668)
        return 0

    def root4(self,num):
        if num == 0:
            return 0
        else:
            return math.sqrt(math.sqrt(abs(num)))
        
    def getTGC(self):
        TGC = self.ROM[0x3C] & 0x00FF
        if TGC > 127:
            TGC = TGC - 256
        
        return TGC
        
    def getVDD(self):
        Kvdd = (self.ROM[0x33] & 0xFF00)/256
        if Kvdd > 127:
            Kvdd = Kvdd -256
        Kvdd = Kvdd*32
        
        Vdd25 = self.ROM[0x33] & 0x00FF
        Vdd25 = (Vdd25-256)*32 - 8192
        
        RAM = self.RAM[0x032A]
        if RAM > 32767:
            RAM = RAM - 65536

        DV = (RAM - Vdd25)/float(Kvdd)  
        
        return DV

    def getTa(self):
        KVptat = (self.ROM[0x32] & 0xFC00)/1024
        if KVptat > 31:
            KVptat = KVptat - 62
        
        KVptat = KVptat/4096.0
        
        KTptat = self.ROM[0x32] & 0x03FF
        if KTptat > 511:
            KTptat = KTptat - 1022
        
        KTptat = KTptat/8.0
        
        Vptat25 = self.ROM[0x31]
        if Vptat25 > 32767:
            Vptat25 = Vptat25 - 65536
        Vptat = self.RAM[0x0320]
        if Vptat > 32767:
            Vptat = Vptat - 65536
        Vbe = self.RAM[0x0300]
        if Vbe > 32767:
            Vbe = Vbe - 65536

        AlphaptatEE = (self.ROM[0x10] & 0xF000)/4096
        Alphaptat = (AlphaptatEE/4)+8
        Vptatart = (Vptat/float(Vptat * Alphaptat + Vbe))*262144
        
        Ta = ((Vptatart/float(1+KVptat*self.DV)-Vptat25)/float(KTptat))+self.Ta0
        return Ta

    def getGain(self):
        GAIN = self.ROM[0x30]
        if GAIN > 32767:
            GAIN = GAIN - 65536

        RAM = self.RAM[0x030A]
        if RAM > 32767:
            RAM = RAM - 65536
        return GAIN/float(RAM)
                
    def pixnum(self,i,j):
        return (i-1)*32 + j

    def patternChess(self,i,j):
        pixnum = self.pixnum(i,j)
        a = (pixnum-1)/32
        b = int((pixnum-1)/32)/2
        return int(a) - int(b)*2
        
    def getKsTa(self):

        KsTaEE = (self.ROM[0x3C] & 0xFF00) >> 256
        if KsTaEE > 127:
            KsTaEE = KsTaEE -256
        
        KsTa = KsTaEE/8192.0
        return KsTa
    
    def getKsTo(self):

        EE1 = self.ROM[0x3D]
        EE2 = self.ROM[0x3E]
        KsTo1 = EE1 & 0x00FF
        KsTo3 = EE2 & 0x00FF
        KsTo2 = (EE1 & 0xFF00) >> 8
        KsTo4 = (EE2 & 0xFF00) >> 8

        if KsTo1 > 127:
            KsTo1 = KsTo1 -256
        if KsTo2 > 127:
            KsTo2 = KsTo2 -256
        if KsTo3 > 127:
            KsTo3 = KsTo3 -256
        if KsTo4 > 127:
            KsTo4 = KsTo4 -256
        
        KsToScale = (self.ROM[0x3F] & 0x000F)+8
        KsTo1 = KsTo1/float(pow(2,KsToScale))
        KsTo2 = KsTo2/float(pow(2,KsToScale))
        KsTo3 = KsTo3/float(pow(2,KsToScale))
        KsTo4 = KsTo4/float(pow(2,KsToScale))
        return KsTo1, KsTo2, KsTo3, KsTo4
        
    def getCorners(self):
        EE = self.ROM[0x3F]
        step = ((EE & 0x3000)>>12)*10
        CT3 = ((EE & 0x00f0)>>4)*step
        CT4 = ((EE & 0x0f00)>>8)*(step+CT3)
        return step, CT3, CT4
        
    def getPixDataRAM(self,i,j):

        Offsetavg = self.ROM[0x11]
        if Offsetavg > 32676:
            Offsetavg = Offsetavg-65536

        scaleVal = self.ROM[0x10]
        OCCscaleRow = (scaleVal&0x0f00)/256
        OCCscaleCol = (scaleVal&0x00F0)/16
        OCCscaleRem = (scaleVal&0x000F)

        rowAdd = 0x12 + int(((i-1)/4))
        colAdd = 0x18 + int(((j-1)/4))
        rowMask = 0xF<<int((4*((i-1)%4)))
        colMask = 0xF<<int((4*((j-1)%4)))
        
        OffsetPixAdd = 0x3F+((i-1)*32)+j
        OffsetPixVal = self.ROM[OffsetPixAdd]
        OffsetPix = (OffsetPixVal & 0xFC00)/1024
        if OffsetPix >31:
            OffsetPix = OffsetPix - 64

        OCCRow = (self.ROM[rowAdd] & rowMask)>>(4*((i-1)%4))
        if OCCRow >7:
            OCCRow = OCCRow -16

        OCCCol = (self.ROM[colAdd] & colMask)>>(4*((j-1)%4))
        if OCCCol > 7:
            OCCCol = OCCCol -16

        pixOffset = Offsetavg + OCCRow*pow(2,OCCscaleRow) + OCCCol*pow(2,OCCscaleCol) + OffsetPix*pow(2,OCCscaleRem)
        
        KtaEE = int((OffsetPixVal & 0x000E)/2)
        if KtaEE > 3:
            KtaEE = KtaEE - 7

        colEven = not (j%2)
        rowEven = not (i%2)
        rowOdd = not rowEven
        colOdd = not colEven

        KtaAvAddr = 0x36 + (colEven)
        KtaAvMask = 0xFF00 >> (8*rowEven)
        

        KtaRC = (self.ROM[KtaAvAddr] & KtaAvMask) >> 8* rowOdd
        if KtaRC > 127:
            KtaRC = KtaAvRC - 256
        
        KtaScale1 = ((self.ROM[0x38] & 0x00F0) >>4)+8
        KtaScale2 = (self.ROM[0x38] & 0x000F)
        Kta = (KtaRC+(KtaEE<<KtaScale2))/float(pow(2,KtaScale1))
        
        shiftNum = (rowOdd*4)+(colOdd*8)
        KvMask = 0x000F << shiftNum

        Kv = (self.ROM[0x34] & KvMask) >> shiftNum
        if Kv > 7:
            Kv = Kv-16
        

        KvScale = (self.ROM[0x38] & 0x0F00)>>8
        
        Kv = Kv/float(KvScale)
       
        RAMaddr = ((i-1)*32)+ j-1
        RAM = self.RAM[RAMaddr]
        if RAM > 32767:
            RAM = RAM - 65536
        pixGain = RAM*self.gain
        pixOs = pixGain - pixOffset*(1+Kta*(self.Ta - self.Ta0)*(1+Kv*(self.VDD - self.VDD0)))
        return pixOs

    
    def getCompensatedPixDataRAM(self,i,j):
        pixOs = self.getPixDataRAM(i,j)
        Kgain = ((self.gain -1)/10)+1

        pixGainCPSP0 = self.RAM[0x0308]     
        if pixGainCPSP0 > 32767:
            pixGainCPSP0 = pixGainCPSP0 - 65482
        
        pixGainCPSP1 = self.RAM[0x0328]
        if pixGainCPSP1 > 32767:
            pixGainCPSP1 = pixGainCPSP1 - 65482
        
        pixGainCPSP0 = pixGainCPSP0*Kgain
        pixGainCPSP1 = pixGainCPSP1*Kgain
        
        OffCPSP0 = self.ROM[0x3A] & 0x03FF
        if OffCPSP0 > 511:
            OffCPSP0 = OffCPSP0-1024
        
        OffCPSP1d = (self.ROM[0x3A] &0xFC00)>>10
        if OffCPSP1d > 31:
            OffCPSP1d = OffCPSP1d-64
        
        OffCPSP1 = OffCPSP1d + OffCPSP0
        
        KvtaCPEEVal = self.ROM[0x3B]
        KvtaScaleVal = self.ROM[0x38]
        
        KtaScale1 = ((KvtaScaleVal & 0x00F0)>>4)+8
        KvScale = (KvtaScaleVal & 0x0F00)>>8
        
        KtaCPEE = KvtaCPEEVal & 0x00FF
        if KtaCPEE > 127:
            KtaCPEE = KtaCPEE -256
        
        KvCPEE = (KvtaCPEEVal & 0xFF00)>>8
        
        KtaCP = KtaCPEE/float(pow(2,KtaScale1))
        KvCP = KvCPEE/float(pow(2,KvScale))
        
        b = (1+KtaCP*(self.Ta - self.Ta0))*(1+ KvCP*(self.VDD - self.VDD0))
        pixOSCPSP0 = pixGainCPSP0 - OffCPSP0*b
        pixOSCPSP1 = pixGainCPSP1 - OffCPSP1*b
        
        if self.chessNotIL:
            pattern = self.patternChess(i,j)
        else:
            pattern = self.patternChess(i,j)
        
        VIREmcomp = pixOs/self.emissivity
        VIRcomp = VIREmcomp - self.TGC*((1-pattern)*pixOSCPSP0 + pattern*pixOSCPSP1)

        reg2439val = self.ROM[0x39]
        reg2420val = self.ROM[0x20]
        alphaScaleCP = ((reg2420val & 0xF000)>>12) + 27
        CPP1P0ratio = (reg2439val & 0xFC00)>>10
        if CPP1P0ratio >31:
            CPP1P0ratio = CPP1P0ratio -64

        alphaRef = self.ROM[0x21]
        alphaScale = ((reg2420val & 0xF000)>>12) + 30
        rowAdd = 0x22 + int(((i-1)/4))
        colAdd = 0x28 + int(((j-1)/4))
        rowMask = 0xF<<int((4*((i-1)%4)))
        colMask = 0xF<<int((4*((j-1)%4)))

        ACCRow = (self.ROM[rowAdd] & rowMask)>>(4*((i-1)%4))
        if ACCRow >7:
            ACCRow = ACCRow -16

        ACCCol = (self.ROM[colAdd] & colMask)>>(4*((j-1)%4))
        if ACCCol > 7:
            ACCCol = ACCCol -16
        
        ACCScaleRow = (reg2420val & 0x0F00)>>8
        ACCScaleCol = (reg2420val & 0x00F0)>>4
        ACCScaleRem = (reg2420val & 0x000F)
        
        alphaPixel = self.ROM[0x1f+self.pixnum(i,j)]&0x03f0>>4
        
        alpha = (alphaRef+(ACCRow<<ACCScaleRow)+(ACCCol<<ACCScaleCol)+(alphaPixel<<ACCScaleRem))/float(pow(2,alphaScale))
        
        alphaCPSP0 = (reg2439val & 0x03ff)/float(pow(2,alphaScaleCP))
        alphaCPSP1 = alphaCPSP0*(1+CPP1P0ratio/128.0)
        alphacomp= alpha - self.TGC*((1-pattern)*alphaCPSP0 + pattern*alphaCPSP1)*(1+self.KsTa*(self.Ta-self.Ta0))
        #print(alphacomp)
        Tak4 = pow(self.Ta + 273.15,4)
        Trk4 = pow(self.Ta-8 + 273.15,4)
        Tar = Trk4-(Trk4-Tak4)/self.emissivity
        Sx = self.KsTo2*self.root4(pow(alphacomp,3)*VIRcomp + pow(alphacomp,4)*Tar)
        return self.root4((VIRcomp/(alphacomp*(1-self.KsTo2*273.15)+Sx))+Tar) - 273.15
