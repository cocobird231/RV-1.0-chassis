from subprocess import run
import serial, time
import RPi.GPIO as GPIO
import crcmod
import binascii

#ser = serial.Serial("/dev/ttyAMA0", 115200)

class TurningModule():
    def crc16Add(self, read):
        crc16 = crcmod.mkCrcFun(0x18005, rev=True, initCrc=0xFFFF, xorOut=0x0000)
        data = read.replace(" ", "")
        readcrcout = hex(crc16(binascii.unhexlify(data))).upper()
        #print(readcrcout)
        str_list = list(readcrcout)
        #print(str_list)
        if len(str_list) == 5:
            str_list.insert(2, '0') 
        crc_data = "".join(str_list)
        #print(crc_data)
        #print('CRC16: %s' % (crc_data[4:] + ' ' + crc_data[2:4]))
        read = data + crc_data[4:] + crc_data[2:4]
        return read


    #old
    def runCommand(self, runCommand):    
        ser = serial.Serial()
        ser.port = "/dev/ttyAMA0"

        #115200,N,8,1
        ser.baudrate = 115200
        ser.bytesize = serial.EIGHTBITS #number of bits per bytes
        ser.parity = serial.PARITY_EVEN #set parity check
        ser.stopbits = serial.STOPBITS_ONE #number of stop bits
        
        ser.timeout = 0.5          #non-block read 0.5s
        ser.writeTimeout = 0.5     #timeout for write 0.5s
        ser.xonxoff = False    #disable software flow control
        ser.rtscts = False     #disable hardware (RTS/CTS) flow control
        ser.dsrdtr = False     #disable hardware (DSR/DTR) flow control


        uart_tx = 7
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(uart_tx, GPIO.OUT)


        try: 
            ser.open()
        except Exception as ex:
            print ("open serial port error " + str(ex))
            exit()
        
        if ser.isOpen():
            try:
                ser.flushInput() #flush input buffer
                ser.flushOutput() #flush output buffer

                #query command
                #command = [0x01, 0x03, 0x00, 0xCC, 0x00, 0x02, 0x04, 0x34]
                #reset command
                #command = [0x01, 0x10, 0x01, 0x80, 0x00, 0x02, 0x04, 0x00, 0x00, 0x00, 0x01, 0x37, 0x9F]

                command0 = [0x01, 0x10, 0x00, 0x58, 0x00, 0x10, 0x20, 
                0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x01,
                0x00, 0x00, 0x01, 0xF4,
                0x00, 0x00, 0x4E, 0x20,
                0x00, 0x0F, 0x42, 0x40,
                0x00, 0x0F, 0x42, 0x40,
                0x00, 0x00, 0x03, 0x20,
                0x00, 0x00, 0x00, 0x01,
                0xD1, 0x3E]

                command1 = [0x01, 0x10, 0x00, 0x58, 0x00, 0x10, 0x20, 
                0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x01,
                0x00, 0x00, 0x15, 0x7C,
                0x00, 0x00, 0x4E, 0x20,
                0x00, 0x0F, 0x42, 0x40,
                0x00, 0x0F, 0x42, 0x40,
                0x00, 0x00, 0x03, 0x20,
                0x00, 0x00, 0x00, 0x01,
                0x93, 0x40]

                if runCommand == 0:
                    #write 8 byte data
                    GPIO.output(uart_tx, GPIO.HIGH)
                    ser.write(command0)
                    print("write command1", command0)
                    time.sleep(0.0001*len(command0))
                    GPIO.output(uart_tx, GPIO.LOW)
                    for i in range(20):
                        count = ser.inWaiting()
                        if count != 0:
                            print('count = ', count)
                            recv = ser.read(count)
                            print(recv)
                            print('\n')
                            break
                            #ser.write(recv)
                        ser.flushInput()
                        time.sleep(0.1)
                        i=i+1
                elif runCommand == 1:
                    GPIO.output(uart_tx, GPIO.HIGH)
                    ser.write(command1)
                    print("write command2", command1)
                    time.sleep(0.0001*len(command1))
                    GPIO.output(uart_tx, GPIO.LOW)
                    for i in range(20):
                        count = ser.inWaiting()
                        if count != 0:
                            print('count = ', count)
                            recv = ser.read(count)
                            print(recv)
                            print('\n')
                            break
                            #ser.write(recv)
                        ser.flushInput()
                        time.sleep(0.1)
                        i=i+1

                
            except Exception as e1:
                print ("communicating error " + str(e1))
        
        else:
            print ("open serial port error")
    def Test(self):
        ser = serial.Serial()
        ser.port = "/dev/ttyAMA0"

        #115200,N,8,1
        ser.baudrate = 115200
        ser.bytesize = serial.EIGHTBITS #number of bits per bytes
        ser.parity = serial.PARITY_EVEN #set parity check
        ser.stopbits = serial.STOPBITS_ONE #number of stop bits
        
        ser.timeout = 0.5          #non-block read 0.5s
        ser.writeTimeout = 0.5     #timeout for write 0.5s
        ser.xonxoff = False    #disable software flow control
        ser.rtscts = False     #disable hardware (RTS/CTS) flow control
        ser.dsrdtr = False     #disable hardware (DSR/DTR) flow control


        uart_tx = 7
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(uart_tx, GPIO.OUT)


        try: 
            ser.open()
        except Exception as ex:
            print ("open serial port error " + str(ex))
            exit()
        
        if ser.isOpen():
            try:
                ser.flushInput() #flush input buffer
                ser.flushOutput() #flush output buffer

                #-100
                motorCommand1 = "\
                    DE061083FF9C\
                    "
                motorCommandCrc1 = bytearray.fromhex(TurningModule().crc16Add(motorCommand1))

                motorCommand2 = "\
                    DE0610830064\
                    "
                motorCommandCrc2 = bytearray.fromhex(TurningModule().crc16Add(motorCommand2))

                counter = 0
                while(True):
                    if counter%2 == 0:
                        motorCommand = motorCommandCrc2
                    else:
                        motorCommand = motorCommandCrc1
                    #write 8 byte data
                    GPIO.output(uart_tx, GPIO.HIGH)
                    ser.write(motorCommand)
                    #print("write command1", command0)
                    time.sleep(0.0001*len(motorCommand))
                    GPIO.output(uart_tx, GPIO.LOW)
                    for i in range(20):
                        count = ser.inWaiting()
                        if count != 0:
                            print('count = ', count)
                            recv = ser.read(count)
                            print(recv)
                            print('\n')
                            break
                            #ser.write(recv)
                        ser.flushInput()
                        time.sleep(0.1)
                        i=i+1

                    counter += 1
                    time.sleep(6)
                
            except Exception as e1:
                print ("communicating error " + str(e1))
        
        else:
            print ("open serial port error")
    def SetDistance(self, distance):
        def to_hex(n):
            # 兩個位元組可以表示的範圍是 -32768 ~ 32767
            # 若輸入的數值超過此範圍，則會有誤差
            if n < -32768 or n > 32767:
                raise ValueError("輸入超出範圍。")
            # 對於負數，我們先將其轉換為相對應的二補數表示，然後轉換為十六進制
            if n < 0:
                n = n + 65536
            return hex(n)[2:].zfill(4)

        ser = serial.Serial()
        ser.port = "/dev/ttyAMA0"

        #115200,N,8,1
        ser.baudrate = 115200
        ser.bytesize = serial.EIGHTBITS #number of bits per bytes
        ser.parity = serial.PARITY_EVEN #set parity check
        ser.stopbits = serial.STOPBITS_ONE #number of stop bits
        
        ser.timeout = 0.5          #non-block read 0.5s
        ser.writeTimeout = 0.5     #timeout for write 0.5s
        ser.xonxoff = False    #disable software flow control
        ser.rtscts = False     #disable hardware (RTS/CTS) flow control
        ser.dsrdtr = False     #disable hardware (DSR/DTR) flow control


        uart_tx = 7
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(uart_tx, GPIO.OUT)


        try: 
            ser.open()
        except Exception as ex:
            print ("open serial port error " + str(ex))
            exit()
        
        if ser.isOpen():
            try:
                ser.flushInput() #flush input buffer
                ser.flushOutput() #flush output buffer

                motorCommandTemp = "DE061083xxxx"

                motorCommandRaw = motorCommandTemp.replace("xxxx", to_hex(distance))

                motorCommandCrc = bytearray.fromhex(TurningModule().crc16Add(motorCommandRaw))                

                #write 8 byte data
                GPIO.output(uart_tx, GPIO.HIGH)
                ser.write(motorCommandCrc)
                #print("write command1", command0)
                time.sleep(0.0001*len(motorCommandCrc))
                GPIO.output(uart_tx, GPIO.LOW)
                for i in range(20):
                    count = ser.inWaiting()
                    if count != 0:
                        print('count = ', count)
                        recv = ser.read(count)
                        print(recv)
                        print('\n')
                        break
                        #ser.write(recv)
                    ser.flushInput()
                    time.sleep(0.1)
                    i=i+1
                
                
                
            except Exception as e1:
                print ("communicating error " + str(e1))
        
        else:
            print ("open serial port error")
    def Initialization():
        ser = serial.Serial()
        ser.port = "/dev/ttyAMA0"

        #115200,N,8,1
        ser.baudrate = 115200
        ser.bytesize = serial.EIGHTBITS #number of bits per bytes
        ser.parity = serial.PARITY_EVEN #set parity check
        ser.stopbits = serial.STOPBITS_ONE #number of stop bits
        
        ser.timeout = 0.5          #non-block read 0.5s
        ser.writeTimeout = 0.5     #timeout for write 0.5s
        ser.xonxoff = False    #disable software flow control
        ser.rtscts = False     #disable hardware (RTS/CTS) flow control
        ser.dsrdtr = False     #disable hardware (DSR/DTR) flow control


        uart_tx = 7
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(uart_tx, GPIO.OUT)


        try: 
            ser.open()
        except Exception as ex:
            print ("open serial port error " + str(ex))
            exit()
        
        if ser.isOpen():
            try:
                ser.flushInput() #flush input buffer
                ser.flushOutput() #flush output buffer


                motorCommand_10000 = "01100058001020\
                    00000001\
                    00000014\
                    00002710\
                    000007D0\
                    000001F4\
                    000001F4\
                    000001F4\
                    00000001"

                motorCommand_5000 = "01100058001020\
                    00000001\
                    00000014\
                    00001388\
                    000007D0\
                    000001F4\
                    000001F4\
                    000001F4\
                    00000001"

                motorCommand_0 = "01100058001020\
                    00000001\
                    00000014\
                    00000000\
                    000007D0\
                    000001F4\
                    000001F4\
                    000001F4\
                    00000001"

                
                motorCommandCrc_10000 = bytearray.fromhex(TurningModule().crc16Add(motorCommand_10000))
                motorCommandCrc_5000 = bytearray.fromhex(TurningModule().crc16Add(motorCommand_5000))
                motorCommandCrc_0 = bytearray.fromhex(TurningModule().crc16Add(motorCommand_0))

                

                commandsToRun = [motorCommandCrc_0, motorCommandCrc_10000, motorCommandCrc_5000]
                
                for command in commandsToRun:                
                    #write 8 byte data
                    GPIO.output(uart_tx, GPIO.HIGH)
                    ser.write(command)
                    #print("write command1", command0)
                    time.sleep(0.0001*len(command))
                    GPIO.output(uart_tx, GPIO.LOW)
                    for i in range(20):
                        count = ser.inWaiting()
                        if count != 0:
                            print('count = ', count)
                            recv = ser.read(count)
                            print(recv)
                            print('\n')
                            break
                            #ser.write(recv)
                        ser.flushInput()
                        time.sleep(0.1)
                        i=i+1

                    try:
                        time.sleep(7.5)
                    except NameError:
                        print("Last element")
                                
            except Exception as e1:
                print ("communicating error " + str(e1))
        
        else:
            print ("open serial port error")
    def Break(position):
        ser = serial.Serial()
        ser.port = "/dev/ttyAMA0"

        #115200,N,8,1
        ser.baudrate = 115200
        ser.bytesize = serial.EIGHTBITS #number of bits per bytes
        ser.parity = serial.PARITY_EVEN #set parity check
        ser.stopbits = serial.STOPBITS_ONE #number of stop bits
        
        ser.timeout = 0.5          #non-block read 0.5s
        ser.writeTimeout = 0.5     #timeout for write 0.5s
        ser.xonxoff = False    #disable software flow control
        ser.rtscts = False     #disable hardware (RTS/CTS) flow control
        ser.dsrdtr = False     #disable hardware (DSR/DTR) flow control


        uart_tx = 7
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(uart_tx, GPIO.OUT)


        try: 
            ser.open()
        except Exception as ex:
            print ("open serial port error " + str(ex))
            exit()
        
        if ser.isOpen():
            try:
                ser.flushInput() #flush input buffer
                ser.flushOutput() #flush output buffer


                motorCommand = "01100058001020\
                    00000001\
                    00000014\
                    0000XXXX\
                    000007D0\
                    000001F4\
                    000001F4\
                    000001F4\
                    00000001"

                motorCommand_5000 = "01100058001020\
                    00000001\
                    00000014\
                    00001388\
                    000007D0\
                    000001F4\
                    000001F4\
                    000001F4\
                    00000001"

                motorCommand_0 = "01100058001020\
                    00000001\
                    00000014\
                    00000000\
                    000007D0\
                    000001F4\
                    000001F4\
                    000001F4\
                    00000001"

                
                motorCommandCrc_10000 = bytearray.fromhex(TurningModule().crc16Add(motorCommand_10000))
                motorCommandCrc_5000 = bytearray.fromhex(TurningModule().crc16Add(motorCommand_5000))
                motorCommandCrc_0 = bytearray.fromhex(TurningModule().crc16Add(motorCommand_0))

                

                commandsToRun = [motorCommandCrc_0, motorCommandCrc_10000, motorCommandCrc_5000]
                
                for command in commandsToRun:                
                    #write 8 byte data
                    GPIO.output(uart_tx, GPIO.HIGH)
                    ser.write(command)
                    #print("write command1", command0)
                    time.sleep(0.0001*len(command))
                    GPIO.output(uart_tx, GPIO.LOW)
                    for i in range(20):
                        count = ser.inWaiting()
                        if count != 0:
                            print('count = ', count)
                            recv = ser.read(count)
                            print(recv)
                            print('\n')
                            break
                            #ser.write(recv)
                        ser.flushInput()
                        time.sleep(0.1)
                        i=i+1

                    try:
                        time.sleep(7.5)
                    except NameError:
                        print("Last element")
                                
            except Exception as e1:
                print ("communicating error " + str(e1))
        
        else:
            print ("open serial port error")
        

#test main
if __name__ == '__main__':
    TurningModule().Test()
    #TurningModule().SetDistance(0)
    