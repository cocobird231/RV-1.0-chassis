import socket
import struct
from BreakModule import BreakModule
from TurningModule_V2 import TurningModule
from AxleModule import AxleModule
import time
from subprocess import check_output
import sys
import os
from threading import Thread
import random
from pathlib import Path
from os.path import expanduser
import pickle

serverIp = '192.168.1.42'
TIME_BETWEEN_ALIVE_SIGNALS = 2

#myID = 0x01 #剎車馬達
#myID = 0x0B #輪軸馬達
#myID = 0x15 #UPS
#myID = 0x29 #轉向馬達

################################
import threading
axleDict = { 'dir' : 0, 'pwm' : 0, 'parking' : 0 }
steeringDict = { 'min' : 0, 'max' : 0, 'center' : 0, 'value' : 0  }
ros2DictLock = threading.Lock()

__sockRecvData = None
__sockRecvDataLock = threading.Lock()

def ModuleProcess(interval_s = 0.1):
    global __sockRecvData
    global __sockRecvDataLock
    unpacked_content = None

    # axle
    lastPwm = 0
    lastRunDirection = 1

    while (True):
        __sockRecvDataLock.acquire()
        unpacked_content = __sockRecvData
        __sockRecvDataLock.release()

        if (unpacked_content == None):
            time.sleep(1)
            continue

        t0 = time.time()

        #command: break
        if len(unpacked_content) > 5 and unpacked_content[4]==0 and unpacked_content[5]==1:
            BreakModule().runCommand(int(LoopReceiveDataAndProcess.breakModuleCommand))
            LoopReceiveDataAndProcess.breakModuleCommand = not LoopReceiveDataAndProcess.breakModuleCommand

        #command: get UPS info
        elif len(unpacked_content) > 5 and unpacked_content[4]==0 and unpacked_content[5]==2:
            pass

        #command: axle
        elif len(unpacked_content) > 11 and unpacked_content[4]==0 and unpacked_content[5]==4:
            runDirection = unpacked_content[9]
            brake = unpacked_content[10]
            pwm = unpacked_content[11]

            ros2DictLock.acquire()
            axleDict['dir'] = runDirection
            axleDict['pwm'] = pwm
            axleDict['parking'] = brake
            ros2DictLock.release()

            if brake:
                AxleModule().runCommand(brake=True)
                lastRunDirection = 0
                lastPwm = 0
            else:
                if runDirection == 1:
                    AxleModule().runCommand(runDirection = 1, pwm = pwm, lastPwm = lastPwm, lastRunDirection = lastRunDirection)
                elif runDirection == 2:
                    AxleModule().runCommand(runDirection = -1, pwm = pwm, lastPwm = lastPwm, lastRunDirection = lastRunDirection)
                elif runDirection == 0:
                    AxleModule().runCommand(runDirection = 0, pwm = pwm, lastPwm = lastPwm, lastRunDirection = lastRunDirection)
                lastPwm = pwm
                lastRunDirection = runDirection
        
        #command:breaking motor initial
        elif len(unpacked_content) > 5 and unpacked_content[4]==0 and unpacked_content[5]==5:
            print('handle Break motor initialization')
            BreakModule.Initialization()
        
        #command:breaking motor setting
        elif len(unpacked_content) > 5 and unpacked_content[4]==0 and unpacked_content[5]==6:
            print('handle Break motor set')
            #distance = ord(unpacked_content[9])*256 + ord(unpacked_content[10])
            distance = unpacked_content[9]*256 + unpacked_content[10]
            print("distance = ", distance)
            BreakModule.SetDistance(distance)

        #command:trunig motor initial
        elif len(unpacked_content) > 5 and unpacked_content[4]==0 and unpacked_content[5]==7:
            print('handle Turning motor initialization')
            TurningModule.Initialization()

        #command:turning motor setting
        elif len(unpacked_content) > 5 and unpacked_content[4]==0 and unpacked_content[5]==8:
            print('handle Tuening motor set')
            #distance = ord(unpacked_content[9])*256 + ord(unpacked_content[10])
            distance = unpacked_content[9]*256 + unpacked_content[10]
            print("distance = ", distance)
            TurningModule().SetDistance(distance)

            ros2DictLock.acquire()
            # value in distance
            steeringDict['min'] = 0
            steeringDict['max'] = 10000
            steeringDict['center'] = 5000
            steeringDict['value'] = distance
            ros2DictLock.release()
        
        procT = time.time() - t0
        print('Module process time: %.2fs'  %procT)
        if (interval_s - procT > 0):
            time.sleep(interval_s - procT)
################################

unpackerLength = struct.Struct('I')
int_size = struct.calcsize("I")


home = expanduser('~') + "/tmp/"
Path(home).mkdir(parents=True, exist_ok=True)

def getUpsInfo():    
    f=open('/tmp/ups_data', 'rb')
    #f=open('z:/ups_data', 'rb')
    upsInfo = f.read(16)
    f.close
    return upsInfo


def LoopReceiveDataAndProcess(sock):
    global __sockRecvData
    global __sockRecvDataLock

    lastPwm = 0
    lastRunDirection = 1
    while LoopReceiveDataAndProcess.bWillStop == False:
        try:
            data1 = sock.recv(unpackerLength.size)
        except socket.error as msg:
            print('Caught exception socket.error : ', msg)
            LoopSendAlive.bWillStop = True
            break
        unpacked_length = unpackerLength.unpack(data1)[0]
        try:
            data2 = sock.recv(unpacked_length)
        except socket.error as msg:
            print('Caught exception socket.error : ', msg)
            LoopSendAlive.bWillStop = True
            break
        unpackerContent = struct.Struct(str(unpacked_length) + 'B')
        unpacked_content = unpackerContent.unpack(data2)

        print('got data ', unpacked_content)

        __sockRecvDataLock.acquire()
        __sockRecvData = unpacked_content
        __sockRecvDataLock.release()

        #command: break
        if len(unpacked_content) > 5 and unpacked_content[4]==0 and unpacked_content[5]==1:
            data = (0x69, 0x74, 0x72, 0x69, 0x00, 0x01, 0x10, 0x00, 0x00)
        #command: get UPS info
        elif len(unpacked_content) > 5 and unpacked_content[4]==0 and unpacked_content[5]==2:
            data = [0x69, 0x74, 0x72, 0x69, 0x00, 0x02, 0x04, 0x00, 0x20]
            data.extend(list(getUpsInfo()))
        #command: axle
        elif len(unpacked_content) > 11 and unpacked_content[4]==0 and unpacked_content[5]==4:
            data = (0x69, 0x74, 0x72, 0x69, 0x00, 0x04, 0x10, 0x00, 0x00)
        #command:breaking motor initial
        elif len(unpacked_content) > 5 and unpacked_content[4]==0 and unpacked_content[5]==5:
            data = [0x69, 0x74, 0x72, 0x69, 0x00, 0x05, 0x10, 0x00, 0x00]
        #command:breaking motor setting
        elif len(unpacked_content) > 5 and unpacked_content[4]==0 and unpacked_content[5]==6:
            data = [0x69, 0x74, 0x72, 0x69, 0x00, 0x06, 0x10, 0x00, 0x00]
        #command:trunig motor initial
        elif len(unpacked_content) > 5 and unpacked_content[4]==0 and unpacked_content[5]==7:
            data = [0x69, 0x74, 0x72, 0x69, 0x00, 0x05, 0x10, 0x00, 0x00]
        #command:turning motor setting
        elif len(unpacked_content) > 5 and unpacked_content[4]==0 and unpacked_content[5]==8:
            data = [0x69, 0x74, 0x72, 0x69, 0x00, 0x06, 0x10, 0x00, 0x00]
        else:
            print('ERROR: insufficient axle data')
            continue
        
        packerContent = struct.Struct('I'+ str(len(data)) + 'B')
        packedContent = packerContent.pack(len(data), *data)
        try:
            sock.sendall(packedContent)
        except socket.error as msg:
            print('Caught exception socket.error : ', msg)
            LoopSendAlive.bWillStop = True
            break

        # #command: break
        # if len(unpacked_content) > 5 and unpacked_content[4]==0 and unpacked_content[5]==1:
        #     #response
        #     data = (0x69, 0x74, 0x72, 0x69, 0x00, 0x01, 0x10, 0x00, 0x00)
        #     packerContent = struct.Struct('I'+ str(len(data)) + 'B')
        #     packedContent = packerContent.pack(len(data), *data)
        #     try:
        #         sock.sendall(packedContent)
        #     except socket.error as msg:
        #         print('Caught exception socket.error : ', msg)
        #         LoopSendAlive.bWillStop = True
        #         break

        #     if LoopReceiveDataAndProcess.breakModuleCommand == 0:
        #         BreakModule().runCommand(0)
        #         LoopReceiveDataAndProcess.breakModuleCommand = 1
        #     elif LoopReceiveDataAndProcess.breakModuleCommand == 1:
        #         BreakModule().runCommand(1)
        #         LoopReceiveDataAndProcess.breakModuleCommand = 0

        # #command: get UPS info
        # elif len(unpacked_content) > 5 and unpacked_content[4]==0 and unpacked_content[5]==2:
        #     print('handle get UPS info')
        #     upsInfo = getUpsInfo()

        #     #response
        #     data = [0x69, 0x74, 0x72, 0x69, 0x00, 0x02, 0x04, 0x00, 0x20]
        #     data.extend(list(upsInfo))
        #     packerContent = struct.Struct('I'+ str(len(data)) + 'B')
        #     packedContent = packerContent.pack(len(data), *data)
        #     try:
        #         sock.sendall(packedContent)
        #     except socket.error as msg:
        #         print('Caught exception socket.error : ', msg)
        #         LoopSendAlive.bWillStop = True
        #         break

        # #command: axle
        # elif len(unpacked_content) > 11 and unpacked_content[4]==0 and unpacked_content[5]==4:
        #     runDirection = unpacked_content[9]
        #     brake = unpacked_content[10]
        #     pwm = unpacked_content[11]

            
        #     #save to temp file for ros2 to read
        #     #---------------------------------------
        #     '''
        #     messageToSend = {
        #         'dir' : runDirection,\
        #         'pwm' : pwm, \
        #         'parking' : brake}
        #     storeData(messageToSend)
        #     '''
        #     ros2DictLock.acquire()
        #     axleDict['dir'] = runDirection
        #     axleDict['pwm'] = pwm
        #     axleDict['parking'] = brake
        #     ros2DictLock.release()
        #     #---------------------------------------
        #     '''
        #     dataToWrite = 'MyID=' + str(myID) +  \
        #         ' dir=' + str(runDirection) +  \
        #         ' pwm = ' + str(pwm) + \
        #         ' parking = ' + str(brake)
        #     strFilePath = home + '/ros2_' + str(myID)
        #     f = open(strFilePath, 'w+')
        #     f.write(dataToWrite)
        #     f.close
        #     '''
            
            
        #     #response
        #     data = (0x69, 0x74, 0x72, 0x69, 0x00, 0x04, 0x10, 0x00, 0x00)
        #     packerContent = struct.Struct('I'+ str(len(data)) + 'B')
        #     packedContent = packerContent.pack(len(data), *data)
        #     try:
        #         sock.sendall(packedContent)
        #     except socket.error as msg:
        #         print('Caught exception socket.error : ', msg)
        #         LoopSendAlive.bWillStop = True
        #         break

        #     if brake:
        #         AxleModule().runCommand(brake=True)
        #         lastRunDirection = 0
        #         lastPwm = 0
        #     else:
        #         if runDirection == 1:
        #             AxleModule().runCommand(runDirection = 1, pwm = pwm, lastPwm = lastPwm, lastRunDirection = lastRunDirection)
        #         elif runDirection == 2:
        #             AxleModule().runCommand(runDirection = -1, pwm = pwm, lastPwm = lastPwm, lastRunDirection = lastRunDirection)
        #         elif runDirection == 0:
        #             AxleModule().runCommand(runDirection = 0, pwm = pwm, lastPwm = lastPwm, lastRunDirection = lastRunDirection)
        #         lastPwm = pwm
        #         lastRunDirection = runDirection
        # #command:breaking motor initial
        # elif len(unpacked_content) > 5 and unpacked_content[4]==0 and unpacked_content[5]==5:
        #     print('handle Break motor initialization')
        #     BreakModule.Initialization()
        #     #response
        #     data = [0x69, 0x74, 0x72, 0x69, 0x00, 0x05, 0x10, 0x00, 0x00]
        #     packerContent = struct.Struct('I'+ str(len(data)) + 'B')
        #     packedContent = packerContent.pack(len(data), *data)
        #     try:
        #         sock.sendall(packedContent)
        #     except socket.error as msg:
        #         print('Caught exception socket.error : ', msg)
        #         LoopSendAlive.bWillStop = True
        #         break
        # #command:breaking motor setting
        # elif len(unpacked_content) > 5 and unpacked_content[4]==0 and unpacked_content[5]==6:
        #     print('handle Break motor set')
        #     #distance = ord(unpacked_content[9])*256 + ord(unpacked_content[10])
        #     distance = unpacked_content[9]*256 + unpacked_content[10]
        #     print("distance = ", distance)
        #     BreakModule.SetDistance(distance)
        #     #response
        #     data = [0x69, 0x74, 0x72, 0x69, 0x00, 0x06, 0x10, 0x00, 0x00]
        #     packerContent = struct.Struct('I'+ str(len(data)) + 'B')
        #     packedContent = packerContent.pack(len(data), *data)
        #     try:
        #         sock.sendall(packedContent)
        #     except socket.error as msg:
        #         print('Caught exception socket.error : ', msg)
        #         LoopSendAlive.bWillStop = True
        #         break
        # #command:trunig motor initial
        # elif len(unpacked_content) > 5 and unpacked_content[4]==0 and unpacked_content[5]==7:
        #     print('handle Turning motor initialization')
        #     TurningModule.Initialization()
        #     #response
        #     data = [0x69, 0x74, 0x72, 0x69, 0x00, 0x05, 0x10, 0x00, 0x00]
        #     packerContent = struct.Struct('I'+ str(len(data)) + 'B')
        #     packedContent = packerContent.pack(len(data), *data)
        #     try:
        #         sock.sendall(packedContent)
        #     except socket.error as msg:
        #         print('Caught exception socket.error : ', msg)
        #         LoopSendAlive.bWillStop = True
        #         break
        # #command:turning motor setting
        # elif len(unpacked_content) > 5 and unpacked_content[4]==0 and unpacked_content[5]==8:
        #     print('handle Tuening motor set')
        #     #distance = ord(unpacked_content[9])*256 + ord(unpacked_content[10])
        #     distance = unpacked_content[9]*256 + unpacked_content[10]
        #     print("distance = ", distance)
        #     TurningModule.SetDistance(distance)
        #     #response
        #     data = [0x69, 0x74, 0x72, 0x69, 0x00, 0x06, 0x10, 0x00, 0x00]
        #     packerContent = struct.Struct('I'+ str(len(data)) + 'B')
        #     packedContent = packerContent.pack(len(data), *data)
        #     try:
        #         sock.sendall(packedContent)
        #     except socket.error as msg:
        #         print('Caught exception socket.error : ', msg)
        #         LoopSendAlive.bWillStop = True
        #         break
        # else:
        #     print('ERROR: insufficient axle data')

def storeData(message): 
    # database 
    db = {} 
    db['device ' + str(myID) + ' message'] = message 
    # Its important to use binary mode 
    dbfile = open(home+'pickle', 'wb')
    # source, destination 
    pickle.dump(db, dbfile)                   
    dbfile.close() 

LoopReceiveDataAndProcess.breakModuleCommand = False
LoopReceiveDataAndProcess.bWillStop = False

def LoopSendAlive(sock):
    while LoopSendAlive.bWillStop == False:
        data = (0x42, 0x42, 0x42, 0x42)
        packerContent = struct.Struct('I'+ str(len(data)) + 'B')
        packedContent = packerContent.pack(len(data), *data)
        try:
            sock.sendall(packedContent)
        except socket.error as msg:
            print('Caught exception socket.error : ', msg)
            LoopReceiveDataAndProcess.bWillStop = True
            break

        try:
            data1 = sock.recv(unpackerLength.size)
        except socket.error as msg:
            print('Caught exception socket.error : ', msg)
            LoopReceiveDataAndProcess.bWillStop = True
            break
        unpacked_length = unpackerLength.unpack(data1)[0]
        try:
            data2 = sock.recv(unpacked_length)
        except socket.error as msg:
            print('Caught exception socket.error : ', msg)
            LoopReceiveDataAndProcess.bWillStop = True
            break
        unpackerContent = struct.Struct(str(unpacked_length) + 'B')
        unpacked_content = unpackerContent.unpack(data2)
        #print('get response ', unpacked_content)

        time.sleep((TIME_BETWEEN_ALIVE_SIGNALS - 1) + random.random())
LoopSendAlive.bWillStop = False
        
        
def sockConnect(his_ip):
    # Create a TCP/IP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((his_ip, 10002))
    return sock

def sockConnectImAlive(his_ip):
    # Create a TCP/IP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((his_ip, 10003))
    return sock

def MakeSureGotLanIp():
    myIpAddr = (str)(check_output(['hostname', '-I'])).strip('b\'')
    print('ip = ', myIpAddr)
    while not myIpAddr.startswith('192.168'):
        time.sleep(1)
        myIpAddr = (str)(check_output(['hostname', '-I'])).strip('b\'')
        print('ip = ', myIpAddr)
    print('get lan ip = ', myIpAddr)
    

def SendDataAndGetResponse(sock, data):

    packerContent = struct.Struct('I'+ str(len(data)) + 'B')
    packedContent = packerContent.pack(len(data), *data)
    try:
        # Send data
        sock.sendall(packedContent)
        #print('sent "%s"' % binascii.hexlify(packedContent))
        # Receive data
        data1 = sock.recv(unpackerLength.size)
        #print('received "%s"' % binascii.hexlify(data1))
        unpacked_length = unpackerLength.unpack(data1)[0]
        data2 = sock.recv(unpacked_length)
        unpackerContent = struct.Struct(str(unpacked_length) + 'B')
        unpacked_content = unpackerContent.unpack(data2)
        #print('unpacked content:', unpacked_content)  

    
    finally:
        #print('closing socket')
        #sock.close()
        pass
    
    return unpacked_content

def sendCommandTellId(sock):
    sendOutData = (0x69, 0x74, 0x72, 0x69, 0x00, 0x03, 0x01, 0x00, 0x00, myID)
    receivedData = SendDataAndGetResponse(sock, data=sendOutData)
    return receivedData



if __name__ == '__main__':

    myID = int(sys.argv[1])

    MakeSureGotLanIp()

    dataSock = sockConnect(serverIp)
    sendCommandTellId(dataSock)
    handler = Thread(target=LoopReceiveDataAndProcess, args=(dataSock, ))
    handler.start()

    imAliveSock = sockConnectImAlive(serverIp)
    receivedData = sendCommandTellId(imAliveSock)
    handler1 = Thread(target=LoopSendAlive, args=(imAliveSock, ))
    handler1.start()

    moduleProcessTh = Thread(target=ModuleProcess, args=(0.1, ))

    
    while True:
        if LoopReceiveDataAndProcess.bWillStop or LoopSendAlive.bWillStop:
            time.sleep(2)
            os.execv(__file__, sys.argv)
            break
        time.sleep(2)


    #test()