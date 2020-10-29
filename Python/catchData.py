import serial
import time
arduinoData=serial.Serial('com4',115200)
time.sleep(1)
while (1==1):
    while (arduinoData.inWaiting()==0):
        pass
    dataPacket=arduinoData.readline()
    print(dataPacket)
