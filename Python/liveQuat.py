from vpython import *
from time import *
import numpy as np
import math
import serial
ad=serial.Serial('com4',115200)
sleep(1)

scene.range=5
scene.background=color.yellow
toRad=2*np.pi/360
toDeg=1/toRad
scene.forward=vector(-1,-1,-1)

scene.width=1200
scene.height=1080

xarrow=arrow(lenght=2, shaftwidth=.1, color=color.red,axis=vector(1,0,0))
yarrow=arrow(lenght=2, shaftwidth=.1, color=color.green,axis=vector(0,1,0))
zarrow=arrow(lenght=4, shaftwidth=.1, color=color.blue,axis=vector(0,0,1))

bBoard=box(length=2,width=2,height=3,opacity=.8,pos=vector(0,0,0,))
while (True):
    try:
        while (ad.inWaiting()==0):
            pass
        dataPacket=ad.readline()
        dataPacket=str(dataPacket,'utf-8')
        splitPacket=dataPacket.split(",")
        roll=float(splitPacket[0])
        pitch=float(splitPacket[1])
        yaw=float(splitPacket[2])

        rate(50)
        k=vector(cos(yaw)*cos(pitch), sin(pitch),sin(yaw)*cos(pitch))
        y=vector(0,1,0)
        s=cross(k,y)
        v=cross(s,k)
        vrot=v*cos(roll)+cross(k,v)*sin(roll)

        bBoard.axis=k
        bBoard.up=vrot
    except:
        pass
