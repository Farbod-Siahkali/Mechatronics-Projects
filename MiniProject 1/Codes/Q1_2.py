import time
import serial
import numpy as np
from vpython import *

np.version.version
def euler_to_rotMat(yaw, pitch, roll):
    Rz_yaw = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                       [np.sin(yaw),  np.cos(yaw), 0],
                       [          0,            0, 1]])
    Ry_pitch = np.array([[ np.cos(pitch), 0, np.sin(pitch)],
                        [             0, 1,             0],
                        [-np.sin(pitch), 0, np.cos(pitch)]])
    Rx_roll = np.array([[1,            0,             0],
                        [0, np.cos(roll), -np.sin(roll)],
                        [0, np.sin(roll),  np.cos(roll)]])
    rotMat = np.dot(Rz_yaw, np.dot(Ry_pitch, Rx_roll))
    return rotMat

arduinoData = serial.Serial('com10', 115200)
time.sleep(1)

scene.range=5
scene.background=color.black

toRad = np.pi/180.0
toDeg = 1/toRad

# Simulate your object using VPython
scene.forward=vector(-1,-1,-1)
scene.width=1200
scene.height=1080

frontArrow=arrow(length=3,shaftwidth=.1,color=color.purple,axis=vector(1,0,0))
upArrow=arrow(length=3,shaftwidth=.1,color=color.magenta,axis=vector(0,1,0))
sideArrow=arrow(length=3,shaftwidth=.1,color=color.orange,axis=vector(0,0,1))
 
bBoard=box(length=6,width=2,height=.2,opacity=.8,pos=vector(0,0,0,))
bn=box(length=1,width=.75,height=.1, pos=vector(-.5,.1+.05,0),color=color.blue)
nano=box(lenght=1.75,width=.6,height=.1,pos=vector(-2,.1+.05,0),color=color.green)
obj=compound([bBoard,bn,nano])

counter = 0

while True:
    while arduinoData.inWaiting() == 0:
        pass
    dataPacket = arduinoData.readline()
    try:
        dataPacket = str(dataPacket, 'utf-8')
        splitPacket = dataPacket.split(",")
        roll = float(splitPacket[0])*toRad
        pitch = float(splitPacket[1])*toRad
        yaw = float(splitPacket[2])*toRad+np.pi
        counter += 1
        if counter % 300 == 0:
            print('roll, pitch and yaw: ')
            print(roll*toDeg, pitch*toDeg, yaw*toDeg)
            print('Rotation Matrix: ')
            rotmat = euler_to_rotMat(yaw, pitch, roll)
            scene.title = str(rotmat)
            print(rotmat)
            print('Determinant: ')
            print(np.linalg.det(rotmat))
            print('\n\n')
        
        k = vector(cos(yaw)*cos(pitch), sin(pitch), sin(yaw)*cos(pitch))
        y = vector(0,1,0)
        s = cross(k,y)
        v = cross(s,k)
        vrot = v*cos(roll)+cross(k,v)*sin(roll)

        frontArrow.axis = k
        sideArrow.axis = cross(k, vrot)
        upArrow.axis = vrot
        obj.axis = k
        obj.up = vrot
    except:
        pass