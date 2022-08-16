import time
import serial
import numpy as np
from vpython import *

def quaternion_to_rotMat(Q):
    q0 = Q[0]
    q1 = Q[1]
    q2 = Q[2]
    q3 = Q[3]
    r00 = 2 * (q0 * q0 + q1 * q1) - 1
    r01 = 2 * (q1 * q2 - q0 * q3)
    r02 = 2 * (q1 * q3 + q0 * q2)
    r10 = 2 * (q1 * q2 + q0 * q3)
    r11 = 2 * (q0 * q0 + q2 * q2) - 1
    r12 = 2 * (q2 * q3 - q0 * q1)
    r20 = 2 * (q1 * q3 - q0 * q2)
    r21 = 2 * (q2 * q3 + q0 * q1)
    r22 = 2 * (q0 * q0 + q3 * q3) - 1
    rot_matrix = np.array([[r00, r01, r02],
                           [r10, r11, r12],
                           [r20, r21, r22]])               
    return rot_matrix

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

import math

counter = 0

while (True):
    try:
        while (arduinoData.inWaiting()==0):
            pass
        dataPacket=arduinoData.readline()
        dataPacket=str(dataPacket,'utf-8')
        splitPacket=dataPacket.split(",")
        q0=float(splitPacket[0])
        q1=float(splitPacket[1])
        q2=float(splitPacket[2])
        q3=float(splitPacket[3])
        counter += 1
        if counter % 200 == 0:
            print('Quarternion:')
            print(q0, q1, q2, q3)
            print('Rotation Matrix:')
            rotmat = quaternion_to_rotMat([q0,q1,q2,q3])
            print(rotmat)
            scene.title = str(rotmat)+str(np.linalg.det(rotmat))
            print('Determinant: ')
            print(np.linalg.det(rotmat))
            print('\n\n')
        roll=math.atan2(2*(q0*q1+q2*q3),1-2*(q1*q1+q2*q2))
        pitch=-math.asin(2*(q0*q2-q3*q1))
        yaw=-math.atan2(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3))-np.pi/2
        #rate(200)
        k=vector(cos(yaw)*cos(pitch), sin(pitch),sin(yaw)*cos(pitch))
        y=vector(0,1,0)
        s=cross(k,y)
        v=cross(s,k)
        vrot=v*cos(roll)+cross(k,v)*sin(roll)
 
        frontArrow.axis=k
        sideArrow.axis=cross(k,vrot)
        upArrow.axis=vrot
        obj.axis=k
        obj.up=vrot
    except:
        pass
