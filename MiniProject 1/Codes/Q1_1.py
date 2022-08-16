import time
import serial
import numpy as np

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

toRad = np.pi/180.0
toDeg = 1/toRad

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
            print(rotmat)
            print('Determinant: ')
            print(np.linalg.det(rotmat))
            print('\n\n')
    except:
        pass