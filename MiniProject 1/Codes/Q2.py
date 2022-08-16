import cv2
import mediapipe as mp
from vpython import *
import numpy as np
import time


mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_hands = mp.solutions.hands
hand_landmarks_index = np.array([0, 4, 17, 20]) #Wrist, Index Finger MCP, Pincky MCP
# For static images:
IMAGE_FILES = []


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
# For webcam input:
cap = cv2.VideoCapture(0)
with mp_hands.Hands(
    model_complexity=0,
    max_num_hands=1,
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5) as hands:
  while cap.isOpened():
    success, image = cap.read()
    # To improve performance, optionally mark the image as not writeable to
    # pass by reference.
    image.flags.writeable = False
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    results = hands.process(image)
    first_transformation_matrix=np.zeros((3,3))
    # Draw the hand annotations on the image.
    image.flags.writeable = True
    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    if results.multi_hand_landmarks:
        
        for hand_landmarks in results.multi_hand_landmarks:
           
            Source = hand_landmarks.landmark[mp_hands.HandLandmark(0).value]
            
            point5 = hand_landmarks.landmark[mp_hands.HandLandmark(5).value]
        
            point17 = hand_landmarks.landmark[mp_hands.HandLandmark(17).value]
            #the first hand that ditected
            if not np.all(first_transformation_matrix):
              x = vector(point5.x-Source.x,point5.y-Source.y,point5.z-Source.z)
              tempvect1 = vector(point17.x-Source.x,point17.y-Source.y,point17.z-Source.z)
              z = cross(tempvect1,x)
              y = cross(z,x)
              z = norm(z)
              y = norm(y)
              x = norm(x)
              first_transformation_matrix=[[x.x,x.y,x.z],[y.x,y.y,y.z],[z.x,z.y,z.z]]
            x = vector(point5.x-Source.x,point5.y-Source.y,point5.z-Source.z)
            tempvect1 = vector(point17.x-Source.x,point17.y-Source.y,point17.z-Source.z)
            z = cross(tempvect1,x)
            y = cross(z,x)
            z = norm(z)
            y = norm(y)
            x = norm(x)
            counter += 1
            if counter % 20 == 1:
              transformation_matrix=[[x.x,x.y,x.z],[y.x,y.y,y.z],[z.x,z.y,z.z]]*np.linalg.inv(first_transformation_matrix)
              print('Rotation Matrix: ')
              print(transformation_matrix[0])
              print(transformation_matrix[1])
              print(transformation_matrix[2])
              print('Determinant: ')
              print(np.linalg.det([[x.x,x.y,x.z],[y.x,y.y,y.z],[z.x,z.y,z.z]]))
              print('\n\n')
            obj.axis = -x
            obj.up = -z
            scene.title = (str(transformation_matrix))
            mp_drawing.draw_landmarks(
                image,
                hand_landmarks,
                mp_hands.HAND_CONNECTIONS,
                mp_drawing_styles.get_default_hand_landmarks_style(),
                mp_drawing_styles.get_default_hand_connections_style())
    # Flip the image horizontally for a selfie-view display.
    cv2.imshow('MediaPipe Hands', cv2.flip(image, 1))
    if cv2.waitKey(5) & 0xFF == 27:
      break
cap.release()


