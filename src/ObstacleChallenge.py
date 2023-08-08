#import necessary libraries
import cv2
from picamera2 import Picamera2
import serial
from time import sleep
import RPi.GPIO as GPIO
import numpy as np


#idea for obstacle challenge
'''
- 3 regions of interest: 
    - left wall
    - middle for signal pillars
    - right wall

stay in middle
if signal pillar detected in middle ROI move to the left or right at a fixed angle until current signal pillar is out of centre ROI or the lane car is approaching reaches a certain area threshold. Then come back to the middle and repeat. 
'''

#function used to send signals to arduino to control speeds of the dc motor and the angle of the servo motor
def write(value): 
    ser.write((str(value)).encode('utf-8'))
    print("signal sent:", value)

#function which displays the Regions of Interest on the image
def displayROI(img, ROI1, ROI2, ROI3):
    image = cv2.line(img, (ROI1[0], ROI1[1]), (ROI1[2], ROI1[1]), (0, 255, 255), 4)
    image = cv2.line(img, (ROI1[0], ROI1[1]), (ROI1[0], ROI1[3]), (0, 255, 255), 4)
    image = cv2.line(img, (ROI1[2], ROI1[3]), (ROI1[2], ROI1[1]), (0, 255, 255), 4)
    image = cv2.line(img, (ROI1[2], ROI1[3]), (ROI1[0], ROI1[3]), (0, 255, 255), 4)
    
    image = cv2.line(img, (ROI2[0], ROI2[1]), (ROI2[2], ROI2[1]), (0, 255, 255), 4)
    image = cv2.line(img, (ROI2[0], ROI2[1]), (ROI2[0], ROI2[3]), (0, 255, 255), 4)
    image = cv2.line(img, (ROI2[2], ROI2[3]), (ROI2[2], ROI2[1]), (0, 255, 255), 4)
    image = cv2.line(img, (ROI2[2], ROI2[3]), (ROI2[0], ROI2[3]), (0, 255, 255), 4)

    image = cv2.line(img, (ROI3[0], ROI3[1]), (ROI3[2], ROI3[1]), (0, 255, 255), 4)
    image = cv2.line(img, (ROI3[0], ROI3[1]), (ROI3[0], ROI3[3]), (0, 255, 255), 4)
    image = cv2.line(img, (ROI3[2], ROI3[3]), (ROI3[2], ROI3[1]), (0, 255, 255), 4)
    image = cv2.line(img, (ROI3[2], ROI3[3]), (ROI3[0], ROI3[3]), (0, 255, 255), 4)

#function to bring the car to a stop
def stopCar():
    write(2098)
    write(1500)

if __name__ == '__main__':

    #initialize camera
    picam2 = Picamera2()
    picam2.preview_configuration.main.size = (640,480)
    picam2.preview_configuration.main.format = "RGB888"
    picam2.preview_configuration.controls.FrameRate = 60
    picam2.preview_configuration.align()
    picam2.configure("preview")
    picam2.start()

    #initialize serial port
    ser = serial.Serial('/dev/ttyACM0', 115200, timeout = 1) 
    ser.flush()
  
    #initialize GPIO pins for button
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(5, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    #lists storing coordinates for the regions of interest to find contours of the lanes
    # order: x1, y1, x2, y2
    ROI1 = [45, 230, 315, 295]
    ROI2 = [380, 205, 640, 255]
    ROI3 = [240, 200, 500, 330]

    #booleans for tracking whether car is in a left or right turn
    lTurn = False
    rTurn = False
  
    kp = 0.005 #value of proportional for proportional steering
    kd = 0.005  #value of derivative for proportional and derivative sterrin
    
    straightConst = 98 #angle in which car goes straight

    turnThresh = 200 #if area of a lane is under this threshold car goes into a turn
    exitThresh = 1500 #if area of both lanes is over this threshold car exits a turn
  
    angle = 2098 #variable for the current angle of the car
    prevAngle = angle #variable tracking the angle of the previous iteration
    tDeviation = 33 #value used to calculate the how far left and right the car turns during a turn
    sharpRight = straightConst - tDeviation + 2000 #the default angle sent to the car during a right turn
    sharpLeft = straightConst + tDeviation + 2000 #the default angle sent to the car during a left turn
    
    speed = 1425 #variable for the speed of the car
    
    aDiff = 0 #value storing the difference of area between contours
    prevDiff = 0 #value storing the previous difference of contours for derivative steering

    redTarget = 70
    greenTarget = 570

    sleep(8) #delay 8 seconds for the servo to be ready

    #if button is pressed break out of loop and proceed with rest of program
    #while True:
        #if GPIO.input(5) == GPIO.LOW:
            #break

    #write initial values to car
    #write(speed) 
    write(angle)

    #main loop
    while True:

        #get an image from pi camera
        img = picam2.capture_array()

        # convert from BGR to HSV
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # Lower red mask (0-10)
        lower_red = np.array([0, 100, 20])
        upper_red = np.array([10, 255, 255])
        r_mask0 = cv2.inRange(img_hsv, lower_red, upper_red)

        # Upper red mask (170-180)
        lower_red = np.array([160, 100, 20])
        upper_red = np.array([175, 255, 255])
        r_mask1 = cv2.inRange(img_hsv, lower_red, upper_red)

        # Join the masks
        raw_mask_r = r_mask0 | r_mask1

        contours_red = cv2.findContours(r_mask1[ROI3[1]:ROI3[3], ROI3[0]:ROI3[2]], cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE)[-2]# Find contours

        # green mask
        lower_green = np.array([60, 100, 20])
        upper_green = np.array([90, 255, 255])

        g_mask = cv2.inRange(img_hsv, lower_green, upper_green)

        contours_green = cv2.findContours(g_mask[ROI3[1]:ROI3[3], ROI3[0]:ROI3[2]], cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE)[-2]# Find contours

        for i in range(len(contours_green)):
          cnt = contours_green[i]
          area = cv2.contourArea(cnt)
          print(area)
          if(area > 100):
              #cv2.drawContours(img, contours_green, i, (0, 255, 0), 2)
              approx=cv2.approxPolyDP(cnt, 0.01*cv2.arcLength(cnt,True),True)
              x,y,w,h=cv2.boundingRect(approx)
              
              x += ROI3[0]
              y += ROI3[1]
              
              cv2.rectangle(img,(x,y),(x+w,y+h),(0,0,255),2)
            


        for i in range(len(contours_red)):
          cnt = contours_red[i]
          area = cv2.contourArea(cnt)
          print(area)
          if(area > 100):
              #cv2.drawContours(img, contours_red, i, (0, 255, 0), 2)
              approx=cv2.approxPolyDP(cnt, 0.01*cv2.arcLength(cnt,True),True)
              x,y,w,h=cv2.boundingRect(approx)
              
              x += ROI3[0]
              y += ROI3[1]
              
              cv2.rectangle(img,(x,y),(x+w,y+h),(0,0,255),2)
            
        if cv2.waitKey(1)==ord('q'):
            stopCar() 
            break

        prevAngle = angle #update previous angle
        
        #display regions of interest
        displayROI(img, ROI1, ROI2, ROI3)

        #show image
        cv2.imshow("finalColor", img)
        
    #close all image windows
    cv2.destroyAllWindows()