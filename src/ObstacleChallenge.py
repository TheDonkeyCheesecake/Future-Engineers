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
    
    image = cv2.line(img, (ROI4[0], ROI4[1]), (ROI4[2], ROI4[1]), (0, 255, 255), 4)
    image = cv2.line(img, (ROI4[0], ROI4[1]), (ROI4[0], ROI4[3]), (0, 255, 255), 4)
    image = cv2.line(img, (ROI4[2], ROI4[3]), (ROI4[2], ROI4[1]), (0, 255, 255), 4)
    image = cv2.line(img, (ROI4[2], ROI4[3]), (ROI4[0], ROI4[3]), (0, 255, 255), 4)

#function to bring the car to a stop
def stopCar():
    write(2098)
    write(1500)

if __name__ == '__main__':

    #initialize camera
    picam2 = Picamera2()
    picam2.preview_configuration.main.size = (640,480)
    picam2.preview_configuration.main.format = "RGB888"
    picam2.preview_configuration.controls.FrameRate = 90
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

    
    redTarget = 150
    greenTarget = 490
    
    turnDir = "none" 
    
    #lists storing coordinates for the regions of interest to find contours of the lanes
    # order: x1, y1, x2, y2
    ROI1 = [25, 220, 260, 300]
    ROI2 = [400, 185, 640, 275]
    ROI3 = [redTarget - 30, 175, greenTarget + 30, 400]
    ROI4 = [200, 350, 440, 400]

    #booleans for tracking whether car is in a left or right turn
    lTurn = False
    rTurn = False
  
    kp = 0.003 #value of proportional for proportional steering
    kd = 0.003  #value of derivative for proportional and derivative sterring

    cKp = 0.2
    cKd = 0.2
    cy = 0.2
  
    straightConst = 98 #angle in which car goes straight

    turnThresh = 200 #if area of a lane is under this threshold car goes into a turn
    exitThresh = 3000 #if area of both lanes is over this threshold car exits a turn
  
    angle = 2098 #variable for the current angle of the car
    prevAngle = angle #variable tracking the angle of the previous iteration
    tDeviation = 40 #value used to calculate the how far left and right the car turns during a turn
    sharpRight = straightConst - tDeviation + 2000 #the default angle sent to the car during a right turn
    sharpLeft = straightConst + tDeviation + 2000 #the default angle sent to the car during a left turn
    
    speed = 1430 #variable for the speed of the car
    targetS = 1440
    
    aDiff = 0 #value storing the difference of area between contours
    prevDiff = 0 #value storing the previous difference of contours for derivative steering

    error = 0
    prevError = 0

    cTarget = 0
    contX = 0
    contY = 0
    
    t = 0
    i = 0
    
    tSignal = False

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
        
        if speed != targetS and i % 4 == 0:
            speed += 1
            #write(speed)
            

        rightArea, leftArea = 0, 0

        #get an image from pi camera
        img = picam2.capture_array()

        imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        #threshold image
        ret, imgThresh = cv2.threshold(imgGray, 55, 255, cv2.THRESH_BINARY_INV)

        #find left and right contours of the lanes
        contours_left, hierarchy = cv2.findContours(imgThresh[ROI1[1]:ROI1[3], ROI1[0]:ROI1[2]], 
        cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    
        contours_right, hierarchy = cv2.findContours(imgThresh[ROI2[1]:ROI2[3], ROI2[0]:ROI2[2]], 
        cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

         #iterate through every contour in both the left and right region of interest and take the largest one in each
        for cnt in contours_left:
            area = cv2.contourArea(cnt)
            
            leftArea = max(area, leftArea) 

        for cnt in contours_right:
            area = cv2.contourArea(cnt)

            rightArea = max(area, rightArea)

        # convert from BGR to HSV
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # Upper red mask (170-180)
        lower_red = np.array([165, 150, 100])
        upper_red = np.array([180, 255, 255])
        r_mask = cv2.inRange(img_hsv, lower_red, upper_red)
 

        contours_red = cv2.findContours(r_mask[ROI3[1]:ROI3[3], ROI3[0]:ROI3[2]], cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE)[-2]# Find contours

        # green mask
        lower_green = np.array([70, 100, 40])
        upper_green = np.array([95, 255, 255])

        g_mask = cv2.inRange(img_hsv, lower_green, upper_green)

        contours_green = cv2.findContours(g_mask[ROI3[1]:ROI3[3], ROI3[0]:ROI3[2]], cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE)[-2]# Find contours
        
        # blue mask
        lower_blue = np.array([100, 100, 100])
        upper_blue = np.array([135, 255, 255])

        b_mask = cv2.inRange(img_hsv, lower_blue, upper_blue)

        contours_blue = cv2.findContours(b_mask[ROI4[1]:ROI4[3], ROI4[0]:ROI4[2]], cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE)[-2]# Find contours
        
        # orange mask
        lower_orange = np.array([0, 100, 20])
        upper_orange = np.array([25, 255, 255])

        o_mask = cv2.inRange(img_hsv, lower_orange, upper_orange)

        contours_orange = cv2.findContours(o_mask[ROI4[1]:ROI4[3], ROI4[0]:ROI4[2]], cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE)[-2]# Find contours
        
        
        

        for i in range(len(contours_green)):
          cnt = contours_green[i]
          area = cv2.contourArea(cnt)
          #print(area)
          if(area > 80):
              #cv2.drawContours(img, contours_green, i, (0, 255, 0), 2)
              approx=cv2.approxPolyDP(cnt, 0.01*cv2.arcLength(cnt,True),True)
              x,y,w,h=cv2.boundingRect(approx)
              
              x += ROI3[0]
              y += ROI3[1]
              
              cv2.rectangle(img,(x,y),(x+w,y+h),(0,0,255),2)

              if y > contY: 
                contY = y
                contX = x + w // 2
                cTarget = greenTarget
            
        for i in range(len(contours_red)):
          cnt = contours_red[i]
          area = cv2.contourArea(cnt)
          #print(area)
          if(area > 80):
              #cv2.drawContours(img, contours_red, i, (0, 255, 0), 2)
              approx=cv2.approxPolyDP(cnt, 0.01*cv2.arcLength(cnt,True),True)
              x,y,w,h=cv2.boundingRect(approx)
              
              x += ROI3[0]
              y += ROI3[1]
              
              cv2.rectangle(img,(x,y),(x+w,y+h),(0,0,255),2)

              if y > contY: 
                contY = y
                contX = x + w // 2
                cTarget = redTarget
                
        for i in range(len(contours_orange)):
          cnt = contours_orange[i]
          area = cv2.contourArea(cnt)
          
          #print(area)
          if(area > 100):
              if turnDir == "none":
                  turnDir = "right"
              #cv2.drawContours(img[ROI4[1]:ROI4[3], ROI4[0]:ROI4[2]], contours_orange, i, (0, 255, 0), 2)
              if turnDir == "right":
                  rTurn = True
                  tSignal = True
        
        for i in range(len(contours_blue)):
          cnt = contours_blue[i]
          area = cv2.contourArea(cnt)
          #print(area)
          if(area > 100):
              if turnDir == "none":
                  turnDir = "left" 
              #cv2.drawContours(img[ROI4[1]:ROI4[3], ROI4[0]:ROI4[2]], contours_blue, i, (0, 255, 0), 2)
              if turnDir == "left":
                  lTurn = True
                  tSignal = True
              

        if cTarget == 0:
            
            aDiff = rightArea - leftArea

            #calculate angle using PD steering
            angle = int(straightConst + aDiff * kp + (aDiff - prevDiff) * kd) + 2000

            prevDiff = aDiff
            
            if t == 12:
                stopCar()
                exit()
            
            
            
        else:
            
            if (lTurn or rTurn) and not tSignal:
                print("pillar detected, end of turn", t)
                lTurn = False
                rTurn = False
                t += 1
            

            error = cTarget - contX
            
            angle = int(straightConst + error * cKp + (error - prevError) * cKd + cy * (contY - 170)) + 2000
            
            #print(angle)
            #print(cTarget)
            #print(contX)
            #print(error) 
            

            prevError = error
            contY = 0
            contX = 0
            cTarget = 0
              
            

        if angle != prevAngle:
            
            if ((rightArea >= exitThresh and rTurn) or (leftArea >= exitThresh and lTurn)) and not tSignal: 
                  #set turn variables to false as turn is over
                  lTurn = False 
                  rTurn = False
                  #increase number of turns by 1
                  t += 1
                  
                  print("lane detected, end of turn", t) 

            if rTurn:
                angle = sharpRight
                
            elif lTurn:
                angle = sharpLeft
                
            
            write(max(min(angle, sharpLeft), sharpRight))
                

            
        if cv2.waitKey(1)==ord('q'):
            stopCar() 
            break

      

        prevAngle = angle #update previous angle
        tSignal = False
        
        #display regions of interest
        displayROI(img, ROI1, ROI2, ROI3)

        #show image
        cv2.imshow("finalColor", img)
        
        i += 1
        print(leftArea, rightArea) 
        
    #close all image windows
    cv2.destroyAllWindows()


