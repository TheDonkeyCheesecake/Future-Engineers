#import necessary libraries
import cv2
import numpy as np
from picamera2 import Picamera2
import serial
from time import sleep
import RPi.GPIO as GPIO

#function used to send signals to arduino to control speeds of the dc motor and the angle of the servo motor
def write(value): 
    ser.write((str(value)).encode('utf-8'))
    print("signal sent:", value)

#function which displays the Regions of Interest on the image
def displayROI(img, ROI1, ROI2):
    image = cv2.line(img, (ROI1[0], ROI1[1]), (ROI1[2], ROI1[1]), (0, 255, 255), 4)
    image = cv2.line(img, (ROI1[0], ROI1[1]), (ROI1[0], ROI1[3]), (0, 255, 255), 4)
    image = cv2.line(img, (ROI1[2], ROI1[3]), (ROI1[2], ROI1[1]), (0, 255, 255), 4)
    image = cv2.line(img, (ROI1[2], ROI1[3]), (ROI1[0], ROI1[3]), (0, 255, 255), 4)
    
    image = cv2.line(img, (ROI2[0], ROI2[1]), (ROI2[2], ROI2[1]), (0, 255, 255), 4)
    image = cv2.line(img, (ROI2[0], ROI2[1]), (ROI2[0], ROI2[3]), (0, 255, 255), 4)
    image = cv2.line(img, (ROI2[2], ROI2[3]), (ROI2[2], ROI2[1]), (0, 255, 255), 4)
    image = cv2.line(img, (ROI2[2], ROI2[3]), (ROI2[0], ROI2[3]), (0, 255, 255), 4)

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
    ROI1 = [10, 215, 270, 295]
    ROI2 = [380, 180, 640, 255]
    
    inTurn = False

    #list storing the previous angles of the car
    #used for counting turns
    prevAng = [2098]

    l = 40 #how many angles prevAng list stores
    t = 0 #number of turns car has completed
    
    kp = 0.004 #value of proportional for proportional steering
    kd = 0.004 #value of derivative for proportional and derivative sterring
    
    straightConst = 98 #angle in which car goes straight
    
    angle = 2098 #variable for the current angle of the car
    prevAngle = angle
    tDeviation = 20 #value used to calculate the sharpest turns left and right the car can make
    sharpRight = straightConst - tDeviation + 2000 #the sharpest right the car may turn
    sharpLeft = straightConst + tDeviation + 2000 #the sharpest left the car may turn
    
    speed = 1430 #variable for the speed of the car
    targetS = 1440 #value speed will reach for acceleration
    
    aDiff = 0 #value storing the difference of area between contours
    prevDiff = 0 #value storing the previous difference of contours for derivative steering

    sleep(8) #delay 8 seconds for the servo to be ready

    #if button is pressed break out of loop and proceed with rest of program
    #while True:
        #if GPIO.input(5) == GPIO.LOW:
            #break

    #write initial values to car
    write(speed) 
    write(angle)

    #main loop
    while True:

        #accelerate if not at target speed
        if speed != targetS:
            speed += 1
          
        #declare variables for the areas of the left and right contours
        rightArea, leftArea = 0, 0

        #get an image from pi camera
        img = picam2.capture_array()

        #convert image into grayscale
        imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        #threshold image
        ret, imgThresh = cv2.threshold(imgGray, 40, 255, cv2.THRESH_BINARY_INV)

        #find left and right contours of the lanes
        contours_left, hierarchy = cv2.findContours(imgThresh[ROI1[1]:ROI1[3], ROI1[0]:ROI1[2]], 
        cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    
        contours_right, hierarchy = cv2.findContours(imgThresh[ROI2[1]:ROI2[3], ROI2[0]:ROI2[2]], 
        cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        
        contours, hierarchy = cv2.findContours(imgThresh, 
        cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        #iterate through every contour in both the left and right region of interest and take the largest one in each
        for cnt in contours_left:
            area = cv2.contourArea(cnt)
            
            leftArea = max(area, leftArea) 

        for cnt in contours_right:
            area = cv2.contourArea(cnt)

            rightArea = max(area, rightArea)
        
        for i in range(len(contours)):
            cnt = contours[i]
            area = cv2.contourArea(cnt)
            
            cv2.drawContours(img, contours, i, (0, 255, 0), 2)
            approx=cv2.approxPolyDP(cnt, 0.01*cv2.arcLength(cnt,True),True)
            x,y,w,h=cv2.boundingRect(approx)
            #cv2.rectangle(img,(x,y),(x+w,y+h),(0,0,255),2)
         
        #calculate difference of areas between the areas of the lanes
        aDiff = rightArea - leftArea
        aSum = rightArea + leftArea
       
        angle = int(straightConst + aDiff * kp + (aDiff - prevDiff) * kd) + 2000

        #if angle is within 5 degrees of the sharpLeft or sharpRight value change angle accordingly
        if leftArea <= 300 and not inTurn:
            angle = max(angle, sharpLeft) 
            inTurn = True

        elif rightArea <= 300 and not inTurn:
            angle = min(angle, sharpRight) 
            inTurn = True
        
        #if 20 or more of the previous 40 angle values are sharpRight or sharpLeft indicating a turn
        #if (prevAng.count(sharpLeft) >= 20 or prevAng.count(sharpRight) >= 20) and (angle != sharpLeft and angle != sharpRight):
            #prevAng = [angle] #reset list 
            #t += 1 #add one turn
        
       
        if angle != prevAngle:
            if angle >= sharpLeft or angle <= sharpRight and inTurn and abs(angle - prevAngle)  < tDeviation:
                write(angle) 
            if (leftArea >= 1500 and rightArea >= 1500 and inTurn) or not inTurn: 
                write(angle)
                if inTurn:
                    inTurn = False
                    t += 1
          
        #update previous area difference
        prevDiff = aDiff
            
        #stop the car and end the program if either q is pressed or the car has done 3 laps (12 turns) and is not still in the turn
        if cv2.waitKey(1)==ord('q') or (t == 12 and abs(angle - (straightConst + 2000)) <= 10) :
            stopCar() 
            break

        #display values
        print(leftArea, rightArea)
        print(aSum) 
        #print(t)
        print(inTurn)
        print(angle)
        print(prevAngle)
        
        prevAngle = angle
        
        #display regions of interest
        displayROI(img, ROI1, ROI2)
        cv2.imshow("finalColor", img)
        
    #close all image windows
    cv2.destroyAllWindows()
