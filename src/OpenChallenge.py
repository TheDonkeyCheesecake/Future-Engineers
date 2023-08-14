#import necessary libraries
import cv2
from picamera2 import Picamera2
import serial
from time import sleep
import RPi.GPIO as GPIO
import numpy as np

#function used to send signals to arduino to control speeds of the dc motor and the angle of the servo motor
def write(value): 
    ser.write((str(value)).encode('utf-8'))
    print("signal sent:", value)

#function which displays the  Regions of Interest on the image
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
    ROI1 = [65, 225, 315, 305]
    ROI2 = [380, 190, 630, 270]
    ROI3 = [200, 350, 440, 400]

    #booleans for tracking whether car is in a left or right turn
    lTurn = False
    rTurn = False
  
    t = 0 #number of turns car has completed
    i = 0 #counts the iterations of the main control loop 
    
    kp = 0.004 #value of proportional for proportional steering
    kd = 0.004  #value of derivative for proportional and derivative sterrin
    
    straightConst = 98 #angle in which car goes straight

    turnThresh = 100 #if area of a lane is under this threshold car goes into a turn
    exitThresh = 1000 #if area of both lanes is over this threshold car exits a turn
  
    angle = 2098 #variable for the current angle of the car
    prevAngle = angle #variable tracking the angle of the previous iteration
    tDeviation = 25 #value used to calculate the how far left and right the car turns during a turn
    sharpRight = straightConst - tDeviation + 2000 #the default angle sent to the car during a right turn
    sharpLeft = straightConst + tDeviation + 2000 #the default angle sent to the car during a left turn
    
    speed = 1435 #variable for the speed of the car
    
    aDiff = 0 #value storing the difference of area between contours
    prevDiff = 0 #value storing the previous difference of contours for derivative steering

    sleep(8) #delay 8 seconds for the servo to be ready

    #if button is pressed break out of loop and proceed with rest of program
    while True:
        if GPIO.input(5) == GPIO.LOW:
            break

    #write initial values to car
    write(speed) 
    write(angle)
    
    lDetected = False

    #main loop
    while True:
          
        #declare variables for the areas of the left and right contours
        rightArea, leftArea = 0, 0

        #get an image from pi camera
        img = picam2.capture_array()

        #convert image into grayscale
        imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        # convert from BGR to HSV
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        
        '''
        #threshold image
        ret, imgThresh = cv2.threshold(imgGray, 50, 255, cv2.THRESH_BINARY_INV)
        '''
        
        # black mask
        lower_black = np.array([0, 0, 0])
        upper_black = np.array([180, 255, 75])
        
        imgThresh = cv2.inRange(img_hsv, lower_black, upper_black)
        
        # orange mask
        lower_orange = np.array([0, 100, 20])
        upper_orange = np.array([25, 255, 255])

        o_mask = cv2.inRange(img_hsv, lower_orange, upper_orange)

        contours_orange = cv2.findContours(o_mask[ROI4[1]:ROI4[3], ROI4[0]:ROI4[2]], cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE)[-2]# Find contours
        #cv2.imshow("black", imgThresh)
        
        #imgThresh = cv2.subtract(imgThresh, b_mask) 
        
        #cv2.imshow("blue", b_mask)
        #cv2.imshow("subtracted", imgThresh) 

        


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
            
         for i in range(len(contours_orange)):
          cnt = contours_orange[i]
          area = cv2.contourArea(cnt)
          
          #print(area)
          if(area > 100):
              lDetected = True
              

        #draw all contours in full image
        
        for i in range(len(contours)):
            cnt = contours[i]
            area = cv2.contourArea(cnt)
            
            cv2.drawContours(img, contours, i, (0, 255, 0), 2)
            approx=cv2.approxPolyDP(cnt, 0.01*cv2.arcLength(cnt,True),True)
            x,y,w,h=cv2.boundingRect(approx)
        
        

        #calculate difference of areas between the areas of the lanes
        aDiff = rightArea - leftArea

        #calculate angle using PD steering
        angle = int(straightConst + aDiff * kp + (aDiff - prevDiff) * kd) + 2000

        #if the area of either lane is less than or equal to turnThresh and the car is not in a turn going the other direction, set the boolean of the respective direction turn to true
        if leftArea <= turnThresh and not rTurn:
            lTurn = True


        elif rightArea <= turnThresh and not lTurn:
            rTurn = True

        #if at the very start of the run, the car does a turn, remove a turn as the car is just adjusting to the center as it was placed to the side and is not in an actual turn
        #if i == 0 and (lTurn or rTurn):
           # t -= 1


        #if angle is different from previous angle
        if angle != prevAngle:
            #if car is in a left or right turn
            if lTurn or rTurn: 

              #if the area of the lane the car is turning towards is greater than or equal to exitThresh, the turn is completed and the booleans are set to false and the number of turns is increased by 1
              if (rightArea >= exitThresh and rTurn) or (leftArea >= exitThresh and lTurn): 
                  #set turn variables to false as turn is over
                  lTurn = False 
                  rTurn = False
                  #increase number of turns by 1
                  
                  if lDetected: 
                      t += 1
                      lDetected = False

              #if car is still in a left turn set the angle to the maximum of angle and sharpLeft
              elif lTurn:
                  angle = max(angle, sharpLeft)
              #if car is still in a right turn set the angle to the minimum of angle and sharpRight
              elif rTurn: 
                  angle = min(angle, sharpRight)

                #write angle to arduino to change servo
              write(angle)
            #if not in a turn write the angle and if the angle is over sharpLeft or sharpRight values it will be rounded down to those values
            else:
                write(max(min(angle, sharpLeft), sharpRight))
          
        #update previous area difference
        prevDiff = aDiff
            
        #stop the car and end the program if either q is pressed or the car has done 3 laps (12 turns) and is not still in the turn (does this by checking whether the angle is within 10 of straight)
        if cv2.waitKey(1)==ord('q') or (t == 12 and abs(angle - (straightConst + 2000)) <= 15) :
            stopCar() 
            break

        #display values
        #print(leftArea, rightArea)
        #print(t)
        #print(angle)
        #print(prevAngle)
        
        prevAngle = angle #update previous angle
        
        #display regions of interest
        #displayROI(img, ROI1, ROI2)

        #show image
        #cv2.imshow("finalColor", img)

        #increase the iterations by 1
        i += 1
        
    #close all image windows
    cv2.destroyAllWindows()
