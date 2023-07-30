#import necessary libraries
import cv2
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
    ROI1 = [55, 205, 315, 295]
    ROI2 = [380, 185, 640, 255]

    #booleans for tracking whether car is in a left or right turn
    lTurn = False
    rTurn = False
  
    t = 0 #number of turns car has completed
    
    kp = 0.005 #value of proportional for proportional steering
    kd = 0.005 #value of derivative for proportional and derivative sterrin
    
    straightConst = 98 #angle in which car goes straight

    turnThresh = 250 #if area of a lane is under this threshold car goes into a turn
    exitThresh = 1500 #if area of both lanes is over this threshold car exits a turn
  
    angle = 2098 #variable for the current angle of the car
    prevAngle = angle #variable tracking the angle of the previous iteration
    tDeviation = 25 #value used to calculate the how far left and right the car turns during a turn
    sharpRight = straightConst - tDeviation + 2000 #the default angle sent to the car during a right turn
    sharpLeft = straightConst + tDeviation + 2000 #the default angle sent to the car during a left turn
    
    speed = 1425 #variable for the speed of the car
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
        ret, imgThresh = cv2.threshold(imgGray, 50, 255, cv2.THRESH_BINARY_INV)

        #find left and right contours of the lanes
        contours_left, hierarchy = cv2.findContours(imgThresh[ROI1[1]:ROI1[3], ROI1[0]:ROI1[2]], 
        cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    
        contours_right, hierarchy = cv2.findContours(imgThresh[ROI2[1]:ROI2[3], ROI2[0]:ROI2[2]], 
        cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        
        #contours, hierarchy = cv2.findContours(imgThresh, 
        #cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        #iterate through every contour in both the left and right region of interest and take the largest one in each
        for cnt in contours_left:
            area = cv2.contourArea(cnt)
            
            leftArea = max(area, leftArea) 

        for cnt in contours_right:
            area = cv2.contourArea(cnt)

            rightArea = max(area, rightArea)

        #draw all contours in full image
        """
        for i in range(len(contours)):
            cnt = contours[i]
            area = cv2.contourArea(cnt)
            
            cv2.drawContours(img, contours, i, (0, 255, 0), 2)
            approx=cv2.approxPolyDP(cnt, 0.01*cv2.arcLength(cnt,True),True)
            x,y,w,h=cv2.boundingRect(approx)
        """
        #calculate difference of areas between the areas of the lanes
        aDiff = rightArea - leftArea

        #calculate angle using PD steering
        angle = int(straightConst + aDiff * kp + (aDiff - prevDiff) * kd) + 2000

        #if the area of either lane is less than or equal to turnThresh and the car is not in a turn going the other direction and the car has fully accelerated, set the boolean of the respective direction turn to true
        if leftArea <= turnThresh and not rTurn and speed == targetS:
            lTurn = True


        elif rightArea <= turnThresh and not lTurn and speed == targetS:
            rTurn = True


        #if angle is different from previous angle
        if angle != prevAngle:
            #if car is in a left or right turn
            if lTurn or rTurn: 

              #if area of both lanes exceed or equal exitThresh, meaning the turn is completed
              if leftArea >= exitThresh and rightArea >= exitThresh: 
                  #set turn variables to false as turn is over
                  lTurn = False 
                  rTurn = False
                  #increase number of turns by 1
                  t += 1

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
                pass
          
        #update previous area difference
        prevDiff = aDiff
            
        #stop the car and end the program if either q is pressed or the car has done 3 laps (12 turns) and is not still in the turn (does this by checking whether the angle is within 10 of straight)
        if cv2.waitKey(1)==ord('q') or (t == 12 and abs(angle - (straightConst + 2000)) <= 10) :
            stopCar() 
            break

        #display values
        print(leftArea, rightArea)
        print(t)
        #print(angle)
        #print(prevAngle)
        
        prevAngle = angle #update previous angle
        
        #display regions of interest
        displayROI(img, ROI1, ROI2)

        #show image
        cv2.imshow("finalColor", img)
        
    #close all image windows
    cv2.destroyAllWindows()
