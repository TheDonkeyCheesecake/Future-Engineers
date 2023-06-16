import cv2
import numpy as np
from picamera2 import Picamera2
import serial
from time import sleep
import RPi.GPIO as GPIO

if __name__ == '__main__':
    
    picam2 = Picamera2()
    picam2.preview_configuration.main.size = (640,480)
    picam2.preview_configuration.main.format = "RGB888"
    picam2.preview_configuration.controls.FrameRate = 60
    picam2.preview_configuration.align()
    picam2.configure("preview")
    picam2.start()

    points = [(120,300), (540,300), (610,320), (60,320)]
    color = (0, 255, 255)
    thickness = 4

    width = 640
    height = 480
    
    ser = serial.Serial('/dev/ttyACM0', 9600, timeout = 1) 
    ser.flush() 
    
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(5, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    angle = 2090
    speed = 1425

    angleRight = 2070
    angleLeft = 2110
    
    sharpRight = 2050
    sharpLeft = 2130
    
    count = 0
    turns = 0
    
    laneThresh = 1000
    val = True

    while True:
       if GPIO.input(5) == GPIO.LOW:
            break
    
    
    
    ser.write((str(speed)).encode('utf-8'))
    print("signal sent", speed)
    ser.write((str(angle)).encode('utf-8'))
    print("signal sent", angle)

    while True: 

        rightArea, leftArea, rightX, leftX = 0, 0, 0, 0

        img = picam2.capture_array()
  
        #image = cv2.line(img, points[0], points[1], color, thickness)
        #image = cv2.line(img, points[1], points[2], color, thickness)
        #image = cv2.line(img, points[2], points[3], color, thickness)
        #image = cv2.line(img, points[3], points[0], color, thickness)
        
        input = np.float32(points)
        output = np.float32([(0,0), (width-1,0), (width-1,height-1), (0,height-1)])

        matrix = cv2.getPerspectiveTransform(input,output)
        imgPerspective = cv2.warpPerspective(img, matrix, (width,height), cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=(0,0,0))

        imgGray = cv2.cvtColor(imgPerspective, cv2.COLOR_BGR2GRAY)

        ret, imgThresh = cv2.threshold(imgGray, 60, 255, cv2.THRESH_BINARY_INV)

        contours, hierarchy = cv2.findContours(imgThresh, 
        cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        
        count = 0

        for i in range(len(contours)):
            cnt = contours[i]
            area = cv2.contourArea(cnt)
            
            cv2.drawContours(imgPerspective, contours, i, (0, 255, 0), 2)
            approx=cv2.approxPolyDP(cnt, 0.01*cv2.arcLength(cnt,True),True)
            x,y,w,h=cv2.boundingRect(approx)
            cv2.rectangle(imgPerspective,(x,y),(x+w,y+h),(0,0,255),2)
            
            if area > laneThresh:
                count += 1
            
            #code that stores the 2 highest areas
            if area > leftArea: 
              leftArea = rightArea
              leftX = rightX
              
              rightArea = area
              rightX = x
              
            elif area > leftArea:    
              leftArea = area
              leftX = x
        
        #print(count)
        
        cv2.imshow("finalColor", imgThresh)
          
        if leftX > rightX: #if left area is actually the right area swap
          leftArea, rightArea = rightArea, leftArea
        
        if max(leftArea, rightArea) >= 100000:
            print("back wall detected")
            if angle == angleRight:
              angle = sharpRight
            elif angle == angleLeft:
              angle = sharpLeft
            
        
        elif leftArea > laneThresh and rightArea > laneThresh and count == 2: # 2 lanes are detected
          if leftArea > rightArea:
             # print("right, 2 lanes")
              angle = angleRight
            
          else:
              #print("left, 2 lanes")
              angle = angleLeft
        
          print("reg turn")        
            
          val = True
        
        elif count >= 2 and val: #1 lane is detected
          
          
          turns += 1
          
          val = False
            

        ser.write((str(angle)).encode('utf-8'))
        #print("signal sent", angle) 
        
        sleep(0.1)
        
        if cv2.waitKey(1)==ord('q'):          
            ser.write((str(1500)).encode('utf-8'))
            print(1500)
            ser.write((str(2090)).encode('utf-8'))
            print(2090)
            break
        
    cv2.destroyAllWindows()