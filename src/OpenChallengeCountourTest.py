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
    picam2.preview_configuration.controls.FrameRate = 30
    picam2.preview_configuration.align()
    picam2.configure("preview")
    picam2.start()
    
    ser = serial.Serial('/dev/ttyACM0', 9600, timeout = 1) #approximately 960 characters per second
    ser.flush() #block the program until all the outgoing data has been sent
    
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(5, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    angle = 2090
    prevAngle = 2090

    points = [(150,280), (510,280), (640,320), (30,320)]
    # read image from disk
    # Yellow color in BGR
    color = (0, 255, 255)

    width = 640
    height = 480
    
    rightArea = 0
    leftArea = 0
    rightX = 0
    leftX = 0
    laneThresh = 1000
      
    # Line thickness of 4 px
    thickness = 4
    val = True
    
    while True:
        if GPIO.input(5) == GPIO.LOW:
            print("Button pressed")
            break
    
    #ser.write((str(1430)).encode('utf-8'))
    ser.write((str(2090)).encode('utf-8'))

    while True: 

        rightArea, leftArea = 0, 0

        img = picam2.capture_array()
  
        # Using cv2.line() method to draw a yellow line with thickness of 4 px
        #image = cv2.line(img, points[0], points[1], color, thickness)
        #image = cv2.line(img, points[1], points[2], color, thickness)
        #image = cv2.line(img, points[2], points[3], color, thickness)
        #image = cv2.line(img, points[3], points[0], color, thickness)
        
        # specify desired output size 
        input = np.float32(points)
        output = np.float32([(0,0), (width-1,0), (width-1,height-1), (0,height-1)])
        # compute perspective matrix
        matrix = cv2.getPerspectiveTransform(input,output)
        imgPerspective = cv2.warpPerspective(img, matrix, (width,height), cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=(0,0,0))
        
        cv2.imshow("finalColor", img)
        #grayscaling
        imgGray = cv2.cvtColor(imgPerspective, cv2.COLOR_BGR2GRAY)

        #Simple Thresholding
        ret, imgThresh = cv2.threshold(imgGray, 90, 255, cv2.THRESH_BINARY_INV)

        contours, hierarchy = cv2.findContours(imgThresh, 
        cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        for i in range(len(contours)):
            cnt = contours[i]
            area = cv2.contourArea(cnt)
            
            cv2.drawContours(imgPerspective, contours, i, (0, 255, 0), 2)
            approx=cv2.approxPolyDP(cnt, 0.01*cv2.arcLength(cnt,True),True)
            x,y,w,h=cv2.boundingRect(approx)
            cv2.rectangle(imgPerspective,(x,y),(x+w,y+h),(0,0,255),2)
            #print(x)
            
            #code that stores the 2 highest areas
            if area > leftArea: 
              leftArea = rightArea
              leftX = rightX
              rightArea = area
              rightX = x
            elif area > leftArea: 
              leftArea = area
              leftX = x
                
        #print(leftArea, end = " ")
        #print(rightArea, end = " ")
        
        cv2.imshow("finalColor", imgPerspective)
          
        if leftX > rightX: #if left area is actually the right area swap
          leftArea, rightArea = rightArea, leftArea
        
        #print(leftArea, end = " ")
        #print(rightArea, end = " ")
        
        #print(leftX, end = " ")
        #print(rightX)
        
        
        if leftArea > laneThresh and rightArea > laneThresh: # 2 lanes are detected
          if leftArea > rightArea:
              print("right")
              angle = 2070
          else:
              print("left")
              angle = 2110
          val = True
        elif (leftArea <= laneThresh or rightArea <= laneThresh) and val: #1 lane is detected
          if angle == 2070: 
              print("left, 1 lane")
              angle = 2110
          elif angle == 2110:
              print("right 1 lane")
              angle = 2070
          val = False

        ser.write((str(angle)).encode('utf-8'))
        
        sleep(0.25)
        
        if cv2.waitKey(1)==ord('q'):
            ser.write((str(1500)).encode('utf-8'))
            ser.write((str(2090)).encode('utf-8'))
            break
        
    cv2.destroyAllWindows()



