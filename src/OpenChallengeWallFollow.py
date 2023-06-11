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

    points = [(150,280), (510,280), (640,320), (30,320)]
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
    speed = 1440

    angleRight = 2070
    angleLeft = 2110
    
    laneThresh = 1000
    val = True
    
    while True:
       if GPIO.input(5) == GPIO.LOW:
           break

    
    ser.write((str(speed)).encode('utf-8'))
    ser.write((str(angle)).encode('utf-8'))

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

        ret, imgThresh = cv2.threshold(imgGray, 80, 255, cv2.THRESH_BINARY_INV)

        imgBlur = cv2.GaussianBlur(imgGray, (3,3), 0)
        
        v = np.median(imgGray)
        
        lowThresh = int(max(0, (1.0 - 0.33) * v))
        highThresh = int(min(255, (1.0 + 0.33) * v))
        
        imgCanny = cv2.Canny(imgBlur, lowThresh, highThresh)
        
        imgFinal = cv2.add(imgThresh, imgCanny)
        
        column = []
        for i in range(640):
            column.append(sum(imgFinal[:, i]))


        max1 = max(column[:319])
        max2 = max(column[320:])
        laneLeft = column.index(max1)
        laneRight = column.index(max2, 320)
        laneCenter = int(laneLeft + (laneRight - laneLeft) / 2)
        
        if laneCenter < 320:
            angle = angleLeft
        elif laneCenter > 320:
            angle = angleRight


        imgFinalColor = cv2.cvtColor(imgFinal, cv2.COLOR_GRAY2BGR)
        cv2.line(imgFinalColor, (laneLeft, 0), (laneLeft, 479), (0, 255, 0), thickness=1, lineType=8)
        cv2.line(imgFinalColor, (laneCenter, 0), (laneCenter, 479), (0, 255, 0), thickness=1, lineType=8) #lane center
        cv2.line(imgFinalColor, (laneRight, 0), (laneRight, 479), (0, 255, 0), thickness=1, lineType=8)


        cv2.line(imgFinalColor, (319, 0), (319, 479), (0, 0, 255), thickness=1, lineType=8) #frame center


        cv2.imshow("finalColor", imgFinalColor)


        ser.write((str(angle)).encode('utf-8'))
        
        sleep(0.25)
        
        if cv2.waitKey(1)==ord('q'):          
            ser.write((str(1500)).encode('utf-8'))
            ser.write((str(2090)).encode('utf-8'))
            break
        
    cv2.destroyAllWindows()
