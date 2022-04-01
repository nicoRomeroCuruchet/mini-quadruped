import cv2
import time
import serial
import mediapipe as mp

cam = cv2.VideoCapture(0)
mpHands = mp.solutions.hands
hands = mpHands.Hands()
mpDraw = mp.solutions.drawing_utils

start = 0 
end = 0
num_of_bytes = 5

serial_port = serial.Serial(
    port="/dev/ttyUSB0",
    baudrate=115200,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE
    )

time.sleep(1)
r = 0
r_x = 0
r_y = 0 

a = True 
 
while True:
        _, img = cam.read()
        imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        results = hands.process(imgRGB)

        if results.multi_hand_landmarks:
                for handLms in results.multi_hand_landmarks:
                        for id, lm in enumerate(handLms.landmark):
                                
                                h,w,c = img.shape
                                cx, cy, cz = int(lm.x*w), int(lm.y*h), int(lm.z*h)

                                if id == 9: 
                                        r_x = -round(0.4*(cx - w/2)/w + 0.6*r_x, 2)
                                        r_y = -round(0.4*(cy - h/2)/h + 0.6*r_y, 2)
                                        data = str(r_x).zfill(num_of_bytes) + str(r_y).zfill(num_of_bytes) + str(int(r)).zfill(num_of_bytes)
                                        serial_port.write(data.encode())

                        mpDraw.draw_landmarks(img, handLms, mpHands.HAND_CONNECTIONS)
                        a=True

        else:
                if(a):
                        data = str('xxx').rjust(15, '0')
                        serial_port.write(data.encode())
                        
                a = False

        start = time.time()
        fps = 1 / (start - end)
        end = start
        cv2.putText(img,str(int(fps)), (10,70), cv2.FONT_HERSHEY_PLAIN, 3,(0,255,0),3)

        cv2.imshow('Webcam', img)
        if cv2.waitKey(1) == ord('q'):
                break

data = str('xxx').rjust(15, '0')
serial_port.write(data.encode())
time.sleep(1)
cam.release()
serial_port.close()
cv2.destroyAllWindows()
