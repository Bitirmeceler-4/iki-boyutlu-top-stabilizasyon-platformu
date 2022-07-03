# yorumlar eklenecek
from threading import Thread

from collections import deque
from imutils.video import VideoStream
import numpy as np
import cv2
import argparse
import time

import imutils

import RPi.GPIO as GPIO

import bluetooth

errorx = 0
errory = 0

PIDx = 0
PIDy = 0

pidflag=0
preverrory=0
preverrorx=0

Kpx = 0.3
Kix = 0.03
Kdx = 0.13
	  
Kpy = 0.3
Kiy = 0.08
Kdy = 0.13

x = 300
y = 300

cX = x
cY = y

xdiff = 300 
ydiff = 300

xrangemin = 250
xrangemax = -250
yrangemin = 250
yrangemax = -250

uuid = "00001101-0000-1000-8000-00805F9B34FB"
addr = "00:18:91:D6:BD:DD"
service_matches = bluetooth.find_service(uuid=uuid, address=addr)

if len(service_matches) == 0:
    print("Couldn't find the SampleServer service.")
    
first_match = service_matches[0]
port = first_match["port"]
name = first_match["name"]
host = first_match["host"]

print("Connecting to \"{}\" on {}".format(name, host))

sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
sock.connect((host, port))

print("Connected.")

centerXc = None
centerYc = None

#communication
def comm():
    global sock, centerXc, centerYc
    while True:
        sock.send(centerXc.__str__() + "x" + centerYc.__str__())
        time.sleep(0.2)
    

def constrain(val, min_val, max_val):
	return min(max_val,max(min_val,val))

def aciayarla(pwm,aci):
	x=(1/180)*aci + 1
	duty=x*5
	pwm.ChangeDutyCycle(constrain(duty,0,100))

GPIO.setmode(GPIO.BOARD)

GPIO.setup(11,GPIO.OUT)
servo2 = GPIO.PWM(11,50)
GPIO.setup(12,GPIO.OUT)
servo1 = GPIO.PWM(12,50)

servo1.start(0)
servo2.start(0)

ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video", help="Path to the (optional) video file")
ap.add_argument("-b", "--buffer", default=64, type=int, help="max buffer size")
args = vars(ap.parse_args())

greenLower = (15,76,87)
greenUpper = (179,255,255)

pts = deque(maxlen=args["buffer"])

thread_running = True

user_input = [None]

kp = 0
ki = 0
kd = 0

def get_user_input(user_input_ref):
	user_input_ref[0] = input('pid: ')

	global user_input
	get_user_input(user_input)

Ts = 50
setpointx = 0
setpointy = 0

from simple_pid import PID

pidx = PID(Kpx, Kix, Kdx, setpointx)
pidy = PID(Kpy, Kiy, Kdy, setpointy)

pidx.set_auto_mode = True
pidy.set_auto_mode = True

pidx.setpoint = 0
pidy.setpoint = 0

def doPID(centerX, centerY):
	global errorx, errory
	global PIDx, PIDy
	global pidflag, preverrory, preverrorx
	global Kpx, Kix, Kdx, Kpy, Kiy, Kdy, x, y, cX, cY, xdiff, ydiff
	global servo1, servo2, greenLower, greenUpper, pts, args
	global pidx, pidy
	
	PIDx = pidx(centerX)
	PIDy = pidy(centerY)

	aciayarla(servo1, constrain(60+PIDx, 0, 180))
	aciayarla(servo2, constrain(120-PIDy, 0, 180))


def mapr(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) // (in_max - in_min) + out_min

def forever_while():
	global thread_running
	global user_input
	global kp, ki, kd
	global errorx, errory
	global PIDx, PIDy
	global pidflag, preverrory, preverrorx
	global Kpx, Kix, Kdx, Kpy, Kiy, Kdy, x, y, cX, cY, xdiff, ydiff
	global servo1, servo2, greenLower, greenUpper, pts, args
	global pidx, pidy
	global centerXc, centerYc

	if not args.get("video", False):
		vs = VideoStream(src=0).start()
	else:
		vs = cv2.VideoCapture(args["video"])

	while thread_running:
		frame = vs.read()
		frame = frame[25:460, 105:540] 
		#print(frame.size().x)
		frame = frame[1] if args.get("video", False) else frame
		if frame is None:
			break

		frame = imutils.resize(frame, width=600)
		blurred = cv2.GaussianBlur(frame, (11, 11), 0)
		hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

		mask = cv2.inRange(hsv, greenLower, greenUpper)
		mask = cv2.erode(mask, None, iterations=2)
		mask = cv2.dilate(mask, None, iterations=2)

		cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		cnts = imutils.grab_contours(cnts)
		center = None
		centerX = None
		centerY = None

		if len(cnts) > 0:
			c = max(cnts, key=cv2.contourArea)
			((x, y), radius) = cv2.minEnclosingCircle(c)
			M = cv2.moments(c)
			center = (int(M['m10']/M['m00']), int(M['m01']/M['m00']))
			centerX = int(M['m10']/M['m00'])
			centerY = int(M['m01']/M['m00'])

			if radius > 10:
				cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
				cv2.circle(frame, center, 5, (0, 0, 255), -1)

		pts.append(center)

		if centerX == None: 
			centerX = cX
		if centerY == None: 
			centerY = cY

		centerX = mapr(centerX, cX-xdiff, cX+xdiff, xrangemin, xrangemax)
		centerY = mapr(centerY, cY-ydiff, cY+ydiff, yrangemax, yrangemin)
		
		centerXc = centerX
		centerYc = centerY
        
		doPID(centerX, centerY)

		for i in range(1, len(pts)):
			if pts[i-1] is None or pts[i] is None:
				continue

			thickness = int(np.sqrt(args["buffer"] / float(i+1)) * 2.5)
			cv2.line(frame, pts[i-1], pts[i], (0, 0, 255), thickness)

		cv2.imshow("Frame", frame)
		key = cv2.waitKey(1) & 0xFF

		if key == ord('q'):
			break
		if user_input[0] is not None:
			try:
				kp, ki, kd = user_input[0].split()
				kp = int(kp)
				ki = int(ki)
				kd = int(kd)

				pidx.Kp = kp
				pidx.Ki = ki
				pidx.Kd = kd

				pidy.Kp = kp
				pidy.Ki = ki
				pidy.Kd = kd

			except: pass

t1 = Thread(target=forever_while)
t2 = Thread(target=get_user_input, args=(user_input,))
t3 = Thread(target=comm)
t1.start()
t2.start()
t3.start()
thread_running = True


if not args.get("video", False):
	vs.stop()
else:
	vs.release()


cv2.destroyAllWindows()

servo1.stop()
servo2.stop()
GPIO.cleanup()
sock.close()
