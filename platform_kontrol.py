# TOBB ETU Elektrik-Elektronik Muhendisligi
# 2021-2022 Yaz Donemi
# ELE 495 Bitirme Projesi - Iki Boyutlu Top Stabilizasyon Platformu
# Grup 4
# Platform Tarafi Goruntu Isleme, Kontrol ve Haberlesme Kodu
# Raspberry Pi 4

# Kontrol kodu ve Bluetooth verileri gonderen kodun ayri thread'lerde calismasini saglayan kutuphane
# Ayrica ekstra ozellik olan kullanicidan gelen veriye bagli PID parametre degisimini runtime'da yapmayi saglayan thread'i de bu sagliyor.
from threading import Thread

# Goruntu isleme icin kullanilan kutuphaneler
from collections import deque
from imutils.video import VideoStream
import numpy as np
import cv2
import argparse
import imutils

# Servolari surmek icin gerekli Raspberry Pi gpio kutuphanesi
import RPi.GPIO as GPIO

# Kontrol kutuphanesi
from simple_pid import PID

# Bluetooth kutuphanesi ve gonderim gecikmesi eklemeyi saglayan zaman kutuphanesi
import bluetooth
import time

# x ekseni icin PID parametreleri
Kpx = 0.27
Kix = 0.11
Kdx = 0.32

# y ekseni icin PID parametreleri
Kpy = 0.27
Kiy = 0.11
Kdy = 0.32

# x ve y eksenleri icin PID sonuclarini tutacak PID degiskenleri
PIDx_result = 0
PIDy_result = 0

# Topun tutulmasi istenen referans x-y noktalari
setpointx = 0
setpointy = 0

# x ve y eksenleri icin kamerada gorulen orta nokta degerleri
midx = 300
midy = 300

# x ve y eksenleri icin kamera degerlerine bagli platform yaricaplari
xdiff = 300
ydiff = 300

# x ve y eksenlerini -250 250 degerleri arasina getirmek icin belirlenen deger araliklari
# yani 50x50 cm platforma 500x500 mm olarak davranilmasi saglaniyor
xrangemin = 250
xrangemax = -250
yrangemin = 250
yrangemax = -250

# RGB DEGERLERİ
#beyaz için
#greenLower = (12,0,154) 
#greenUpper = (117,26,170)

#siyah için
#greenLower = (0,0,0) 
#greenUpper = (179,255,42)

#her renk için
greenLower = (0,60,0) 
greenUpper = (179,251,235)

# Bluetooth ile gonderilecek top konum bilgisi degiskenleri
ball_pos_x_c = None
ball_pos_y_c = None

# Servo motorlarin surulmeye baslanmasi
GPIO.setmode(GPIO.BOARD)

GPIO.setup(11,GPIO.OUT)
servo2 = GPIO.PWM(11,50)
GPIO.setup(12,GPIO.OUT)
servo1 = GPIO.PWM(12,50)

servo1.start(0)
servo2.start(0)

# Video icin argumanlarin alinmasi
ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video", help="Path to the (optional) video file")
ap.add_argument("-b", "--buffer", default=64, type=int, help="max buffer size")
args = vars(ap.parse_args())

pts = deque(maxlen=args["buffer"])

# Ayri bir thread'de Kp, Ki, Kd degiskenlerini runtime'da degistirebilmek icin gereken degiskenler
user_input = [None]

ui_kp = 0
ui_ki = 0
ui_kd = 0

# Arduino tarafinda bulunan HC-05'in id'si ve adresi veriliyor, iletisim baslatiliyor.
uuid = "00001101-0000-1000-8000-00805F9B34FB"
addr = "00:18:91:D6:BD:DD"
service_matches = bluetooth.find_service(uuid=uuid, address=addr)
if len(service_matches) == 0:
    print("Cihaz bulunamadi.")
    
first_match = service_matches[0]
port = first_match["port"]
name = first_match["name"]
host = first_match["host"]
print("Baglaniliyor... \"{}\" {}".format(name, host))

sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
sock.connect((host, port))
print("Baglandi.")

# PID nesneleri olusturulup Kp, Ki, Kd degerleri veriliyor
pidx = PID(Kpx, Kix, Kdx, setpointx)
pidy = PID(Kpy, Kiy, Kdy, setpointy)

pidx.sample_time = 0.02
pidy.sample_time = 0.02

# Raspberry'nin kendi bluetooth'u üzerinden Arduino'daki HC-05 modulu uzerine veri gonderilmesi
def comm():
	global sock, ball_pos_x_c, ball_pos_y_c
	while True:
		# Her yarim saniyede bir x ve y pozisyonlarini tek seferde arasinda 'x' karakteri ile gonderiyor
		# ve arduino kodunda buna gore ayristirilip ekrana basiliyor.
		sock.send(ball_pos_x_c.__str__() + "x" + ball_pos_y_c.__str__())
		# Ayri thread'de calistigi icin ana fonksiyonda gecikme olusturmuyor.
		time.sleep(0.5)

# Degerleri belli bir aralikta hizalamak icin kullanilan yardimci fonksiyon
def map_value(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) // (in_max - in_min) + out_min

# Degerleri verilen minimum ve maksimum degerler arasinda kisitlamak icin kullanilan yardimci fonksiyon
def constrain(val, min_val, max_val):
	return min(max_val,max(min_val,val))

# Servo motor acilarini ayarlamak icin kullanilan fonksiyon
def set_degree(servo, degree):
	servo.ChangeDutyCycle(map_value(degree, 0, 180, 5, 10))

# Ayri thread'de calisip kullanicidan surekli deger alip PID degerlerini runtime'da degistirmeye yarayan fonksiyon
# Ekstra ozellik olarak ekledik
def get_user_input(user_input_ref):
	user_input_ref[0] = input('pid: ')
	global user_input
	get_user_input(user_input)

# PID kontrolu saglayan fonksiyon
def do_PID(ball_pos_x, ball_pos_y):
	global pidx, pidy
	global PIDx_result, PIDy_result
	global servo1, servo2

	# Top x-y konumu verilip PID sonuclari aliniyor
	PIDx_result = pidx(ball_pos_x)
	PIDy_result = pidy(ball_pos_y)

	# Servo motorlarin acilari PID degerlerine gore ayarlaniyor
	# Platformun dik durusu 90 derece oldugundan uzerine toplaniyor
	# Dereceler 0-180 fiziksel sinirlara sinirlandiriliyor
	set_degree(servo1, constrain(90+PIDx_result, 0, 180))
	# servo2 ters oldugundan eksili
	set_degree(servo2, constrain(90-PIDy_result, 0, 180))

# Surekli calisip goruntu isleyecek ve buna bagli PID kontrol yapacak ana fonksiyon
def main_while():
	global pidx, pidy
	global PIDx_result, PIDy_result
	global midx, midy, xdiff, ydiff
	global greenLower, greenUpper, pts, args
	global ball_pos_x_c, ball_pos_y_c
	global user_input
	global ui_kp, ui_ki, ui_kd

	# Kameradan video verisi aliniyor
	if not args.get("video", False):
		vs = VideoStream(src=0).start()
	else:
		vs = cv2.VideoCapture(args["video"])

	# Sonsuz donguye giriliyor
	while True:
		# Her bir resim karesi okunuyor
		frame = vs.read()
		# Resim karesinin kenar fazlaliklari kesiliyor
		frame = frame[25:460, 105:540]

		# Resim karesi gercekten var mi yok mu kontrolu
		frame = frame[1] if args.get("video", False) else frame
		if frame is None:
			break

		# Resim karesi boyutlandirilip gauss filtresi uygulaniyor
		frame = imutils.resize(frame, width=600)
		blurred = cv2.GaussianBlur(frame, (11, 11), 0)

		# rgb'den hsv renk koduna geciliyor
		hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

		# Onceden belirlenen renk kodlari icin resme maske ayarlaniyor ve daraltilip genisletiliyor
		mask = cv2.inRange(hsv, greenLower, greenUpper)
		mask = cv2.erode(mask, None, iterations=2)
		mask = cv2.dilate(mask, None, iterations=2)

		# Topu tespit eden konturlar maskeyle atiliyor
		cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		cnts = imutils.grab_contours(cnts)

		# Eger top bulunamadiysa, konum degeri yok
		ball_pos_center = None
		ball_pos_x = None
		ball_pos_y = None

		# Eger konturlar varsa yani top bulunduysa topun orta noktasinin x-y konumlarini bul
		if len(cnts) > 0:
			c = max(cnts, key=cv2.contourArea)
			((x, y), radius) = cv2.minEnclosingCircle(c)
			M = cv2.moments(c)
			ball_pos_center = (int(M['m10']/M['m00']), int(M['m01']/M['m00']))
			ball_pos_x = int(M['m10']/M['m00'])
			ball_pos_y = int(M['m01']/M['m00'])

			# Topun etrafina daire sekli ciz
			if radius > 10:
				cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
				cv2.circle(frame, ball_pos_center, 5, (0, 0, 255), -1)

		pts.append(ball_pos_center)

		# frame'de topun uzerinde cizgi ciz
		for i in range(1, len(pts)):
			if pts[i-1] is None or pts[i] is None:
				continue

			thickness = int(np.sqrt(args["buffer"] / float(i+1)) * 2.5)
			cv2.line(frame, pts[i-1], pts[i], (0, 0, 255), thickness)
		
		# frame'i goster
		cv2.imshow("Frame", frame)
		
		# q tusuna basilirsa cik
		key = cv2.waitKey(1) & 0xFF
		if key == ord('q'):
			break

		# Eger top pozisyonlari yoksa yani platformun uzerinde top yoksa platformu duzeltmek icin degerleri orta noktaya cek
		if ball_pos_x == None: 
			ball_pos_x = midx
		if ball_pos_y == None: 
			ball_pos_y = midy

		# Top pozisyon degerlerini verilen araliktaki degerlere hizala
		ball_pos_x = map_value(ball_pos_x, midx-xdiff, midx+xdiff, xrangemin, xrangemax)
		ball_pos_y = map_value(ball_pos_y, midy-ydiff, midy+ydiff, yrangemax, yrangemin)
		
		# Bluetooth ile gonderilecek top konum bilgilerini ayri degiskenlere ata
		ball_pos_x_c = ball_pos_x
		ball_pos_y_c = ball_pos_y
        
		# Topu ortada tutmaya calisan PID kontrolu yap
		do_PID(ball_pos_x, ball_pos_y)
		
		# Eger bu sirada kullanicidan deger geldiyse yeni PID degerlerini hala calisiyorken ata
		if user_input[0] is not None:
			try:
				ui_kp, ui_ki, ui_kd = user_input[0].split()
				ui_kp = float(ui_kp)
				ui_ki = float(ui_ki)
				ui_kd = float(ui_kd)

				pidx.Kp = ui_kp
				pidx.Ki = ui_ki
				pidx.Kd = ui_kd

				pidy.Kp = ui_kp
				pidy.Ki = ui_ki
				pidy.Kd = ui_kd

			except: pass

# Fonksiyonlari 3 ayri thread olarak ac ve baslat
# Thread olmalari sayesinde fonksiyonlar tamamen ayri islemleri yapip bu islemleri yaparken birbirini bloklamiyor ve gecikme olusturmuyor
t1 = Thread(target=main_while)
t2 = Thread(target=get_user_input, args=(user_input,))
t3 = Thread(target=comm)
t1.start()
t2.start()
t3.start()

# En son videoyu durdur
if not args.get("video", False):
	vs.stop()
else:
	vs.release()

# goruntuleri kapat
cv2.destroyAllWindows()

# servolari kapat
servo1.stop()
servo2.stop()
GPIO.cleanup()

# bluetoothu kapat
sock.close()
