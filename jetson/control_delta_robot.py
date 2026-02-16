import serial
import math
import time
import numpy as np
import serial.tools.list_ports
import cv2
import cv2.aruco as aruco
import os

# 자동 포트 감지 및 연결
def auto_connect(baudrate=115200, timeout=1):
	ports = list(serial.tools.list_ports.comports())
	for port in ports:
		# 이름이나 설명에 'usbmodem' 또는 'usbserial'이 포함된 포트 우선 연결
		if 'usbmodem' in port.device or 'usbserial' in port.device or 'ttyACM' in port.device:
			try:
				return serial.Serial(port.device, baudrate, timeout=timeout)
			except Exception:
				continue
	# 못 찾으면 첫 번째 포트 시도
	if ports:
		try:
			return serial.Serial(ports[0].device, baudrate, timeout=timeout)
		except Exception:
			pass
	raise RuntimeError("연결 가능한 시리얼 포트를 찾을 수 없습니다.")

class DeltaRobot:
	"""
	R: 베이스 모터 반지름 (mm)
	r: 말단 플랫폼 반지름 (mm)
	L: 상부 링크 길이 (mm)
	l: 하부 링크 길이 (mm)
	"""
	def __init__(self, R, r, L, l):
		self.R = R
		self.r = r
		self.rf = L
		self.re = l
		# 상수
		self.sqrt3 = math.sqrt(3.0)
		self.tan30 = 1.0 / self.sqrt3
		self.sin120 =  self.sqrt3 / 2.0
		self.cos120 = -0.5
		# 플랫폼 변환
		self.f = 2.0 * R / self.tan30
		self.e = 2.0 * r / self.tan30

		self.angles = []
		self.current_pos = [0, 0, 0]
		self.mcu = auto_connect(115200, 0)

	def _angle_yz(self, x0, y0, z0):
		# 베이스·말단 플랫폼 오프셋
		y1  = -0.5 * self.f * self.tan30   # = -R
		y0p =  y0 - 0.5 * self.e * self.tan30  # = y0 - r

		# 이차 방정식 계수
		a = (x0*x0 + y0p*y0p + z0*z0
			 + self.rf*self.rf - self.re*self.re
			 - y1*y1) / (2.0 * z0)
		b = (y1 - y0p) / z0

		# 판별식
		d = -(a + b*y1)**2 + self.rf*self.rf * (b*b + 1.0)
		if d < 0:
			return None

		# 하위 분기: -sqrt(d)
		sd = math.sqrt(d)
		yj = (y1 - a*b - sd) / (b*b + 1.0)
		zj = a + b*yj

		# 각도 계산 (라디안→도)
		theta = math.degrees(math.atan2(-zj, (y1 - yj)))
		return theta

	def inverse(self, x0, y0, z0):
		thetas = []
		for (c, s) in [(1, 0),
					   (self.cos120,  self.sin120),
					   (self.cos120, -self.sin120)]:
			x = x0 * c + y0 * s
			y = -x0 * s + y0 * c
			th = self._angle_yz(x, y, z0)
			if th is None:
				return None
			thetas.append(th)
		self.angles = thetas
		return thetas
	
	def angle_to_steps(self, target_angle_deg):
		# 기본 파라미터
		step_angle = 1.8               # 스텝당 회전 각도 (도)
		microstep_division = 16        # 마이크로스테핑 비율
		gear_ratio = 10                # 감속기 비율 (출력:입력)

		# 마이크로스텝 한 스텝당 각도 (도)
		microstep_angle = step_angle / microstep_division

		# 출력축 기준 목표 각도 → 모터 기준으로 환산
		motor_angle = target_angle_deg * gear_ratio

		# 필요한 마이크로스텝 수 계산
		steps = motor_angle / microstep_angle

		return round(steps)

	def send_command(self, com):
		self.mcu.write(com)	

	def send_move_command(self):
		# 명령어(동작 각도)를 시리얼 포트로 전송
		steps = [-self.angle_to_steps(angle) for angle in self.angles]
		command = f"m {steps[0]} {steps[1]} {steps[2]}\n".encode()
		self.send_command(command)
		self.current_pos = steps
	
	def send_downmove_command(self):
		# 명령어(동작 각도)를 시리얼 포트로 전송
		steps = [-self.angle_to_steps(angle) for angle in self.angles]
		command = f"n {steps[0]} {steps[1]} {steps[2]}\n".encode()
		self.send_command(command)
		self.current_pos = steps
	
	def send_set_downmode_command(self):
		self.send_command("d\n".encode())
	
	def send_reset_command(self):
		self.send_command("r\n".encode())
		self.current_pos = [0, 0, 0]
	
	def send_grip_command(self, onoff):
		if onoff != 0 and onoff != 1:
			print("send_grip_command: value error\n")
			return
		self.send_command(f"g{onoff}\n".encode())
	
	def send_set_speed_command(self, speed):
		s1 = s2 = s3 = 0
		if len(speed) == 1:
			s1 = s2 = s3 = speed
		elif len(speed) == 3:
			s1 = speed[0]
			s2 = speed[1]
			s3 = speed[2]
		else:
			print("send_set_speed_command: value error\n")
			return
		self.send_command(f"s {s1} {s2} {s3}\n".encode())

	def readline(self):
		return self.mcu.readline()

	def reset_input_buffer(self):
		self.mcu.reset_input_buffer()

	def grip_and_throw(self, Cx, Cy, Num, cap):
		Cx += 80
		DRx, DRy, DRz = 0, 250, -700
		can_x = [0, 400, 400, 0, -400, -400]
		can_y = [0, 300, -300, -300, -300, 300]

		self.inverse(DRx, DRy, DRz)
		self.send_move_command()
		
		self.send_grip_command(1)
		
		while True:
			rec = self.readline()
			if rec is None:
				continue
			if 'ON'.encode() in rec:
				break
		self.send_set_downmode_command()
		while True:
			Mxy = None
			while Mxy is None:
				# rec = self.readline()
				# if rec is not None and 'FS'.encode() in rec:
				# 	return
				Mxy = detect_aruco_marker(cap)
			print(Mxy)
			Mx, My = Mxy
			DRx += (Cx - Mx) * 0.1
			DRy -= (Cy - My) * 0.1
			DRz -= 10

			print((DRx, DRy, DRz))
			self.inverse(DRx, DRy, DRz)
			self.send_downmove_command()	
			time.sleep(0.1)

			rec = self.readline()
			# print(rec)
			if rec is not None and 'FS'.encode() in rec:
				break
		
		DRz = -700
		self.inverse(DRx, DRy, DRz)
		self.send_move_command()
		# print(self.readline())

		self.inverse(can_x[Num], can_y[Num], DRz)
		self.send_move_command()

		self.send_grip_command(0)

		self.inverse(0, 0, -800)
		self.send_move_command()

def detect_aruco_marker(cap):
	aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250) #DICT_6X6_250에 해당하는 마크를 탐지

	rtn, frame = cap.read()
	if not rtn:
		print("Error: Unable to read from webcam.")
		return

	h, w = frame.shape[:2]
	# frame = cv2.getRectSubPix(frame, (480, 480), (w/2.0, h/2.0))

	gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
	corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict)
	if ids is None:
		print("detect_aruco_marker: no detect marker\n")
		return None
	if len(ids) != 3:
		print("detect_aruco_marker: less marker\n")
		return None
	
	# 탐지된 마커를 표시
	aruco.drawDetectedMarkers(frame, corners)
	frame_x, frame_y = 0, 0
	for corner, id in zip(corners, ids):
		# 꼭짓점 좌표 가져오기
		pts = corner[0]
		
		# 중심 좌표 계산
		center_x = int((pts[0][0] + pts[2][0]) / 2)
		center_y = int((pts[0][1] + pts[2][1]) / 2)
		frame_x += center_x
		frame_y += center_y

	# cv2.imshow("yaho", frame)
	# cv2.waitKey(1)

	frame_x = frame_x / 3
	frame_y = frame_y / 3	
	return frame_x, frame_y

if __name__ == "__main__":

	# === Step 1: Lock camera settings (Linux-specific using v4l2-ctl) ===
	os.system('v4l2-ctl -d /dev/video0 --set-ctrl exposure_auto=1')  # 수동 노출 모드
	os.system('v4l2-ctl -d /dev/video0 --set-ctrl exposure_absolute=200')  # 값은 50~200 범위에서 실험
	os.system('v4l2-ctl -d /dev/video0 --set-ctrl white_balance_temperature_auto=0')
	os.system('v4l2-ctl -d /dev/video0 --set-ctrl white_balance_temperature=4000')  # 실내 기준

	# 웹캠 연결 (0은 기본 웹캠)
	cap = cv2.VideoCapture(0) 
	cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
	cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 640)

	myDeltaRobot = DeltaRobot(R=300.0, r=60.0, L=400.0, l=890.0)
	# myDeltaRobot.send_reset_command()
	
	while True:
		can_x = [0, 400, 400, 0, -400, -400]
		can_y = [0, 300, -300, -300, -300, 300]

		data = input("Cx, Cy, Num: ")
		if 'r' in data:
			myDeltaRobot.send_reset_command()
			continue
		if 'm' in data:
			_, x, y, z = data.split(' ')
			x = float(x)
			y = float(y)
			z = float(z)
			myDeltaRobot.inverse(x, y, z)
			myDeltaRobot.send_move_command()
			continue
		if 'g' in data:
			myDeltaRobot.send_grip_command(int(data[1]))
			continue
		if 'c' in data:
			_, Cx, Cy, Num = data.split(' ')
			myDeltaRobot.grip_and_throw(float(Cx), float(Cy), int(Num))
		

		# print(myDeltaRobot.readline())

	cap.release()
	cv2.destroyAllWindows()

		# command = input("명령어를 입력하세요 (예: m 0 0 0): ")
		# if command[0] == 'm':
		# 	# 명령어를 파싱하여 x, y, z 좌표 추출
		# 	_, x, y, z = command.split(' ')
		# 	x = float(x)
		# 	y = float(y)
		# 	z = float(z)
		# 	myDeltaRobot.inverse(x, y, z)
		# 	myDeltaRobot.send_move_command()
		# elif command[0] == 's':
		# 	data = command.split(' ')
		# 	if len(data) == 2:
		# 		myDeltaRobot.send_set_speed_command(int(data[1]))
		# 	elif len(data) == 4:
		# 		myDeltaRobot.send_set_speed_command(data[1:])
		# 	# myDeltaRobot.send_command(f"s {x} {y} {z}\n".encode())
		# 	# continue
		# elif command[0] == 'g':
		# 	myDeltaRobot.send_grip_command(int(command[1]))
		# 	# print(f"g{command[1:]}\n".encode())
		# 	# myDeltaRobot.send_command(f"g{command[1:]}\n".encode())
		# 	# continue
		# elif command[0] == 'r':
		# 	myDeltaRobot.send_reset_command()
		# 	# print(f"r{command[1:]}\n".encode())
		# 	# myDeltaRobot.send_command(f"r{command[1:]}\n".encode())
		# 	# continue

		
		