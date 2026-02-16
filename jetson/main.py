import logging
logging.getLogger('werkzeug').setLevel(logging.ERROR)  # Suppress Flask debug logs

from flask import Flask, Response, render_template
import cv2
import cv2.aruco as aruco
from ultralytics import YOLO
import torch
import os
import time
import requests
import detection
import control_delta_robot
os.environ['QT_QPA_PLATFORM'] = 'offscreen'

app = Flask(__name__)
detection.initialize_exposure()

# 허용 오차 및 안정화 시간 설정
TOL_CENTER = 3		 	# 픽셀 단위 허용 오차
STABLE_DURATION = 3.0	# 안정화 시간 (초 단위)

# 최종 확정된 중심점 (나중에 전송할 용도)
confirmed_center = None
trashcan_index = None
robot_running = 0

# ——— 설정 ———
CAM_ID = 0
VIDEO_DEV = '/dev/video0'
DET_CROP = (480, 480)
DEVICE = 'cuda' if torch.cuda.is_available() else 'cpu'
model = YOLO("final.pt")

# ——— 카메라 초기화 + 수동 노출 설정 ———
cap = cv2.VideoCapture(CAM_ID)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

myDeltaRobot = control_delta_robot.DeltaRobot(300, 60, 400, 890)
myDeltaRobot.inverse(0, 0, -800)
myDeltaRobot.send_move_command()
myDeltaRobot.send_reset_command()

def generate_frames():
	global confirmed_center, trashcan_index
	candidate_center = None		# 안정화 후보 중심점
	stable_start_time = None	# 안정화 시작 시점

	while True:
		success, frame = cap.read()
		if not success:
			continue

		h, w = frame.shape[:2]
		det_roi = cv2.getRectSubPix(frame, DET_CROP, (w/2.0, h/2.0))

		if robot_running:
			# 로봇이 동작 중이면 프레임을 처리하지 않고 넘어감
			# print("로봇이 동작 중입니다. 프레임을 처리하지 않습니다.")
			aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250) #DICT_6X6_250에 해당하는 마크를 탐지
			gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
			corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict)
			if len(corners) == 3:
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
				frame_x = frame_x / len(corners)
				frame_y = frame_y / len(corners)

				# 중심점 표시
				cv2.circle(frame, (int(frame_x), int(frame_y)), 5, (0,0,255), -1)
				cv2.circle(frame, (confirmed_center[0] + 80, confirmed_center[1]), 5, (255,0,0), -1)

			ret, buffer = cv2.imencode('.jpg', frame)
			if not ret:
				continue
			frame_bytes = buffer.tobytes()
			yield (b'--frame\r\n'
				   b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
			time.sleep(0.1)
			continue

		results = model(det_roi, device=DEVICE, show=False, verbose=False)[0]

		# 중심점 안정화 로직
		if len(results.boxes) > 0:
			x1, y1, x2, y2 = map(int, results.boxes[0].xyxy[0])
			cx, cy = (x1 + x2) // 2, (y1 + y2) // 2

			# 바운딩 박스 내에서의 상대적 위치 비율 (중심점 기준)
			box_w = x2 - x1
			box_h = y2 - y1
			rel_cx = (cx - x1) / box_w  # 0~1, 0.5가 중심
			rel_cy = (cy - y1) / box_h

			# 왜곡 보정: 중심에서 멀수록 더 크게 보정 (예: k=0.2)
			k = 0.15
			# 중심(0.5, 0.5)에서의 거리 비율
			dx = rel_cx - 0.5
			dy = rel_cy - 0.5
			# 보정 적용 (바운딩 박스 내에서만 이동)
			cx_corr = int(cx + dx * k * box_w)
			cy_corr = int(cy + dy * k * box_h)

			# 이후 로직에서 cx_corr, cy_corr 사용
			if candidate_center is None:
				candidate_center = (cx_corr, cy_corr)
				stable_start_time = time.time()
			else:
				dx2 = abs(cx_corr - candidate_center[0])
				dy2 = abs(cy_corr - candidate_center[1])
				if dx2 <= TOL_CENTER and dy2 <= TOL_CENTER:
					if (time.time() - stable_start_time >= STABLE_DURATION and confirmed_center is None):
						confirmed_center = candidate_center
						print(f'확정된 중심점: {confirmed_center}')
				else:
					confirmed_center = None
					candidate_center = (cx_corr, cy_corr)
					stable_start_time = time.time()
		else:
			# 객체가 없으면 리셋
			candidate_center = None
			stable_start_time = None

		for box in results.boxes:
			x1, y1, x2, y2 = map(int, box.xyxy[0])
			cls_id = int(box.cls[0])
			conf = float(box.conf[0])
			label = model.names[cls_id]
			cv2.rectangle(det_roi, (x1, y1), (x2, y2), (0,255,0), 2)
			cv2.putText(det_roi, f'{label} {conf:.2f}', (x1, y1-5),
						cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)

			if label == 'Metal':
				trashcan_index = 3
			elif label == 'Plastic':
				trashcan_index = 4
			elif label == 'Paper pack' or label == 'Paper':
				trashcan_index = 5
			else: # General trash
				trashcan_index = 1 # or 2

			# 중심점 찍기
			cv2.circle(det_roi, (cx_corr, cy_corr), 5, (0,0,255), -1)
		
		# 중심점 및 쓰레기통 인덱스 출력
		# print(f'확정된 중심점: {confirmed_center}, 쓰레기통 인덱스: {trashcan_index}')
		
		# 전송할 프레임 인코딩
		ret, buffer = cv2.imencode('.jpg', det_roi)
		if not ret:
			continue

		frame_bytes = buffer.tobytes()
		yield (b'--frame\r\n'
			   b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
		time.sleep(0.1)  # 프레임 속도 조절

@app.route('/')
def index():
	return render_template('index.html')

@app.route('/run_delta_robot', methods=['POST'])
def run_delta_robot():
	global confirmed_center, trashcan_index, robot_running
	if robot_running == 1:
		return {"status": "error", "message": "로봇이 이미 동작 중입니다."}
	robot_running = 1
	if confirmed_center is not None and trashcan_index is not None:
		Cx, Cy = confirmed_center
		print(f'확정된 중심점으로 로봇 동작: {Cx}, {Cy}, {trashcan_index}')
		myDeltaRobot.grip_and_throw(Cx, Cy, trashcan_index, cap)
		time.sleep(3)  # 로봇 동작 완료 대기
		robot_running = 0
		return {"status": "success", "message": f"로봇이 {Cx}, {Cy}, {trashcan_index} 위치로 동작했습니다."}
	return {"status": "error", "message": "확정된 중심점이 없습니다."}

@app.route('/reset_delta_robot', methods=['POST'])
def reset_delta_robot():
	myDeltaRobot.inverse(0, 0, -800)
	myDeltaRobot.send_move_command()
	myDeltaRobot.send_reset_command()
	return {"status": "success", "message": f"robot reset."}

@app.route('/video_feed')
def video_feed():
	return Response(
		generate_frames(),
		mimetype='multipart/x-mixed-replace; boundary=frame'
	)

@app.route('/get_center')
def get_center():
	global confirmed_center
	return {"center": str(confirmed_center) if confirmed_center else None}

if __name__ == '__main__':
	app.run(host='0.0.0.0', port=5000, threaded=True)
	print('Model is ready! Please Reload on Web!')
