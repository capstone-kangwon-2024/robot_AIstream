from flask import Flask, render_template, Response
from ultralytics import YOLO
import pyrealsense2 as rs
import numpy as np
import cv2
import os
import subprocess
import signal
import smbus  # i2c 통신을 위해

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 424, 240, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 360, rs.format.z16, 30)

align_to = rs.stream.color
align = rs.align(align_to)

profile = pipeline.start(config)

depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()

# 타격지점 예측박스 크기는 60x40
# 거리 160~180, 180~200, 200~220 에 따라 타격예상 지점 바뀜
# 기준 박스 좌표계산
# x: 중앙인 212px에 -30px한 x(182) +30px한 x(242)
# y: 중앙인 120px에 -20px한 y(100) +20px한 y(140)

center_box = [
    (182, 85, 242, 115),
    (182, 110, 242, 130),
    (182, 125, 242, 155),
    (182, 85, 242, 155),
]

 # 파손위험성이 크거나 위험한 물체 목록
target_classes = ['tv', 'laptop', 'dog', 'vase', 'person', 'toaster', 'bottle', 'microwave', 'potted plant', 'chair']

os.environ['YOLO_VERBOSE'] = 'False'
model = YOLO("yolov8n.pt")

# 메모리에 모델 미리적재
print("AI준비중....")
color_frame = None    
while True:
    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    
    if not color_frame: continue
    
    frame = np.asanyarray(color_frame.get_data())
        
    results = model(frame, conf=0.6, verbose=False) 
    
    break
print("AI실행 준비완료!")

app = Flask(__name__)

def is_overlap(box1, box2):
    x1_min, y1_min, x1_max, y1_max = box1
    x2_min, y2_min, x2_max, y2_max = box2
    
    return not (x1_max < x2_min or x1_min > x2_max or y1_max < y2_min or y1_min > y2_max)

def get_depth_at_pixel(depth_frame, pixel_x, pixel_y):
	return depth_frame.as_depth_frame().get_distance(round(pixel_x), round(pixel_y))

# AI분석과 실시간 영상 송출과 하는 코드부분
def generate_frame():
        while True:
            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)
            
            # 컬러영상과 거리정보영상을 흭득
            color_frame = aligned_frames.get_color_frame()
            aligned_depth_frame = aligned_frames.get_depth_frame() 
            
            # OpenCV로 개발가능하도록 변환
            color_frame    = np.asanyarray(color_frame.get_data())
            depth_raw      = np.asanyarray(aligned_depth_frame.get_data())
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_raw, alpha=0.1), cv2.COLORMAP_JET)
            
            # 컬러영상 AI분석 시작 ( 수행시간 80ms ~ 100ms 소요 )
            results = model(color_frame, conf=0.6, verbose=False) 
            
            # AI분석 결과 수신 및 처리
            for result in results:
                for box in result.boxes:
                    # AI분석으로 판독된 물체이름
                    class_name = model.names[int(box.cls)]
                    
                    # 해당 물체가 사전에 정의해둔 파손위험, 위험물체 인가
                    if class_name in target_classes:
                        # 컬러영상에서 해당 물체의 좌표 저장
                        bbox = box.xyxy[0].tolist()
                        
                        x = [int(bbox[0]), int(bbox[2])]
                        y = [int(bbox[1]), int(bbox[3])]
                        
                        # 물체 거리 측정 알고리즘 수행시작.
                        # 측정 알고리즘 수행완료 시 물체에서 가장 가까운 몸체의 거리를 흭득함.
                        rows = int((y[1]-y[0])/4)
                        cols = int((x[1]-x[0])/4)
                        dis_mat = np.full((rows, cols), 999, dtype=int)
                        for yi in range(0, rows):
                            for xi in range(0, cols):
                                dis = int(get_depth_at_pixel(aligned_depth_frame, x[0]+xi*4, y[0]+yi*4)*100)
                                
                                if not dis == 0: dis_mat[yi][xi] = dis
                        
                        # 알고리즘으로 흭득한 물체 거리와 화면에서의 좌표저장
                        target_dis = np.min(dis_mat.ravel())
                        mi         = np.argmin(dis_mat.ravel())
                        mi_y = int(mi/cols)
                        mi_x = mi - mi_y*cols
                        mi_x = x[0] + (mi_x*4)
                        mi_y = y[0] + (mi_y*4)
                        
                        # 물체 거리가 공 발사 시 착탄 지점과 가까워지면 경고를 하도록함
                        # 경고방식은 물체 테두리 박스, 물체이름은 하얀색 -> 빨간색에 가까워 지도록 함
                        other_val = 255
                        if 130 <= target_dis and target_dis <= 150:
                            other_val = int((target_dis-130)/1) * 12
                        elif target_dis < 130:
                            other_val = 0
                            if target_dis < 130:  cv2.putText(color_frame, f'too close! {target_dis}cm', (int((x[0]+x[1])/2), y[0]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
                            
                            cv2.rectangle(color_frame, (x[0], y[0]), (x[1], y[1]), (other_val, other_val, 255), 1)
                            cv2.putText(color_frame, f'{class_name}', (mi_x, mi_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (other_val, other_val, 255), 1)
                        
                        # 물체 거리에 따라 착탄지점을 예측합니다
                        # 기본적으로 하단, 중단, 상단으로 3부분만 예측합니다.
                        dis_conf = -1
                        if 105 < target_dis and target_dis < 130:     dis_conf = 0
                        elif 130 <= target_dis and target_dis <= 140: dis_conf = 1
                        elif 140 < target_dis and target_dis <= 160:  dis_conf = 2
                        
                        # 계산된 착탄지점에 물체가 있는지 검사합니다.
                        # 검사결과, 착탄지점에 있을 시 경고창을 띄웁니다.
                        target_closest_box = (mi_x-5, mi_y-5, mi_x+5, mi_y+5)
                        if dis_conf != -1 and is_overlap(center_box[3], target_closest_box):
                            cv2.rectangle(color_frame, \
                                    (center_box[dis_conf][0], center_box[dis_conf][1]), \
                                    (center_box[dis_conf][2], center_box[dis_conf][3]), \
                                    (0, 0, 255), 2)
                            cv2.putText(color_frame, f'warning! {m_dis}cm', (center_box[dis_conf][0], center_box[dis_conf][1]-5), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 1) 
            
            # 지금까지 수행한 AI분석 결과를 이미지로 변환합니다
            ret, buffer = cv2.imencode('.jpg', color_frame)
            
            # 이미지를 웹스트리밍 할 수 있도록 바이트코드로 변환하여 송출합니다.
            res = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + res + b'\r\n\r\n')

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/video_feed')
def video_feed():
    return Response(generate_frame(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

def exit_handler(signum, frame):
    print("프로그램 종료")
    
    pipeline.stop()
    cv2.destroyAllWindows() 
    
    exit()
    
# 프로그램 종료시 종료절차를 수행하도록 함
signal.signal(signal.SIGINT, exit_handler)  

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, debug=False)
    
    # gevent 비동기 방식은 core dump 문제를 발생하기에 기존 방식으로 롤백
    # http_server = WSGIServer(('0.0.0.0', 5000), app)
    # http_server.serve_forever()