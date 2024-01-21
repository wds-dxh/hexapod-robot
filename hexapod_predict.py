import json
import time
import os
import cv2
import socket
os.environ['YOLO_VERBOSE'] = str(False)
from ultralytics import YOLO
from get_need_result import convert_boxes

# host = socket.gethostname()
host = "192.168.1.36"
port = 6524

client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)# 创建客户端套接字
client_socket.connect((host, port))# 连接服务器

def send_data_to_raspi_aal(xywh, cls):
    data_to_send = {"xywh": xywh, "cls": cls}
    json_data = json.dumps(data_to_send)
    client_socket.send(json_data.encode())


def send_data_to_raspi(xywh, cls):
    detect_color = 0
    min_x_data = None  # 初始化一个空列表
    min_x_value = 640  # 初始化最小x值为正无穷大
    for i in range(len(cls)):
        # print(cls[i])
        if cls[i] == detect_color:
            if xywh[i][0] < min_x_value:
                min_x_value = xywh[i][0]
                min_x_data = str(xywh[i])
                print(min_x_data)
    if min_x_data is not None:
        # time.sleep(0.05)
        client_socket.send(min_x_data.encode())      # 发送数据

model = YOLO('hexapod_cup.pt')
video_stream_url = 'http://192.168.1.36:5000/video_feed'
# video_stream_url = "save_video/hexapod.avi"
cap = cv2.VideoCapture(video_stream_url)
start_time = time.time()

cout = 0
if __name__ == '__main__':
    while cap.isOpened():
        cout += 1
        success, frame = cap.read()
        if not success:
            break
        else:
            if cout%3 == 0:
                results = model.predict(frame,conf=0.4,imgsz=(640,480),max_det=3,save=True)
                annotated_frame = results[0].plot()
                boxes = results[0].boxes
                xywh,cls = convert_boxes(boxes)#得到类别和框
                # if cout%5 == 0:
                # send_data_to_raspi(xywh,cls)
                send_data_to_raspi_aal(xywh,cls)
                # print(xywh)

                # 显示带注释的帧
                cv2.imshow("hexapod", annotated_frame)
                # 显示FPS
                # cv2.waitKey(0)
                fps = 1.0 / (time.time() - start_time)
                start_time = time.time()
                cv2.putText(annotated_frame, f"{fps:.1f} FPS", (0, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    # 释放视频捕获对象并关闭显示窗口
    cap.release()
    cv2.destroyAllWindows()
    client_socket.close()# 关闭连接