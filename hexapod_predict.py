import time
import os
import cv2
import socket
os.environ['YOLO_VERBOSE'] = str(False)
from ultralytics import YOLO


# 获取服务器的主机名和端口号
# host = "192.168.1.36"
# port = 6524
# client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# client_socket.connect((host, port))# 连接服务器

#要识别的颜色
# red:0
# blue:1
# cup:2
# detect_color = 1

# def send_data_to_raspi(xywh,cls):
#     global detect_x
#     global detect_color
#     for i in range(len(cls)):
#         print(cls[i])
#         if cls[i] == detect_color:
#             if xywh[i][0] < detect_x:
#                 detect_x = xywh[i][0]
#                 data = str(xywh[i])
#             else:
#                 data = str("no")
#         else:
#             data = str("no")
#     client_socket.send(data.encode())# 发送数据


host = "192.168.1.36"
port = 6524
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)# 创建客户端套接字
client_socket.connect((host, port))# 连接服务器
def send_data_to_raspi(xywh, cls):
    detect_color = 1
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
        time.sleep(0.1)
        client_socket.send(min_x_data.encode())  # 发送数据


def convert_boxes(boxes):
    xywh = boxes.xywh
    xywh = xywh.tolist()
    #将二维列表转化为整数二维列表
    xywh = [[int(j) for j in i] for i in xywh]
    cls = boxes.cls
    cls = cls.tolist()
    cls = [int(i) for i in cls]
    return xywh,cls


model = YOLO('hexapod.pt')
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
                results = model.predict(frame,conf=0.6,imgsz=(640,480),max_det=1,save=True)
                annotated_frame = results[0].plot()
                boxes = results[0].boxes
                xywh,cls = convert_boxes(boxes)#得到类别和框
                send_data_to_raspi(xywh,cls)
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