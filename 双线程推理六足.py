import threading
import time
import os
import cv2
import socket
os.environ['YOLO_VERBOSE'] = str(False)
from ultralytics import YOLO


host = "192.168.1.36"
port = 6524
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)# 创建客户端套接字
client_socket.connect((host, port))# 连接服务器
xywh = None
cls = None
def send_data_to_raspi():#(xywh, cls)
    global xywh
    global cls
    detect_color = 1
    min_x_data = None  # 初始化一个空列表
    min_x_value = 640  # 初始化最小x值为正无穷大
    if cls is None and xywh is None:
        for i in range(len(cls)):
            # print(cls[i])
            if cls[i] == detect_color:
                if xywh[i][0] < min_x_value:
                    min_x_value = xywh[i][0]
                    min_x_data = str(xywh[i])
                    # print(min_x_data)

    if min_x_data is not None:
        print(min_x_data)
        # time.sleep(0.05)
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



current_frame = None
frame_lock = threading.Lock()
def video_stream_thread():
    global current_frame
    video_stream_url = 'http://192.168.1.36:5000/video_feed'
    # video_stream_url = "save_video/hexapod.avi"
    cap = cv2.VideoCapture(video_stream_url)

    while cap.isOpened():
        success, frame = cap.read()
        #视频文件的情况下，延时等待推理线程
        time.sleep(0.03)
        if success:
            with frame_lock:
                current_frame = frame
    cap.release()

model = YOLO('hexapod.pt')
def inference_thread():
    global current_frame
    global xywh
    global cls
    threading.Thread(target=send_data_to_raspi, daemon=True).start()
    threading.Thread(target=video_stream_thread, daemon=True).start()
    while True:
        with frame_lock:
            if current_frame is not None:
                    # frame = current_frame
                results = model.predict(current_frame,conf=0.6,imgsz=(640,480),max_det=1,save=True)
                annotated_frame = results[0].plot()
                boxes = results[0].boxes
                xywh,cls = convert_boxes(boxes)#得到类别和框
                # send_data_to_raspi(xywh,cls)
                # print(xywh)

                # 显示带注释的帧
                cv2.imshow("hexapod", annotated_frame)
                # 显示FPS
                # cv2.waitKey(0)
                # fps = 1.0 / (time.time() - start_time)
                # start_time = time.time()
                # cv2.putText(annotated_frame, f"{fps:.1f} FPS", (0, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
    cv2.destroyAllWindows()
    client_socket.close()# 关闭连接
if __name__ == '__main__':
    send = threading.Thread(target=inference_thread)
    send.start()
    send.join()
    video = threading.Thread(target=video_stream_thread)
    video.start()
    video.join()
    inference = threading.Thread(target=inference_thread)
    inference.start()
    inference.join()
    cv2.destroyAllWindows()