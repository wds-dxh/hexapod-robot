import time
from get_need_result import convert_boxes
import cv2
import os
os.environ['YOLO_VERBOSE'] = str(False)#不打印yolov8信息
from ultralytics import YOLO


# 加载YOLOv8模型
model = YOLO('hexapod_cup.pt')

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
        time.sleep(0.1)
        print("mini_x_data:",min_x_data)



# 打开视频文件
# video_stream_url = 'http://192.168.137.1:5000/video_feed'
# video_stream_url = 'http://192.168.9.9:5000/video_feed'
video_stream_url = 'http://192.168.1.36:5000/video_feed'
# video_stream_url = 'save_video/hexapod.avi'
cap = cv2.VideoCapture(video_stream_url)
start_time = time.time()
# 遍历视频帧
while cap.isOpened():
    # 从视频中读取一帧
    success, frame = cap.read()
    # frame = cv2.imread("two.png")
    # if success:
    if frame is not None:
        # 在该帧上运行YOLOv8推理
        frame = cv2.resize(frame, (640, 480), interpolation=cv2.INTER_LINEAR)
        results = model.predict(frame,conf=0.6,imgsz=(640, 480),max_det=3,save=True)

        # 在帧上可视化结果
        annotated_frame = results[0].plot()
        boxes = results[0].boxes
        xywh, cls = convert_boxes(boxes)  # 得到类别和坐标
        print(xywh)
        send_data_to_raspi(xywh, cls)

        fps = 1.0 / (time.time() - start_time)
        cv2.putText(annotated_frame, f"{fps:.1f} FPS", (0, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        # 显示带注释的帧
        cv2.imshow("YOLOv8推理", annotated_frame)
        # 显示FPS
        # cv2.waitKey(0)
        start_time = time.time()
        # 如果按下'q'则中断循环
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    else:
        # 如果视频结束则中断循环
        break

# 释放视频捕获对象并关闭显示窗口
cap.release()
cv2.destroyAllWindows()