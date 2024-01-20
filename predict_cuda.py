import time
import cv2
from ultralytics import YOLO
import threading

# 加载YOLOv8模型
model = YOLO('best.pt')

# 用于存储当前帧的全局变量
current_frame = None
frame_lock = threading.Lock()


# 读取视频流的线程
def video_stream_thread():
    global current_frame
    cap = cv2.VideoCapture('http://172.20.10.4:5000/video_feed')

    while cap.isOpened():
        success, frame = cap.read()
        if success:
            with frame_lock:
                current_frame = frame

    cap.release()


# 推理和处理数据的线程
def inference_thread():
    global current_frame
    start_time = time.time()

    while True:
        with frame_lock:
            if current_frame is not None:
                # 在当前帧上运行YOLOv8推理
                results = model.predict(current_frame, conf=0.6, imgsz=(640, 480), max_det=1, save=True)

                # 在帧上可视化结果
                annotated_frame = results[0].plot()

                fps = 1.0 / (time.time() - start_time)
                cv2.putText(annotated_frame, f"{fps:.1f} FPS", (0, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

                # 显示带注释的帧
                cv2.imshow("YOLOv8推理", annotated_frame)

                start_time = time.time()

        # 如果按下'q'则中断循环
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break


# 创建并启动两个线程
video_thread = threading.Thread(target=video_stream_thread)
inference_thread = threading.Thread(target=inference_thread)

video_thread.start()
inference_thread.start()

# 等待两个线程结束
video_thread.join()
inference_thread.join()

# 关闭所有窗口
cv2.destroyAllWindows()
