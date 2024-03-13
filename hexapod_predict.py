import json
import time
import os
import cv2
import socket
os.environ['YOLO_VERBOSE'] = str(False)
from ultralytics import YOLO
from get_need_result import convert_boxes

host = "192.168.149.1"   # 修改为树莓派的IP地址
port = 1024


client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect((host, port))
 
def send_data_to_raspi_aal(xywh, cls):
    data_to_send = {"xywh": xywh, "cls": cls}
    json_data = json.dumps(data_to_send)
    client_socket.send(json_data.encode())

model = YOLO('hexapod_cup_v2.pt')
video_stream_url = f'http://{host}:5000/video_feed'
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
            if cout % 3 == 0:
                results = model.predict(frame, conf=0.4, imgsz=(640, 480), max_det=3, save=True)
                annotated_frame = results[0].plot()
                boxes = results[0].boxes
                xywh, cls = convert_boxes(boxes)
                send_data_to_raspi_aal(xywh, cls)
                print(cls)

                cv2.imshow("hexapod", annotated_frame)
                fps = 1.0 / (time.time() - start_time)
                start_time = time.time()
                cv2.putText(annotated_frame, f"{fps:.1f} FPS", (0, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()
    client_socket.close()
