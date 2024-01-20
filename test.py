import cv2
import os
os.environ['YOLO_VERBOSE'] = str(False)
from ultralytics import YOLO
import cv2
# 加载模型
model = YOLO('yolov8n.pt')  # 预训练的 YOLOv8n 模型

# 在图片列表上运行批量推理
results = model("ultralytics/assets/zidane.jpg")  # 返回 Results 对象列表
# frame = results[0].plot()  # 可视化结果
boxes = results[0].boxes
xywh = boxes.xywh
xywh = xywh.tolist()
#将二维列表转化为整数二维列表
xywh = [[int(j) for j in i] for i in xywh]
# print(xywh)
# print(len(xywh))

cls = boxes.cls
cls = cls.tolist()
cls = [int(i) for i in cls]
# print(cls)
for i in range(len(cls)):
    if cls[i] == 0:
        print(cls[i])
        print(xywh[i])
        print("------------")






# cv2.imshow("hexapod", frame)
# cv2.waitKey(0)
# cv2.destroyAllWindows()
