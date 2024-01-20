import os
os.environ['YOLO_VERBOSE'] = str(False)
from ultralytics import YOLO

# 加载预训练的YOLOv8n模型
model = YOLO('yolov8n.pt')

# 在图片上运行推理
results = model('ultralytics/assets/zidane.jpg',conf=0.3,imgsz=(640,480),max_det=3,stream=True,save=True)

# 查看结果
# for r in results:
results = list(results) # 转换为列表
need_class_dit = results[0].names#字典类别
print(need_class_dit)
need_result = results[0].boxes
# print("类别字典：",need_class_dit)
#
need_class_list = need_result.cls  # 类别
# need_class_level = need_result.conf # 置信度
need_class_box = need_result.xyxy # 框
print(need_class_list)
# # print(need_class_level)
print(need_class_box)

# for r in results:
#     need_class_dit = r.names#字典类别
#     print(need_class_dit)
#     need_result = r.boxes
#     # print("类别字典：",need_class_dit)
#     #
#     # need_class_list = need_result.cls  # 类别
#     # need_class_level = need_result.conf # 置信度
#     need_class_box = need_result.xyxy # 框
#     # # print(need_class_list)
#     # # print(need_class_level)
#     print(need_class_box)