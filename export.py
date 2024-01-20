from ultralytics import YOLO

# 加载模型
model = YOLO('yolov8n.pt')  # 加载官方模型
model = YOLO('hexapod.pt')  # 加载自定义训练的模型

# 导出模型
model.export(format='onnx')