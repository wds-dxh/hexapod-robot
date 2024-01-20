# 训练

* python

```python
from ultralytics import YOLO

# 加载一个模型
model = YOLO('yolov8n.yaml')  # 从YAML建立一个新模型
model = YOLO('yolov8n.pt')  # 加载预训练模型（推荐用于训练）
model = YOLO('yolov8n.yaml').load('yolov8n.pt')  # 从YAML建立并转移权重

# 训练模型
results = model.train(data='coco128.yaml', epochs=100, imgsz=640)
```

* cli

```shell
# 从YAML构建新模型，从头开始训练
yolo detect train data=coco128.yaml model=yolov8n.yaml epochs=100 imgsz=640

# 从预训练*.pt模型开始训练
yolo detect train data=coco128.yaml model=yolov8n.pt epochs=100 imgsz=640

# 从YAML构建一个新模型，转移预训练权重，然后开始训练
yolo detect train data=coco128.yaml model=yolov8n.yaml pretrained=yolov8n.pt epochs=100 imgsz=640
```

* 恢复训练

python:

```shell
from ultralytics import YOLO

# 加载模型
model = YOLO('path/to/last.pt')  # 加载部分训练的模型

# 恢复训练
results = model.train(resume=True)
```

cli

```shell
# 恢复中断的训练
yolo train resume model=path/to/last.pt
```



* 记录

```shell
tensorboard --logdir ultralytics/runs  # 替换为'runs'目录
```

# 推理

```python
from ultralytics import YOLO

# 加载模型
model = YOLO('yolov8n.pt')  # 预训练的 YOLOv8n 模型

# 在图片列表上运行批量推理
results = model(['im1.jpg', 'im2.jpg'])  # 返回 Results 对象列表

# 处理结果列表
for result in results:
    boxes = result.boxes  # 边界框输出的 Boxes 对象
    masks = result.masks  # 分割掩码输出的 Masks 对象
    keypoints = result.keypoints  # 姿态输出的 Keypoints 对象
    probs = result.probs  # 分类输出的 Probs 对象
```



Results`对象具有以下属性：

| 属性         | 类型              | 描述                                                     |
| :----------- | :---------------- | :------------------------------------------------------- |
| `orig_img`   | `numpy.ndarray`   | 原始图像的numpy数组。                                    |
| `orig_shape` | `tuple`           | 原始图像的形状，格式为（高度，宽度）。                   |
| `boxes`      | `Boxes, 可选`     | 包含检测边界框的Boxes对象。                              |
| `masks`      | `Masks, 可选`     | 包含检测掩码的Masks对象。                                |
| `probs`      | `Probs, 可选`     | 包含每个类别的概率的Probs对象，用于分类任务。            |
| `keypoints`  | `Keypoints, 可选` | 包含每个对象检测到的关键点的Keypoints对象。              |
| `speed`      | `dict`            | 以毫秒为单位的每张图片的预处理、推理和后处理速度的字典。 |
| `names`      | `dict`            | 类别名称的字典。                                         |
| `path`       | `str`             | 图像文件的路径。                                         |











