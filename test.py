import torch
print(torch.cuda.is_available())  # 打印是否可用
print(torch.version.cuda)  # 打印CUDA版本
print(torch.cuda.get_device_name(0))  # 打印默认GPU设备的名称
