import torch

# 给定三个张量
tensor1 = torch.tensor([0., 0., 27.], device='cuda:0')
tensor2 = torch.tensor([0.8360, 0.8190, 0.2910], device='cuda:0')
tensor3 = torch.tensor([[114.8699, 197.4121, 1114.4688, 711.8894],
                        [748.4614, 41.8552, 1143.0757, 713.0239],
                        [439.4747, 437.0735, 524.3459, 709.1597]], device='cuda:0')

# 访问元素
print(len(tensor1))  # 返回张量的长度
result = int(tensor1[0].item() ) # 通过索引访问元素,item()的作用是返回一个Python数值
print("Element:", result)  #item()方法返回一个Python数值





