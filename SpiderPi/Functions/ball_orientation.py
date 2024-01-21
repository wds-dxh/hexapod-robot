#!/usr/bin/python3
# coding=utf8
import sys
sys.path.append('/home/pi/SpiderPi/')
import cv2
import math
import time
import Camera
import threading
import kinematics
import import_path
import numpy as np
import yaml_handle
import ArmIK.ArmMoveIK as AMK
import HiwonderSDK.Misc as Misc
import HiwonderSDK.Board as Board

# 小球定位

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

# 逆运动学定义
ik = kinematics.IK()
AK = AMK.ArmIK()

# 初始姿态参数定义
# 姿态数据为3x6的数组，表示6条腿的的末端的x，y，z坐标，单位mm
# 头部朝前的方向为x轴， 头部朝前位置为负方向，右边为y轴正， 竖直朝上为z轴正， 从中间两条腿的连线的中点做垂线与上下板相交，取连线中心为零点
# 第一条腿表示头部朝前时左上角所在腿, 逆时针表示1-6
current_pos = [[-199.53, -177.73, -100.0],
               [ -35.0, -211.27, -100.0],
               [199.53, -177.73, -100.0],
               [199.53,  177.73, -100.0],
               [ -35.0,  211.27, -100.0],
               [-199.53, 177.73, -100.0]]

# 左右两只前脚与摄像头坐标零点对应的参数
left_pos =  [-220, -0, -80]
right_pos = [-220,  0, -80]


range_rgb = {
    'red': (0, 0, 255),
    'blue': (255, 0, 0),
    'green': (0, 255, 0),
    'black': (0, 0, 0),
    'white': (255, 255, 255)}

# 变量定义
i = 0
HWSONAR = None
step = False
size = (640, 480)
old_x, old_y = 0, 0
initial_coord = (0, 18, 10)
world_x, world_y = -1, -1
lab_data = None
K,R,T = None,None,None

# 读取颜色阈值及坐标变换参数
def load_config():
    global lab_data,K,R,T
    
    lab_data = yaml_handle.get_yaml_data(yaml_handle.lab_file_path)
    camera_cal = yaml_handle.get_yaml_data(yaml_handle.camera_file_path)['block_params']
    K = np.array(camera_cal['K'], dtype=np.float64).reshape(3, 3)
    R = np.array(camera_cal['R'], dtype=np.float64).reshape(3, 1)
    T = np.array(camera_cal['T'], dtype=np.float64).reshape(3, 1)
        

# 初始位置
def initMove():
    HWSONAR.setRGBMode(0)
    HWSONAR.setRGB(1, (0, 0, 0))
    HWSONAR.setRGB(2, (0, 0, 0))
    Board.setBusServoPulse(25, 180, 1000)
    ik.stand(current_pos)
    AK.setPitchRangeMoving(initial_coord, -90, -90, 100, 2000)


# 找出面积最大的轮廓
# 参数为要比较的轮廓的列表
def getAreaMaxContour(contours):
    contour_area_temp = 0
    contour_area_max = 0
    area_max_contour = None
    max_area = 0

    for c in contours:  # 历遍所有轮廓
        contour_area_temp = math.fabs(cv2.contourArea(c))  # 计算轮廓面积
        if contour_area_temp > contour_area_max:
            contour_area_max = contour_area_temp
            if contour_area_temp >= 100:  # 只有在面积大于设定时，最大面积的轮廓才是有效的，以过滤干扰
                area_max_contour = c
                max_area = contour_area_temp

    return area_max_contour, max_area  # 返回最大的轮廓, 面积

# 像素坐标变换到现实坐标函数
def camera_to_world(cam_mtx, r, t, img_points):
    inv_k = np.asmatrix(cam_mtx).I
    r_mat = np.zeros((3, 3), dtype=np.float64)
    cv2.Rodrigues(r, r_mat)
    inv_r = np.asmatrix(r_mat).I  # 3*3
    transPlaneToCam = np.dot(inv_r, np.asmatrix(t))  # 3*3 dot 3*1 = 3*1
    world_pt = []
    coords = np.zeros((3, 1), dtype=np.float64)
    for img_pt in img_points:
        coords[0][0] = img_pt[0][0]
        coords[1][0] = img_pt[0][1]
        coords[2][0] = 1.0
        worldPtCam = np.dot(inv_k, coords)  # 3*3 dot 3*1 = 3*1
        worldPtPlane = np.dot(inv_r, worldPtCam)  # 3*3 dot 3*1 = 3*1
        scale = transPlaneToCam[2][0] / worldPtPlane[2][0]
        scale_worldPtPlane = np.multiply(scale, worldPtPlane)
        worldPtPlaneReproject = np.asmatrix(scale_worldPtPlane) - np.asmatrix(transPlaneToCam)  # 3*1 dot 1*3 = 3*3
        pt = np.zeros((3, 1), dtype=np.float64)
        pt[0][0] = worldPtPlaneReproject[0][0]
        pt[1][0] = worldPtPlaneReproject[1][0]
        pt[2][0] = 0
        world_pt.append(pt.T.tolist())
    return world_pt

# 机器人移动函数
def move():
    global step
    global world_x, world_y
    
    while True:
        if step:
            if world_x > 0: # 目标在右边
                # 抬起右前脚
                current_pos[5] = [-300, 100, -50]
                ik.stand(current_pos) 
                time.sleep(2)
                
            else: # 目标在左边
                # 抬起左前脚
                current_pos[0] = [-300, -100, -50]
                ik.stand(current_pos)
                time.sleep(2)
                
        else:
            # 回到初始姿态
            current_pos[0] = [-199.53, -177.73, -100]
            ik.stand(current_pos)
            current_pos[5] = [-199.53, 177.73, -100]
            ik.stand(current_pos)
            time.sleep(1)
        

# 运行子线程
th = threading.Thread(target=move)
th.setDaemon(True)
th.start()

# 颜色识别函数
def ColorDetect(img, color='green'):
    global world_x, world_y,i
    global step, old_x, old_y
    
    img_copy = img.copy()
    img_h, img_w = img.shape[:2]
    frame_resize = cv2.resize(img_copy, size, interpolation=cv2.INTER_NEAREST)
    frame_gb = cv2.GaussianBlur(frame_resize, (3, 3), 3)      
    frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)  # 将图像转换到LAB空间
    frame_mask = cv2.inRange(frame_lab,
                             (lab_data[color]['min'][0],
                              lab_data[color]['min'][1],
                              lab_data[color]['min'][2]),
                             (lab_data[color]['max'][0],
                              lab_data[color]['max'][1],
                              lab_data[color]['max'][2]))  #对原图像和掩模进行位运算
    eroded = cv2.erode(frame_mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  #腐蚀
    dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))) #膨胀
    contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  #找出轮廓
    areaMaxContour, area_max = getAreaMaxContour(contours)  #找出最大轮廓
    
    if area_max > 500:  # 有找到最大面积
        ((centerX, centerY), radius) = cv2.minEnclosingCircle(areaMaxContour)  # 获取最小外接圆
        centerX = int(Misc.map(centerX, 0, size[0], 0, img_w))
        centerY = int(Misc.map(centerY, 0, size[1], 0, img_h))
        radius = int(Misc.map(radius, 0, size[0], 0, img_w))
        cv2.circle(img, (centerX, centerY), radius, range_rgb[color], 2)#画圆
        cv2.putText(img, "Color: " + color, (10, img.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.65, range_rgb[color], 2) #显示颜色名字
        
        if abs(centerX-old_x) < 8 and abs(centerY-old_y) < 8: # 判断目标坐标有没有变化
            i += 1
        else:
            i = 0
            old_x, old_y = centerX, centerY
            
        if i > 5: # 多次判断，确定目标位置稳定
            i = 0
            # 转换成现实距离
            center = np.array([centerX,centerY])
            w = camera_to_world(K, R, T, center.reshape((1, 1, 2)))[0][0] 
            world_x, world_y = int(-w[0]), int(w[1])
            step = True 
                      
    else:
        world_x, world_y = -1, -1
        step = False
            
    return img

# 主要图像处理函数
def run(img):
    img = ColorDetect(img, color='green') # 进行颜色识别，目标颜色:green
    return img

if __name__ == '__main__':
    import HiwonderSDK.Sonar as Sonar
    from CameraCalibration.CalibrationConfig import *
    
    #加载参数
    param_data = np.load(calibration_param_path + '.npz')

    #获取参数
    mtx = param_data['mtx_array']
    dist = param_data['dist_array']
    newcameramtx, _ = cv2.getOptimalNewCameraMatrix(mtx, dist, (640, 480), 0, (640, 480))
    mapx, mapy = cv2.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (640, 480), 5)
    
    HWSONAR = Sonar.Sonar()
    initMove()
    load_config()
    my_camera = Camera.Camera()
    my_camera.camera_open()
    while True:
        img = my_camera.frame
        if img is not None:
            frame = img.copy()
            frame = cv2.remap(frame, mapx, mapy, cv2.INTER_LINEAR)  # 畸变矫正
            Frame = run(frame)
            cv2.imshow('Frame', Frame)
            key = cv2.waitKey(1)
            if key == 27:
                break
        else:
            time.sleep(0.01)
    my_camera.camera_close()
    cv2.destroyAllWindows()
    
    
