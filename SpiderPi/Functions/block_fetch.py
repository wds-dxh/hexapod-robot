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

# 色块定点夹取

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)
    
# 逆运动学定义  
ik = kinematics.IK()
AK = AMK.ArmIK()
HWSONAR = None


range_rgb = {
    'red': (0, 0, 255),
    'blue': (255, 0, 0),
    'green': (0, 255, 0),
    'black': (0, 0, 0),
    'white': (255, 255, 255)}

# 变量定义
start = False
face_detect = False
size = (640, 480)
initial_coord = (0, 15, 5)
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

# 初始位置
def initMove():
    HWSONAR.setRGBMode(0)
    HWSONAR.setRGB(1, (0, 0, 0))
    HWSONAR.setRGB(2, (0, 0, 0))
    Board.setBusServoPulse(25, 120, 1000)
    ik.stand(ik.initial_pos)
    AK.setPitchRangeMoving(initial_coord, -90, -90, 100, 2000)

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


def setBuzzer(s):
    Board.setBuzzer(1)  # 打开
    time.sleep(s)  # 延时
    Board.setBuzzer(0)  #关闭

# 机器人移动函数
def move():
    global world_x, world_y, start
    
    pulse21 = 500
    dn = 1
    while True:
        if start:
            time.sleep(0.5)
            Board.setBusServoPulse(25, 120, 500) # 张开爪子
            x,y = initial_coord[0]+world_x, initial_coord[1]+world_y # 转换成机械臂坐标
            AK.setPitchRangeMoving((x, y, 5), -90, -90, 100, 1000) # 移动到目标位置
            time.sleep(5)
            Board.setBusServoPulse(25, 350, 500) # 夹取目标
            time.sleep(0.5)
            AK.setPitchRangeMoving((12, 24, 5), -90, -90, 100, 1500) # 抬起来
            time.sleep(1.5)
            AK.setPitchRangeMoving((12, 24, -5), -90, -90, 100, 1000) # 移动到放置位置
            time.sleep(1)
            Board.setBusServoPulse(25, 120, 500) # 张开爪子
            time.sleep(0.5)
            AK.setPitchRangeMoving((12, 24, 5), -90, -90, 100, 1000) # 抬起来
            time.sleep(1)
            AK.setPitchRangeMoving(initial_coord, -90, -90, 100, 2000) # 回到检测色块姿态
            time.sleep(2)
            start = False
            world_x, world_y = 666, 666
            
        else:
            setBuzzer(0.1)
            time.sleep(1)
        

# 运行子线程
th = threading.Thread(target=move)
th.setDaemon(True)
th.start()


num = 0
old_x, old_y = 0, 0
# 颜色识别函数
def ColorDetect(img, color='red'):
    global num, start
    global old_x, old_y 
    global world_x, world_y
    
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
        cv2.putText(img, "Color: " + color, (10, img.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.65, range_rgb[color], 2)
        
        if abs(centerX-old_x) < 8 and abs(centerY-old_y) < 8: # 判断目标坐标有没有变化
            num += 1
        else:
            num = 0
            old_x, old_y = centerX, centerY
            
        if num > 10: # 多次判断，确定目标位置稳定
            # 转换成现实距离
            center = np.array([centerX,centerY])
            w = camera_to_world(K, R, T, center.reshape((1, 1, 2)))[0][0] 
            world_x, world_y = int(-w[0])/10, int(-w[1])/10
            print(world_x, world_y)
            start = True
            num = 0
            
        if abs(world_x) > 8 and world_y > 8:
            cv2.putText(img, "Position out of range!!!", (180, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,0,255), 2)
            start = False
                                
    else:
        num = 0
        start = False
        world_x, world_y = 666, 666
            
    return img

# 主要图像处理函数
def run(img):
    img = ColorDetect(img)
        
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
    
    