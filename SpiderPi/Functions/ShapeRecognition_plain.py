#!/usr/bin/python3
# coding=utf8
import sys
sys.path.append('/home/pi/SpiderPi/')
import cv2
import math
import time
import signal
import Camera
import threading
import kinematics
import yaml_handle
import numpy as np
import HiwonderSDK.tm1640 as tm
import HiwonderSDK.Board as Board
import ArmIK.ArmMoveIK as AMK


if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)
 
# 红色形状识别

# 点阵接口：扩展板io7、io8
 

# 逆运动学定义
ik = kinematics.IK()
AK = AMK.ArmIK()

lab_data = None
move_st = True

# 读取颜色阈值函数
def load_config():
    global lab_data
    
    lab_data = yaml_handle.get_yaml_data(yaml_handle.lab_file_path)
    
# 初始位置
def initMove():
    HWSONAR.setRGBMode(0)
    HWSONAR.setRGB(1, (0, 0, 0))
    HWSONAR.setRGB(2, (0, 0, 0))
    ik.stand(ik.initial_pos)
    AK.setPitchRangeMoving((0, 12, 18), -60, -90, 100, 2000)


# 找出面积最大的轮廓
# 参数为要比较的轮廓的列表
def getAreaMaxContour(contours):
    contour_area_temp = 0
    contour_area_max = 0
    area_max_contour = None

    for c in contours:  # 历遍所有轮廓
        contour_area_temp = math.fabs(cv2.contourArea(c))  # 计算轮廓面积
        if contour_area_temp > contour_area_max:
            contour_area_max = contour_area_temp
            if contour_area_temp > 50:  # 只有在面积大于50时，最大面积的轮廓才是有效的，以过滤干扰
                area_max_contour = c

    return area_max_contour, contour_area_max  # 返回最大的轮廓

shape_length = 0

def setBuzzer(s):
    Board.setBuzzer(1)  # 打开
    time.sleep(s)  # 延时
    Board.setBuzzer(0)  #关闭

# 主要控制函数
def move():
    global shape_length
    
    while move_st:
        if shape_length == 3:
            print('三角形')
            setBuzzer(0.1)
            time.sleep(0.1)
            
        elif shape_length == 4:
            print('矩形')
            setBuzzer(0.1)
            time.sleep(0.1)
            setBuzzer(0.1)
            time.sleep(0.1)
            
        elif shape_length >= 6:
            print('圆')
            setBuzzer(0.1)
            time.sleep(0.1)
            setBuzzer(0.1)
            time.sleep(0.1)
            setBuzzer(0.1)
            time.sleep(0.1)
        else:
            print('None')
            setBuzzer(0)
            
       
        
# 运行子线程
th = threading.Thread(target=move)
th.setDaemon(True)
th.start()

shape_list = []

# 退出关闭函数
def Stop(signum, frame):
    global move_st
    move_st = False
    tm.display_buf = [0] * 16
    tm.update_display()
    print('关闭中...')
    AGC.runActionGroup('lift_down')

signal.signal(signal.SIGINT, Stop)

# 主要图像处理函数
def run(img):
    global shape_length, shape_list
    
    img_h, img_w = img.shape[:2]
    frame_gb = cv2.GaussianBlur(img, (3, 3), 3)      
    frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)  # 将图像转换到LAB空间
    max_area = 0
    color_area_max = None    
    areaMaxContour_max = 0
    color = 'blue'
    frame_mask = cv2.inRange(frame_lab,
                     (lab_data[color]['min'][0],
                      lab_data[color]['min'][1],
                      lab_data[color]['min'][2]),
                     (lab_data[color]['max'][0],
                      lab_data[color]['max'][1],
                      lab_data[color]['max'][2]))  #对原图像和掩模进行位运算
    opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((6,6),np.uint8))  #开运算
    closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((6,6),np.uint8)) #闭运算
    contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  #找出轮廓
    areaMaxContour, area_max = getAreaMaxContour(contours)  #找出最大轮廓
    
    if area_max > 200:                   
        cv2.drawContours(img, areaMaxContour, -1, (0, 0, 255), 2)
        # 识别形状
        # 周长  0.035 根据识别情况修改，识别越好，越小
        epsilon = 0.035 * cv2.arcLength(areaMaxContour, True)
        # 轮廓相似
        approx = cv2.approxPolyDP(areaMaxContour, epsilon, True)
        shape_list.append(len(approx))
        if len(shape_list) == 24:
            shape_length = int(round(np.mean(shape_list)))                            
            shape_list = []
            print(shape_length)
    else:
        shape_length = 0
            
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
    
    load_config()
    initMove()
    my_camera = Camera.Camera()
    my_camera.camera_open()
    while move_st:
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

