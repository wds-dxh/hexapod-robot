#!/usr/bin/python3
#coding=utf8
import os
import sys
import import_path
import time
import numpy as np
import ArmIK.ArmMoveIK as AMK
import kinematics as kinematics
import HiwonderSDK.Board as Board
import HiwonderSDK.ActionGroupControl as AGC
import cv2



if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

ik = kinematics.IK()
AK = AMK.ArmIK()
HWSONAR = None
TextColor = (0, 255, 255)
TextSize = 12

__isRunning = False
distance = 0

def reset():
    AK.setPitchRangeMoving((0, 15, 30), 0, -90, 100, 1000) # 机械臂复位  

def init():
    reset()
    print('Avoidance Init')

def exit():
    global __isRunning
    
    HWSONAR.setRGBMode(0)
    HWSONAR.setRGB(1, (0, 0, 0))
    HWSONAR.setRGB(2, (0, 0, 0))
    __isRunning = False
    print('Avoidance Exit')


def start():
    global __isRunning
    __isRunning = True
    print('Avoidance Start')

def stop():
    global __isRunning
    __isRunning = False
    ik.stand(ik.initial_pos)
    print('Avoidance Stop')


def preprocess_frame(frame, lower_threshold, upper_threshold):
    #图像预处理
    #高斯模糊
    frame = cv2.GaussianBlur(frame, (5, 5), 0)
    #先腐蚀后膨胀
    kernel = np.ones((5, 5), np.uint8)
    frame = cv2.erode(frame, kernel, iterations=1)#iterations:意思是迭代次数，迭代次数越多，模糊程度(腐蚀程度)越大
    frame = cv2.dilate(frame, kernel, iterations=1)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_threshold, upper_threshold)
    res = cv2.bitwise_and(frame, frame, mask=mask)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return res, contours

def draw_bounding_boxes(frame, contours, color):
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if 200 < area < 1500:  # 设置颜色块的面积范围
            x, y, w, h = cv2.boundingRect(cnt)
            cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)




import HiwonderSDK.Sonar as Sonar   
from CameraCalibration.CalibrationConfig import *
init()
start()
AGC.runAction('init_actions')



# 设置红色和蓝色的阈值
# lower_red = np.array([0, 50, 100])
# upper_red = np.array([10, 255, 255])
# lower_blue = np.array([100, 50, 100])
# upper_blue = np.array([124, 255, 255])
#访问array

#斜方向看过
# lower_red = np.array([0, 255, 195])
# upper_red = np.array([15, 255, 255])
# lower_blue = np.array([26, 125, 71])
# upper_blue = np.array([170, 255, 255])

# lower_red = np.array([1, 160, 100])
# upper_red = np.array([255, 255, 255])
# lower_blue = np.array([0, 0, 0])
# upper_blue = np.array([255, 140, 110])

lower_red = np.array([78, 146, 136])
upper_red = np.array([211, 218, 255])
lower_blue = np.array([0, 0, 0])
upper_blue = np.array([255, 165, 97])




if __name__ == '__main__':
    cap = cv2.VideoCapture(0)
    #加载参数
    param_data = np.load(calibration_param_path + '.npz')
    #获取参数
    mtx = param_data['mtx_array']
    dist = param_data['dist_array']
    newcameramtx, _ = cv2.getOptimalNewCameraMatrix(mtx, dist, (640, 480), 0, (640, 480))#自由比例参数
    mapx, mapy = cv2.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (640, 480), 5)  # 获取映射方程
    while True:
        ret, frame = cap.read()
        if frame is None:
            break
        # time.sleep(0.1)
        else:
            frame = cv2.remap(frame, mapx, mapy, cv2.INTER_LINEAR)  # 畸变矫正
            # 预处理红色
            res_red, contours_red = preprocess_frame(frame, lower_red, upper_red)
            draw_bounding_boxes(frame, contours_red, (0, 0, 255))

            # 预处理蓝色
            res_blue, contours_blue = preprocess_frame(frame, lower_blue, upper_blue)
            draw_bounding_boxes(frame, contours_blue, (255, 0, 0))

            # 展示原图
            cv2.imshow('color', frame)
        if cv2.waitKey(1) == 27:
            break


    cv2.destroyAllWindows()
