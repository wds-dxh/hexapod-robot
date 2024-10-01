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
import time


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

# 设置红色和蓝色的阈值
# lower_red = np.array([0, 50, 100])
# upper_red = np.array([10, 255, 255])
# lower_blue = np.array([100, 50, 100])
# upper_blue = np.array([124, 255, 255])
#访问array





lower_red = np.array([0, 47, 125])
upper_red = np.array([76, 255, 255])
lower_blue = np.array([59, 110, 93])    
upper_blue = np.array([126, 255, 97])

# 添加六个滑块设置 HSV 阈值
cv2.namedWindow("HSV 阈值", cv2.WINDOW_NORMAL)
cv2.createTrackbar("H 下限", "HSV 阈值", 0, 255, lambda x: None)
cv2.createTrackbar("H 上限", "HSV 阈值", 255, 255, lambda x: None)
cv2.createTrackbar("S 下限", "HSV 阈值", 0, 255, lambda x: None)
cv2.createTrackbar("S 上限", "HSV 阈值", 255, 255, lambda x: None)
cv2.createTrackbar("V 下限", "HSV 阈值", 0, 255, lambda x: None)
cv2.createTrackbar("V 上限", "HSV 阈值", 255, 255, lambda x: None)


import HiwonderSDK.Sonar as Sonar   
from CameraCalibration.CalibrationConfig import *
init()
start()
AGC.runAction('init_actions')

# 读取摄像头
cap = cv2.VideoCapture(0)
# frame = cv2.imread("image.jpg")
while True:
    ret, frame = cap.read()
    if frame is None:
        break
    time.sleep(0.1)
      #加载参数
    param_data = np.load(calibration_param_path + '.npz')
    #获取参数
    mtx = param_data['mtx_array']
    dist = param_data['dist_array']
    newcameramtx, _ = cv2.getOptimalNewCameraMatrix(mtx, dist, (640, 480), 0, (640, 480))#自由比例参数
    mapx, mapy = cv2.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (640, 480), 5)  # 获取映射方程


    # 获取 HSV 阈值
    h_lower = cv2.getTrackbarPos("H 下限", "HSV 阈值")
    h_upper = cv2.getTrackbarPos("H 上限", "HSV 阈值")
    s_lower = cv2.getTrackbarPos("S 下限", "HSV 阈值")
    s_upper = cv2.getTrackbarPos("S 上限", "HSV 阈值")
    v_lower = cv2.getTrackbarPos("V 下限", "HSV 阈值")
    v_upper = cv2.getTrackbarPos("V 上限", "HSV 阈值")
    img = frame.copy()
    # 预处理帧
    frame1, contours = preprocess_frame(img, (h_lower, s_lower, v_lower), (h_upper, s_upper, v_upper))

    # # 预处理红色
    # res_red, contours_red = preprocess_frame(frame, lower_red, upper_red)
    # draw_bounding_boxes(frame, contours_red, (0, 0, 255))
    #
    # # 预处理蓝色
    # res_blue, contours_blue = preprocess_frame(frame, lower_blue, upper_blue)
    # draw_bounding_boxes(frame, contours_blue, (255, 0, 0))

    # 展示原图
    cv2.imshow('video', frame1)

    if cv2.waitKey(1) == 27:
        break


cv2.destroyAllWindows()
