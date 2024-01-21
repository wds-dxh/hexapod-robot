#!/usr/bin/python3
#coding=utf8
import os
import sys
import cv2
import import_path
import time
import Camera
import threading
import numpy as np
import pandas as pd
import ArmIK.ArmMoveIK as AMK
import kinematics as kinematics
import HiwonderSDK.Board as Board
import HiwonderSDK.ActionGroupControl as AGC


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

# def move():
#     while True:
#         if __isRunning:
#             if 0 < distance < Threshold:
#                 while distance < 25: # 小于25cm时后退
#                     ik.back(ik.initial_pos, 2, 80, 50, 1)
#                 for i in range(6): # 左转6次，每次15度，一共90度
#                     if __isRunning:
#                         ik.turn_left(ik.initial_pos, 2, 15, 50, 1)
#             else: 
#                 ik.go_forward(ik.initial_pos, 2, 80, 50, 1)
#         else:
#             time.sleep(0.01)

# threading.Thread(target=move, daemon=True).start()

distance_data = []

def run(img):
    global __isRunning
    global distance
    global distance_data

    if __isRunning:
        
        # 数据处理，过滤异常值
        distance_ = HWSONAR.getDistance() / 10.0
        distance_data.append(distance_)
        data = pd.DataFrame(distance_data)
        data_ = data.copy()
        u = data_.mean()  # 计算均值
        std = data_.std()  # 计算标准差

        data_c = data[np.abs(data - u) <= std]
        distance = data_c.mean()[0]# 计算均值
        if len(distance_data) == 5:
            distance_data.remove(distance_data[0])

        cv2.putText(img, "Dist:%.1fcm" % distance, (30, 480 - 30), cv2.FONT_HERSHEY_SIMPLEX, 1.2, TextColor, 2)
        return img

#让程序一直读取超声波数据通过全局变量distance传递给run函数        
# threading.Thread(target=run, daemon=True).start()#daemon=True表示该线程会随着主线程的退出而退出


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

    HWSONAR = Sonar.Sonar()# 超声波初始化
    init()
    start()
    my_camera = Camera.Camera()
    my_camera.camera_open()
    AGC.runAction('init_actions')

    # 设置视频编解码器和保存路径
    fourcc = cv2.VideoWriter_fourcc(*'MJPG')
    out = cv2.VideoWriter('output.avi', fourcc, 20.0, (640, 480))  # 参数分别为保存路径、编解码器、帧率、分辨率
    # out = cv2.VideoWriter('outpy.avi', cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'), 10, (320, 240))  # 保存视频

    while True:
        img = my_camera.frame
        if img is not None:
            frame = img.copy()
            frame = cv2.remap(frame, mapx, mapy, cv2.INTER_LINEAR)  # 畸变矫正
            # frame = run(frame)
            # print("distance:", distance)
            cv2.imshow('Frame', frame)
            if cv2.waitKey(1) == ord('s'):
                # out.write(frame)
                print('save image')
            key = cv2.waitKey(1)
            if key == 27:
                break
        else:
            time.sleep(0.01)
    my_camera.camera_close()
    cv2.destroyAllWindows()
    out.release()