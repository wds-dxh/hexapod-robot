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
import pandas as pd#用于数据处理
import kinematics as kinematics
import HiwonderSDK.Board as Board
import HiwonderSDK.ActionGroupControl as AGC#动作组控制
import socket

#自己写的函数
import Functions.action as action


if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)


ik = kinematics.IK()
HWSONAR = None
TextColor = (0, 255, 255)
TextSize = 12
__isRunning = False
distance = 0
def init():
    AGC.runAction('init_actions')#初始化视角，可以正常看到识别的色块
    print('Avoidance Init')


def start():
    global __isRunning
    __isRunning = True
    print('Avoidance Start')


def stop():
    global __isRunning
    __isRunning = False
    ik.stand(ik.initial_pos)#机器人站立
    print('Avoidance Stop')


data = None#全局变量,传递坐标数据
def get_xywh():
    global data
    host = "192.168.1.36"
    print(host)
    port = 6524
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((host, port))
    server_socket.listen(1)  # 参数表示最大等待连接数,单位是个
    print(f"等待连接在 {host}:{port}...")
    client_socket, addr = server_socket.accept()
    print(f"连接来自 {addr}")
    while True:
        data = client_socket.recv(1024)  # 接收数据
        # print(f"接收到的数据: {data.decode()}")  # 输出接收到的数据
        # print(eval(data)[0])
        # print(eval(data)[1])
        # if not data:
        #     print("未收到数据")
    client_socket.close()  # 关闭连接
#启用socket通信线程
# threading.Thread(target=get_xywh, daemon=True).start()#daemon=True表示该线程会随着主线程的退出而退出


distance_data = []#全局变量,传递超声波数据
def get_distance():
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
    init()
    start()
    action = action.action(speed=100,move_distance=20,PID_P=1)
    action.Grab_correction_location(ik, distance = 0)
    threading.Thread(target=get_xywh, daemon=True).start()#daemon=True表示该线程会随着主线程的退出而退出
    cv2.waitKey(0)
    while True:
        if data != None:
            break
        print('等待数据')
        time.sleep(0.5)

        
    while True:
        flag = action.alignment_PID(eval(data)[0],eval(data)[1],ik,high = 420)
        if flag == True:
            print('对准了')
            break 
        if flag == False:
            print('没对准')
    action.grap()   
    print('抓取完成')
    
    

