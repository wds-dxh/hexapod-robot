#!/usr/bin/python3
#coding=utf8
import json
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

xywh = None#全局变量,传递坐标数据
cls = None#全局变量,传递类别数据
def recv_data_all():
    global xywh
    global cls
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
        received_data = client_socket.recv(1024).decode()
        parsed_data = json.loads(received_data)
        if parsed_data != None:
            xywh = parsed_data["xywh"]
            cls = parsed_data["cls"]
    client_socket.close()  # 关闭连接


#detect_color
#0:blue
#1:red
#2:cup
def get_color(detect_color):    #def get_color(detect_color, xywh, cls):
    global xywh
    global cls
    min_x_value = 640
    if xywh != None and cls != None:
        for i in range(len(cls)):
            # print(cls[i])
            if cls[i] == detect_color:
                if xywh[i][0] < min_x_value:
                    need_xywh = xywh[i]# need_xywh = xywh[i][0]
                    # print(need_xywh)
                    return need_xywh

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
    detect_color = 0
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
    action = action.action(speed=100,move_distance=20,
                           PID_Px=0.6,PID_Ix=0.1,PID_Dx=0.1,
                           PID_Py=0.4,PID_Iy=0.05,PID_Dy=0.15)
    action.Grab_correction_location(ik, distance = 0)
    threading.Thread(target=recv_data_all, daemon=True).start()#daemon=True表示该线程会随着主线程的退出而退出
    # ik.left_move(ik.initial_pos, 2, 0, 100, 1)  # 左移10mm
    while True:
        if xywh != None and cls != None:
            break
        print('等待数据')
        time.sleep(0.5)

    while True:
        if get_color(detect_color) != None: #and get_color(detect_color)[0] !=None and get_color(1)[1] != None
            flag = action.alignment_PID(get_color(detect_color)[0],get_color(detect_color)[1],ik,width= 320, high = 395,range=5)
            if flag == False:
                print('没对准')
            else:
                print("对准了")
                break
    action.grap()   
    print('抓取完成')
    
    

