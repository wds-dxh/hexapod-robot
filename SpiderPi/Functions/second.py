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
import pandas as pd
import kinematics as kinematics
import HiwonderSDK.Board as Board
import HiwonderSDK.ActionGroupControl as AGC
import socket
import Metal_detection

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
    AGC.runAction('init_actions')
    print('Avoidance Init')

def start():
    global __isRunning
    __isRunning = True
    print('Avoidance Start')

def stop():
    global __isRunning
    __isRunning = False
    ik.stand(ik.initial_pos)
    print('Avoidance Stop')

def get_local_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.connect(("8.8.8.8", 80))  # 连接到一个外部IP地址
    local_ip_address = s.getsockname()[0]
    s.close()
    return local_ip_address



data = None
def get_xywh():
    global data
    host = socket.gethostbyname(socket.getfqdn())
    # host = "172.20.10.3"
    print(host)
    port = 1024
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((host, port))
    server_socket.listen(1)
    print(f"Waiting for connection at {host}:{port}...")
    client_socket, addr = server_socket.accept()
    print(f"Connection from {addr}")
    while True:
        data = client_socket.recv(1024)
    client_socket.close()

xywh = None
cls = None
def recv_data_all():
    global xywh
    global cls
    host = get_local_ip()
    print(host)
    port = 1024
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((host, port))
    server_socket.listen(1)
    print(f"Waiting for connection at {host}:{port}...")
    client_socket, addr = server_socket.accept()
    print(f"Connection from {addr}")
    while True:
        received_data = client_socket.recv(1024).decode()
        try:
            parsed_data = json.loads(received_data)
        except json.decoder.JSONDecodeError as e:
            print(f"JSON parsing error: {e}")
            print(f"Received data: {received_data}")

        if parsed_data != None:
            xywh = parsed_data["xywh"]
            cls = parsed_data["cls"]
    client_socket.close()

def get_color(detect_color):
    global xywh
    global cls
    need_xywh = None
    min_x_value = 640
    if xywh != None and cls != None:
        for i in range(len(cls)):
            if cls[i] == detect_color:
                if xywh[i][0] < min_x_value:
                    min_x_value = xywh[i][0]
                    need_xywh = xywh[i]
        if need_xywh != None:
            return need_xywh

distance_data = []
def get_distance():
    global __isRunning
    global distance
    global distance_data
    if __isRunning:
        distance_ = HWSONAR.getDistance() / 10.0
        distance_data.append(distance_)
        data = pd.DataFrame(distance_data)
        data_ = data.copy()
        u = data_.mean()
        std = data_.std()
        data_c = data[np.abs(data - u) <= std]
        distance = data_c.mean()[0]
        if len(distance_data) == 5:
            distance_data.remove(distance_data[0])

if __name__ == '__main__':
    detect_color = 7
    import HiwonderSDK.Sonar as Sonar
    from CameraCalibration.CalibrationConfig import *

    param_data = np.load(calibration_param_path + '.npz')
    mtx = param_data['mtx_array']
    dist = param_data['dist_array']
    newcameramtx, _ = cv2.getOptimalNewCameraMatrix(mtx, dist, (640, 480), 0, (640, 480))
    mapx, mapy = cv2.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (640, 480), 5)
    init()
    start()
    metal_detection = Metal_detection.Metal_detection()

    action = action.action(speed=100, move_distance=20, PID_Px=0.4, PID_Ix=0.1, PID_Dx=0.1, PID_Py=0.4, PID_Iy=0.05, PID_Dy=0.15)
    threading.Thread(target=recv_data_all, daemon=True).start()

    while True:
        if xywh != None and cls != None:
            break
        print('Waiting for data')
        time.sleep(0.5)

    while True:
        if get_color(detect_color) != None:
            flag = action.alignment_PID_color(get_color(detect_color)[0], get_color(detect_color)[1], ik, width=320, high=395, range=5,
                                              PID_Px=0.4, PID_Ix=0.1, PID_Dx=0.1, PID_Py=0.2, PID_Iy=0.1, PID_Dy=0.4)
            if flag == False:
                print('Not aligned')
            else:
                print("Aligned")
                break
    action.grap()
    print('Grabbing completed')

    metal_detection.IS_OR_NOT_Metal()
    if metal_detection.IS_OR_NOT_Metal() == False:
        ik.back(ik.initial_pos, 2, 80, 100, 1)
        action.discard()
        while True:
            if get_color(detect_color) != None:
                flag = action.alignment_PID_color(get_color(detect_color)[0], get_color(detect_color)[1], ik, width=320, high=380, range=5,
                                                  PID_Px=0.4, PID_Ix=0.1, PID_Dx=0.1, PID_Py=0.2, PID_Iy=0.1, PID_Dy=0.4)
                if flag == False:
                    print('Not aligned')
                else:
                    print("Aligned")
                    action.grap()
                    time.sleep(1)
                    print('Grabbing completed')
                    break
    action.Step_back_and_place_the_cup(ik, distance_back=2, distance_right=3)
    while True:
        if get_color(3) != None:
            flag = action.alignment_PID_CUP(get_color(3)[0], get_color(3)[1], ik, width=320, high=212, range=10,
                                            pid_px=0.3, pid_ix=0.05, pid_dx=0.3, pid_py=0.3, pid_iy=0.05, pid_dy=0.4)
            if flag == False:
                print('Not aligned')
            else:
                print("Aligned")
                break
    action.place()
    print('Placement completed')
