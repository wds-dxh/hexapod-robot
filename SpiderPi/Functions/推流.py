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

from flask import Flask, render_template, Response
import cv2

app = Flask(__name__)



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


import HiwonderSDK.Sonar as Sonar   
from CameraCalibration.CalibrationConfig import *
# #加载参数
# param_data = np.load(calibration_param_path + '.npz')
# #获取参数
# mtx = param_data['mtx_array']
# dist = param_data['dist_array']
# newcameramtx, _ = cv2.getOptimalNewCameraMatrix(mtx, dist, (640, 480), 0, (640, 480))#自由比例参数
# mapx, mapy = cv2.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (640, 480), 5)  # 获取映射方程
init()
start()
AGC.runAction('init_actions')




def generate_frames():
    #加载参数
    param_data = np.load(calibration_param_path + '.npz')
    #获取参数
    mtx = param_data['mtx_array']
    dist = param_data['dist_array']
    newcameramtx, _ = cv2.getOptimalNewCameraMatrix(mtx, dist, (640, 480), 0, (640, 480))#自由比例参数
    mapx, mapy = cv2.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (640, 480), 5)  # 获取映射方程
    # my_camera = Camera.Camera()
    # my_camera.camera_open()
    cap = cv2.VideoCapture(0)
    while True:
        success, img = cap.read()
        if img is not None:
            frame = img.copy()
            frame = cv2.remap(frame, mapx, mapy, cv2.INTER_LINEAR)  # 畸变矫正
            # 将帧编码为JPEG格式
            frame = cv2.resize(frame, (640, 480))
            ret, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
            frame = buffer.tobytes()

        # 使用生成器函数输出帧
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/video_feed')
def video_feed():
    # 使用Response对象包装生成器函数，以便将其作为视频流输出
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    app.run(debug=False, host='0.0.0.0' ,port=5000)#host为0.0.0.0代表可以通过ip访问，port为端口号
