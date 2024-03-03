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


from flask import Flask, render_template, Response
import cv2
ik = kinematics.IK()
import HiwonderSDK.Sonar as Sonar   
from CameraCalibration.CalibrationConfig import *
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
            cv2.imshow('frame', frame)
            cv2.waitKey(1)
            if cv2.waitKey(1) == ord('q'):
                break


if __name__ == '__main__':
    ik.go_forward(ik.initial_pos, 2, 10, 100, 1)
    generate_frames()

