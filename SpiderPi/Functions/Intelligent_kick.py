#!/usr/bin/python3
# coding=utf8
import sys
sys.path.append('/home/pi/SpiderPi/')
import cv2
import math
import time
import Camera
import threading
import numpy as np
import yaml_handle
import import_path
import ArmIK.ArmMoveIK as AMK
import kinematics as kinematics
from HiwonderSDK.PID import PID
import HiwonderSDK.Misc as Misc
import HiwonderSDK.Board as Board

ik = kinematics.IK()
AK = AMK.ArmIK()
debug = False
HWSONAR = None

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

__target_color = ('green',)

x_pid = PID(P=0.2, I=0.01, D=0.008)#pid初始化
y_pid = PID(P=0.2, I=0.02, D=0.008)
X_pid = PID(P=0.2, I=0.01, D=0.008)
Y_pid = PID(P=0.8, I=0.01, D=0.008)

range_rgb = {
    'red': (0, 0, 255),
    'blue': (255, 0, 0),
    'green': (0, 255, 0),
    'violet':(255,0,255),
    'black': (0, 0, 0),
    'white': (255, 255, 255),
}

lab_data = None
def load_config():
    global lab_data
    
    lab_data = yaml_handle.get_yaml_data(yaml_handle.lab_file_path)

# 初始姿态参数定义
# 姿态数据为3x6的数组，表示6条腿的的末端的x，y，z坐标，单位mm
# 头部朝前的方向为x轴， 头部朝前位置为负方向，右边为y轴正， 竖直朝上为z轴正， 从中间两条腿的连线的中点做垂线与上下板相交，取连线中心为零点
# 第一条腿表示头部朝前时左上角所在腿, 逆时针表示1-6
current_pos = [[-199.53, -177.73, -100.0],
               [ -35.0, -211.27, -100.0],
               [199.53, -177.73, -100.0],
               [199.53,  177.73, -100.0],
               [ -35.0,  211.27, -100.0],
               [-199.53, 177.73, -100.0]]

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
            if contour_area_temp > 50:  # 只有在面积大于设定时，最大面积的轮廓才是有效的，以过滤干扰
                area_max_contour = c
                max_area = contour_area_temp

    return area_max_contour, max_area  # 返回最大的轮廓

x_dis = 500
y_dis = 170
# 初始位置
def initMove():
    HWSONAR.setRGBMode(0)
    HWSONAR.setRGB(1, (0, 0, 0))
    HWSONAR.setRGB(2, (0, 0, 0))
    ik.stand(ik.initial_pos, t=500)
    AK.setPitchRangeMoving((0, 10, 16), -45, -90, 100, 1000) 
    Board.setBusServoPulse(24, y_dis, 500)
    Board.setBusServoPulse(21, x_dis, 500)   

state = False
# 变量重置
def reset():
    global x_dis, y_dis, state
       
    x_dis = 500
    y_dis = 170
    x_pid.clear()
    y_pid.clear()
    X_pid.clear()
    Y_pid.clear()
    state = False
    HWSONAR.setRGBMode(0)
    HWSONAR.setRGB(1, (0, 0, 0))
    HWSONAR.setRGB(2, (0, 0, 0))

# 初始化调用
def init():
    initMove()
    load_config()
    reset()

def hisEqulColor(img):
    ycrcb = cv2.cvtColor(img, cv2.COLOR_BGR2YCR_CB)
    channels = cv2.split(ycrcb)
    cv2.equalizeHist(channels[0], channels[0])
    cv2.merge(channels, ycrcb)
    img_eq = cv2.cvtColor(ycrcb, cv2.COLOR_YCR_CB2BGR)
    return img_eq


search = True
p = [0, 0, 0]
o = [0, 0, 0]
# 机器人移动函数
def move():
    global x_dis, y_dis, state
    
    d_x,d_y = 5,100
    direction = 0
    peak = False
    initial = True
    move_st = False
    dx,dy,dz = 0,0,0
    ago_direction = 'right'
    
    while True:
        if state:
            X_pid.SetPoint = 500    #设定
            X_pid.update(x_dis)     #当前
            dx = int(X_pid.output)
            
            Y_pid.SetPoint = 170    #设定
            Y_pid.update(y_dis)     #当前
            dy = int(Y_pid.output)
            dz = abs(dy)
            
            dx = 10 if dx > 10 else dx
            dx = -10 if dx < -10 else dx
            dy = 100 if dy > 100 else dy
            dy = -100 if dy < -100 else dy
            dz = 50 if dz > 50 else dz
            dz = 20 if dz < 20 else dz
            
            if abs(dx) > 5 or dy > 20 or dy < -10:
                if dy < -10:
                    direction = 0
                elif dy > 20 and abs(dx) < 15:
                    direction = 180
                    dx = -dx
                    
                if abs(dy) > 50:
                    amplitude = int(abs(dy)-abs(dx)*2)
                    
                elif abs(dy) < 20:
                    amplitude = int(abs(dy) + abs(dx)*1.5)
                    
                else:
                    amplitude = abs(dy)
                if dx > 5:
                    ago_direction = 'right'
                elif dx < -5:
                    ago_direction = 'left'
                ik.setStepMode(ik.initial_pos, 2, 2, amplitude, dz, direction, 0, dx/10, p, o, 50, 1)
                move_st = True
            
            else:
                ik.setStepMode(ik.initial_pos, 2, 2, 40, 20, 0, 0, 0, p, o, 50, 1)  # 朝前直走40mm
                ik.stand(ik.initial_pos, t=500)
                if x_dis <= 510:
                    ik.stand(current_pos)
                    time.sleep(0.5)
                    # 抬起右前脚
                    current_pos[5] = [-199.53, 177.73, -60]
                    ik.stand(current_pos) 
                    time.sleep(0.2)
                    # 右脚踢球
                    current_pos[5] = [-340, -30, -80]
                    ik.stand(current_pos,200) 
                    time.sleep(0.5)
                    # 回到初始姿态
                    current_pos[5] = [-199.53, 177.73, -100]
                    ik.stand(current_pos)
                    ik.stand(ik.initial_pos, t=500)
                else:
                    ik.stand(current_pos)
                    time.sleep(0.5)
                    # 抬起左前脚
                    current_pos[0] = [-199.53, 177.73, -60]
                    ik.stand(current_pos) 
                    time.sleep(0.2)
                    # 左脚踢球
                    current_pos[0] = [-340, 30, -80]
                    ik.stand(current_pos,200) 
                    time.sleep(0.5)
                    # 回到初始姿态
                    current_pos[0] = [-199.53, -177.73, -100]
                    ik.stand(current_pos)
                    ik.stand(ik.initial_pos, t=500)
        else:
            if move_st:
                move_st = False
                ik.stand(ik.initial_pos, t=500)
            if x_dis >= 800:
                d_x = -5
                y_dis += d_y
                y_dis = 260 if y_dis > 260 else y_dis
                y_dis = 160 if y_dis < 160 else y_dis
                Board.setBusServoPulse(24, y_dis, 200)
                if initial:
                    initial = False
                else:
                    peak = True
            elif x_dis <= 200:
                d_x = 5
                y_dis += d_y
                y_dis = 260 if y_dis > 260 else y_dis
                y_dis = 160 if y_dis < 160 else y_dis
                Board.setBusServoPulse(24, y_dis, 200)
                peak = True
                
            if abs(x_dis - 500) <= 10 and peak:     
                if y_dis >= 260:
                    d_y = -100
                    if ago_direction == 'right':
                        ik.turn_right(ik.initial_pos, 2, 30, 120, 3)
                    elif ago_direction == 'left':
                        ik.turn_left(ik.initial_pos, 2, 30, 120, 3)
                    ik.stand(ik.initial_pos, t = 300)
                    peak = False
                elif y_dis <= 160:
                    d_y = 100
                
            x_dis += d_x
            Board.setBusServoPulse(21, x_dis, 50)
            time.sleep(0.05)


# 运行子线程
th = threading.Thread(target=move)
th.setDaemon(True)
th.start()


size = (320, 240)
def run(img):
    global x_dis, y_dis, state, search
    
    img_copy = img.copy()
    img_h, img_w = img.shape[:2]
    
    if __target_color == ():
        return img

    cv2.line(img, (int(img_w/2 - 10), int(img_h/2)), (int(img_w/2 + 10), int(img_h/2)), (0, 255, 255), 2)
    cv2.line(img, (int(img_w/2), int(img_h/2 - 10)), (int(img_w/2), int(img_h/2 + 10)), (0, 255, 255), 2)
    img_hisEqul = hisEqulColor(img_copy)
    frame_resize = cv2.resize(img_hisEqul, size, interpolation=cv2.INTER_NEAREST)
    frame_gb = cv2.GaussianBlur(frame_resize, (5, 5), 5)   
    frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)  # 将图像转换到LAB空间
    
    area_max = 0
    areaMaxContour = 0
    for i in lab_data:
        if i in __target_color:
            detect_color = i
            frame_mask = cv2.inRange(frame_lab,
                                         (lab_data[i]['min'][0],
                                          lab_data[i]['min'][1],
                                          lab_data[i]['min'][2]),
                                         (lab_data[i]['max'][0],
                                          lab_data[i]['max'][1],
                                          lab_data[i]['max'][2]))  #对原图像和掩模进行位运算
            eroded = cv2.erode(frame_mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  #腐蚀
            dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))) #膨胀
            if debug:
                cv2.imshow(i, dilated)
            contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  # 找出轮廓
            areaMaxContour, area_max = getAreaMaxContour(contours)  # 找出最大轮廓

    if area_max > 50:  # 有找到最大面积
        (centerX, centerY), radius = cv2.minEnclosingCircle(areaMaxContour) #获取最小外接圆
        centerX = int(Misc.map(centerX, 0, size[0], 0, img_w))
        centerY = int(Misc.map(centerY, 0, size[1], 0, img_h))
        radius = int(Misc.map(radius, 0, size[0], 0, img_w))
        cv2.circle(img, (int(centerX), int(centerY)), int(radius), range_rgb[detect_color], 2)
        if search:
            if abs(centerX - img_w/2) < 50:
                search = False
        else:
            if abs(centerX - img_w/2) > 15: 
                x_pid.SetPoint = img_w/2  #设定
            else:
                x_pid.SetPoint = centerX
            x_pid.update(centerX)  #当前
            dx = int(x_pid.output)
            x_dis += dx  #输出
             
            if abs(centerY - img_h/2) > 15:
                y_pid.SetPoint = img_h/2
            else:
                y_pid.SetPoint = centerY
            y_pid.update(centerY)
            dy = int(y_pid.output)
            y_dis += dy
            
            x_dis = 0 if x_dis < 0 else x_dis          
            x_dis = 1000 if x_dis > 1000 else x_dis
            y_dis = 0 if y_dis < 0 else y_dis
            y_dis = 240 if y_dis > 240 else y_dis    
            
            if not debug:
                state = True
                Board.setBusServoPulse(24, y_dis, 20)
                Board.setBusServoPulse(21, x_dis, 20)
                time.sleep(0.02)
            
    else:
        state = False
        search = True
            
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
    init()
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