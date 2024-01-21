#!/usr/bin/python3
# coding=utf8
import sys
sys.path.append('/home/pi/SpiderPi/')
import cv2
import math
import time
import Camera
import apriltag
import threading
import kinematics
import import_path
import numpy as np
import yaml_handle
import ArmIK.ArmMoveIK as AMK
import HiwonderSDK.Misc as Misc
import HiwonderSDK.Board as Board

# 巡线搬运

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

# 逆运动学定义
ik = kinematics.IK()
AK = AMK.ArmIK()

range_rgb = {
    'red': (0, 0, 255),
    'blue': (255, 0, 0),
    'green': (0, 255, 0),
    'black': (0, 0, 0),
    'white': (255, 255, 255)}

# 变量定义
HWSONAR = None
step = 'move'
carry = False
place = False
exist = False
target_color = None
color_list = ('green','blue')
line_width = 0
line_angle = 0
line_centerX = -1
line_centerY = -1
img_center_x = 320
last_line_centerX = 0
line_coord = (0, 10, 15) # 机械臂巡线坐标
detect_coord = (0, 15, 5) # 机械臂识别坐标

old_x, old_y = 999, 999
world_x, world_y = 999, 999


lab_data = None
K,R,T = None,None,None
# 读取颜色阈值及坐标变换参数
def load_config():
    global lab_data,K,R,T
    
    lab_data = yaml_handle.get_yaml_data(yaml_handle.lab_file_path)
    camera_cal = yaml_handle.get_yaml_data(yaml_handle.camera_file_path)['block_params']
    K = np.array(camera_cal['K'], dtype=np.float64).reshape(3, 3)
    R = np.array(camera_cal['R'], dtype=np.float64).reshape(3, 1)
    T = np.array(camera_cal['T'], dtype=np.float64).reshape(3, 1)
        

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
            if contour_area_temp >= 100:  # 只有在面积大于设定时，最大面积的轮廓才是有效的，以过滤干扰
                area_max_contour = c
                max_area = contour_area_temp

    return area_max_contour, max_area  # 返回最大的轮廓, 面积

# 初始位置
def initMove():
    HWSONAR.setRGBMode(0)
    HWSONAR.setRGB(1, (0, 0, 0))
    HWSONAR.setRGB(2, (0, 0, 0))
    Board.setBusServoPulse(25, 120, 1000)
    ik.stand(ik.initial_pos)
    AK.setPitchRangeMoving(line_coord, -60, -90, 90, 2000)

# 像素坐标变换到现实坐标函数
def camera_to_world(cam_mtx, r, t, img_points):
    inv_k = np.asmatrix(cam_mtx).I
    r_mat = np.zeros((3, 3), dtype=np.float64)
    cv2.Rodrigues(r, r_mat)
    inv_r = np.asmatrix(r_mat).I  # 3*3
    transPlaneToCam = np.dot(inv_r, np.asmatrix(t))  # 3*3 dot 3*1 = 3*1
    world_pt = []
    coords = np.zeros((3, 1), dtype=np.float64)
    for img_pt in img_points:
        coords[0][0] = img_pt[0][0]
        coords[1][0] = img_pt[0][1]
        coords[2][0] = 1.0
        worldPtCam = np.dot(inv_k, coords)  # 3*3 dot 3*1 = 3*1
        worldPtPlane = np.dot(inv_r, worldPtCam)  # 3*3 dot 3*1 = 3*1
        scale = transPlaneToCam[2][0] / worldPtPlane[2][0]
        scale_worldPtPlane = np.multiply(scale, worldPtPlane)
        worldPtPlaneReproject = np.asmatrix(scale_worldPtPlane) - np.asmatrix(transPlaneToCam)  # 3*1 dot 1*3 = 3*3
        pt = np.zeros((3, 1), dtype=np.float64)
        pt[0][0] = worldPtPlaneReproject[0][0]
        pt[1][0] = worldPtPlaneReproject[1][0]
        pt[2][0] = 0
        world_pt.append(pt.T.tolist())
    return world_pt


# 机器人移动函数
def move():
    global step,carry,place,target_color
    global last_line_centerX,line_width
    
    n = 0
    dn = 0
    sp = 0
    direction = None
    while True:
        if step == 'detect': # 色块识别、抓取
            
            if place: # 放置状态
                
                if target_color == 'green':
                    x, y = -6, 24
                elif target_color == 'blue':
                    x, y =  8, 26
                AK.setPitchRangeMoving((x, y, 0), -90, -90, 100, 1000)   # 移动到目标位置上方
                time.sleep(1)
                AK.setPitchRangeMoving((x, y, -5), -90, -90, 100, 1000)  # 移动到目标位置
                time.sleep(1)
                Board.setBusServoPulse(25, 120, 500)                     # 放下木块
                time.sleep(0.5)
                AK.setPitchRangeMoving((x, y, 0), -90, -90, 100, 1000)   # 抬起来
                time.sleep(1)
                AK.setPitchRangeMoving(line_coord, -60, -90, 90, 1500)   # 机械臂回到巡线姿态
                time.sleep(1.5)
                place = False
                step = 'move'
                ik.back(ik.initial_pos, 2, 30, 50, 5)        # 向后退几步
                ik.stand(ik.initial_pos, t=500)              # 立正姿态
                ik.turn_right(ik.initial_pos, 2, 15, 50, 9)  # 右转180°                       
                ik.stand(ik.initial_pos, t=500)
                time.sleep(0.5)
                line_width = 0
            
            else:   #夹取状态
                if carry:
                    if world_x > 7:
                        ik.right_move(ik.initial_pos, 2, 30, 50, 1)
                        direction = 'right'
                        sp += 1
                        carry = False
                    elif world_x > 4 and world_y < -2:
                        ik.right_move(ik.initial_pos, 2, 30, 50, 1)
                        direction = 'right'
                        sp += 1
                        carry = False
                    elif world_x < -7:
                        ik.left_move(ik.initial_pos, 2, 30, 50, 1)
                        direction = 'left'
                        sp += 1
                        carry = False
                    elif world_x < -4 and world_y < -2:
                        ik.left_move(ik.initial_pos, 2, 30, 50, 1)
                        direction = 'left'
                        sp += 1
                        carry = False
                    elif world_y > 7:
                        ik.go_forward(ik.initial_pos, 2, 50, 60, 1)
                        carry = False
                    else:
                        ik.stand(ik.initial_pos)
                        time.sleep(0.5)
                        Board.setBusServoPulse(25, 120, 500)                    # 张开爪子
                        x,y = detect_coord[0]+world_x, detect_coord[1]+world_y  # 转换成机械臂坐标
                        AK.setPitchRangeMoving((x, y, 0), -90, -90, 100, 1000)  # 移动到目标位置上方
                        time.sleep(1)
                        AK.setPitchRangeMoving((x, y, -5), -90, -90, 100, 1000) # 移动到目标位置
                        time.sleep(1)
                        Board.setBusServoPulse(25, 500, 500)                    # 夹取目标
                        time.sleep(0.5)
                        AK.setPitchRangeMoving((x, y, 0), -90, -90, 100, 1000) # 抬起来
                        time.sleep(1)
                        AK.setPitchRangeMoving(line_coord, -60, -90, 90, 1500) # 机械臂回到巡线姿态
                        time.sleep(1.5)
                        if direction == 'left':
                            ik.right_move(ik.initial_pos, 2, 30, 50, sp)
                            direction = None
                            sp = 0
                        elif direction == 'right':
                            ik.left_move(ik.initial_pos, 2, 30, 50, sp)
                            direction = None
                            sp = 0            
                        place = True
                        carry = False
                        step = 'move'
                        ik.back(ik.initial_pos, 2, 30, 50, 5)        # 向后退几步
                        ik.stand(ik.initial_pos, t=500)              # 立正姿态
                        ik.turn_left(ik.initial_pos, 2, 15, 50, 9)   # 右转180°                       
                        ik.stand(ik.initial_pos, t=500)
                        time.sleep(0.5)
                        line_width = 0
                
        elif step == 'move': # 机器人巡线移动
            if line_centerX >= 0 and line_width < 100:    # 判断是否检测到线条        
                if abs(line_centerX -img_center_x) < 30:  # 判断是否偏离
                    ik.go_forward(ik.initial_pos, 2, 50, 60, 1)
                    
                elif line_centerX -img_center_x >= 30:    #线条在画面右侧，右转
                    ik.turn_right(ik.initial_pos, 2, 5, 50, 1)
                    
                elif line_centerX -img_center_x <= -30:   # 线条在画面左侧，左转
                    ik.turn_left(ik.initial_pos, 2, 5, 50, 1)
                    
                last_line_centerX = line_centerX
                n = 0
                
            elif line_centerY < 380 and 230 > line_width > 100:
                ik.go_forward(ik.initial_pos, 2, 50, 50, 1)
            
            elif line_centerX == -2: # 未检测到线条，根据最后检测到的坐标，左转或右转
                if n < 5:
                    n += 1
                    if last_line_centerX >= img_center_x:
                        ik.turn_left(ik.initial_pos, 2, 5, 50, 1)
                    else:
                        ik.turn_right(ik.initial_pos, 2, 5, 50, 1)
                  
            if line_width > 230: # 检测到横线
                dn += 1
                time.sleep(0.01)
                if dn >= 5: # 多次检测
                    ik.go_forward(ik.initial_pos, 2, 30, 60, 3) # 向前走两步
                    ik.stand(ik.initial_pos, t=500)
                    if place:
                        
                        while abs(line_centerX -img_center_x) > 20 and line_centerY < 260:
                            print(abs(line_centerX -img_center_x))
                            if line_centerX -img_center_x >= 20:    #线条在画面右侧
                                ik.right_move(ik.initial_pos, 2, 20, 50, 1)
                                
                            elif line_centerX -img_center_x <= -20:   # 线条在画面左侧
                                ik.left_move(ik.initial_pos, 2, 20, 50, 1)
                            
                    AK.setPitchRangeMoving(detect_coord, -90, -90, 90, 1500) # 机械臂回到识别姿态
                    ik.stand(ik.initial_pos)
                    step = 'detect'
                    line_width = 0
                    dn = 0
            else:
                dn = 0       
        else:
           time.sleep(0.01)
        

# 运行子线程
th = threading.Thread(target=move)
th.setDaemon(True)
th.start()


size = (640, 480)
roi = [(150, 400,  0, 640)]
# 线条识别函数
def lineDetect(img,color):
    global line_centerX, line_centerY, line_width, line_angle
    
    img_w = img.shape[:2][1]
    img_h = img.shape[:2][0]
    frame_gb = cv2.GaussianBlur(img, (3, 3), 3)
    
    for r in roi:
        blobs = frame_gb[r[0]:r[1], r[2]:r[3]]
        frame_lab = cv2.cvtColor(blobs, cv2.COLOR_BGR2LAB)  # 将图像转换到LAB空间
        frame_mask = cv2.inRange(frame_lab,
                                 (lab_data[color]['min'][0],
                                  lab_data[color]['min'][1],
                                  lab_data[color]['min'][2]),
                                 (lab_data[color]['max'][0],
                                  lab_data[color]['max'][1],
                                  lab_data[color]['max'][2]))  #对原图像和掩模进行位运算
        eroded = cv2.erode(frame_mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  #腐蚀
        dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))    #膨胀
        dilated[:, 0:100] = 0
        dilated[:, 540:640] = 0
        cnts = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)[-2]  #找出所有轮廓
        cnt_large, area = getAreaMaxContour(cnts)  #找到最大面积的轮廓
        if area > 10:
            rect = cv2.minAreaRect(cnt_large)  #最小外接矩形
            box = np.int0(cv2.boxPoints(rect))  #最小外接矩形的四个顶点
            for j in range(4):
                box[j, 1] = box[j, 1] + r[0]

            cv2.drawContours(img, [box], -1, (0, 255, 255), 2)  #画出四个点组成的矩形
            #获取矩形的对角点
            pt1_x, pt1_y = box[0, 0], box[0, 1]
            pt3_x, pt3_y = box[2, 0], box[2, 1]
            line_center_x, line_center_y = (pt1_x + pt3_x) / 2, (pt1_y + pt3_y) / 2  #中心点
            
            cv2.circle(img, (int(line_center_x), int(line_center_y)), 5, (0, 0, 255), -1)  #画出中心点
            line_centerX = line_center_x
            line_centerY = line_center_y
            line_width = int(abs(pt1_x - pt3_x))
            
            down_x = (cnt_large[cnt_large[:,:,1].argmax()][0])[0]
            down_y = (cnt_large[cnt_large[:,:,1].argmax()][0])[1]
            left_x = (cnt_large[cnt_large[:,:,0].argmin()][0])[0]
            left_y = (cnt_large[cnt_large[:,:,0].argmin()][0])[1]
            right_x = (cnt_large[cnt_large[:,:,0].argmax()][0])[0]
            right_y = (cnt_large[cnt_large[:,:,0].argmax()][0])[1]
            if pow(down_x-left_x,2)+pow(down_y-left_y,2) > pow(down_x-right_x,2)+pow(down_y-right_y, 2):
                left_x = int(Misc.map(left_x, 0, size[0], 0, img_w))
                left_y = int(Misc.map(left_y, 0, size[1], 0, img_h))  
                right_x = int(Misc.map(down_x, 0, size[0], 0, img_w))
                right_y = int(Misc.map(down_y, 0, size[1], 0, img_h))
            else:
                left_x = int(Misc.map(down_x, 0, size[0], 0, img_w))
                left_y = int(Misc.map(down_y, 0, size[1], 0, img_h))
                right_x = int(Misc.map(right_x, 0, size[0], 0, img_w))
                right_y = int(Misc.map(right_y, 0, size[1], 0, img_h))
                
            line_angle = int(math.degrees(math.atan2(right_y - left_y, right_x - left_x)))
            print(line_centerX,line_width,line_angle)
            
        else:
            line_width = 0
            if line_centerX != -1:
                line_centerX = -2
        
    return img

i = 0
# 颜色识别函数
def ColorDetect(img, color):
    global world_x, world_y, i,target_color
    global old_x, old_y, carry, exist
    
    img_copy = img.copy()
    img_h, img_w = img.shape[:2]
    frame_resize = cv2.resize(img_copy, size, interpolation=cv2.INTER_NEAREST)
    frame_gb = cv2.GaussianBlur(frame_resize, (3, 3), 3)      
    frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)  # 将图像转换到LAB空间
    frame_mask = cv2.inRange(frame_lab,
                             (lab_data[color]['min'][0],
                              lab_data[color]['min'][1],
                              lab_data[color]['min'][2]),
                             (lab_data[color]['max'][0],
                              lab_data[color]['max'][1],
                              lab_data[color]['max'][2]))  #对原图像和掩模进行位运算
    eroded = cv2.erode(frame_mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  #腐蚀
    dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))) #膨胀
    contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  #找出轮廓
    areaMaxContour, area_max = getAreaMaxContour(contours)  #找出最大轮廓
    
    if area_max > 300:  # 有找到最大面积
        exist = True
        ((centerX, centerY), radius) = cv2.minEnclosingCircle(areaMaxContour)  # 获取最小外接圆
        centerX = int(Misc.map(centerX, 0, size[0], 0, img_w))
        centerY = int(Misc.map(centerY, 0, size[1], 0, img_h))
        radius = int(Misc.map(radius, 0, size[0], 0, img_w))
        cv2.circle(img, (centerX, centerY), radius, range_rgb[color], 2)#画圆
        cv2.putText(img, "Color: " + color, (10, img.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.65, range_rgb[color], 2) #显示颜色名字
        
        if abs(centerX-old_x) < 8 and abs(centerY-old_y) < 8: # 判断目标坐标有没有变化
            i += 1
        else:
            i = 0
            old_x, old_y = centerX, centerY
            
        if i > 3: # 多次判断，确定目标位置稳定
            # 转换成现实距离
            center = np.array([centerX,centerY])
            w = camera_to_world(K, R, T, center.reshape((1, 1, 2)))[0][0] 
            world_x, world_y = int(-w[0])/10, int(-w[1])/10
            print(world_x, world_y)
            target_color = color
            carry = True
            i = 0                                
    else:
        world_x, world_y = 999, 999
        carry = False
        exist = False
            
    return img


skip = 0
# 主要图像处理函数
def run(img):
    global skip
    
    if step == 'move':
        img = lineDetect(img, 'red')
    elif step == 'detect' and not place:
        img = ColorDetect(img, color_list[skip])
        if not exist:
            skip += 1
            skip  = 0 if skip > 1 else skip
        
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
    initMove()
    load_config()
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
    
    