#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import sys
sys.path.append('/home/pi/SpiderPi/ArmIK')
import time
import numpy as np
import kinematics  # 运动学库已加密，只供调用
import ArmMoveIK

AK = ArmMoveIK.ArmIK()
ik = kinematics.IK()  # 实例化逆运动学库

ik.stand(ik.initial_pos, t=500)  # 立正，姿态为ik.initial_pos，时间500ms
print('姿态:\n', np.array(ik.initial_pos))  # 打印查看姿态
# 姿态数据为3x6的数组，表示6条腿的的末端的x，y，z坐标，单位mm
# 头部朝前的方向为x轴， 头部朝前位置为负方向，右边为y轴正， 竖直朝上为z轴正， 从中间两条腿的连线的中点做垂线与上下板相交，取连线中心为零点
# 第一条腿表示头部朝前时左上角所在腿, 逆时针表示1-6
# [[-199.53, -177.73, -100.0],
#  [0.0,     -211.27, -100.0],
#  [199.53,  -177.73, -100.0],
#  [199.53,  177.73,  -100.0],
#  [0.0,     211.27,  -100.0],
#  [-199.53, 177.73,  -100.0]]

# 参数1：姿态；参数2：模式，2为六足模式，4为四足模式，当为4足模式时相应的姿态需要为initial_pos_quadruped
# 参数3：步幅，单位mm （转弯时为角度，单位度）；参数4：速度，单位mm/s；参数5：执行次数，单位0时表示无限循环
# ik.circle()
'''
moveToPosition(leg, leg_p, p, o):
控制机体的中心运动
:param leg: 1~6, 表示6条腿，1表示头部朝前时左上角所在腿, 逆时针表示1-6
:param leg_p: 足末端点相对中心的坐标,单位mm
:param p: 方向角
:param o: 旋转角
:param speed: 运行该动作的速度,单位ms
:return:
'''
'''
setStepMode(pos, mode, step_velocity, step_amplitude, step_height, movement_direction, rotation_angle, rotation, p, o, speed, times):
:param pos: 初始位置/mm
:param mode: 步态模式， 1:Ripple Gait，2:Tripod Gait, 3:四足，4：四足
:param step_velocity: 速度
:param step_amplitude: 幅度/mm
:param step_height: 高度/mm
:param movement_direction: 方向/0-360
:param rotation: 旋转角/-1~1
:param speed: 舵机速度/ms, 一个周期10个动作，前进幅度为step_amplitude，所以移动速度为step_amplitude/(speed*10)
:param times: 执行次数
:return:
'''
AK.setPitchRangeMoving((0, 10, 16), -45, -90, 100, 1000) 
time.sleep(1)

for i in range(2):
    AK.setPitchRangeMoving((0, 10 + 10*(i + 1), 30), 0, -90, 100, 800)
    ik.setStepMode_whitout_delay(ik.initial_pos, 2, 2, -100, 20, 0, 0, 0, [0, 0, 0], [0, 0, 0], 80, 1)
ik.stand(ik.initial_pos, t=500)

for i in range(3):
    AK.setPitchRangeMoving((0, 30.1 - 10*(i + 1), 30), 0, -90, 100, 800)
    ik.setStepMode_whitout_delay(ik.initial_pos, 2, 2, 100, 20, 0, 0, 0, [0, 0, 0], [0, 0, 0], 80, 1)
ik.stand(ik.initial_pos, t=500)


ik.go_forward(ik.initial_pos, 2, 50, 80, 1)  # 朝前直走50mm

ik.back(ik.initial_pos, 2, 100, 80, 1)  # 朝后直走100mm

ik.turn_left(ik.initial_pos, 2, 30, 100, 1)  # 原地左转30度

ik.turn_right(ik.initial_pos, 2, 30, 100, 1)  # 原地右转30读

ik.left_move(ik.initial_pos, 2, 100, 100, 1)  # 左移100mm

ik.right_move(ik.initial_pos, 2, 100, 100, 1)  # 右移100mm

ik.stand(ik.initial_pos, t=500)



