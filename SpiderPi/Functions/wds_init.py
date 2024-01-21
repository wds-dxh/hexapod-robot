#!/usr/bin/python3
# coding=utf8
import sys
sys.path.append('/home/pi/SpiderPi/')
import HiwonderSDK.Board as Board
import time
import kinematics

ik = kinematics.IK()  # 实例化逆运动学库

# 以下为初始化动作
def init():
    ik.stand(ik.initial_pos) # 立正
    servo21,servo22,servo23,servo24,servo25 = 500,705,90,330,0 
    Board.setBusServoPulse(21, servo21, 1000)
    Board.setBusServoPulse(22, servo22, 1000)
    Board.setBusServoPulse(23, servo23, 1000)
    Board.setBusServoPulse(24, servo24, 1000)
    Board.setBusServoPulse(25, servo25, 1000)
    time.sleep(1)





if __name__ == '__main__':
    ik.stand(ik.initial_pos) # 立正
    servo21,servo22,servo23,servo24,servo25 = 500,705,90,330,500 
    Board.setBusServoPulse(21, servo21, 1000)
    Board.setBusServoPulse(22, servo22, 1000)
    Board.setBusServoPulse(23, servo23, 1000)
    Board.setBusServoPulse(24, servo24, 1000)
    Board.setBusServoPulse(25, servo25, 1000)
    time.sleep(1)
