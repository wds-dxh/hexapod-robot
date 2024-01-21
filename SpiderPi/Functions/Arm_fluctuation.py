#!/usr/bin/python3
# coding=utf8
import sys
sys.path.append('/home/pi/SpiderPi/')
import time
import copy
import ArmIK.ArmMoveIK as ArmIK
import kinematics as kinematics

AK = ArmIK.ArmIK()
ik = kinematics.IK()

# 机械臂上下移动     
     

if __name__ == '__main__':
    AK.setPitchRangeMoving((0, 15, 30), 0, -90, 100, 2000)
    ik.stand(ik.initial_pos)
    time.sleep(2)
    
    AK.setPitchRangeMoving((0, 15, 35), 0, -90, 100, 1000)
    time.sleep(1)
    for i in range(3):
        AK.setPitchRangeMoving((0, 15, 25), 0, -90, 100, 2000)
        time.sleep(2)
        AK.setPitchRangeMoving((0, 15, 35), 0, -90, 100, 2000)
        time.sleep(2)
        
    AK.setPitchRangeMoving((0, 15, 30), 0, -90, 100, 1000)
    time.sleep(1)