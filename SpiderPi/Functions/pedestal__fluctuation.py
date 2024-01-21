#!/usr/bin/python3
# coding=utf8
import sys
sys.path.append('/home/pi/SpiderPi/')
import time
import copy
import ArmIK.ArmMoveIK as ArmIK
import kinematics as kinematics

# 机体上下移动  

AK = ArmIK.ArmIK()
ik = kinematics.IK()


def Stand(height, mode, t):
    if height > 160:
        ik.current_pos = copy.deepcopy(ik.initial_pos_high)
    else:
        ik.current_pos = copy.deepcopy(ik.initial_pos)
        
    pos = ik.current_pos
    for i in range(6):
        pos[i][2] = -float(height)
        
    ik.stand(pos, t)
        
      

if __name__ == '__main__':
    AK.setPitchRangeMoving((0, 15, 30), 0, -90, 100, 2000)
    ik.stand(ik.initial_pos)
    time.sleep(2)

    Stand(50,2,1000)
    for i in range(3):
        Stand(150,2,2000)
        
        Stand(50,2,2000)
    
    Stand(100,2,1000)