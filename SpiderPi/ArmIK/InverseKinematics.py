#!/usr/bin/env python3
# encoding: utf-8
# 4自由度机械臂逆运动学：给定相应的坐标（X,Y,Z），以及俯仰角，计算出每个关节转动的角度
# 2020/07/20 Aiden
import logging
from math import *

# CRITICAL, ERROR, WARNING, INFO, DEBUG
logging.basicConfig(level=logging.ERROR)
logger = logging.getLogger(__name__)

class IK:
    # 舵机从下往上数
    # 公用参数，即4自由度机械臂的连杆参数
    l1 = 13.0    #机械臂底盘中心到第二个舵机中心轴的距离/cm
    l2 = 13.0   #第二个舵机到第三个舵机的距离/cm
    l3 = 13.0    #第三个舵机到第四个舵机的距离/cm
    l4 = 15.0    #这里不做具体赋值，根据初始化时的选择进行重新赋值

    def setLinkLength(self, L1=l1, L2=l2, L3=l3, L4=l4):
        # 更改机械臂的连杆长度，为了适配相同结构不同长度的机械臂
        self.l1 = L1
        self.l2 = L2
        self.l3 = L3
        self.l4 = L4

    def getLinkLength(self):
        # 获取当前设置的连杆长度
        return {"L1":self.l1, "L2":self.l2, "L3":self.l3, "L4":self.l4}

    def getRotationAngle(self, coordinate_data, Alpha):
        # 给定指定坐标和俯仰角，返回每个关节应该旋转的角度，如果无解返回False
        # coordinate_data为夹持器末端坐标，坐标单位cm， 以元组形式传入，例如(0, 5, 10)
        # Alpha为夹持器与水平面的夹角，单位度

        # 设夹持器末端为P(X, Y, Z), 坐标原点为O, 原点为云台中心在地面的投影， P点在地面的投影为P_
        # l1与l2的交点为A, l2与l3的交点为B，l3与l4的交点为C
        # CD与PD垂直，CD与z轴垂直，则俯仰角Alpha为DC与PC的夹角, AE垂直DP_， 且E在DP_上， CF垂直AE，且F在AE上
        # 夹角表示：例如AB和BC的夹角表示为ABC
        X, Y, Z = coordinate_data
        #求底座旋转角度
        theta21 = degrees(atan2(Y, X))
 
        P_O = sqrt(X*X + Y*Y) #P_到原点O距离
        CD = self.l4 * cos(radians(Alpha))
        PD = self.l4 * sin(radians(Alpha)) #当俯仰角为正时，PD为正，当俯仰角为负时，PD为负
        AF = P_O - CD
        CF = Z - self.l1 - PD
        AC = sqrt(pow(AF, 2) + pow(CF, 2))
        if round(CF, 4) < -self.l1:
            logger.debug('高度低于0, CF(%s)<l1(%s)', CF, -self.l1)
            return False
        if self.l2 + self.l3 < round(AC, 4): #两边之和小于第三边
            logger.debug('不能构成连杆结构, l2(%s) + l3(%s) < AC(%s)', self.l2, self.l3, AC)
            return False

        #求theat4
        cos_ABC = round(-(pow(AC, 2)- pow(self.l2, 2) - pow(self.l3, 2))/(2*self.l2*self.l3), 4) #余弦定理
        if abs(cos_ABC) > 1:
            logger.debug('不能构成连杆结构, abs(cos_ABC(%s)) > 1', cos_ABC)
            return False
        ABC = acos(cos_ABC) #反三角算出弧度
        theta23 = 180.0 - degrees(ABC)

        #求theta22
        CAF = acos(AF / AC)
        cos_BAC = round((pow(AC, 2) + pow(self.l2, 2) - pow(self.l3, 2))/(2*self.l2*AC), 4) #余弦定理
        if abs(cos_BAC) > 1:
            logger.debug('不能构成连杆结构, abs(cos_BAC(%s)) > 1', cos_BAC)
            return False
        if CF < 0:
            zf_flag = -1
        else:
            zf_flag = 1
        theta22 = degrees(CAF * zf_flag + acos(cos_BAC))

        #求theta24
        theta24 = Alpha - theta22 + theta23

        return {"theta24":theta24, "theta23":theta23, "theta22":theta22, "theta21":theta21} # 有解时返回角度字典
            
if __name__ == '__main__':
    ik = IK()
    ik.setLinkLength(L1=ik.l1 , L4=ik.l4)
    print('连杆长度：', ik.getLinkLength())
    print(ik.getRotationAngle((0, 0, ik.l1 + ik.l2 + ik.l3 + ik.l4), 90))
