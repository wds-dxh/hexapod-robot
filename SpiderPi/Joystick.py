# #!/usr/bin/python3
# # coding=utf8
# import sys
# sys.path.append('/home/pi/SpiderPi/')
# import os
# import time
# import json
# import pygame
# import asyncio
# import threading
# import websockets
# import HiwonderSDK.Board as Board
# import Functions.kinematics as kinematics
# import HiwonderSDK.ActionGroupControl as AGC
# import Functions.Robot_dance as dance
# import pygame
# import sys

# servo21,servo22,servo23,servo24,servo25 = 500,705,90,330,0
# def init():
#     # ik = kinematics.IK()
#     # ik.stand(ik.initial_pos) # 立正
#     Board.setBusServoPulse(21, servo21, 1000)
#     Board.setBusServoPulse(22, servo22, 1000)
#     Board.setBusServoPulse(23, servo23, 1000)
#     Board.setBusServoPulse(24, servo24, 1000)
#     Board.setBusServoPulse(25, servo25, 1000)
#     time.sleep(1)

# button0 = "up"
# button1 = "up"
# button3 = "up"
# button4 = "up"
# button6 = "up"
# button7 = "up"
# button8 = "up"
# button9 = "up"
# if __name__ == '__main__':
#     # init()
#     pygame.init()
#     ik = kinematics.IK()
#     # 初始化手柄
#     pygame.joystick.init()
    
#     try:
#         # 获取手柄的数量
#         joystick_count = pygame.joystick.get_count()
        
#         print(f"Number of joysticks: {joystick_count}")

#         if joystick_count > 0:
#             # 获取第一个手柄
#             joystick = pygame.joystick.Joystick(0)#创建一个操纵杆对象
#             joystick.init()#初始化这个操纵杆对象

#             print(f"Name of the joystick: {joystick.get_name()}")
#             print(f"Number of axes: {joystick.get_numaxes()}")
#             print(f"Number of buttons: {joystick.get_numbuttons()}")
#             print(f"Number of balls: {joystick.get_numballs()}")
#             print(f"Number of hats: {joystick.get_numhats()}")

#             while True:
#                 # 处理事件
#                 for event in pygame.event.get():
#                     if event.type == pygame.QUIT:#如果单击关闭窗口，则退出
#                         pygame.quit()
#                         sys.exit()

#                     # 处理手柄事件
#                     if event.type == pygame.JOYAXISMOTION:#如果是摇杆事件
#                         print(f"Axis {event.axis}: {event.value}")

#                     elif event.type == pygame.JOYBUTTONDOWN:#如果是按键事件
#                         print(f"Button {event.button} down")
#                         if event.button == 11:  #初始化
#                             init()
#                         if event.button == 0:   #×
#                             # servo22 += 3
#                             print(servo22)
#                             button0 = "down"
#                             # Board.setBusServoPulse(22, servo22, 30)
#                             time.sleep(0.03)
#                         if  event.button == 3:#方框
#                             # servo23 += 3
#                             button3 = "down"
#                             # Board.setBusServoPulse(22, servo23, 30)
#                             time.sleep(0.03)
#                         if  event.button == 8:  #L2
#                             # servo24 += 3
#                             button8 = "down"
#                             # Board.setBusServoPulse(22, servo24, 30)
#                             time.sleep(0.03)
#                         if  event.button == 9:  #L2
#                             # servo25 += 3
#                             button9 = "down"
#                             # Board.setBusServoPulse(22, servo25, 30)
#                             time.sleep(0.03)

#                         if event.button == 1:   #圆圈
#                             # servo22 -= 3
#                             print(servo22)
#                             button1 = "down"
#                             # Board.setBusServoPulse(22, servo22, 30)
#                             time.sleep(0.03)
#                         if  event.button == 4:#三角形
#                             # servo23 -= 3
#                             button4 = "down"
#                             # Board.setBusServoPulse(22, servo23, 30)
#                             time.sleep(0.03)
#                         if  event.button == 6:  #L1
#                             # servo24 -= 3
#                             button6 = "down"
#                             # Board.setBusServoPulse(22, servo24, 30)
#                             time.sleep(0.03)
#                         if  event.button == 7:  #R1
#                             # servo25 -= 3
#                             button7 = "down"
#                             # Board.setBusServoPulse(22, servo25, 30)
#                             time.sleep(0.03)
                            
#                     elif event.type == pygame.JOYBUTTONUP:#如果是松开按键事件
#                         print(f"Button {event.button} up")
#                         #所有按键全部设置为0
#                         button0 = "up"
#                         button1 = "up"
#                         button3 = "up"
#                         button4 = "up"
#                         button6 = "up"
#                         button7 = "up"
#                         button8 = "up"
#                         button9 = "up"

#                     elif event.type == pygame.JOYHATMOTION:#如果是摇杆事件
#                         print(f"Hat {event.hat} moved to {event.value}")
#                         if event.hat == 0:#速度全部为30
#                             if event.value[1] == 1:
#                                 ik.go_forward(ik.initial_pos, 2, 50, 30, 1)  # 朝前直走100mm
#                             if event.value[1] == -1:
#                                 ik.back(ik.initial_pos, 2, 50, 30, 1)# 朝后直走100mm
#                             if event.value[0] == -1:
#                                 ik.left_move(ik.initial_pos, 2, 50, 30, 1)  # 左移100mm
#                             if event.value[0] == 1:
#                                 ik.right_move(ik.initial_pos, 2, 50, 30, 1)# 右移100mm

#                 if button0 == "down":
#                     servo22 += 3
#                     print(servo22)
#                     Board.setBusServoPulse(22, servo22, 30)
#                     time.sleep(0.03)
#                 if button1 == "down":
#                     servo22 -= 3
#                     print(servo22)
#                     Board.setBusServoPulse(22, servo22, 30)
#                     time.sleep(0.03)
#                 if button3 == "down":
#                     servo23 += 3
#                     print(servo23)
#                     Board.setBusServoPulse(23, servo23, 30)
#                     time.sleep(0.03)
#                 if button4 == "down":
#                     servo23 -= 3
#                     print(servo23)
#                     Board.setBusServoPulse(23, servo23, 30)
#                     time.sleep(0.03)
#                 if button8 == "down":
#                     servo24 += 3
#                     print(servo24)
#                     Board.setBusServoPulse(24, servo24, 30)
#                     time.sleep(0.03)
#                 if button9 == "down":
#                     servo25 += 3
#                     print(servo25)
#                     Board.setBusServoPulse(25, servo25, 30)
#                     time.sleep(0.03)
#                 if button6 == "down":
#                     servo24 -= 3
#                     print(servo24)
#                     Board.setBusServoPulse(24, servo24, 30)
#                     time.sleep(0.03)
#                 if button7 == "down":
#                     servo25 -= 3
#                     print(servo25)
#                     Board.setBusServoPulse(25, servo25, 30)
#                     time.sleep(0.03)

#                 # Board.setBusServoPulse(22, servo22, 30)
#                 # time.sleep(0.03)
#                 # Board.setBusServoPulse(23, servo23, 30)
#                 # time.sleep(0.03)
#                 # Board.setBusServoPulse(24, servo24, 30)
#                 # time.sleep(0.03)
#                 # Board.setBusServoPulse(25, servo25, 30)
#                 # time.sleep(0.03)
#     finally:
#         pygame.quit()#意思是退出pygame，清理pygame占用资源
