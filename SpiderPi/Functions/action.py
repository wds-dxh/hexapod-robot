import HiwonderSDK.ActionGroupControl as AGC#动作组控制
import time

class action:
    
    def __init__(self, speed = 10,move_distance = 30,PID_P = 1,PID_I = 0.1,PID_D = 0.1):
        self.__speed = speed
        self.__move_distance = move_distance
        self.__PID_P = PID_P
        print('action init')
        pass

    def init_actions(self):#已经直接写了，不需要再调用
        AGC.runAction('init_actions')#初始化视角，可以正常看到识别的色块

 #定义校准动作，传入xywh坐标
    def alignment_PID(self, x,y,ik,high = 420):
        speed = self.__speed
        move_distance = self.__move_distance
        PID_P = self.__PID_P
        if x == None or y == None:
            return False
        else:        
            if x >325:
                # ik.turn_left(ik.initial_pos, 2, move_distance, speed, 1)  # 原地左转10度
                Error = x - 325
                control = PID_P * Error
                #限制控制量的大小，在10-100之间
                if control > 100:
                    control = 100
                elif control < 10:
                    control = 10
                ik.right_move(ik.initial_pos, 2, control , speed, 1)
                time.sleep(0.03)
                return False
            elif x < 315:
                Error = 315 - x
                control = PID_P * Error
                #限制控制量的大小，在10-100之间
                if control > 100:
                    control = 100
                elif control < 10:
                    control = 10
                # ik.turn_right(ik.initial_pos, 2, move_distance, speed, 1)
                ik.left_move(ik.initial_pos, 2, control, speed, 1)  # 左移10mm
                time.sleep(0.03)
                return False
            if y > 400:
                Error = y - 400
                control = PID_P * Error
                #限制控制量的大小，在10-100之间
                if control > 100:
                    control = 100
                elif control < 10:
                    control = 10
                ik.back(ik.initial_pos, 2, control, speed, 1)# 朝前直走10mm
                time.sleep(0.03)
                return False
            elif y < 390:
                Error = 390 - y
                control = PID_P * Error
                #限制控制量的大小，在10-100之间
                if control > 100:
                    control = 100
                elif control < 10:
                    control = 10
                ik.go_forward(ik.initial_pos, 2, control, speed, 1)  # 朝后直走10mm
                time.sleep(0.03)
                return False
            # elif 410< y:
            #     if y < 430:
            #         AGC.runAction('init_actions')#初始化视角，可以正常看到识别的色块
            #         print("y对准了")
            #         print("y:",y)         
            #         return True
            # elif 310 < x :
            #     if x >330:
            #         AGC.runAction('init_actions')#初始化视角，可以正常看到识别的色块
            #         print("x对准了",x)
            #         print("x:",x)
            #         return True   
            elif 390< y and y < 400 and x <325 and 315 < x :
                AGC.runAction('init_actions')#初始化视角，可以正常看到识别的色块
                time.sleep(1)
                return True



    #定义校准动作，传入xywh坐标
    def alignment(self, x,y,ik,high = 420):
        speed = self.__speed
        move_distance = self.__move_distance
        PID_P = self.__PID_P
        if x == None or y == None:
            return False
        else:        
            if x >325:
                # ik.turn_left(ik.initial_pos, 2, move_distance, speed, 1)  # 原地左转10度
                Error = x - 325
                ik.right_move(ik.initial_pos, 2,move_distance , speed, 1)
                time.sleep(0.1)
                return False
            elif x < 315:
                Error = 315 - x
                # ik.turn_right(ik.initial_pos, 2, move_distance, speed, 1)
                ik.left_move(ik.initial_pos, 2, move_distance, speed, 1)  # 左移10mm
                time.sleep(0.1)
                return False
            if y > 400:
                Error = y - 400
                ik.back(ik.initial_pos, 2, move_distance, speed, 1)# 朝前直走10mm
                time.sleep(0.1)
                return False
            elif y < 390:
                Error = 390 - y
                ik.go_forward(ik.initial_pos, 2,move_distance, speed, 1)  # 朝后直走10mm
                time.sleep(0.1)
                return False
            # elif 410< y:
            #     if y < 430:
            #         AGC.runAction('init_actions')#初始化视角，可以正常看到识别的色块
            #         print("y对准了")
            #         print("y:",y)         
            #         return True
            # elif 310 < x :
            #     if x >330:
            #         AGC.runAction('init_actions')#初始化视角，可以正常看到识别的色块
            #         print("x对准了",x)
            #         print("x:",x)
            #         return True   
            elif 390< y and y < 400 and x <325 and 315 < x :
                AGC.runAction('init_actions')#初始化视角，可以正常看到识别的色块
                time.sleep(1)
                return True



#定义x校准动作，传入xywh坐标
    def alignment_x(self, x,ik,high = 420):
        speed = self.__speed
        move_distance = self.__move_distance
        if x == None:
            return False
        else:
            print("x:",x)
            if x >310:
                # ik.turn_left(ik.initial_pos, 2, move_distance, speed, 1)  # 原地左转10度
                ik.right_move(ik.initial_pos, 2, move_distance, speed, 1)
                time.sleep(0.1)
                return False
            elif x < 290:
                # ik.turn_right(ik.initial_pos, 2, move_distance, speed, 1)
                ik.left_move(ik.initial_pos, 2, move_distance, speed, 1)  # 左移10mm
                time.sleep(0.1)
                return False
            elif 310 < x :
                if x >330:
                    AGC.runAction('init_actions')#初始化视角，可以正常看到识别的色块
                    print("x对准了",x)
                    print("x:",x)
                    return True

    #定义y校准动作，传入xywh坐标
    def alignment_y(self, y,ik,high = 420):
        speed = self.__speed
        move_distance = self.__move_distance
        y_distance = 10
        if y == None:
            return False
        else:
            if y > (430):
                ik.back(ik.initial_pos, 2, move_distance, speed, 1)# 朝前直走10mm
                time.sleep(0.1)
                return False
            elif y < (410):
                ik.go_forward(ik.initial_pos, 2, move_distance, speed, 1)  # 朝后直走10mm
                time.sleep(0.1)
                return False
            elif 410< y:
                if y < 430:
                    AGC.runAction('init_actions')#初始化视角，可以正常看到识别的色块
                    print("y对准了")
                    print("y:",y)         
                    return True
    #前进到抓取纠正位置
    def  Grab_correction_location(self, ik, distance = 50):
        speed = self.__speed
        ik.go_forward(ik.initial_pos, 2, distance, speed, 1) 


    #抓取物体并返回到摄像头可以看到的位置
    def grap(self):#这里抓取的英文是grab，但是为了避免和函数名重复，所以改成了grap
        AGC.runAction('init_grap')
        AGC.runAction('init_actions')#这个动作可以用于回到摄像头可以看到的位置，并且物体不会掉落
        

if __name__ == '__main__':
    #打印对象的所有属性和方法
    print(dir(action))