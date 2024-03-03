import HiwonderSDK.ActionGroupControl as AGC#动作组控制
import time

class action:
    
    def __init__(self, speed = 10,move_distance = 30,
                 PID_Px = 1,PID_Ix = 0.08,PID_Dx = 0.16,
                 PID_Py = 1,PID_Iy = 0.08,PID_Dy = 0.16):
        self.__speed = speed
        self.__move_distance = move_distance
        self.__PID_Px = PID_Px
        self.__PID_Ix = PID_Ix#是微分项
        self.__PID_Dx = PID_Dx
        self.__PID_Py = PID_Py
        self.__PID_Iy = PID_Iy
        self.__PID_Dy = PID_Dy

        self.__integral_x = 0
        self.__integral_y = 0
        self.__prev_error_x = 0
        self.__prev_error_y = 0
        print('action init')
        pass

    def init_actions(self):#已经直接写了，不需要再调用
        AGC.runAction('init_actions')#初始化视角，可以正常看到识别的色块




    def alignment_PID_color(self, x, y, ik, width=320, high=395, range=5,
                           PID_Px=0.4,PID_Ix=0.1,PID_Dx=0.1,
                           PID_Py=0.4,PID_Iy=0.05,PID_Dy=0.15):
        speed = self.__speed
        move_distance = self.__move_distance
        # PID_Px = self.__PID_Px
        # PID_Ix = self.__PID_Ix
        # PID_Dx = self.__PID_Dx
        # PID_Py = self.__PID_Py
        # PID_Iy = self.__PID_Iy
        # PID_Dy = self.__PID_Dy
        x_reduce = width - range
        x_add = width + range
        y_reduce = high - range
        y_add = high + range
        if x is None or y is None:
            return False
        else:
            # 计算误差
            error_x = x - width
            error_y = y - high

            # 计算积分项
            self.__integral_x += error_x
            self.__integral_y += error_y
            #限制积分项的大小
            self.__integral_x = max(-100, min(100, self.__integral_x))
            self.__integral_y = max(-100, min(100, self.__integral_y))

            # 计算微分项
            derivative_x = error_x - self.__prev_error_x
            derivative_y = error_y - self.__prev_error_y

            # 计算控制量
            control_x = PID_Px * error_x + PID_Ix * self.__integral_x + PID_Dx * derivative_x
            control_y = PID_Py * error_y + PID_Iy * self.__integral_y + PID_Dy * derivative_y

            # 限制控制量的大小，在10-100之间
            # control_x = max(10, min(100, control_x))
            # control_y = max(10, min(100, control_y))
            # 限制控制量的大小，在10-100之间
            control_x = max(10, min(100, abs(control_x)))
            control_y = max(10, min(100, abs(control_y)))

            if x > x_add:
                ik.right_move(ik.initial_pos, 2, abs(control_x), speed, 1)
                time.sleep(0.03)
                return False
            elif x < x_reduce:
                ik.left_move(ik.initial_pos, 2, abs(control_x), speed, 1)
                time.sleep(0.03)
                return False
            if y > y_add:
                # print(control_y)
                print("y:",y)
                print("后退")
                ik.back(ik.initial_pos, 2, abs(control_y), speed, 1)
                time.sleep(0.03)
                return False
            elif y < y_reduce:
                # print(control_y)
                # print("y:",y)
                # print("前进")
                ik.go_forward(ik.initial_pos, 2, abs(control_y), speed, 1)
                time.sleep(0.03)
                return False
            if high+5< y and y < high-5 and x <width-5 and width+5 < x :
                AGC.runAction('init_actions')#初始化视角，可以正常看到识别的色块
                time.sleep(1)
                return True
            # 更新之前的误差
            self.__prev_error_x = error_x
            self.__prev_error_y = error_y
            # 休眠时间



    def alignment_PID_CUP(self, x, y, ik, width=320, high=395, range=5,
                      pid_px = 1,pid_ix = 0.08,pid_dx = 0.16,
                      pid_py = 1,pid_iy = 0.08,pid_dy = 0.16):
        speed = self.__speed
        move_distance = self.__move_distance
        # PID_Px = self.__PID_Px
        # PID_Ix = self.__PID_Ix
        # PID_Dx = self.__PID_Dx
        # PID_Py = self.__PID_Py
        # PID_Iy = self.__PID_Iy
        # PID_Dy = self.__PID_Dy
        PID_Px = pid_px
        PID_Ix = pid_ix
        PID_Dx = pid_dx
        PID_Py = pid_py
        PID_Iy = pid_iy
        PID_Dy = pid_dy
        x_reduce = width - range
        x_add = width + range
        y_reduce = high - range
        y_add = high + range
        if x is None or y is None:
            return False
        else:
            # 计算误差
            error_x = x - width
            error_y = y - high

            # 计算积分项
            self.__integral_x += error_x
            self.__integral_y += error_y
            #限制积分项的大小
            self.__integral_x = max(-100, min(100, self.__integral_x))
            self.__integral_y = max(-100, min(100, self.__integral_y))

            # 计算微分项
            derivative_x = error_x - self.__prev_error_x
            derivative_y = error_y - self.__prev_error_y

            # 计算控制量
            control_x = PID_Px * error_x + PID_Ix * self.__integral_x + PID_Dx * derivative_x
            control_y = PID_Py * error_y + PID_Iy * self.__integral_y + PID_Dy * derivative_y

            # 限制控制量的大小，在10-100之间
            # control_x = max(10, min(100, control_x))
            # control_y = max(10, min(100, control_y))
            # 限制控制量的大小，在10-100之间
            control_x = max(10, min(100, abs(control_x)))
            control_y = max(10, min(100, abs(control_y)))

            if x > x_add:
                ik.right_move(ik.initial_pos, 2, abs(control_x), speed, 1)
                time.sleep(0.03)
                return False
            elif x < x_reduce:
                ik.left_move(ik.initial_pos, 2, abs(control_x), speed, 1)
                time.sleep(0.03)
                return False
            if y > y_add:
                # print(control_y)
                print("y:",y)
                print("后退")
                ik.back(ik.initial_pos, 2, abs(control_y), speed, 1)
                time.sleep(0.03)
                return False
            elif y < y_reduce:
                # print(control_y)
                # print("y:",y)
                # print("前进")
                ik.go_forward(ik.initial_pos, 2, abs(control_y), speed, 1)
                time.sleep(0.03)
                return False
            if high+5< y and y < high-5 and x <width-5 and width+5 < x :
                AGC.runAction('init_actions')#初始化视角，可以正常看到识别的色块
                time.sleep(1)
                return True
            # 更新之前的误差
            self.__prev_error_x = error_x
            self.__prev_error_y = error_y
            # 休眠时间

    
    def alignment_PID_DOOR(self, x, y, ik, width=320, high=395, range=5,
                  pid_px = 1,pid_ix = 0.08,pid_dx = 0.16,
                  pid_py = 1,pid_iy = 0.08,pid_dy = 0.16):
        speed = self.__speed
        move_distance = self.__move_distance
        # PID_Px = self.__PID_Px
        # PID_Ix = self.__PID_Ix
        # PID_Dx = self.__PID_Dx
        # PID_Py = self.__PID_Py
        # PID_Iy = self.__PID_Iy
        # PID_Dy = self.__PID_Dy
        PID_Px = pid_px
        PID_Ix = pid_ix
        PID_Dx = pid_dx
        PID_Py = pid_py
        PID_Iy = pid_iy
        PID_Dy = pid_dy
        x_reduce = width - range
        x_add = width + range
        y_reduce = high - range
        y_add = high + range
        if x is None or y is None:
            return False
        else:
            # 计算误差
            error_x = x - width
            error_y = y - high
            # 计算积分项
            self.__integral_x += error_x
            self.__integral_y += error_y
            #限制积分项的大小
            self.__integral_x = max(-100, min(100, self.__integral_x))
            self.__integral_y = max(-100, min(100, self.__integral_y))
            # 计算微分项
            derivative_x = error_x - self.__prev_error_x
            derivative_y = error_y - self.__prev_error_y
            # 计算控制量
            control_x = PID_Px * error_x + PID_Ix * self.__integral_x + PID_Dx * derivative_x
            control_y = PID_Py * error_y + PID_Iy * self.__integral_y + PID_Dy * derivative_y
            # 限制控制量的大小，在10-100之间
            # control_x = max(10, min(100, control_x))
            # control_y = max(10, min(100, control_y))
            # 限制控制量的大小，在10-100之间
            control_x = max(10, min(100, abs(control_x)))
            control_y = max(10, min(100, abs(control_y)))
            if x > x_add:
                ik.right_move(ik.initial_pos, 2, abs(control_x), speed, 1)
                time.sleep(0.03)
                return False
            elif x < x_reduce:
                ik.left_move(ik.initial_pos, 2, abs(control_x), speed, 1)
                time.sleep(0.03)
                return False
            if y > y_add:
                # print(control_y)
                print("y:",y)
                print("后退")
                ik.back(ik.initial_pos, 2, abs(control_y), speed, 1)
                time.sleep(0.03)
                return False
            elif y < y_reduce:
                # print(control_y)
                # print("y:",y)
                # print("前进")
                ik.go_forward(ik.initial_pos, 2, abs(control_y), speed, 1)
                time.sleep(0.03)
                return False
            if high+5< y and y < high-5 and x <width-5 and width+5 < x :
                AGC.runAction('init_actions')#初始化视角，可以正常看到识别的色块
                time.sleep(1)
                return True
            # 更新之前的误差
            self.__prev_error_x = error_x
            self.__prev_error_y = error_y
            # 休眠时间



 #定义校准动作，传入xywh坐标
    def alignment_P(self, x,y,ik,width = 320, high = 395,range = 5):
        speed = self.__speed
        move_distance = self.__move_distance
        PID_P = self.__PID_P
        x_reduce = width-range
        x_add = width+range
        y_reduce = high-range
        y_add = high+range
        if x == None or y == None:
            return False
        else:        
            if x >x_add:
                Error = x - x_add
                control_x = PID_P * Error
                #限制控制量的大小，在10-100之间
                max(10, min(100, control_x))
                ik.right_move(ik.initial_pos, 2, control_x , speed, 1)
                time.sleep(0.03)
                return False
            elif x < x_reduce:
                Error = x_reduce - x
                control_x = PID_P * Error
                #限制控制量的大小，在10-100之间
                max(10, min(100, control_x))
                ik.left_move(ik.initial_pos, 2, control_x, speed, 1)  # 左移10mm
                time.sleep(0.03)
                return False
            if y > y_add:
                Error = y - y_add
                control_y = PID_P * Error
                #限制控制量的大小，在10-100之间
                control_y = max(10, min(100, control_y))
                ik.back(ik.initial_pos, 2, control_y, speed, 1)# 朝前直走10mm
                time.sleep(0.03)
                return False
            elif y < y_reduce:
                Error = y_reduce - y
                control_y = PID_P * Error
                #限制控制量的大小，在10-100之间
                control_y = max(10, min(100, control_y))
                ik.go_forward(ik.initial_pos, 2, control_y, speed, 1)  # 朝后直走10mm
                time.sleep(0.03)
                return False
            elif (high+5) < y and y < (high-5) and x < (width-5) and (width+5) < x :
                # AGC.runAction('init_actions')#初始化视角，可以正常看到识别的色块
                time.sleep(1)
                return True



    #定义校准动作，传入xywh坐标,不使用pid
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
                time.sleep(0.03)
                return False
            elif x < 315:
                Error = 315 - x
                # ik.turn_right(ik.initial_pos, 2, move_distance, speed, 1)
                ik.left_move(ik.initial_pos, 2, move_distance, speed, 1)  # 左移10mm
                time.sleep(0.03)
                return False
            if y > 400:
                Error = y - 400
                ik.back(ik.initial_pos, 2, move_distance, speed, 1)# 朝前直走10mm
                time.sleep(0.03)
                return False
            elif y < 390:
                Error = 390 - y
                ik.go_forward(ik.initial_pos, 2,move_distance, speed, 1)  # 朝后直走10mm
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
    def  Grab_correction_location(self, ik, distance = 1):
        speed = self.__speed
        for i in range(distance):
            ik.go_forward(ik.initial_pos, 2, 100, speed, 1)
            time.sleep(0.1)


    #抓取物体并返回到摄像头可以看到的位置
    def grap(self):#这里抓取的英文是grab，但是为了避免和函数名重复，所以改成了grap
        AGC.runAction('init_grap2')
        AGC.runAction('init_actions')#这个动作可以用于回到摄像头可以看到的位置，并且物体不会掉落

    def Step_back_and_place_the_cup(self, ik, distance_back = 1,distance_right = 3):
        speed = self.__speed
        for i in range(distance_back):
            ik.back(ik.initial_pos, 2, 80, speed, 1)
            time.sleep(0.1)
        for i in range(distance_right):
            ik.right_move(ik.initial_pos, 2, 80, speed, 1)
            time.sleep(0.1)

    def Step_back_and_enter_the_door(self, ik, distance_back = 1,distance_left = 3):
        speed = self.__speed
        for i in range(distance_back):
            ik.back(ik.initial_pos, 2, 80, speed, 1)
            time.sleep(0.1)
        for i in range(distance_left):
            ik.left_move(ik.initial_pos, 2, 80, speed, 1)
            time.sleep(0.1)

    def Entrance(self,distance = 1):
        for i in range(distance):
            AGC.runAction('forward_flutter')
        print("进门动作完成")


    def place(self):
        AGC.runAction('init_putdown')
        AGC.runAction('init_actions')

    def discard(self):
        AGC.runAction('init_discard')
        # AGC.runAction('init_actions')


if __name__ == '__main__':
    #打印对象的所有属性和方法
    print(dir(action))
