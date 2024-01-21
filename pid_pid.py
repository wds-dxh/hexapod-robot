import time


class YourClass:
    def __init__(self, speed=10, move_distance=30, PID_P=1, PID_I=0.1, PID_D=0.1):
        self.__speed = speed
        self.__move_distance = move_distance
        self.__PID_P = PID_P
        self.__PID_I = PID_I
        self.__PID_D = PID_D
        self.__integral_x = 0
        self.__integral_y = 0
        self.__prev_error_x = 0
        self.__prev_error_y = 0
        print('action init')
        pass

    def init_actions(self):  # 已经直接写了，不需要再调用
        AGC.runAction('init_actions')  # 初始化视角，可以正常看到识别的色块

    # 定义校准动作，传入xywh坐标
    def alignment_PID(self, x, y, ik, width=320, high=395, range=5):
        speed = self.__speed
        move_distance = self.__move_distance
        PID_P = self.__PID_P
        PID_I = self.__PID_I
        PID_D = self.__PID_D
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

            # 计算微分项
            derivative_x = error_x - self.__prev_error_x
            derivative_y = error_y - self.__prev_error_y

            # 计算控制量
            control_x = PID_P * error_x + PID_I * self.__integral_x + PID_D * derivative_x
            control_y = PID_P * error_y + PID_I * self.__integral_y + PID_D * derivative_y

            # 限制控制量的大小，在10-100之间
            control_x = max(10, min(100, control_x))
            control_y = max(10, min(100, control_y))

            # 应用控制量
            if x > x_add:
                ik.right_move(ik.initial_pos, 2, control_x, speed, 1)
                return False
            elif x < x_reduce:
                ik.left_move(ik.initial_pos, 2, control_x, speed, 1)
                return False
            if y > y_add:
                ik.back(ik.initial_pos, 2, control_y, speed, 1)
                return False
            elif y < y_reduce:
                ik.go_forward(ik.initial_pos, 2, control_y, speed, 1)
                return False
            elif high+5< y and y < high-5 and x <width-5 and width+5 < x :
                AGC.runAction('init_actions')#初始化视角，可以正常看到识别的色块
                time.sleep(1)
                return True
            # 更新之前的误差
            self.__prev_error_x = error_x
            self.__prev_error_y = error_y
            # 休眠时间
            time.sleep(0.03)


