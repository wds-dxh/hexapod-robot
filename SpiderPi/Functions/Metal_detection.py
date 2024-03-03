import RPi.GPIO as GPIO
import time

# 秒级延迟
class Metal_detection():
    def __init__(self):
        self.read_pin = 18
        # 设置GPIO口为输出
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.read_pin, GPIO.IN)
        # GPIO.setmode(gpio.BOARD)
    #定义析构函数
    def __del__(self):
        GPIO.cleanup()

    def IS_OR_NOT_Metal(self):
        if GPIO.input(self.read_pin) == 1:
            print("没有有金属")
            return False
        else: 
            print("有金属")
            return True
        # 读取GPIO口电平
            # return gpio.input(self.read_pin)

if __name__ == '__main__':
    metal_detection = Metal_detection()
    while True:
        time.sleep(0.5)
        metal_detection.IS_OR_NOT_Metal()