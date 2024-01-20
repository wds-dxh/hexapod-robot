import cv2
import time

def record_video():
    # 打开摄像头
    cap = cv2.VideoCapture(0)

    # 设置视频编解码器和帧率
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out = cv2.VideoWriter('save_video/小车.avi', fourcc, 20.0, (640, 480))

    start_time = time.time()
    recording = False

    while True:
        # 读取一帧
        ret, frame = cap.read()

        # 检查是否成功读取帧
        if not ret:
            print("无法读取视频帧")
            break

        # 显示当前帧
        cv2.imshow('Frame', frame)

        # 检查是否按下 's' 键，开始录制
        key = cv2.waitKey(1) & 0xFF
        if key == ord('s'):
            if not recording:
                print("开始录制")
                start_time = time.time()
                recording = True
        if key == ord('p'):
            if recording:
                print("暂停录制")
                recording = False
                break
        elif key == 27:  # 按下 'Esc' 键退出
            break




        # 如果正在录制，则将当前帧写入输出视频
        if recording:
            out.write(frame)

        # 每隔一秒打印时间
        if recording and time.time() - start_time >= 1:
            print("已录制时间: {:.1f}秒".format(time.time() - start_time))
            start_time = time.time()

    # 释放资源
    cap.release()
    out.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    record_video()
