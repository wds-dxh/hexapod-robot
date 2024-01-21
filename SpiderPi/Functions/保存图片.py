import cv2
import os
import argparse

if not os.path.exists('images'):
    os.mkdir('images')

# 解析命令行参数
parser = argparse.ArgumentParser()
parser.add_argument('--img', type=str, default='640 480', help='Image width and height (e.g., "width height")')
args = parser.parse_args()

img_size = args.img.split()     # 以空格分割输入的宽高
if len(img_size) != 2:          # 如果输入的宽高不是两个数
    print("Error: Invalid input for image size. Please provide width and height separated by a space.")
    exit()

video_stream_url = 'http://172.20.10.4:5000/video_feed'
cap = cv2.VideoCapture(0)
# frame_width = 640
# frame_height = 640
frame_width = int(img_size[0])
frame_height = int(img_size[1])

cap.set(3, frame_width)
cap.set(4, frame_height)

frame_count = 3295
save_image = False

while True:
    ret, frame = cap.read()
    if not ret:
        break
    #cv2.putText(frame, f'FPS: {cap.get(cv2.CAP_PROP_FPS)}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    cv2.imshow('Video', frame)

    if cv2.waitKey(1) & 0xFF == ord('s'):
        save_image = True

    if save_image:
        image_filename = os.path.join('images', f'image_{frame_count:04d}.jpg')
        cv2.imwrite(image_filename, frame)
        print(f"Saved: {image_filename}")  # 打印保存的文件名
        frame_count += 1
        save_image = False

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
