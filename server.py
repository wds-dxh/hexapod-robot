# 获取本地主机名和端口号
import json
import socket
host = socket.gethostname()
port = 8888
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind((host, port))
server_socket.listen(1)  # 参数表示最大等待连接数
print(f"等待连接在 {host}:{port}...")
def recv_data():
    client_socket, addr = server_socket.accept()
    print(f"连接来自 {addr}")
    while True:
        data = client_socket.recv(1024)  # 接收数据
        print(f"接收到的数据: {data.decode()}")  # 输出接收到的数据
        if not data:
            print("未收到数据")
    client_socket.close()  # 关闭连接

def get_color(detect_color, xywh, cls):
    # global xywh
    # global cls
    min_x_value = 640
    if xywh != None and cls != None:
        for i in range(len(cls)):
            # print(cls[i])
            if cls[i] == detect_color:
                if xywh[i][0] < min_x_value:
                    need_xywh = xywh[i][0]
                    return need_xywh

def recv_data_all():
    client_socket, addr = server_socket.accept()
    print(f"连接来自 {addr}")
    while True:
        received_data = client_socket.recv(1024).decode()
        parsed_data = json.loads(received_data)
        xywh = parsed_data["xywh"]
        cls = parsed_data["cls"]
        # print(xywh[0])
        # print(cls[0])
        print(get_color(0, xywh, cls))
        # return xywh, cls
    client_socket.close()  # 关闭连接



if __name__ == '__main__':
    recv_data_all()