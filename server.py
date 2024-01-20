# 获取本地主机名和端口号


import socket
host = socket.gethostname()
port = 8888
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind((host, port))
server_socket.listen(1)  # 参数表示最大等待连接数
print(f"等待连接在 {host}:{port}...")
if __name__ == '__main__':
    client_socket, addr = server_socket.accept()
    print(f"连接来自 {addr}")
    while True:
        data = client_socket.recv(1024)  # 接收数据
        print(f"接收到的数据: {data.decode()}")
        #如果data为空
        if not data:
            break
    client_socket.close()  # 关闭连接
