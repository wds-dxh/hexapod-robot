import socket


# 获取服务器的主机名和端口号
host = socket.gethostname()
port = 8888
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect((host, port))# 连接服务器
message = "Hello, 树莓派!"
client_socket.send(message.encode())
client_socket.close()
