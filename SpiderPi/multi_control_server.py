#!/usr/bin/env python3

# websocket中继广播服务器， 接受up连接的数据 发送到 down连接
# ==============================================================================
import asyncio
import websockets

down_clients = set()  # 已连接的下行客户端

async def broadcaster(socket: websockets.WebSocketClientProtocol, url_path):
    connected = server.ws_server.websockets
    if url_path.endswith('down'):  # 下行连接管理
        down_clients.add(socket)
        await socket.wait_closed()
        down_clients.remove(socket)
    elif url_path.endswith('up'):  # 接收上行数据并转发
        async for msg in socket:
            for ws in connected:
                await ws.send(msg)
    else:
        pass

server = websockets.serve(broadcaster, '0.0.0.0', 7788)
asyncio.get_event_loop().run_until_complete(server)
asyncio.get_event_loop().run_forever()

