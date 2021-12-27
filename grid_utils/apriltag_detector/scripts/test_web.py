#!/usr/bin/env python
import websocket
ws= websocket.WebSocket()
ws.connect("ws://192.168.29.244:8888")
msg = "0,0,1"
# for i in range(85,120):
#     msg = "{},{},0".format(i,i)
#     print(msg)
#     ws.send(msg)
#     ws.recv()
#     k=input()
ws.send(msg)
k=input()
ws.close()