#!/usr/bin/env python3

# import telnetlib
# HOST = "192.168.29.80"
# tn = telnetlib.Telnet(HOST,8888)
# msg = "255,255,0\r"
# tn.write(msg.encode("ascii"))
# # for i in range(1,100):
#     tn.write(msg.encode("ascii"))
#     msg = "{},{},0".format(i,i)
#     print(msg)
#     a= input()
import websocket
ws= websocket.WebSocket()
ws.connect("ws://192.168.29.105:8888")
msg = "220,0,500"
# for i in range(85,120):
#     msg = "{},{},0".format(i,i)
#     print(msg)
#     ws.send(msg)
#     ws.recv()
#     k=input()
ws.send(msg)
k=input()
ws.close()
