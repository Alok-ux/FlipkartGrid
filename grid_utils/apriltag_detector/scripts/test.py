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
ws.connect("ws://192.168.29.232:8888")
# msg = "150,0,0"
for i in range(0,3):
    if i==0:
        msg = "-120,-110,0"
    elif i==1:
        msg = "0,0,0"
    else:
        msg = "0,0,1"
    print(msg)
    ws.send(msg)
    k=input()
# msg = "0,0,1"
# ws.send(msg)
# ws.recv()
# k=input()
# ws.send(msg)
k=input()
ws.close()
