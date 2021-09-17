#!/usr/bin/env python3

import telnetlib
HOST = "192.168.29.105"
tn = telnetlib.Telnet(HOST,8888)
msg = "96,100,0\r"
tn.write(msg.encode("ascii"))
# for i in range(1,100):
#     tn.write(msg.encode("ascii"))
#     msg = "{},{},0".format(i,i)
#     print(msg)
#     a= input()
