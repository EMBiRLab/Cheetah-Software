import lcm
import numpy as np
import sys
sys.path.append('../')
sys.path.append('./lcm-types/python/')
from robot_server_command_lcmt import robot_server_command_lcmt
from robot_server_response_lcmt import robot_server_response_lcmt


def my_handler(channel, data):
    msg = robot_server_command_lcmt.decode(data)
    print("Received message on channel \"%s\"" % channel)
    for i in range(12):
      print("tau_ff", i, " = ",str(msg.tau_ff[i]))
      print("q_des", i, " = ",str(msg.q_des[i]))
      print("qd_des", i, " = ",str(msg.qd_des[i]))
      print("kp_joint", i, " = ",str(msg.kp_joint[i]))
      print("kd_joint", i, " = ",str(msg.kd_joint[i]))


def my_handler2(channel, data):
    msg = robot_server_response_lcmt.decode(data)
    print("Received message on channel \"%s\"" % channel)
    for i in range(12):
    #   print("tau_ff", i, " = ",str(msg.tau_ff[i]))
      print("q", i, " = ",str(msg.q[i]))
      print("tau_est", i, " = ",str(msg.tau_est[i]))
      print("qd", i, " = ",str(msg.qd[i]))
    #   print("kd_joint", i, " = ",str(msg.kd_joint[i]))
    
lc = lcm.LCM()
subscription = lc.subscribe("robot_server_command", my_handler)
lc1 = lcm.LCM()
subscription2 = lc1.subscribe("robot_server_response", my_handler2)
try:
    while True:
        lc.handle()
        # lc1.handle()
except KeyboardInterrupt:
    pass