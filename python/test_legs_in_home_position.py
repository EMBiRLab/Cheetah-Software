# from cmath import nan
import lcm
import numpy as np
import sys
import random
sys.path.append('../')
sys.path.append('./lcm-types/python/')
from robot_server_command_lcmt import robot_server_command_lcmt
from robot_server_response_lcmt import robot_server_response_lcmt


def my_cmd_handler(channel, data):
    msg = robot_server_command_lcmt.decode(data)
    print("Received message on channel \"%s\"" % channel)
    for i in range(12):
      print("tau_ff", i, " = ",str(msg.tau_ff[i]))
      print("q_des", i, " = ",str(msg.q_des[i]))
      print("qd_des", i, " = ",str(msg.qd_des[i]))
      print("kp_joint", i, " = ",str(msg.kp_joint[i]))
      print("kd_joint", i, " = ",str(msg.kd_joint[i]))


def my_response_handler(channel, data):
    msg = robot_server_response_lcmt.decode(data)
    print("Received message on channel \"%s\"" % channel)
    for i in range(12):
    #   print("tau_ff", i, " = ",str(msg.tau_ff[i]))
      print("q", i, " = ",str(msg.q[i]))
      print("tau_est", i, " = ",str(msg.tau_est[i]))
      print("qd", i, " = ",str(msg.qd[i]))
    #   print("kd_joint", i, " = ",str(msg.kd_joint[i]))
    
lc = lcm.LCM()
# robservcommand = lc.subscribe("robot_server_command", my_cmd_handler)
lc1 = lcm.LCM()
robservresponse = lc1.subscribe("robot_server_response", my_response_handler)

cmd = robot_server_command_lcmt()
desired_q   = [0.05, -0.5*.7, 1.0*.7, 0.05, 0.5*.7, -1.0*.7, 0.05, -0.5*.7, 1.0*.7, 0.05, 0.5*.7, -1.0*.7]
desired_qd  = [0.0,  0.0,  0.0, 0.0,  0.0,  0.0, 0.0,  0.0,  0.0, 0.0,  0.0,  0.0]
desired_tau = [0.0,  0.0,  0.0, 0.0,  0.0,  0.0, 0.0,  0.0,  0.0, 0.0,  0.0,  0.0]

# desired_q   = [float("nan"), float("nan"), float("nan"), float("nan"), float("nan"), float("nan"), float("nan"), float("nan"), float("nan"), 0.05, 0.5*.7, -1.0*.7]
# desired_qd  = [float("nan"), float("nan"), float("nan"), float("nan"), float("nan"), float("nan"), float("nan"), float("nan"), float("nan"), 0.0,  0.0,  0.0]
# desired_tau = [float("nan"), float("nan"), float("nan"), float("nan"), float("nan"), float("nan"), float("nan"), float("nan"), float("nan"), 0.0,  0.0,  0.0]

try:
    while True:
        lc.handle_timeout(10)
        lc1.handle_timeout(10)
        
        for i in range(12):
          cmd.q_des[i] = desired_q[i]
          cmd.qd_des[i] = desired_qd[i]
          cmd.tau_ff[i] = desired_tau[i]
          cmd.kp_joint[i] = 0
          cmd.kd_joint[i] = 0

        print("q_des 3 = ",str(cmd.q_des[3]))
        lc.publish("robot_server_command", cmd.encode())
        
except KeyboardInterrupt:
    pass