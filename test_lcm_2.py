import lcm
import numpy as np
import sys
import time
import random
import math
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
# subscription = lc.subscribe("robot_server_command", my_handler)
lc1 = lcm.LCM()
# subscription2 = lc1.subscribe("robot_server_response", my_handler2)
cmd = robot_server_command_lcmt()
desired_q = [-0.05, -0.8, 1.7, 0.05, -0.8, 1.7, -0.05, -0.8, 1.7, 0.05, -0.8, 1.7]
start = time.time()

try:
    while True:
        t = time.time() - start
        lc.handle_timeout(10)
        # lc1.handle()
        for i in range(12):
          cmd.q_des[i] = float("nan") #desired_q[i] + 0.01*(random.random()-.5)
          cmd.qd_des[i] = float("nan") #0.01*(random.random()-.5)
          # if i == 7:
          #   cmd.q_des[i] = 0.0
          #   cmd.qd_des[i] = 0.0
          if i==2:
            cmd.tau_ff[i] = (1*math.sin(2*math.pi*t*0.5)-1) #1*(random.random()-.5)
          else:
            cmd.tau_ff[i] = 0.0

          # print("q",i ,"=", cmd.q[i])
        # cmd.fsm_state = 0
        lc.publish("robot_server_command", cmd.encode())
        # if np.mod(ii,2) == 0:
        # print("publishing")
except KeyboardInterrupt:
    pass
