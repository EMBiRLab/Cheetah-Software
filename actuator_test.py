import lcm
import numpy as np
import sys
import time
import math
# sys.path.append('../')
sys.path.append('lcm-types/python/')
from robot_server_command_lcmt import robot_server_command_lcmt
from robot_server_response_lcmt import robot_server_response_lcmt
# import robot_server_command_lcmt.py
# import ../lcm-types/python/robot_server_response_lcmt.py
# import robot_server_response_lcmt.py

def robot_server_response_handler(channel, data):
  response = robot_server_response_lcmt.decode(data)
  print("received message on channel \"{}\": q[0]={}", channel, response.q[0])

lc = lcm.LCM()

tau_feedforward = [0,0,0]
kp = [20, 20, 20]
kd = [3, 3, 3]
cmd = robot_server_command_lcmt()
# cmd.kp_joint = [0.1 for ii in range(12)]

# lc.publish("robot_server_command", cmd.encode())

subscription = lc.subscribe("robot_server_response", robot_server_response_handler)

# lc.publish("robot_server_command", cmd.encode())
iter = 1
start = time.time()
frq = 1
speed = 5
try:
  while True:
    # lc.handle_timeout(10)

    curr_time = time.time()-start
    pos_hip =       .2*math.sin(2*math.pi*curr_time*frq)
    pos_shoulder =  .2*math.sin(2*math.pi*curr_time*frq)
    pos_knee =      .2*math.sin(2*math.pi*curr_time*frq)
    pos = [pos_hip, pos_shoulder, pos_knee]

    dpos_hip =       2*math.pi*frq*.2*math.cos(2*math.pi*curr_time*frq)
    dpos_shoulder =  2*math.pi*frq*.2*math.cos(2*math.pi*curr_time*frq)
    dpos_knee =      2*math.pi*frq*.2*math.cos(2*math.pi*curr_time*frq)
    dpos = [dpos_hip, dpos_shoulder, dpos_knee]
    pos = [float("nan"),float("nan"),float("nan")]
    dpos = [speed,speed,speed]

    for leg in range(4):
        cmd.tau_ff[3*leg:3*(leg+1)] = tau_feedforward      
        cmd.q_des[3*leg:3*(leg+1)] = pos
        cmd.qd_des[3*leg:3*(leg+1)] = dpos
        cmd.kp_joint[3*leg:3*(leg+1)] = kp
        cmd.kd_joint[3*leg:3*(leg+1)] = kd



    iter = iter + 1
    lc.publish("robot_server_command", cmd.encode())
    # print("publishing")
    print("sending over positions:", pos, "and vels:", dpos)

    time.sleep(0.05)
except KeyboardInterrupt:
  pass

lc.unsubscribe(subscription)
