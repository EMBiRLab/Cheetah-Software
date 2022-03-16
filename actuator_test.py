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
frq_Hz = 1.0
ampl_rad = 0.5
speed = 5
messaging_period = 0.0025
use_vel_cmd = True
curr_time = 0
try:
  while curr_time < 1.25:
    # lc.handle_timeout(10)

    curr_time = time.time()-start
    trig_time = 2*math.pi*curr_time*frq_Hz
    chain_rule =  2*math.pi*frq_Hz
    pos_hip =       ampl_rad*math.sin(trig_time)
    pos_shoulder =  ampl_rad*math.sin(trig_time)
    pos_knee =      ampl_rad*math.sin(trig_time)
    pos = [pos_hip, pos_shoulder, pos_knee]

    dpos_hip =       ampl_rad*chain_rule*math.cos(trig_time)
    dpos_shoulder =  ampl_rad*chain_rule*math.cos(trig_time)
    dpos_knee =      ampl_rad*chain_rule*math.cos(trig_time)
    pos = [pos_hip, pos_shoulder, pos_knee]
    dpos = [dpos_hip, dpos_shoulder, dpos_knee]
    #pos = [float("nan"),float("nan"),float("nan")]
    #dpos = [speed, speed, speed]
    if not use_vel_cmd:
        dpos = [0,0,0]

    for leg in range(4):
        cmd.tau_ff[3*leg:3*(leg+1)] = tau_feedforward
        cmd.q_des[3*leg:3*(leg+1)] = pos
        cmd.qd_des[3*leg:3*(leg+1)] = dpos
        cmd.kp_joint[3*leg:3*(leg+1)] = kp
        cmd.kd_joint[3*leg:3*(leg+1)] = kd


    iter = iter + 1
    lc.publish("robot_server_command", cmd.encode())
    # print("publishing")
    print("t = ", round(curr_time, 2), ", cmd pos = ", [round(e,2) for e in pos], ", cmd vel = ", [round(e,2) for e in dpos])

    time.sleep(messaging_period)
except KeyboardInterrupt:
    cmd.tau_ff = [0 for e in range(12)]
    cmd.q_des = [float("nan") for e in range(12)]
    cmd.qd_des = [0 for e in range(12)]
    lc.publish("robot_server_command", cmd.encode())
    pass

cmd.tau_ff = [0 for e in range(12)]
cmd.q_des = [float("nan") for e in range(12)]
cmd.qd_des = [0 for e in range(12)]
lc.publish("robot_server_command", cmd.encode())
lc.publish("robot_server_command", cmd.encode())
lc.unsubscribe(subscription)
