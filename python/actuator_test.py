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

from dataclasses import dataclass

@dataclass
class SineParams:
  frq_Hz: float
  ampl_rad: float
  knee_gain: float
  duration: float

def robot_server_response_handler(channel, data):
  response = robot_server_response_lcmt.decode(data)
  print("received message on channel \"{}\": q[0]={}", channel, response.q[0])

lc = lcm.LCM()

tau_feedforward = [0,0,0]
kp = [20, 20, 20]
kd = [3, 3, 3]
cmd = robot_server_command_lcmt()
nan_cmd = robot_server_command_lcmt()
nan_cmd.q_des = [float("nan") for e in range(12)]
nan_cmd.qd_des = [float("nan") for e in range(12)]
nan_cmd.tau_ff = [float("nan") for e in range(12)]
# cmd.kp_joint = [0.1 for ii in range(12)]

# lc.publish("robot_server_command", cmd.encode())

subscription = lc.subscribe("robot_server_response", robot_server_response_handler)

# lc.publish("robot_server_command", cmd.encode())
iter = 1
start = time.time()
frq_Hz = 2.0
ampl_rad = 0.3
speed = -18
messaging_period = 1/500
use_vel_cmd = True
curr_time = 0

action1 = SineParams(1, 0.3, 1, 600)
action2 = SineParams(1, 0.3, -1, 0)
action3 = SineParams(2, 0.3, -2, 0)


try:
  while curr_time < 1:
    curr_time = time.time()-start
    lc.publish("robot_server_command", nan_cmd.encode())
    print("sending nans; time = ", curr_time)
except KeyboardInterrupt:
  pass

actions = [action1, action2, action3]

try:

  for ii in range(len(actions)):
    ca = actions[ii]

    start = time.time()
    curr_time = 0

    while curr_time < ca.duration:
      # lc.handle_timeout(10)

      curr_time = time.time()-start
      trig_time = 2*math.pi*curr_time*ca.frq_Hz
      chain_rule =  2*math.pi*ca.frq_Hz
      pos_hip =       ca.ampl_rad*math.sin(trig_time)
      pos_hip = 0
      pos_shoulder =  ca.ampl_rad*math.sin(trig_time)
      pos_knee =      ca.knee_gain*ca.ampl_rad*math.sin(trig_time)
      pos = [pos_hip, pos_shoulder, pos_knee]

      dpos_hip =       ca.ampl_rad*chain_rule*math.cos(trig_time)
      dpos_hip = 0
      dpos_shoulder =  ca.ampl_rad*chain_rule*math.cos(trig_time)
      dpos_knee =      ca.knee_gain*ca.ampl_rad*chain_rule*math.cos(trig_time)
      pos = [pos_hip, pos_shoulder, pos_knee]
      dpos = [dpos_hip, dpos_shoulder, dpos_knee]
      pos = [float("nan"),float("nan"),float("nan")]
      dpos = [speed, speed, speed]
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
lc.publish("robot_server_command", nan_cmd.encode())
lc.publish("robot_server_command", nan_cmd.encode())
lc.unsubscribe(subscription)
