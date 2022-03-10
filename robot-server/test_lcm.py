import lcm
import numpy as np
import sys
sys.path.append('../')
sys.path.append('../lcm-types/python/')
from robot_server_command_lcmt import robot_server_command_lcmt
from robot_server_response_lcmt import robot_server_response_lcmt
# import robot_server_command_lcmt.py
# import ../lcm-types/python/robot_server_response_lcmt.py
# import robot_server_response_lcmt.py

def robot_server_response_handler(channel, data):
  response = robot_server_response_lcmt.decode(data)
  print("received message on channel \"{}\": q[0]={}", channel, response.q[0])

lc = lcm.LCM()

cmd = robot_server_command_lcmt()
# cmd.kp_joint = [0.1 for ii in range(12)]
cmd.q_des[0] = float("nan")
cmd.qd_des[0] = 0.1

lc.publish("robot_server_command", cmd.encode())

subscription = lc.subscribe("robot_server_response", robot_server_response_handler)

lc.publish("robot_server_command", cmd.encode())
ii = 0
try:
  while True:
    lc.handle()
    ii = ii + 1
    if np.mod(ii,2) == 0:
      lc.publish("robot_server_command", cmd.encode())
      print("publishing")
except KeyboardInterrupt:
  pass

lc.unsubscribe(subscription)