import lcm
import numpy as np
import sys
sys.path.append('../')
sys.path.append('./lcm-types/python/')
from gamepad_lcmt import gamepad_lcmt
from robot_server_response_lcmt import robot_server_response_lcmt
# import robot_server_command_lcmt.py
# import ../lcm-types/python/robot_server_response_lcmt.py
# import robot_server_response_lcmt.py

# def robot_server_response_handler(channel, data):
#   response = robot_server_response_lcmt.decode(data)
#   print("received message on channel \"{}\": q[0]={}", channel, response.q[0])

lc = lcm.LCM()

# cmd = gamepad_lcmt()
# cmd.kp_joint = [0.1 for ii in range(12)]
# cmd.leftBumper = 0
# cmd.rightBumper = 0
# cmd.leftTriggerButton = 0
# cmd.rightTriggerButton = 0
# cmd.back = 0
# cmd.start = 0
# cmd.a = 0
# cmd.x = 0
# cmd.b = 0
# cmd.y = 0
# cmd.leftStickButton = 0
# cmd.rightStickButton = 0
# cmd.leftTriggerAnalog = 0
# cmd.rightTriggerAnalog = 0
# for i in range(0,2):
#   cmd.leftStickAnalog[i] = 1
#   cmd.rightStickAnalog[i] = 1
cmd = robot_server_response_lcmt()
for i in range(12):
  cmd.q[i] = 10
  cmd.qd[i] = 10
  cmd.tau_est[i] = 10
cmd.fsm_state = 1


# lc.publish("robot_server_response", cmd.encode())

# subscription = lc.subscribe("robot_server_response", robot_server_response_handler)

lc.publish("robot_server_response", cmd.encode())
# ii = 0
try:
  while True:
    print("yes")
    lc.handle_timeout(100)
    # ii = ii + 1
    # if np.mod(ii,2) == 0:
    lc.publish("robot_server_response", cmd.encode())
    print("publishing")
except KeyboardInterrupt:
  pass

# lc.unsubscribe(subscription)