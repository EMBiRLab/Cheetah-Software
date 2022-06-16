import lcm
import numpy as np
import sys
sys.path.append('../')
sys.path.append('./lcm-types/python/')
# from gamepad_lcmt import gamepad_lcmt
from leg_control_command_lcmt import leg_control_command_lcmt
from leg_control_data_lcmt import leg_control_data_lcmt

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
ctrl_cmd = leg_control_command_lcmt()
ctrl_dat = leg_control_data_lcmt()



# lc.publish("robot_server_response", cmd.encode())

# subscription = lc.subscribe("robot_server_response", robot_server_response_handler)

ii = 0
try:
  while True:

    for i in range(12):
      ctrl_cmd.tau_ff[i] = ii + 0
      ctrl_cmd.f_ff[i] = ii + 1
      ctrl_cmd.q_des[i] = ii + 2
      ctrl_cmd.qd_des[i] = ii + 3
      ctrl_cmd.p_des[i] = ii + 4
      ctrl_cmd.v_des[i] = ii + 5
      ctrl_dat.q[i] = ii + 6
      ctrl_dat.qd[i] = ii + 7
      ctrl_dat.p[i] = ii + 8
      ctrl_dat.v[i] = ii + 9
      ctrl_dat.tau_est[i] = ii + 10

    lc.publish("leg_control_command", ctrl_cmd.encode())
    lc.publish("leg_control_data", ctrl_dat.encode())


    ii = ii + 1

    print("publishing")

except KeyboardInterrupt:
  pass

