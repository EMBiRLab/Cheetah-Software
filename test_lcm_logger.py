import lcm
import numpy as np
import sys
import random
import csv
sys.path.append('../')
sys.path.append('./lcm-types/python/')
from leg_control_command_lcmt import leg_control_command_lcmt
from leg_control_data_lcmt import leg_control_data_lcmt
from state_estimator_lcmt import state_estimator_lcmt


"""
struct leg_control_command_lcmt {
    float tau_ff[12];
    float f_ff[12];
    float q_des[12];
    float qd_des[12];
    float p_des[12];
    float v_des[12];
    float kp_cartesian[12];
    float kd_cartesian[12];
    float kp_joint[12];
    float kd_joint[12];
}
"""
def ctrl_cmd_handler(channel, data):
    msg = leg_control_command_lcmt.decode(data)
    
    # Fill in the corresponding section of buffer and set flag
    global ctrl_cmd_ready
    global buffer

    buffer[0:11]  = msg.tau_ff
    buffer[12:23] = msg.f_ff
    buffer[24:35] = msg.q_des
    buffer[36:47] = msg.qd_des
    buffer[48:59] = msg.p_des
    buffer[60:71] = msg.v_des

    ctrl_cmd_ready = True

"""
struct leg_control_data_lcmt {
    float q[12];
    float qd[12];
    float p[12];
    float v[12];
    float tau_est[12];
}
"""
def ctrl_data_handler(channel, data):
    msg = leg_control_data_lcmt.decode(data)

    # Fill in corresponding section of buffer and set flag
    global ctrl_data_ready
    global buffer

    buffer[72:83]   = msg.q
    buffer[84:95]   = msg.qd
    buffer[96:107]  = msg.p
    buffer[108:119] = msg.v
    buffer[120:131] = msg.tau_est

    ctrl_data_ready = True



"""
struct state_estimator_lcmt {
    float p[3];
    float vWorld[3];
    float vBody[3];
    float rpy[3];
    float omegaBody[3];
    float omegaWorld[3];
    float quat[4];
}
"""
def state_estim_handler(channel, data):
    msg = state_estimator_lcmt.decode(data)
    # do nothing for now

buffer = [0 for x in range(132)]
header = ['cmd_tau_ff[0]','cmd_tau_ff[1]','cmd_tau_ff[2]','cmd_tau_ff[3]','cmd_tau_ff[4]','cmd_tau_ff[5]','cmd_tau_ff[6]','cmd_tau_ff[7]','cmd_tau_ff[8]','cmd_tau_ff[9]','cmd_tau_ff[10]','cmd_tau_ff[11]',
          'cmd_f_ff[0]','cmd_f_ff[1]','cmd_f_ff[2]','cmd_f_ff[3]','cmd_f_ff[4]','cmd_f_ff[5]','cmd_f_ff[6]','cmd_f_ff[7]','cmd_f_ff[8]','cmd_f_ff[9]','cmd_f_ff[10]','cmd_f_ff[11]',
          'cmd_q_des[0]','cmd_q_des[1]','cmd_q_des[2]','cmd_q_des[3]','cmd_q_des[4]','cmd_q_des[5]','cmd_q_des[6]','cmd_q_des[7]','cmd_q_des[8]','cmd_q_des[9]','cmd_q_des[10]','cmd_q_des[11]',
          'cmd_qd_des[0]','cmd_qd_des[1]','cmd_qd_des[2]','cmd_qd_des[3]','cmd_qd_des[4]','cmd_qd_des[5]','cmd_qd_des[6]','cmd_qd_des[7]','cmd_qd_des[8]','cmd_qd_des[9]','cmd_qd_des[10]','cmd_qd_des[11]',
          'cmd_p_des[0]','cmd_p_des[1]','cmd_p_des[2]','cmd_p_des[3]','cmd_p_des[4]','cmd_p_des[5]','cmd_p_des[6]','cmd_p_des[7]','cmd_p_des[8]','cmd_p_des[9]','cmd_p_des[10]','cmd_p_des[11]',
          'cmd_v_des[0]','cmd_v_des[1]','cmd_v_des[2]','cmd_v_des[3]','cmd_v_des[4]','cmd_v_des[5]','cmd_v_des[6]','cmd_v_des[7]','cmd_v_des[8]','cmd_v_des[9]','cmd_v_des[10]','cmd_v_des[11]',
          'data_q[0]','data_q[1]','data_q[2]','data_q[3]','data_q[4]','data_q[5]','data_q[6]','data_q[7]','data_q[8]','data_q[9]','data_q[10]','data_q[11]',
          'data_qd[0]','data_qd[1]','data_qd[2]','data_qd[3]','data_qd[4]','data_qd[5]','data_qd[6]','data_qd[7]','data_qd[8]','data_qd[9]','data_qd[10]','data_qd[11]',
          'data_p[0]','data_p[1]','data_p[2]','data_p[3]','data_p[4]','data_p[5]','data_p[6]','data_p[7]','data_p[8]','data_p[9]','data_p[10]','data_p[11]',
          'data_v[0]','data_v[1]','data_v[2]','data_v[3]','data_v[4]','data_v[5]','data_v[6]','data_v[7]','data_v[8]','data_v[9]','data_v[10]','data_v[11]',
          'data_tau_est[0]','data_tau_est[1]','data_tau_est[2]','data_tau_est[3]','data_tau_est[4]','data_tau_est[5]','data_tau_est[6]','data_tau_est[7]','data_tau_est[8]','data_tau_est[9]','data_tau_est[10]','data_tau_est[11]']
          
f = open('test1.csv','w')
writer = csv.writer(f)
writer.writerow(header)

ctrl_cmd_ready = False
ctrl_data_ready = False

lc = lcm.LCM()
subscription = lc.subscribe("leg_control_command", ctrl_cmd_handler)
subscription = lc.subscribe("leg_control_data", ctrl_data_handler)
# subscription = lc.subscribe("state_estimator", state_estim_handler)

iter = 0
try:
    while True:
        lc.handle_timeout(10)
        
        if ctrl_cmd_ready and ctrl_data_ready:
          writer.writerow(buffer)
          ctrl_cmd_ready = False
          ctrl_data_ready = False

        if iter % 300 == 0:
            print("logging is still alive")

        iter += 1
except KeyboardInterrupt:
    f.close()