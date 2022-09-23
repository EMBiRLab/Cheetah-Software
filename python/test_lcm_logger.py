# from ctypes.wintypes import HANDLE
import lcm
import numpy as np
import sys
import random
import csv
import time
sys.path.append('../')
sys.path.append('../lcm-types/python/')
from leg_control_command_lcmt import leg_control_command_lcmt
from leg_control_data_lcmt import leg_control_data_lcmt
from state_estimator_lcmt import state_estimator_lcmt
from robot_server_response_lcmt import robot_server_response_lcmt
from mpc_state_data_t import mpc_state_data_t

class Handler:

    def __init__(self):
        # self.buffer = [0 for x in range(413)]
        self.buffer = [0 for x in range(353)]
        self.header = ['cmd_tau_ff_0','cmd_tau_ff_1','cmd_tau_ff_2','cmd_tau_ff_3','cmd_tau_ff_4','cmd_tau_ff_5','cmd_tau_ff_6','cmd_tau_ff_7','cmd_tau_ff_8','cmd_tau_ff_9','cmd_tau_ff_10','cmd_tau_ff_11',
                'cmd_f_ff_0','cmd_f_ff_1','cmd_f_ff_2','cmd_f_ff_3','cmd_f_ff_4','cmd_f_ff_5','cmd_f_ff_6','cmd_f_ff_7','cmd_f_ff_8','cmd_f_ff_9','cmd_f_ff_10','cmd_f_ff_11',
                'cmd_q_des_0','cmd_q_des_1','cmd_q_des_2','cmd_q_des_3','cmd_q_des_4','cmd_q_des_5','cmd_q_des_6','cmd_q_des_7','cmd_q_des_8','cmd_q_des_9','cmd_q_des_10','cmd_q_des_11',
                'cmd_qd_des_0','cmd_qd_des_1','cmd_qd_des_2','cmd_qd_des_3','cmd_qd_des_4','cmd_qd_des_5','cmd_qd_des_6','cmd_qd_des_7','cmd_qd_des_8','cmd_qd_des_9','cmd_qd_des_10','cmd_qd_des_11',
                'cmd_p_des_0','cmd_p_des_1','cmd_p_des_2','cmd_p_des_3','cmd_p_des_4','cmd_p_des_5','cmd_p_des_6','cmd_p_des_7','cmd_p_des_8','cmd_p_des_9','cmd_p_des_10','cmd_p_des_11',
                'cmd_v_des_0','cmd_v_des_1','cmd_v_des_2','cmd_v_des_3','cmd_v_des_4','cmd_v_des_5','cmd_v_des_6','cmd_v_des_7','cmd_v_des_8','cmd_v_des_9','cmd_v_des_10','cmd_v_des_11',
                'kp_cartesian_0','kp_cartesian_1','kp_cartesian_2','kp_cartesian_3','kp_cartesian_4','kp_cartesian_5','kp_cartesian_6','kp_cartesian_7','kp_cartesian_8','kp_cartesian_9','kp_cartesian_10','kp_cartesian_11',
                'kd_cartesian_0','kd_cartesian_1','kd_cartesian_2','kd_cartesian_3','kd_cartesian_4','kd_cartesian_5','kd_cartesian_6','kd_cartesian_7','kd_cartesian_8','kd_cartesian_9','kd_cartesian_10','kd_cartesian_11',
                'kp_joint_0','kp_joint_1','kp_joint_2','kp_joint_3','kp_joint_4','kp_joint_5','kp_joint_6','kp_joint_7','kp_joint_8','kp_joint_9','kp_joint_10','kp_joint_11',
                'kd_joint_0','kd_joint_1','kd_joint_2','kd_joint_3','kd_joint_4','kd_joint_5','kd_joint_6','kd_joint_7','kd_joint_8','kd_joint_9','kd_joint_10','kd_joint_11',
                'data_q_0','data_q_1','data_q_2','data_q_3','data_q_4','data_q_5','data_q_6','data_q_7','data_q_8','data_q_9','data_q_10','data_q_11',
                'data_qd_0','data_qd_1','data_qd_2','data_qd_3','data_qd_4','data_qd_5','data_qd_6','data_qd_7','data_qd_8','data_qd_9','data_qd_10','data_qd_11',
                'data_p_0','data_p_1','data_p_2','data_p_3','data_p_4','data_p_5','data_p_6','data_p_7','data_p_8','data_p_9','data_p_10','data_p_11',
                'data_v_0','data_v_1','data_v_2','data_v_3','data_v_4','data_v_5','data_v_6','data_v_7','data_v_8','data_v_9','data_v_10','data_v_11',
                'data_tau_est_0','data_tau_est_1','data_tau_est_2','data_tau_est_3','data_tau_est_4','data_tau_est_5','data_tau_est_6','data_tau_est_7','data_tau_est_8','data_tau_est_9','data_tau_est_10','data_tau_est_11',
                'J0_0', 'J0_1', 'J0_2','J0_3','J0_4','J0_5','J0_6','J0_7','J0_8',
                'J1_0', 'J1_1', 'J1_2','J1_3','J1_4','J1_5','J1_6','J1_7','J1_8',
                'J2_0', 'J2_1', 'J2_2','J2_3','J2_4','J2_5','J2_6','J2_7','J2_8',
                'J3_0', 'J3_1', 'J3_2','J3_3','J3_4','J3_5','J3_6','J3_7','J3_8',
                'p_0', 'p_1', 'p_2',
                'vWorld_0', 'vWorld_1', 'vWorld_2',
                'vBody_0', 'vBody_1', 'vBody_2',
                'rpy_0', 'rpy_1', 'rpy_2',
                'omegaBody_0', 'omegaBody_1', 'omegaBody_2',
                'omegaWorld_0', 'omegaWorld_1', 'omegaWorld_2',
                'quat_0', 'quat_1', 'quat_2', 'quat_3',
                'pBody_des_0','pBody_des_1','pBody_des_2',
                'vBody_des_0','vBody_des_1','vBody_des_2',
                'aBody_des_0','aBody_des_1','aBody_des_2',
                'pBody_rpy_des_0','pBody_rpy_des_1','pBody_rpy_des_2',
                'vBody_ori_des_0','vBody_ori_des_1','vBody_ori_des_2',
                'pFoot_des_0','pFoot_des_1','pFoot_des_2','pFoot_des_3','pFoot_des_4','pFoot_des_5','pFoot_des_6','pFoot_des_7','pFoot_des_8','pFoot_des_9','pFoot_des_10','pFoot_des_11',
                'vFoot_des_0','vFoot_des_1','vFoot_des_2','vFoot_des_3','vFoot_des_4','vFoot_des_5','vFoot_des_6','vFoot_des_7','vFoot_des_8','vFoot_des_9','vFoot_des_10','vFoot_des_11',
                'aFoot_des_0','aFoot_des_1','aFoot_des_2','aFoot_des_3','aFoot_des_4','aFoot_des_5','aFoot_des_6','aFoot_des_7','aFoot_des_8','aFoot_des_9','aFoot_des_10','aFoot_des_11',
                'Fr0_des_0','Fr0_des_1','Fr0_des_2','Fr0_des_3','Fr0_des_4','Fr0_des_5','Fr0_des_6','Fr0_des_7','Fr0_des_8','Fr0_des_9','Fr0_des_10','Fr0_des_11',
                'contact_state_0', 'contact_state_1', 'contact_state_2', 'contact_state_3',
                # 'Fr1_des_0','Fr1_des_1','Fr1_des_2','Fr1_des_3','Fr1_des_4','Fr1_des_5','Fr1_des_6','Fr1_des_7','Fr1_des_8','Fr1_des_9','Fr1_des_10','Fr1_des_11',
                # 'Fr2_des_0','Fr2_des_1','Fr2_des_2','Fr2_des_3','Fr2_des_4','Fr2_des_5','Fr2_des_6','Fr2_des_7','Fr2_des_8','Fr2_des_9','Fr2_des_10','Fr2_des_11',
                # 'Fr3_des_0','Fr3_des_1','Fr3_des_2','Fr3_des_3','Fr3_des_4','Fr3_des_5','Fr3_des_6','Fr3_des_7','Fr3_des_8','Fr3_des_9','Fr3_des_10','Fr3_des_11',
                # 'Fr4_des_0','Fr4_des_1','Fr4_des_2','Fr4_des_3','Fr4_des_4','Fr4_des_5','Fr4_des_6','Fr4_des_7','Fr4_des_8','Fr4_des_9','Fr4_des_10','Fr4_des_11',
                # 'Fr5_des_0','Fr5_des_1','Fr5_des_2','Fr5_des_3','Fr5_des_4','Fr5_des_5','Fr5_des_6','Fr5_des_7','Fr5_des_8','Fr5_des_9','Fr5_des_10','Fr5_des_11',
                # 'Fr6_des_0','Fr6_des_1','Fr6_des_2','Fr6_des_3','Fr6_des_4','Fr6_des_5','Fr6_des_6','Fr6_des_7','Fr6_des_8','Fr6_des_9','Fr6_des_10','Fr6_des_11',
                # 'Fr7_des_0','Fr7_des_1','Fr7_des_2','Fr7_des_3','Fr7_des_4','Fr7_des_5','Fr7_des_6','Fr7_des_7','Fr7_des_8','Fr7_des_9','Fr7_des_10','Fr7_des_11',
                # 'Fr8_des_0','Fr8_des_1','Fr8_des_2','Fr8_des_3','Fr8_des_4','Fr8_des_5','Fr8_des_6','Fr8_des_7','Fr8_des_8','Fr8_des_9','Fr8_des_10','Fr8_des_11',
                # 'Fr9_des_0','Fr9_des_1','Fr9_des_2','Fr9_des_3','Fr9_des_4','Fr9_des_5','Fr9_des_6','Fr9_des_7','Fr9_des_8','Fr9_des_9','Fr9_des_10','Fr9_des_11'
                'q_0', 'q_1', 'q_2', 'q_3', 'q_4', 'q_5', 'q_6', 'q_7', 'q_8', 'q_9', 'q_10', 'q_11',
                'qd_0', 'qd_1', 'qd_2', 'qd_3', 'qd_4', 'qd_5', 'qd_6', 'qd_7', 'qd_8', 'qd_9', 'qd_10', 'qd_11',
                'tau_est_0', 'tau_est_1', 'tau_est_2', 'tau_est_3', 'tau_est_4', 'tau_est_5', 'tau_est_6', 'tau_est_7', 'tau_est_8', 'tau_est_9', 'tau_est_10', 'tau_est_11',
                'fsm_state',
                'accelerometer_0', 'accelerometer_1', 'accelerometer_2', 
                'gyro_0', 'gyro_1', 'gyro_2', 
                'quat_0', 'quat_1', 'quat_2', 'quat_3',
                'time'
                ]
        self.ctrl_cmd_ready  = False
        self.ctrl_data_ready = False
        self.state_est_ready = False
        self.rob_serv_response_ready = False
        self.mpc_state_data_ready = False

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
    def ctrl_cmd_handler(self, channel, data):
        msg = leg_control_command_lcmt.decode(data)

        self.buffer[0:12]  = msg.tau_ff
        self.buffer[12:24] = msg.f_ff
        self.buffer[24:36] = msg.q_des
        self.buffer[36:48] = msg.qd_des
        self.buffer[48:60] = msg.p_des
        self.buffer[60:72] = msg.v_des
        self.buffer[72:84] = msg.kp_cartesian
        self.buffer[84:96] = msg.kd_cartesian
        self.buffer[96:108] = msg.kp_joint
        self.buffer[108:120] = msg.kd_joint

        self.ctrl_cmd_ready = True

    """
    struct leg_control_data_lcmt {
        float q[12];
        float qd[12];
        float p[12];
        float v[12];
        float tau_est[12];
        float jacobian0[9];
        float jacobian1[9];
        float jacobian2[9];
        float jacobian3[9];
    }
    """
    def ctrl_data_handler(self, channel, data):
        msg = leg_control_data_lcmt.decode(data)

        self.buffer[120:132] = msg.q
        self.buffer[132:144] = msg.qd
        self.buffer[144:156] = msg.p
        self.buffer[156:168] = msg.v
        self.buffer[168:180] = msg.tau_est
        self.buffer[180:189] = msg.jacobian0
        self.buffer[189:198] = msg.jacobian1
        self.buffer[198:207] = msg.jacobian2
        self.buffer[207:216] = msg.jacobian3

        self.ctrl_data_ready = True



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
    def state_estim_handler(self, channel, data):
        msg = state_estimator_lcmt.decode(data)

        self.buffer[216:219] = msg.p
        self.buffer[219:222] = msg.vWorld
        self.buffer[222:225] = msg.vBody
        self.buffer[225:228] = msg.rpy
        self.buffer[228:231] = msg.omegaBody
        self.buffer[231:234] = msg.omegaWorld
        self.buffer[234:238] = msg.quat

        self.state_est_ready = True

    """
    struct mpc_state_data_t
    {
        float pBody_des[3];
        float vBody_des[3];
        float aBody_des[3];
        
        float pBody_rpy_des[3];
        float vBody_ori_des[3];
        
        float pFoot_des[12];
        float vFoot_des[12];
        float aFoot_des[12];
        float Fr_des[12];

        float contact_state[4];

        float future_Fr[108];
    }
    """
    def mpc_data_handler(self, channel, data):
        msg = mpc_state_data_t.decode(data)

        self.buffer[238:241] = msg.pBody_des
        self.buffer[241:244] = msg.vBody_des
        self.buffer[244:247] = msg.aBody_des
        self.buffer[247:250] = msg.pBody_rpy_des
        self.buffer[250:253] = msg.vBody_ori_des
        self.buffer[253:265] = msg.pFoot_des
        self.buffer[265:277] = msg.vFoot_des
        self.buffer[277:289] = msg.aFoot_des
        self.buffer[289:301] = msg.Fr_des
        self.buffer[301:305] = msg.contact_state
        # self.buffer[305:412] = msg.future_Fr

        self.mpc_state_data_ready = True

    """
    struct robot_server_response_lcmt {
        float q[12];
        float qd[12];
        float tau_est[12];
        byte fsm_state;
        float accelerometer[3];
        float gyro[3];
        float quat[4];
    }
    """
    def rob_serv_response_handler(self, channel, data):
        msg = robot_server_response_lcmt.decode(data)

        self.buffer[305:317] = msg.q
        self.buffer[317:329] = msg.qd
        self.buffer[329:341] = msg.tau_est
        self.buffer[341]     = msg.fsm_state
        self.buffer[342:345] = msg.accelerometer
        self.buffer[345:348] = msg.gyro
        self.buffer[348:352] = msg.quat

        self.rob_serv_response_ready = True

handler = Handler()          
# print(len(handler.buffer))          
# print(len(handler.header))          
# f = open("/home/ursk/muadquad_data/mq_telem" + time.strftime("_%d_%m_%Y_%H-%M-%S") + ".csv",'w+')
f = open("/home/embir/data_quad/mq_telem" + time.strftime("_%d_%m_%Y_%H-%M-%S") + ".csv",'w+')
# f = open("/home/adsm/mq_telem/mq_telem" + time.strftime("_%d_%m_%Y_%H-%M-%S") + ".csv",'w+')
# f = open("/home/mrako/Documents/EMBIR/mq_telem" + time.strftime("_%d_%m_%Y_%H-%M-%S") + ".csv",'w+')
writer = csv.writer(f)
writer.writerow(handler.header)

# ctrl_cmd_ready = False
# ctrl_data_ready = False

lc = lcm.LCM()
lc_mpc = lcm.LCM()
subscription = lc.subscribe("leg_control_command", handler.ctrl_cmd_handler)
subscription = lc.subscribe("leg_control_data", handler.ctrl_data_handler)
subscription = lc.subscribe("state_estimator", handler.state_estim_handler)
subscription = lc.subscribe("robot_server_response", handler.rob_serv_response_handler)
subscription = lc_mpc.subscribe("mpc_data", handler.mpc_data_handler)

iter = 0
start_time = time.time()
try:
    while True:
        lc.handle_timeout(100)
        
        if handler.ctrl_cmd_ready and handler.ctrl_data_ready and handler.state_est_ready and handler.rob_serv_response_ready:
            lc_mpc.handle_timeout(1)
            cur_time = time.time() - start_time
            # buf2 = [cur_time] + handler.buffer[:413]
            handler.buffer[352] = cur_time
            # print(len(handler.buffer))
            writer.writerow([round(e, 4) for e in handler.buffer])
            # handler.buffer = [0 for x in range(413)]
            handler.buffer = [0 for x in range(353)]
            handler.ctrl_cmd_ready = False
            handler.ctrl_data_ready = False
            handler.state_est_ready = False
            handler.rob_serv_response_ready = False
            handler.mpc_state_data_ready = False

        if iter % 300 == 0:
            print("logging is still alive")

        iter += 1
except KeyboardInterrupt:
    f.close()
