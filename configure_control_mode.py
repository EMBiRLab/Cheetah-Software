import lcm
import numpy as np
import sys
import random
import threading
import struct

sys.path.append('../')
sys.path.append('./lcm-types/python/')
from control_parameter_request_lcmt import control_parameter_request_lcmt
from control_parameter_respones_lcmt import control_parameter_respones_lcmt


def control_param_response_handler(channel, data):
    msg = control_parameter_respones_lcmt.decode(data)
    print("Received message on channel \"%s\"" % channel)
    # Now update the requestNumber according to the value in the response
    requestNumLock.acquire()
    global requestNum
    requestNum = msg.requestNumber
    requestNumLock.release()

# Thread for handling the responses regarding control parameter requests
def control_response_thread(lc):
    while True:
        global stop_listening
        if(stop_listening):
            break
        lcmLock.acquire()
        lc.handle_timeout(10)
        lcmLock.release()


# Main script
# Establish variables
lc = lcm.LCM()
subscription = lc.subscribe("interface_response", control_param_response_handler)
cmd = control_parameter_request_lcmt()
requestNum = 0
lcmLock = threading.Lock()
requestNumLock = threading.Lock()
stop_listening = False

t = threading.Thread(target=control_response_thread, args=(lc,))
t.start()


# Run main loop to change control modes
while True:
    try:
        print(
            "Avaiable robot states are:\n"
            "K_PASSIVE - 0\n"
            "K_STAND_UP - 1\n"
            "K_BALANCE_STAND - 3\n"
            "K_LOCOMOTION - 4\n"
            "K_LOCOMOTION_TEST - 5\n"
            "K_RECOVERY_STAND - 6\n"
            "K_VISION - 8\n"
            "K_BACKFLIP - 9\n"
            "K_FRONTJUMP - 11\n")

        desired_control_mode = input("What control mode would you like? enter the number: ")
        desired_control_mode = (int)(desired_control_mode)
        if(desired_control_mode > 11 or desired_control_mode < 0):
            stop_listening = True
            t.join()
            sys.exit()

        value = [0 for x in range(64)]
        value[0] = desired_control_mode

        extended_name = "control_mode"
        extended_name = extended_name.ljust(64," ")
        # cmd.name = struct.pack('%c' % len(extended_name),*extended_name)                 #'@c',*extended_name)
        cmd.name = [ord(e) for e in list(extended_name)]
        print("cmd.name is:",cmd.name, "with len:",len(cmd.name))
        print("the value is:",value, "with size:",len(value))
        # print("CMD.NAME TYPE:",typeof())
        requestNumLock.acquire()
        cmd.requestNumber = requestNum + 1
        requestNumLock.release()
        cmd.value = value
        cmd.parameterKind = 1 #appears to be the default in the mini_cheetah code
        cmd.requestKind = 3 #corresponds to ControlParameterRequestKind::SET_USER_PARAM_BY_NAME
        
        lc.publish("robot_server_response", cmd.encode())

        print("cmd.name is:",cmd.name)
        print("iteration of while loop is complete\n\n")

    except KeyboardInterrupt:
        stop_listening = True
        t.join()
        sys.exit()