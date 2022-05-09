"""
PyTeapot module for drawing rotating cube using OpenGL as per
quaternion or yaw, pitch, roll angles received over serial port.
"""
import numpy as np
from cmath import pi
import pygame
import math
from OpenGL.GL import *
from OpenGL.GLU import *
from pygame.locals import *
import sys
sys.path.append('../')
sys.path.append('./lcm-types/python/')
import lcm

from pyquaternion import Quaternion
from robot_server_response_lcmt import robot_server_response_lcmt

useSerial = False # set true for using serial for data transmission, false for wifi
useLCM = True
useQuat = True   # set true for using quaternions, false for using y,p,r angles

first_rx = True
rx_thresh = 15
rx_count = 0

latest_quat = [1, 0, 0, 0]
qtransform = Quaternion(.9999999, 0, 0, 0)

def rs_handler(channel, data):
    global rx_count
    rx_count = rx_count + 1
    msg = robot_server_response_lcmt.decode(data)
    global latest_quat
    global qtransform
    global first_rx
    latest_quat = msg.quat

    qimu = Quaternion(latest_quat[0],latest_quat[1],latest_quat[2],latest_quat[3])
    # Rimu = qimu.rotation_matrix


    if rx_count >= rx_thresh and rx_count < rx_thresh + 1:
        qmit = Quaternion(.9999,0,0,0)
        # qtransform = qmit / qimu
        qtransform = qmit * qimu.inverse
        # qtransform = qimu * qmit.inverse
        first_rx = False

    # Rtransform = qtransform.rotation_matrix
    q1 = Quaternion(axis=[0,0,1], angle=np.pi/2)

    # qimu = q1 * qimu
    # q2 = q1 * qimu

    qy90 = Quaternion(0.7071068, 0, 0.7071068, 0)
    # rotatedq = qtransform * qimu
    # rotatedq = qtransform * qimu #* qtransform.conjugate
    # rotatedq = qimu * qtransform #* qtransform.conjugate
    rotatedq = qy90*qimu

    latest_quat = [rotatedq[0], rotatedq[1], rotatedq[2], rotatedq[3]]

if(useSerial):
    import serial
    ser = serial.Serial('/dev/ttyUSB0', 38400)
else:
    import socket

    UDP_IP = "0.0.0.0"
    UDP_PORT = 5005
    sock = socket.socket(socket.AF_INET, # Internet
                         socket.SOCK_DGRAM) # UDP
    sock.bind((UDP_IP, UDP_PORT))

def main():
    video_flags = OPENGL | DOUBLEBUF
    pygame.init()
    screen = pygame.display.set_mode((640, 480), video_flags)
    pygame.display.set_caption("PyTeapot IMU orientation visualization")
    resizewin(640, 480)
    init()
    frames = 0
    ticks = pygame.time.get_ticks()

    lc = lcm.LCM()
    subscription = lc.subscribe("robot_server_response", rs_handler)

    while 1:
        lc.handle_timeout(10)
        event = pygame.event.poll()
        if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
            break
        if(useQuat):
            [w, nx, ny, nz] = read_data()
        else:
            [yaw, pitch, roll] = read_data()
        if(useQuat):
            draw(w, nx, ny, nz)
        else:
            draw(1, yaw, pitch, roll)
        pygame.display.flip()
        frames += 1
    print("fps: %d" % ((frames*1000)/(pygame.time.get_ticks()-ticks)))
    if(useSerial):
        ser.close()


def resizewin(width, height):
    """
    For resizing window
    """
    if height == 0:
        height = 1
    glViewport(0, 0, width, height)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(45, 1.0*width/height, 0.1, 100.0)
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()


def init():
    glShadeModel(GL_SMOOTH)
    glClearColor(0.0, 0.0, 0.0, 0.0)
    glClearDepth(1.0)
    glEnable(GL_DEPTH_TEST)
    glDepthFunc(GL_LEQUAL)
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST)


def cleanSerialBegin():
    if(useQuat):
        try:
            line = ser.readline().decode('UTF-8').replace('\n', '')
            w = float(line.split('w')[1])
            nx = float(line.split('a')[1])
            ny = float(line.split('b')[1])
            nz = float(line.split('c')[1])
        except Exception:
            pass
    else:
        try:
            line = ser.readline().decode('UTF-8').replace('\n', '')
            yaw = float(line.split('y')[1])
            pitch = float(line.split('p')[1])
            roll = float(line.split('r')[1])
        except Exception:
            pass


def read_data():
    # if(useSerial):
    #     ser.reset_input_buffer()
    #     cleanSerialBegin()
    #     line = ser.readline().decode('UTF-8').replace('\n', '')
    #     print(line)
    # else:
    #     # Waiting for data from udp port 5005
    #     data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
    #     line = data.decode('UTF-8').replace('\n', '')
    #     print(line)
    if(useLCM):
        return latest_quat
                
    # if(useQuat):
    #     w = float(line.split('w')[1])
    #     nx = float(line.split('a')[1])
    #     ny = float(line.split('b')[1])
    #     nz = float(line.split('c')[1])
    #     return [w, nx, ny, nz]
    # else:
    #     yaw = float(line.split('y')[1])
    #     pitch = float(line.split('p')[1])
    #     roll = float(line.split('r')[1])
    #     return [yaw, pitch, roll]


def draw(w, nx, ny, nz):
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    glLoadIdentity()
    glTranslatef(0, 0.0, -7.0)

    drawText((-2.6, 1.8, 2), "PyTeapot", 18)
    drawText((-2.6, 1.6, 2), "Module to visualize quaternion or Euler angles data", 16)
    drawText((-2.6, -2, 2), "Press Escape to exit.", 16)

    if(useQuat):
        [yaw, pitch , roll] = quat_to_ypr([w, nx, ny, nz])
        drawText((-2.6, -1.8, 2), "Yaw: %f, Pitch: %f, Roll: %f" %(yaw, pitch, roll), 16)
        glRotatef(2 * math.acos(w) * 180.00/math.pi, -1 * nx, nz, ny)
    else:
        yaw = nx
        pitch = ny
        roll = nz
        drawText((-2.6, -1.8, 2), "Yaw: %f, Pitch: %f, Roll: %f" %(yaw, pitch, roll), 16)
        glRotatef(-roll, 0.00, 0.00, 1.00)
        glRotatef(pitch, 1.00, 0.00, 0.00)
        glRotatef(yaw, 0.00, 1.00, 0.00)

    glBegin(GL_QUADS)
    glColor3f(0.0, 1.0, 0.0)
    glVertex3f(1.0, 0.2, -1.0)
    glVertex3f(-1.0, 0.2, -1.0)
    glVertex3f(-1.0, 0.2, 1.0)
    glVertex3f(1.0, 0.2, 1.0)

    glColor3f(1.0, 0.5, 0.0)
    glVertex3f(1.0, -0.2, 1.0)
    glVertex3f(-1.0, -0.2, 1.0)
    glVertex3f(-1.0, -0.2, -1.0)
    glVertex3f(1.0, -0.2, -1.0)

    glColor3f(1.0, 0.0, 0.0)
    glVertex3f(1.0, 0.2, 1.0)
    glVertex3f(-1.0, 0.2, 1.0)
    glVertex3f(-1.0, -0.2, 1.0)
    glVertex3f(1.0, -0.2, 1.0)

    glColor3f(1.0, 1.0, 0.0)
    glVertex3f(1.0, -0.2, -1.0)
    glVertex3f(-1.0, -0.2, -1.0)
    glVertex3f(-1.0, 0.2, -1.0)
    glVertex3f(1.0, 0.2, -1.0)

    glColor3f(0.0, 0.0, 1.0)
    glVertex3f(-1.0, 0.2, 1.0)
    glVertex3f(-1.0, 0.2, -1.0)
    glVertex3f(-1.0, -0.2, -1.0)
    glVertex3f(-1.0, -0.2, 1.0)

    glColor3f(1.0, 0.0, 1.0)
    glVertex3f(1.0, 0.2, -1.0)
    glVertex3f(1.0, 0.2, 1.0)
    glVertex3f(1.0, -0.2, 1.0)
    glVertex3f(1.0, -0.2, -1.0)
    glEnd()


def drawText(position, textString, size):
    font = pygame.font.SysFont("Courier", size, True)
    textSurface = font.render(textString, True, (255, 255, 255, 255), (0, 0, 0, 255))
    textData = pygame.image.tostring(textSurface, "RGBA", True)
    glRasterPos3d(*position)
    glDrawPixels(textSurface.get_width(), textSurface.get_height(), GL_RGBA, GL_UNSIGNED_BYTE, textData)

def quat_to_ypr(q):
    yaw   = math.atan2(2.0 * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3])
    pitch = -math.asin(2.0 * (q[1] * q[3] - q[0] * q[2]))
    roll  = math.atan2(2.0 * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3])
    pitch *= 180.0 / math.pi
    yaw   *= 180.0 / math.pi
    yaw   -= -0.13  # Declination at Chandrapur, Maharashtra is - 0 degress 13 min
    roll  *= 180.0 / math.pi
    return [yaw, pitch, roll]


if __name__ == '__main__':
    main()
