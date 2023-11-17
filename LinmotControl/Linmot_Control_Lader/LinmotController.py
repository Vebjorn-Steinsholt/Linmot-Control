import math
import socket
import struct
import numpy as np
from matplotlib import pyplot as plt
from interval_timer import IntervalTimer


choice = 1.0
setPoint = 0.0
maxVel = 0.0
Acceleration = 0.0
Deceleration = 0.0
timeInterval = 0.02
tic = 0
UDP_IP = "192.168.1.5"
UDP_PORT = 8888


def messagehandler(mode, position, velocity, acceleration, deceleration):
    msg = [mode, position, velocity, acceleration, deceleration]
    message = struct.pack(">5f", *msg)
    # print('UDP target IP: %s' % UDP_IP)
    # print('UDP target port: %s' % UDP_PORT)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # UDP # Internet
    sock.sendto(message, (UDP_IP, UDP_PORT))


def go_to_pos():
    pos = float(input("setPoint: "))
    vel = float(input("maxVel: "))
    acc = float(input("Acceleration: "))
    dacc = float(input("Deceleration: "))
    messagehandler(3, pos, vel, acc, dacc)


def calc_sine(amplitude,f,phi, time):
    omega = float(2 * math.pi * f)
    time_series = np.arange(0, time, timeInterval)
    y = []
    v_max = []
    a_max = []
    for t in time_series:
        y.append((100  + amplitude * math.sin(omega * t + phi)))
        # v = float((amplitude * omega * math.cos(omega * t + phi)))
        v = float(amplitude*omega)
        v_max.append((math.fabs(v) / 1000))
        # a = float(-amplitude * omega ** 2 * math.sin(omega * t + phi))
        a = float(-amplitude * omega ** 2)
        a_max.append((math.fabs(a) / 1000))
    j = 0
    for interval in IntervalTimer(timeInterval):
        messagehandler(3, y[j], v_max[j], a_max[j], a_max[j])
        j += 1
        if j == time / timeInterval:
            break


def sin_go_to_pos():
    messagehandler(8, 120, 1, 1, 1)


#while choice != 0:
#    userInput = input("Operation mode: ")
#    choice = float(userInput)
#    if choice == 3.0:
#        go_to_pos()
#    if choice == 7:
#        messagehandler(3, 120, 1, 1, 1)
#        calc_sine()
#    if choice == 8:
#        sin_go_to_pos()
#    if choice == 1:
#        messagehandler(choice, setPoint, maxVel, Acceleration, Deceleration)
#    if choice == 2:
#        messagehandler(choice, setPoint, maxVel, Acceleration, Deceleration)
#    if choice == 4:
#        messagehandler(choice, setPoint, maxVel, Acceleration, Deceleration)
#    if choice == 5:
#        messagehandler(choice, setPoint, maxVel, Acceleration, Deceleration)
