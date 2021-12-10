#!/usr/bin/python3

import threading
import time
import zmq
import libioplus as io
import sys
import random
import rospy
import rosgraph
import re

# Possible light states:
# - On                          - The light is on 
# - Off                         - The light is off
# - Single                      - The light flashes alternating on off .5 sec
# - Double                      - The light flashes twice then pauses for 1.5 sec
# - Single Slow                 - The light flashes once then pauses for 2.5 sec
# - Single Super Slow           - The light flashes once then pauses for 5.5 sec
# - Double Super Slow           - The light flashes twice then pauses for 10.5 sec
# - Single Alternating          - The light flashes once, alternating with another light
# - Double Alternating          - The light flashes twice, then alternates with another light

flash_dict = {
    'ON': [ 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
    'OFF': [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], 
    'Single': [ 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0],
    'Double': [ 1, 0, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0],
    'Single Slow': [ 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    'Single Super Slow': [ 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    'Double Super Slow': [ 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    'Single Alt1': [1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0],
    'Single Alt2': [0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1],
    'Double Alt1': [1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    'Double Alt2': [0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0]
}
#Lights current states
#         Red    Amber  White  Green  BRed
states = ['OFF', 'OFF', 'OFF', 'OFF', 'OFF']
colors = ['red', 'amber', 'white', 'green', 'b_red']
color_bits = [1, 1, 1, 1, 1]

def start_up():
    ########  Starting Up   ########
    for j in range(3):
        for i in range(5):
            states[i] = 'ON'
            time.sleep(0.05)
            if i < 4:
                states[i+1] = 'ON'
                time.sleep(0.05)
            if i > 0:
                states[i-1] = 'OFF'
                time.sleep(0.05)
    #################################
        
def get_bit_val():
    value = str('000') + ''.join(str(e) for e in color_bits)
    val = int(value, 2)
    return val

def set_bits(color, state, loop_pos, j) :
    flash = flash_dict.get(state)
    color_bits[j] = flash[loop_pos]

def flash_loop():
    while True:
        for i in range(24):
            time.sleep(0.25)
            for j in range(5):
                set_bits(colors[j], states[j], i, j)
            value = get_bit_val()
            #print(value)
            io.setRelays(0,value)
            

t1 = threading.Thread(target=flash_loop)
t1.start()
start_up()
print('Starting Server')

context = zmq.Context()
socket = context.socket(zmq.REP)
socket.bind('tcp://*:5555')
print('Socket open')
 
####   Systems to Monitor   ####      id:    (T/F denoted by: [system_id],T or [system_id],F
master_power: bool = False            #MP 
payload_power: bool = False           #PP
vp_scm_running: bool = False          #VP
Ros_Core_Up: bool = False             #ROS            Can I connected to the roscore on mystique? (192.168.100.112)
engine_running: bool = False          #ER
valid_gps_fix: bool = False           #GPS
joystick_controll: bool = False       #JC
e_stop_active: bool = False           #ES
radio_link_to_op: bool = False        #RL
################################

while True:
    update = socket.recv()
    socket.send(b"Ack")
    update = (update.decode("utf-8")).split(',')
    id = update[0]
    status = update[1]
    print(id + ' connection status: ' + status)
    
    if id == 'MP':
        if status == 'T':
            master_power == True
        elif status == 'F':
            master_power == False
    if id == 'PP':
        if status == 'T':
            payload_power== True
        elif status == 'F':
            payload_power== False
    if id == 'VP':
        if status == 'T':
            vp_scm_running == True
        elif status == 'F':
            vp_scm_running == False
    if id == 'ROS':
        if status == 'T':
            Ros_Core_Up == True
        elif status == 'F':
            Ros_Core_Up == False
    if id == 'ER':
        if status == 'T':
            engine_running == True
        elif status == 'F':
            engine_running == False
    if id == 'GPS':
        if status == 'T':
            valid_gps_fix == True
        elif status == 'F':
            valid_gps_fix == False
    if id == 'JC':
        if status == 'T':
            joystick_controll == True
        elif status == 'F':
            joystick_controll == False
    if id == 'ES':
        if status == 'T':
            e_stop_active == True
        elif status == 'F':
            e_stop_active == False
    if id == 'RL':
        if status == 'T':
           radio_link_to_op == True
        elif status == 'F':
           radio_link_to_op == False

    #TODO: Define state combos and corresponding lighting here  