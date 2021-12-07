#!/usr/bin/python3

import threading
import time
import libioplus as io
import sys
import random
import rospy
import rosgraph

# Possible light states:
# - On                          - The light is on 
# - Off                         - The light is off
# - Single                      - The light flashes once then pauses for 1 sec
# - Double                      - The light flashes twice then pauses for 1 sec
# - Single Slow                 - The light flashes once then pauses for 3 sec
# - Double Slow                 - The light flashes twice then pauses for 3 sec
# - Single Super Slow           - The light flashes once then pauses for 5 sec
# - Double Super Slow           - The light flashes twice then pauses for 5 sec
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
#         Red   Amber White Green botton red
states = ['OFF', 'OFF', 'OFF', 'OFF', 'OFF']
colors = ['red', 'amber', 'white', 'green', 'b_red']
color_bits = [1, 1, 1, 1, 1]

def get_bit_val():
    value = str('000') + ''.join(str(e) for e in color_bits)
    val = int(value, 2)
    return val

def set_bits(color, state, loop_pos, j) :
    flash = flash_dict.get(state)
    color_bits[j] = flash[loop_pos]

def flash_loop():
    
    while 1:
        for i in range(24):
            time.sleep(0.25)
            for j in range(5):
                set_bits(colors[j], states[j], i, j)
            value = get_bit_val()
            #print(value)
            io.setRelays(0,value)
            

t1 = threading.Thread(target=flash_loop)

t1.start()

# stl = flash_dict.keys()

# while 1:
#     for i in range(5):
#         states[i] = random.choice(stl)
#     time.sleep(5)

# for i in range(5):
#     if i%2 == 0: 
#         states[i] = 'Single Alt1'

# for i in range(5):
#     if i%2 == 1: 
#         states[i] = 'Single Alt2'
        

while 1:

    if rosgraph.is_master_online():
        print('ROS MASTER CONNECTED')
        states[4] = 'OFF'
        states[1] = 'ON'
        rospy.init_node('ben_lights_node')
        rospy.spin()
    else:
        print('ROS MASTER OFFLINE')
        states[4] = 'Double'
    time.sleep(2)