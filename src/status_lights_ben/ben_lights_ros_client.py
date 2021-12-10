#!/usr/bin/python3

import zmq
import time
import rosbag
import rospy
import rosgraph

context = zmq.Context()

#  Socket to talk to server
socket = context.socket(zmq.REQ)
socket.connect("tcp://localhost:5555")
print('Connected')
while True:
    if rosgraph.is_master_online():
        socket.send(b"ROS,T")
    else:
        socket.send(b"ROS,F")
    message = socket.recv()
    time.sleep(5)
    