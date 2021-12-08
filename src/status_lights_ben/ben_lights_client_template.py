#!/usr/bin/python3

import zmq
import time

context = zmq.Context()

#  Socket to talk to server
print("Connecting to hello world server…")
socket = context.socket(zmq.REQ)
socket.connect("tcp://localhost:5555")

for request in range(1000000):
    socket.send(b"Hello")
    # time.sleep(1)
    message = socket.recv()
    print("Received reply %s [ %s ]" % (request, message))