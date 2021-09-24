import libioplus as io
import time
import sys


stackLevel = 0   


for i in range(5):
    io.setRelayCh(stackLevel, i +1, 0)
    time.sleep(0.05)
