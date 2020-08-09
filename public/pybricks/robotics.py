# Needed to prevent loops from locking up the javascript thread
SENSOR_DELAY = 0.001

# Import the necessary libraries
import simPython, time
import math
from ev3dev2.motor import *

print("### pybrick.py imported ###")
def straightPy(speed):
    tank_drive = MoveTank(OUTPUT_A, OUTPUT_B)
    tank_drive.on(speed, speed) 
    print("### MoveStraight ###; speed " + str(speed))

