# Needed to prevent loops from locking up the javascript thread
SENSOR_DELAY = 0.001

# Import the necessary libraries
import simPython, time
import math
from ev3dev2.motor import *

print("!!!!!! importing parameters.py")

class PORT:
    A = 'outA'
    B = 'outB'
    C = 'outC'
    D = 'outD'

    INPUT_S1 = 'inS1'
    INPUT_S2 = 'inS2'
    INPUT_S3 = 'inS3'
    INPUT_S4 = 'inS4'

class Direction:
    CLOCKWISE = 'CLOCKWISE'
    COUNTERCLOCKWISE = 'COUNTERCLOCKWISE'

class Stop:
    COAST = 'COAST'
    BRAKE = 'BRAKE'
    HOLD = 'HOLD'

class Color:
    BLACK = 'BLACK'
    BLUE = 'BLUE'
    GREEN = 'GREEN'
    YELLOW = 'YELLOW'
    RED = 'RED'
    WHITE = 'WHITE'    
    BROWN = 'BROWN'
    ORANGE = 'ORANGE'
    PURPLE = 'PURPLE'    

class Button:
    print("not implemented") 