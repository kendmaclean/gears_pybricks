# Needed to prevent loops from locking up the javascript thread
SENSOR_DELAY = 0.001

# Import the necessary libraries
import simPython, time
import math
from ev3dev2.motor import *

from pybricks.parameters import *

class Motor:
    MAX_SPEED = 300    
    MAX_DURATION = 1000    

    def __init__(self, port, positive_direction=Direction.CLOCKWISE, gears=None):
        if isinstance(port, Port):
            self.port = port
        else:
            raise TypeError("port not of type Port")        
        if isinstance(positive_direction, Direction):
            self.positive_direction = positive_direction
        else:
            raise TypeError("positive_direction not of type Direction")             
        if (gears is not None):
            raise ValueError("gears not implemented")

    # Measuring
    def speed(self):
        print("speed not implemented")
        if (self.speed is not None):
            return self.speed
        else:
            raise ValueError("speed not implemented")

    def angle(self):
        if (self.angle is not None):
            return self.angle
        else:
            raise ValueError("angle not implemented")

    def reset_angle(self, angle):
        self.angle = angle

    # Stopping
    def stop(self):
        print("stop not implemented")

    def brake(self):
        print("brake not implemented")

    def hold(self):
        print("hold not implemented")

    # Action
    def run(self, speed):
        if (-MAX_SPEED <= speed <= MAX_SPEED):
            self.speed = speed
        else:
            raise ValueError("speed outside allowable bounds")        

    def run_time(self, speed, time, then=Stop.HOLD, wait=True):
        if -MAX_SPEED <= speed <= MAX_SPEED:
            self.speed = speed
        else:
            raise ValueError("speed outside allowable bounds")          
        self.time = time
        if 0 <= time <= MAX_DURATION:
            self.time = time
        else:
            raise ValueError("speed outside allowable bounds")              
        self.then = then        
        self.wait = wait                    

    def run_angle(self, speed, rotation_angle, then=Stop.HOLD, wait=True):
        if (-MAX_SPEED <= speed <= MAX_SPEED):
            self.speed = speed
        else:
            raise ValueError("speed outside allowable bounds")          
        self.rotation_angle = rotation_angle   
        self.then = then        
        self.wait = wait            
        print("not implemented")

    def run_target(self, speed, target_angle, then=Stop.HOLD, wait=True):
        if -MAX_SPEED <= speed <= MAX_SPEED:
            self.speed = speed
        else:
            raise ValueError("speed outside allowable bounds")          
        self.target_angle = target_angle   
        self.then = then        
        self.wait = wait            
        print("not implemented")

    def run_until_stalled(self, speed, then=Stop.COAST, duty_limit=None):
        if -MAX_SPEED <= speed <= MAX_SPEED:
            self.speed = speed
        else:
            raise ValueError("speed outside allowable bounds")        
        self.then = then          
        self.duty_limit = duty_limit   
        print("not implemented")

class TouchSensor:
    def __init__(self, port):
        self.port = port

    def pressed(self):
        print("TouchSensor not implemented")

class ColorSensor:
    def __init__(self, port):
        self.port = port

    def color(self):
        print("ColorSensor not implemented")       

    def ambient(self):
        print("ColorSensor not implemented") 

    def reflection(self):
        print("ColorSensor not implemented")               

    def rgb(self):
        print("ColorSensor not implemented")

class UltrasonicSensor:
    def __init__(self, port):
        self.port = port

    def distance(self, silent=False):
        print("ColorSensor not implemented")       

class GyroSensor:
    def __init__(self, port, positive_direction=Direction.CLOCKWISE):
        self.port = port
        self.positive_direction = positive_direction        

    def speed(self):
        print("GyroSensor not implemented")            

    def angle(self):
        print("GyroSensor not implemented")       

    def reset_angle(self):
        print("GyroSensornot implemented")     