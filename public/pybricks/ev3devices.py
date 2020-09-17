# Needed to prevent loops from locking up the javascript thread
SENSOR_DELAY = 0.001

# Import the necessary libraries
import simPython, time
import math
from ev3dev2.motor import *

from pybricks.parameters import *

class MotorP:
    MAX_SPEED = 300    
    MAX_DURATION = 1000    

    def __init__(self, port, positive_direction=DirectionP.CLOCKWISE, gears=None):
        self.port = port
        self.positive_direction = positive_direction
        if (gears is not None):
            raise ValueError("gears not implemented")

        self.motor = LargeMotor(port)
        self.wheelDiameter = self.motor.wheel.wheelDiameter()
        self.wheelRadius = self.motor.wheel.wheelRadius()        
        self.axleTrack = self.motor.wheel.axleTrack()

    def __str__(self):
        return "Port: " + str(self.port) + ";\n robotTemplates.js wheelDiameter: " + str(self.wheelDiameter) + ";\n robotTemplates.js axleTrack: " + str(self.axleTrack) 

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

    def run_time(self, speed, time, then=StopP.HOLD, wait=True):
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

    def run_angle(self, speed, rotation_angle, then=StopP.HOLD, wait=True):
        if (-MAX_SPEED <= speed <= MAX_SPEED):
            self.speed = speed
        else:
            raise ValueError("speed outside allowable bounds")          
        self.rotation_angle = rotation_angle   
        self.then = then        
        self.wait = wait            
        print("not implemented")

    def run_target(self, speed, target_angle, then=StopP.HOLD, wait=True):
        if -MAX_SPEED <= speed <= MAX_SPEED:
            self.speed = speed
        else:
            raise ValueError("speed outside allowable bounds")          
        self.target_angle = target_angle   
        self.then = then        
        self.wait = wait            
        print("not implemented")

    def run_until_stalled(self, speed, then=StopP.COAST, duty_limit=None):
        if -MAX_SPEED <= speed <= MAX_SPEED:
            self.speed = speed
        else:
            raise ValueError("speed outside allowable bounds")        
        self.then = then          
        self.duty_limit = duty_limit   
        print("not implemented")

class TouchSensorP:
    def __init__(self, port):
        self.port = port

    def pressed(self):
        print("TouchSensor not implemented")

class ColorSensorP:
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

class UltrasonicSensorP:
    def __init__(self, port):
        self.port = port

    def distance(self, silent=False):
        print("ColorSensor not implemented")       

class GyroSensorP:
    def __init__(self, port, positive_direction=DirectionP.CLOCKWISE):
        self.port = port
        self.positive_direction = positive_direction        

    def speed(self):
        print("GyroSensor not implemented")            

    def angle(self):
        print("GyroSensor not implemented")       

    def reset_angle(self):
        print("GyroSensornot implemented")     