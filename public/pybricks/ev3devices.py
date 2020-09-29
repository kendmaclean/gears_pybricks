# Needed to prevent loops from locking up the javascript thread
SENSOR_DELAY = 0.001

# Import the necessary libraries
import simPython, time
import math
import ev3dev2.motor
from pybricks.parameters import *

class Motor:
    MAX_SPEED = 300    
    MAX_DURATION = 1000    
    # TODO not a ful implementation
    
    def __init__(self, port, positive_direction=Direction.CLOCKWISE, gears=None):
        self.port = port
        self.positive_direction = positive_direction
        if (gears is not None):
            raise ValueError("gears not implemented")

        self.motor = ev3dev2.motor.LargeMotor(port)
        self.wheel = self.motor.wheel

        self.wheelDiameter = self.wheel.wheelDiameter()
        self.wheelRadius = self.wheel.wheelRadius()        
        self.axleTrack = self.wheel.axleTrack() 

        # TODO access to this should be through Motor methods


    def __str__(self):
        return "Port: " + str(self.port) + ";\n robotTemplates.js wheelDiameter: " + str(self.wheelDiameter) + ";\n robotTemplates.js axleTrack: " + str(self.axleTrack) 

    # Measuring
    #def speed(self):
    #    return self.motor.wheel.axleTrack() 

    def angle(self):
        if (self.angle is not None):
            return self.angle
        else:
            raise ValueError("angle not implemented")

    def reset_angle(self, angle):
        print("not implemented")

    # Stopping
    def stop(self):
        print("stop not implemented")

    def brake(self):
        print("brake not implemented")

    def hold(self):
        print("hold not implemented")

    # Action
    def run(self, speed):
        if (-Motor.MAX_SPEED <= speed <= Motor.MAX_SPEED):
            self.speed = speed
        else:
            raise ValueError("speed outside allowable bounds")        

    def run_time(self, speed, time, then=Stop.HOLD, wait=True):
        if -Motor.MAX_SPEED <= speed <= Motor.MAX_SPEED:
            self.speed = speed
        else:
            raise ValueError("speed outside allowable bounds")          
        self.time = time
        if 0 <= time <= Motor.MAX_DURATION:
            self.time = time
        else:
            raise ValueError("speed outside allowable bounds")              
        self.then = then        
        self.wait = wait                    

    def run_angle(self, speed, rotation_angle, then=Stop.HOLD, wait=True):
        if (-Motor.MAX_SPEED <= speed <= Motor.MAX_SPEED):
            self.speed = speed
        else:
            raise ValueError("speed outside allowable bounds")          
        self.rotation_angle = rotation_angle   
        self.then = then        
        self.wait = wait            
        print("not implemented")

    def run_target(self, speed, target_angle, then=Stop.HOLD, wait=True):
        if -Motor.MAX_SPEED <= speed <= Motor.MAX_SPEED:
            self.speed = speed
        else:
            raise ValueError("speed outside allowable bounds")          
        self.target_angle = target_angle   
        self.then = then        
        self.wait = wait            
        print("not implemented")

    def run_until_stalled(self, speed, then=Stop.COAST, duty_limit=None):
        if -Motor.MAX_SPEED <= speed <= Motor.MAX_SPEED:
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

class UltrasonicSensor:
    def __init__(self, address=None):
        self.sensor = simPython.UltrasonicSensor(address)

    def distance(self): # in cm not mm
        time.sleep(SENSOR_DELAY)
        return float(self.sensor.dist())

    @property
    def presence(self):
        print("Error UltrasonicSensor presence not implemented")
        return False

class ColorSensor:
    def __init__(self, address=None):
        self.sensor = simPython.ColorSensor(address)

    def reflection(self):
        time.sleep(SENSOR_DELAY)
        return int(list(self.sensor.value())[0] / 2.55)

    def color(self):
        time.sleep(SENSOR_DELAY)
        hsv = self.hsv

        if hsv[1] < 20:
            if hsv[2] < 30:
                return Color.BLACK
            else:
                return Color.WHITE

        elif hsv[0] < 30:
            return Color.RED

        elif hsv[0] < 90:
            return Color.YELLOW

        elif hsv[0] < 163:
            return Color.GREEN

        elif hsv[0] < 283:
            return Color.BLUE

        else:
            return Color.RED

    def rgb(self):
        time.sleep(SENSOR_DELAY)
        rgb = list(self.sensor.value())
        for i in range(3):
            rgb[i] = int(rgb[i])
        return rgb

    @property
    def hsv(self):
        time.sleep(SENSOR_DELAY)
        hsv = list(self.sensor.valueHSV())
        for i in range(3):
            hsv[i] = int(hsv[i])
        return hsv

# TODO how to test to see if Gyro actually in the port selected???
class GyroSensor:
    def __init__(self, port, positive_direction=Direction.CLOCKWISE):
        if positive_direction is not Direction.CLOCKWISE:
            print("ERROR: cannot set positive_direction in this virtual environment")        
        self.port = port
        self.positive_direction = positive_direction 
        self.sensor = simPython.GyroSensor(port)

    def speed(self):
        # rate is actually: angularVelocity
        time.sleep(SENSOR_DELAY)
        rate = self.angle_and_rate()
        return rate[1]

    def angle(self):
        # angle() is in degrees
        time.sleep(SENSOR_DELAY)        
        angle = self.angle_and_rate()
        return angle[0]   

    def angle_and_rate(self):
        time.sleep(SENSOR_DELAY)
        angle_and_rate = list(self.sensor.angleAndRate())
        for i in range(2):
            angle_and_rate[i] = int(angle_and_rate[i])
        return angle_and_rate

    #def reset_angle(self, angle):
    #    if angle is not None:
    #        print("ERROR: cannot set Gyro to specified angle in this virtual environment")
    def reset_angle(self):            
        self.sensor.reset()
        return  
