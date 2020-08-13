# Needed to prevent loops from locking up the javascript thread
SENSOR_DELAY = 0.001

# Import the necessary libraries
import simPython, time
import math
from ev3dev2.motor import *

from pybricks.ev3devices import *
from pybricks.robotics import *

print("### robotics.py imported ###")
def straight(speed):
    tank_drive = MoveTank(OUTPUT_A, OUTPUT_B)
    tank_drive.on(speed, speed) 
    print("### MoveStraight ###; speed " + str(speed))

class DriveBase:
    SMALLEST_TIRE = 50
    LARGEST_TIRE = 300
    SMALLEST_AXLE_TRACK = 100
    LARGEST_AXLE_TRACK = 300
    MAX_SPEED = 300
    MAX_ACCEL = 100
    MAX_DEGREES = 360

    def __init__(self, left_motor, right_motor, wheel_diameter, axle_track):
        if isinstance(left_motor, MotorP):
            self.left_motor = left_motor
        else:
            raise TypeError("left_motor not of type MotorP")
        if isinstance(right_motor, MotorP):
            self.right_motor = right_motor
        else:
            raise TypeError("right_motor not of type MotorP")
        if DriveBase.SMALLEST_TIRE <= wheel_diameter <= DriveBase.LARGEST_TIRE:
            self.wheel_diameter = wheel_diameter
        else:
            raise ValueError("Wheel diameter must be between " +
            DriveBase.SMALLEST_TIRE + "mm and " + DriveBase.LARGEST_TIRE + "mm")
        if DriveBase.SMALLEST_AXLE_TRACK <= axle_track <= DriveBase.LARGEST_AXLE_TRACK:
            self.axle_track = axle_track
        else:
            raise ValueError("Wheel diameter must be between " +
            DriveBase.SMALLEST_AXLE_TRACK + "mm and " + DriveBase.LARGEST_AXLE_TRACK + "mm")                

    def settings(self, straight_speed=100, straight_acceleration=100, turn_rate=0, turn_acceleration=100):
        if -DriveBase.MAX_SPEED <= straight_speed <= DriveBase.MAX_SPEED:
            self.straight_speed = straight_speed
        else:
            raise ValueError("straight_speed outside allowable bounds")
        if -DriveBase.MAX_ACCEL <= straight_acceleration <= DriveBase.MAX_ACCEL:
            self.straight_acceleration = straight_acceleration
        else:
            raise ValueError("straight_acceleration outside allowable bounds")
        if -DriveBase.MAX_DEGREES <= turn_rate <= DriveBase.MAX_DEGREES:
            self.turn_rate = turn_rate
        else:
            raise ValueError("turn_rate outside allowable bounds")        
        self.turn_acceleration = turn_acceleration
        if -DriveBase.MAX_ACCEL <= turn_acceleration <= DriveBase.MAX_ACCEL:
            self.turn_acceleration = turn_acceleration
        else:
            raise ValueError("turn_acceleration outside allowable bounds")        

    def straight(self, speed):
        tank_drive = MoveTank(self.left_motor.port, self.right_motor.port)
        tank_drive.on(speed, speed) 
        print("### MoveStraight ###; speed " + str(speed))
        print("### left_motor " + self.left_motor.port)        
        print("### right_motor " + self.right_motor.port)     

    def turn(self, angle):
        self.angle = angle        
        print("not implemented")

    def drive(self, drive_speed, turn_rate):
        self.drive_speed = drive_speed
        self.turn_rate = turn_rate    
        print("not implemented")

    def stop(self):
        print("not implemented")

    def distance(self):
        print("not implemented")       
        return self.distance

    def angle(self):
        print("not implemented")      
        return self.angle        
      
    def state(self):
        print("not implemented")       
        return (self.distance, self.drive_speed, self.angle, self.turn_rate)

    def reset(self):
        self.distance = 0
        self.angle = 0    



'''
# works
class MoveStraight(MotorSet):
    def __init__(self, left_motor_port, right_motor_port, desc=None, motor_class=LargeMotor):
        motor_specs = {
        left_motor_port : motor_class,
        right_motor_port : motor_class,
        }
        MotorSet.__init__(self, motor_specs, desc)
        self.left_motor = self.motors[left_motor_port]
        self.right_motor = self.motors[right_motor_port]
        self.max_speed = self.left_motor.max_speed

    def on(self, left_speed):
        if not isinstance(left_speed, SpeedValue):
            if -100 <= left_speed <= 100:
                left_speed_obj = SpeedPercent(left_speed)
                left_speed_var = int(round(left_speed_obj.to_native_units(self.left_motor)))
            else:
                raise Exception("Invalid Speed Percentage. Speed must be between -100 and 100)")
        else:
            left_speed_var = int(round(left_speed.to_native_units(self.left_motor)))

        self.left_motor.speed_sp = left_speed_var
        self.right_motor.speed_sp = left_speed_var
        self.left_motor.run_forever()
        self.right_motor.run_forever()
'''
