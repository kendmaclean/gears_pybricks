# Needed to prevent loops from locking up the javascript thread
SENSOR_DELAY = 0.001

# Import the necessary libraries
import simPython, time
import math
from ev3dev2.motor import *

print("### pybrick.py imported ###")
def straight(distance):
    tank_drive = MoveTank(OUTPUT_A, OUTPUT_B)
    tank_drive.on(distance, distance) 
    print("### MoveStraight ###; distance_var " + str(distance))

'''
class MoveStraight():
    def __init__(self, distance):
        tank_drive = MoveTank(OUTPUT_A, OUTPUT_B)
        tank_drive.on(distance, distance) 
        print("### MoveStraight ###; distance_var " + str(distance))
''''
'''
classMotor()
    def __init__(self, port, positive_direction=Direction.CLOCKWISE, gears=None)
        self.port = port
        self.positive_direction = positive_direction        
        self.gears = gears
 
classDriveBase()
    def __init__(self, left_motor, right_motor, wheel_diameter, axle_track)
        self.left_motor = left_motor.port
        self.right_motor = right_motor.port
        self.wheel_diameter = wheel_diameter
        self.axle_track = axle_track

    def straight(distance):
        tank_drive = MoveTank(OUTPUT_A, OUTPUT_B)
        tank_drive.on(distance, distance) 
        print("### MoveStraight ###; distance_var " + str(distance))
'''
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
