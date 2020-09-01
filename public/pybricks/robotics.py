# Needed to prevent loops from locking up the javascript thread
SENSOR_DELAY = 0.001

# Import the necessary libraries
import simPython
import math, time
from ev3dev2.motor import *

from pybricks.ev3devices import *
from pybricks.robotics import *

from ev3dev2.motor import *
from ev3dev2.sound import Sound
from ev3dev2.sensor import *
from ev3dev2.sensor.lego import *
from ev3dev2.sensor.virtual import *

class DriveBase:
    SMALLEST_TIRE_DIAMETER = 10
    LARGEST_TIRE_DIAMETER = 200
    SMALLEST_AXLE_TRACK = 20
    LARGEST_AXLE_TRACK = 250
    MAX_SPEED = 1000
    MAX_ACCEL = 300
    MAX_DEGREES = 360

    def __init__(self, left_motor, right_motor, wheel_diameter, axle_track):
        # TODO are these checks even required since motor.py does them anyway...
        def check_motors():
            if isinstance(left_motor, MotorP):
                self.left_motor = left_motor
            else:
                raise TypeError("left_motor not of type MotorP")
            if isinstance(right_motor, MotorP):
                self.right_motor = right_motor
            else:
                raise TypeError("right_motor not of type MotorP")
        def check_wheel_diameter():
            if not math.isclose(wheel_diameter, left_motor.wheelDiameter) or not math.isclose(wheel_diameter, right_motor.wheelDiameter):
                print("DriveBase wheel diameter of " + str(wheel_diameter) + " not the same as described in robotTemples.js: " + 
                str(left_motor.wheelDiameter) + "overriding robotTemplates.js")
            if DriveBase.SMALLEST_TIRE_DIAMETER <= wheel_diameter <= DriveBase.LARGEST_TIRE_DIAMETER:
                self.wheel_diameter = wheel_diameter
            else:
                raise ValueError("Wheel circumference must be between " + 
                str(DriveBase.SMALLEST_TIRE_DIAMETER) + "mm and " + str(DriveBase.LARGEST_TIRE_DIAMETER) + "mm")
        def check_axle_track():
            if not math.isclose(axle_track, left_motor.axleTrack) or not math.isclose(axle_track, right_motor.axleTrack):
                print("DriveBase wheel axle_track of " + str(axle_track) + " not the same as described in robotTemples.js: " + 
                str(left_motor.axleTrack) + "overriding robotTemplates.js")

            if DriveBase.SMALLEST_AXLE_TRACK <= axle_track <= DriveBase.LARGEST_AXLE_TRACK:
                self.axle_track = axle_track
            else:
                if DriveBase.SMALLEST_AXLE_TRACK < axle_track:
                    self.axle_track = DriveBase.SMALLEST_AXLE_TRACK        
                if axle_track > DriveBase.LARGEST_AXLE_TRACK:
                    self.axle_track = DriveBase.LARGEST_AXLE_TRACK
                raise ValueError("Wheel diameter must be between " +
                str(DriveBase.SMALLEST_AXLE_TRACK) + "mm and " + str(DriveBase.LARGEST_AXLE_TRACK) + "mm")       
       
        check_motors()
        check_wheel_diameter()
        self.wheel_circumference = self.wheel_diameter * math.pi   # in millimetres          
        check_axle_track()
        self.settings()

    def settings(self, straight_speed=200, straight_acceleration=100, turn_rate=100, turn_acceleration=100):
        def check_straight_speed():
            if -DriveBase.MAX_SPEED <= straight_speed <= DriveBase.MAX_SPEED:
                # straight_speed in mm/s
                self.straight_speed = straight_speed
            else:
                raise ValueError("straight_speed outside allowable bounds")
        def check_straight_acceleration():
            if -DriveBase.MAX_ACCEL <= straight_acceleration <= DriveBase.MAX_ACCEL:
                self.straight_acceleration = straight_acceleration
            else:
                raise ValueError("straight_acceleration outside allowable bounds")
        def check_turn_rate():
            if -DriveBase.MAX_DEGREES <= turn_rate <= DriveBase.MAX_DEGREES:
                self.turn_rate = turn_rate
            else:
                raise ValueError("turn_rate outside allowable bounds")   
        def check_turn_acceleration():       
            if -DriveBase.MAX_ACCEL <= turn_acceleration <= DriveBase.MAX_ACCEL:
                self.turn_acceleration = turn_acceleration
            else:
                raise ValueError("turn_acceleration outside allowable bounds")        

        check_straight_speed()
        check_straight_acceleration()        
        check_turn_rate()    
        check_turn_acceleration()               

    def straight(self, distance):
        def getSpeedDPSObj(): # Speed in degrees-per-second
            rotations = self.straight_speed / self.wheel_circumference
            degrees = rotations * 360
            return SpeedDPS(degrees)
        def getDistanceInDegrees():
            dist_in_rotations = distance / self.wheel_circumference
            return dist_in_rotations * 360

        tank_drive = MoveTank(self.left_motor.port, self.right_motor.port)
        speedDPS_obj = getSpeedDPSObj()
        tank_drive.on_for_degrees(speedDPS_obj, speedDPS_obj, getDistanceInDegrees(), brake=True, block=True)

    '''
        Robot:
        b = robotWheelbase
        Cturn = (b * pi) = circumferenceOfTurn
        dCT = distanceAlongTurn
        theta = desired robot angle of rotation

        Wheel:
        dw = wheel diameter
        Cwheel = (dw * pi) = wheelCircumference
        alpha = motor angle of rotation

        goal: get robot to turn a set number of degrees (theta) by turning the power wheels
            in opposite directions

        relations:
        robot:
            theta /3 60degrees = dCT / Cturn

            solve for dCT:

            dCT = (theta / 360degrees) / Cturn

        wheels:
        dCT is the distance the wheels must rotate along Cturn

        therefore:

            motorRotationsInDegrees = 360degrees * (dCT / Cturn)

        replace dCT with above formula:

            motorRotationsInDegrees = 360degrees * (theta / 360degrees) / Cturn) / Cwheel

        reduce:

            motorRotationsInDegrees =  theta * (Cturn / Cwheel)     

        # see: https://sheldenrobotics.com/tutorials/Detailed_Turning_Tutorial.pdf    
    '''
    def turn(self, angle):
        def getSpeedDPSObj(): # turn_rate (speed) in degrees-per-second
            rotations = self.turn_rate / self.wheel_circumference
            degrees = rotations * 360
            return SpeedDPS(degrees)        
        def getTurnInDegrees():
            robot_circumference_of_turn = self.axle_track * math.pi
            return angle * (robot_circumference_of_turn / self.wheel_circumference)

        steering_drive = MoveSteering(self.left_motor.port, self.right_motor.port)
        steering=100
        # turn rate is still in percentages!!!
        steering_drive.on_for_degrees(steering, getSpeedDPSObj(), getTurnInDegrees(), brake=True, block=True)

    ###########################################################################

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



