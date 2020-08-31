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

def straight(speed):
    tank_drive = MoveTank(OUTPUT_A, OUTPUT_B)
    tank_drive.on(speed, speed) 

class DriveBase:
    SMALLEST_TIRE_DIAMETER = 1.0
    LARGEST_TIRE_DIAMETER = 150.0
    SMALLEST_AXLE_TRACK = 20.0
    LARGEST_AXLE_TRACK = 250.0
    MAX_SPEED = 300
    MAX_ACCEL = 100
    MAX_DEGREES = 360

    def __init__(self, left_motor, right_motor, wheel_diameter, axle_track):
        print("robotics wheel_diameter:"  + str(wheel_diameter) + "; axle_track:"  + str(axle_track))

        if isinstance(left_motor, MotorP):
            self.left_motor = left_motor
        else:
            raise TypeError("left_motor not of type MotorP")
        if isinstance(right_motor, MotorP):
            self.right_motor = right_motor
        else:
            raise TypeError("right_motor not of type MotorP")

        if not math.isclose(wheel_diameter, left_motor.wheelDiameter) or not math.isclose(wheel_diameter, right_motor.wheelDiameter):
            print("DriveBase wheel diameter of " + str(wheel_diameter) + " not the same as described in robotTemples.js: " + 
            str(left_motor.wheelDiameter) + "... using value from robotTemplates.js")
            wheel_diameter = left_motor.wheelDiameter

        if DriveBase.SMALLEST_TIRE_DIAMETER <= wheel_diameter <= DriveBase.LARGEST_TIRE_DIAMETER:
            self.wheel_diameter = wheel_diameter
        else:
            raise ValueError("Wheel circumference must be between " +
            DriveBase.SMALLEST_TIRE_DIAMETER + "mm and " + DriveBase.LARGEST_TIRE_DIAMETER + "mm")

        self.wheel_circ = self.wheel_diameter * math.pi

        if not math.isclose(axle_track, left_motor.axleTrack) or not math.isclose(axle_track, right_motor.axleTrack):
            print("DriveBase wheel axle_track of " + str(axle_track) + " not the same as described in robotTemples.js: " + 
            str(left_motor.axleTrack) + "... using value from robotTemplates.js")
            axle_track = left_motor.axleTrack

        if DriveBase.SMALLEST_AXLE_TRACK <= axle_track <= DriveBase.LARGEST_AXLE_TRACK:
            self.axle_track = axle_track
        else:
            if DriveBase.SMALLEST_AXLE_TRACK < axle_track:
                self.axle_track = DriveBase.SMALLEST_AXLE_TRACK        
            if axle_track > DriveBase.LARGEST_AXLE_TRACK:
                self.axle_track = DriveBase.LARGEST_AXLE_TRACK
            raise ValueError("Wheel diameter must be between " +
            DriveBase.SMALLEST_AXLE_TRACK + "mm and " + DriveBase.LARGEST_AXLE_TRACK + "mm")                

        self.settings()

    def settings(self, straight_speed=50, straight_acceleration=100, turn_rate=25, turn_acceleration=100):
        # TODO straight_speed is a percentage of motor top speed!!!!!!
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


    def straight(self, distance):
        tank_drive = MoveTank(self.left_motor.port, self.right_motor.port)
        rotations = distance / self.wheel_circ
        rot_degrees = rotations * 360
        print( "rotations: " + str(rotations) )  
        tank_drive.on_for_degrees(self.straight_speed, self.straight_speed, rot_degrees, brake=False, block=True)

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
        steering_drive = MoveSteering(self.left_motor.port, self.right_motor.port)

        if angle > 0:
            steering = 100
        else:
            steering = -100

        robot_circumference_of_turn = self.axle_track * math.pi
        wheel_circumference = self.wheel_diameter * math.pi
        degrees = angle * (robot_circumference_of_turn / wheel_circumference)

        steering_drive.on_for_degrees(steering, self.turn_rate, degrees)

    """
    def straight(self, distance):
        tank_drive = MoveTank(self.left_motor.port, self.right_motor.port)

        while tank_drive.right_motor.position < distance and tank_drive.left_motor.position < distance:
            tank_drive.on(self.straight_speed, self.straight_speed)
            print( "left_motor position: " + str(tank_drive.left_motor.position ))   
            #time.sleep(0.25)             

        tank_drive.stop()
    """

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
