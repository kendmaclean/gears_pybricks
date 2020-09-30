#!/usr/bin/env python3

# Proportional line follower
# adapted from: https://github.com/holyokecodes/FLL-2020/blob/master/programs/comboOne.py
# when turn rate gets too high, slow down the robot so that color sensor can catch up
# should be able to make speed reduction/acceleration proportional too

# see also: https://medium.com/kidstronics/lego-a-proportional-linefollower-robot-d9a951debbef

# Import the necessary libraries
import math
import time
from pybricks.ev3devices import *
from pybricks.parameters import *
from pybricks.robotics import *
from pybricks.tools import wait
from pybricks.hubs import EV3Brick

ev3 = EV3Brick()
motorA = Motor(Port.A)
motorB = Motor(Port.B)
left_motor = motorA
right_motor = motorB
robot = DriveBase(left_motor, right_motor, wheel_diameter=56, axle_track=152)

color_sensor_in1 = ColorSensor(Port.S1)
obstacle_sensor = UltrasonicSensor(Port.S2)
gyro_sensor= GyroSensor(Port.S3)



# Here is where your code starts



def comboOne(robot, ev3, library):
    ## Use Library for line following to get to treadmill? Line follow to get back for sure

    robot.straight(100)
    robot.turn(90)
    robot.straight(240)    
    library.line_follow_for_distance(p=-.9, distance=1450)
    robot.straight(220)
    pass

class FUNCTION_LIBRARY:
    def __init__(self, robot, ev3, left_motor, right_motor, color_sensor):
        #self, DriveBase, Hub
        self.robot = robot
        self.hub = ev3
        self.left_motor = left_motor
        self.right_motor = right_motor
        self.color_sensor = color_sensor
        
        self.robot.reset()

    def line_follow_for_distance(self, p=1, DRIVE_SPEED=150, BLACK=7, WHITE= 88, distance=1000, debug=True):
        print("===starting")

        PROPORTIONAL_GAIN = p
        threshold = (BLACK + WHITE) / 2 #the center of black+white
        low_color = 50
        high_color = 50
        speed = DRIVE_SPEED
        
        while self.robot.distance() < distance: 
            reflection = self.color_sensor.reflection()
            if reflection < low_color:
                low_color = reflection
            if reflection > high_color:
                high_color = reflection              

            
            #Calculate the turn rate from the devation and set the drive base speed and turn rate.
            turn = PROPORTIONAL_GAIN * (reflection - threshold)
            if turn > 25:
                speed = max(speed .7 , DRIVE_SPEED / 2) # slow robot down
            else:
                speed = min(speed * 1.2, DRIVE_SPEED) # speed robot up
                
            print ("turn " + str(turn) + " ;speed " + str(speed))
            self.robot.drive(speed, turn)

        self.robot.stop()
        print("I have reached " + str(math.floor(distance/25.4)) + "inches")

        print ("low_color " + str(low_color))
        print ("high_color " + str(high_color)) 
        
##################################
library = FUNCTION_LIBRARY(robot, ev3, left_motor, right_motor, color_sensor_in1)
comboOne(robot, ev3, library)