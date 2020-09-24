#!/usr/bin/env python3

# Import the necessary libraries
import math
import time
from pybricks.ev3devices import *
from pybricks.parameters import *
from pybricks.robotics import *

left_motor = MotorP(PortP.A)
right_motor = MotorP(PortP.B)
robot = DriveBase(left_motor, right_motor, wheel_diameter=56, axle_track=152)
robot.settings(straight_speed=200, straight_acceleration=100, turn_rate=100, turn_acceleration=100)

gyro_sensor = GyroSensorP(PortP.S3)
color_sensor_in1 = ColorSensorP(PortP.S1)
color_sensor_in2 = ColorSensorP(PortP.S2)



# Here is where your code starts

robot.drive(200, 15)
while True:
  print(color_sensor_in1.reflection())
  time.sleep(0.25)
