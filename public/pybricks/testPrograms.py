#!/usr/bin/env python3

# Import the necessary libraries
import math
import time
from pybricks.ev3devices import *
from pybricks.parameters import *
from pybricks.robotics import *

left_motor = Motor(Port.A)
right_motor = Motor(Port.B)
robot = DriveBase(left_motor, right_motor, wheel_diameter=56, axle_track=152)
robot.settings(straight_speed=200, straight_acceleration=100, turn_rate=100, turn_acceleration=100)

gyro_sensor = GyroSensorP(Port.S3)
color_sensor_in1 = ColorSensorP(Port.S1)
color_sensor_in2 = ColorSensorP(Port.S2)



# Here is where your code starts

robot.drive(200, 15)
while True:
  print(color_sensor_in1.reflection())
  time.sleep(0.25)


=======================

robot.drive(200, 0)
while True:
  print('=======')
  print(motorA.speed())
  motorSpeed_dps = motorA.speed() 
  motorMMperSec = (motorSpeed_dps / 360) * 56
  print("angle" + str(motorA.angle()))
  print("motorSpeed_dps" + str(motorSpeed_dps) ) 
  print("motorMMperSec" + str(motorMMperSec)  )
  rotations = motorA.angle() / 360

  print("rotations" + str(rotations))  
  if rotations > 2:
    motorA.reset_angle(0)
    rotations = 0
  wait(250)


  ===========================


  robot.straight(500)


print('=======')
motorSpeed_dps = motorA.speed() 

print("angle_degrees" + str(motorA.angle()))
print("motorSpeed_dps" + str(motorSpeed_dps) ) 

rotations = motorA.angle() / 360
print("rotations" + str(rotations))  
print("distance" + str(rotations * 56) + "mm")  