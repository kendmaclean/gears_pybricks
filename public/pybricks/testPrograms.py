testPrograms


robot.drive(200, 0)
while (robot.angle()) < 100:
  print("=====")
  time.sleep(0.25)  
robot.stop()


===================================
from pybricks.ev3devices import *
from pybricks.parameters import *
from pybricks.robotics import *

robot = DriveBase(left_motor, right_motor, wheel_diameter=56, axle_track=152)
robot.settings(straight_speed=200, straight_acceleration=100, turn_rate=100, turn_acceleration=100)

tank_drive.on(20, 20)
while (robot.angle()) < 100:
  time.sleep(0.25)
  print('angle')
  print(robot.angle())
robot.stop()

============================

#!/usr/bin/env python3

# Import the necessary libraries
import math
import time
from ev3dev2.motor import *
from ev3dev2.sound import Sound
from ev3dev2.sensor import *
from ev3dev2.sensor.lego import *
from ev3dev2.sensor.virtual import *

from pybricks.ev3devices import *
from pybricks.parameters import *
from pybricks.robotics import *

# Create the sensors and motors objects
motorA = LargeMotor(OUTPUT_A)
motorB = LargeMotor(OUTPUT_B)
left_motor = motorA
right_motor = motorB
tank_drive = MoveTank(OUTPUT_A, OUTPUT_B)
steering_drive = MoveSteering(OUTPUT_A, OUTPUT_B)

spkr = Sound()

color_sensor_in1 = ColorSensor(INPUT_1)
ultrasonic_sensor_in2 = UltrasonicSensor(INPUT_2)
gyro_sensor_in3 = GyroSensor(INPUT_3)
gps_sensor_in4 = GPSSensor(INPUT_4)

motorC = LargeMotor(OUTPUT_C) # Magnet

# Here is where your code starts



left_motorP = MotorP(PortP.A)
right_motorP = MotorP(PortP.B)
robot = DriveBase(left_motorP, right_motorP, wheel_diameter=56, axle_track=152)
robot.settings(straight_speed=200, straight_acceleration=100, turn_rate=100, turn_acceleration=100)

tank_drive.on(20, 20)
while (robot.angle()) < 100:
  time.sleep(0.25)
  print('angle')
  print(robot.angle())
robot.stop()


===================================
robot.angle()
robot.drive(200,45)
time.sleep(5)
robot.angle()
robot.stop()