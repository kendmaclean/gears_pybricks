#!/usr/bin/env python3

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
robot.settings(straight_speed=200,turn_rate=100)

color_sensor_in1 = ColorSensor(Port.S1)
color_sensor_in2 = ColorSensor(Port.S2)
obstacle_sensor = UltrasonicSensor(Port.S3)
gyro_sensor= GyroSensor(Port.S4)

motorC = Motor(Port.C) # Magnet

# Here is where your code starts


gyro_sensor.reset_angle(0)
left_angle = 180
right_angle = 1 # dont use zero
left_motor.run_angle(25,left_angle,Stop.HOLD,False)
right_motor.run_angle(25,right_angle,Stop.HOLD,True)
print("waiting")
wait(8000)
print("done waiting")
wheel_angle_diff = left_angle - right_angle
print("wheel_angle_diff ",wheel_angle_diff)
axle_track=152
ratio = wheel_angle_diff / axle_track
print("ratio ",ratio)

radians = math.atan(ratio)
angle = math.degrees(radians)
print("calculated angle ",angle)

angleOut = gyro_sensor.angle()
print("gyro phase I ",angleOut)

print("robot.angle ",robot.angle() )


===========================

#!/usr/bin/env python3

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
robot.settings(straight_speed=200,turn_rate=100)

color_sensor_in1 = ColorSensor(Port.S1)
color_sensor_in2 = ColorSensor(Port.S2)
obstacle_sensor = UltrasonicSensor(Port.S3)
gyro_sensor= GyroSensor(Port.S4)

motorC = Motor(Port.C) # Magnet

# Here is where your code starts

DRIVE_SPEED = 200
PROPORTIONAL_GAIN = 1.2
INTEGRAL_GAIN = 0.1
DERIVATIVE_GAIN = 0.2
integral = 0
derivative = 0
last_error = 0

robot.drive(25, 0)
wait(30)
robot.drive(50, 0)
wait(30)
robot.drive(75, 0)
wait(30)
robot.drive(100, 0)
wait(30)
# Start following the line endlessly.
while True:
    # Calculate the deviation from the threshold.
    error = color_sensor_in1.reflection() - color_sensor_in2.reflection() 
    integral = integral + error
    derivative = error - last_error

    turn_rate = PROPORTIONAL_GAIN * error + INTEGRAL_GAIN * integral + DERIVATIVE_GAIN * derivative
    print("turn_rate " + str(turn_rate))
    # Set the drive base speed and turn rate.
    robot.drive(DRIVE_SPEED, turn_rate)
    last_error = error
    # You can wait for a short time or do other things in this loop.
    wait(10)

