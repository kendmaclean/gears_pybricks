# Needed to prevent loops from locking up the javascript thread
SENSOR_DELAY = 0.001

# Import the necessary libraries

import math, time
import simPython
import ev3dev2.motor
from pybricks.ev3devices import *

class DriveBase:
    # TODO rationalize the assertions which use these constants - duplicates
    # what is already done elsewhere
    # TODO these are initial guesses
    SMALLEST_TIRE_DIAMETER = 10
    LARGEST_TIRE_DIAMETER = 200
    SMALLEST_AXLE_TRACK = 20
    LARGEST_AXLE_TRACK = 250
    MAX_SPEED = 1000
    MAX_ACCEL = 500

    def __init__(self, left_motor, right_motor, wheel_diameter, axle_track):
        def check_motors():
            '''
                see: EV3DEV2 on github:
                     lego-linux-drivers/motors/tacho_motor_class.c
                     lego-linux-drivers/ev3/legoev3_motor.c
                lego motor specs: https://www.philohome.com/motors/motorcomp.html   
            '''         
            if isinstance(left_motor, Motor):
                self.left_motor = left_motor
            else:
                raise TypeError("left_motor not of type Motor")

            if isinstance(right_motor, Motor):
                self.right_motor = right_motor
            else:
                raise TypeError("right_motor not of type Motor")

        def check_wheel_diameter():
            if not math.isclose(wheel_diameter, left_motor.wheelDiameter) or not math.isclose(wheel_diameter, right_motor.wheelDiameter):
                print("DriveBase wheel diameter of " + str(wheel_diameter) + " not the same as described in robotTemples.js: " + 
                str(left_motor.wheelDiameter) + "overriding robotTemplates.js")

            if DriveBase.SMALLEST_TIRE_DIAMETER <= wheel_diameter <= DriveBase.LARGEST_TIRE_DIAMETER:
                self.wheel_diameter = wheel_diameter # in mm; robotTemplates uses cm
                self.wheel_radius = left_motor.wheelRadius # in mm; robotTemplates uses cm
            else:
                raise ValueError("Wheel circumference must be between " + 
                str(DriveBase.SMALLEST_TIRE_DIAMETER) + "mm and " + str(DriveBase.LARGEST_TIRE_DIAMETER) + "mm")

        def check_axle_track():
            if not math.isclose(axle_track, self.robot.axle_track()):
                print("DriveBase wheel axle_track of " + str(axle_track) + " not the same as described in robotTemples.js: " + 
                str( self.robot.axle_track) + "overriding robotTemplates.js")

            if DriveBase.SMALLEST_AXLE_TRACK <= axle_track <= DriveBase.LARGEST_AXLE_TRACK:
                self.axle_track = axle_track
            else:
                if DriveBase.SMALLEST_AXLE_TRACK < axle_track:
                    self.axle_track = DriveBase.SMALLEST_AXLE_TRACK        
                if axle_track > DriveBase.LARGEST_AXLE_TRACK:
                    self.axle_track = DriveBase.LARGEST_AXLE_TRACK
                raise ValueError("Wheel diameter must be between " +
                str(DriveBase.SMALLEST_AXLE_TRACK) + "mm and " + str(DriveBase.LARGEST_AXLE_TRACK) + "mm")       
       
        self.robot = simPython.Robot()

        check_motors()
        self.tank_drive = ev3dev2.motor.MoveTank(left_motor.port, right_motor.port) 

        check_wheel_diameter()
        self.wheel_circumference = self.wheel_diameter * math.pi   # in millimetres          
        check_axle_track()
        self.settings()

        self.pivot_turn_angle = 0
        self.drive_speed = 0 
        self.straight_speed = 0

    def settings(self, straight_speed=200, straight_acceleration=100, turn_rate=100, turn_acceleration=100):
        '''
            # TODO straight_acceleration and turn_acceleration not implemented
            # TODO how to get this so that settings is paired with straight or turn command; but only once 
            # TODO only really need settings delta from the one set up by default in virtual environment - i.e. if
            # user only changes straight_speed, don't need to reset all other parms - creates too much visual noise


            straight_speed (speed: mm/s) – Speed of the robot during straight().
            straight_acceleration (linear acceleration: mm/s/s) – Acceleration and deceleration 
                of the robot at the start and end of straight().
            turn_rate (rotational speed: deg/s) – Turn rate of the robot during turn().
            turn_acceleration (rotational acceleration: deg/s/s) – Angular acceleration and deceleration 
                of the robot at the start and end of turn().            
        '''
        # TODO are these max checks/assertions even required
        def check_straight_speed():
            if -DriveBase.MAX_SPEED <= straight_speed <= DriveBase.MAX_SPEED:
                self.straight_speed = straight_speed
            else:
                if straight_speed < - DriveBase.MAX_SPEED:
                    self.straight_speed = - DriveBase.MAX_SPEED        
                if straight_speed > DriveBase.MAX_SPEED:
                    self.straight_speed = DriveBase.MAX_SPEED     
                print("Warning: straight_speed outside allowable bounds +/- " + str(DriveBase.MAX_SPEED)) 

        def check_straight_acceleration():
            # do not allow negative value for acceleration
            if 0 <= straight_acceleration <= DriveBase.MAX_ACCEL:
                self.straight_acceleration = straight_acceleration
            else:
                if straight_acceleration < 0:
                    self.straight_acceleration = 0        
                if straight_acceleration > DriveBase.MAX_ACCEL:
                    self.straight_acceleration = DriveBase.MAX_ACCEL     
                print("Warning: straight_acceleration outside allowable bounds of 0 and " + str(DriveBase.MAX_ACCEL))   

        #def check_turn_acceleration():       
        #    print("Warning: turn_acceleration not implemented")

        check_straight_speed()
        check_straight_acceleration()
        self.settings_turn_rate = turn_rate
        #check_turn_acceleration() # not implemented

    def straight(self, distance):
        speed_sp_mms = self.straight_speed # millimetres per second
        speed_sp_rps = speed_sp_mms / self.wheel_circumference # rotations per second (wheel_circumference = wheel_diameter * pi)
        speed_sp_dps = speed_sp_rps * 360 # dps = degrees per second

        accel_sp_mms = self.straight_acceleration # only looking at mm/s not true acceleration of mm/s/s here
        accel_sp_rps = accel_sp_mms / self.wheel_circumference
        accel_sp_dps = accel_sp_rps * 360

        def robot_speed():
            return (self.left_motor.wheel.speed_sp() + self.right_motor.wheel.speed_sp() ) / 2

        def rampTime(): # 
            '''
                ramp_up_sp 
                Units are in milliseconds and must be positive. When set to a non-zero
                value, the motor speed will increase from 0 to 100% of ``max_speed``
                over the span of this setpoint.         

                The actual ramp time is [the ratio of [the difference between the ``speed_sp`` 
                and the current ``speed``] and max_speed] multiplied by ``ramp_up_sp``. 

                see: https://github.com/ev3dev/lego-linux-drivers/blob/ev3dev-buster/motors/tacho_motor_class.c
            '''
            speed = robot_speed()
            max_speed = self.left_motor.max_dps
            ramp_up_sp = accel_sp_dps
            ramp_time =( (speed_sp_dps - speed) / max_speed ) * ramp_up_sp

            segments_per_second = 10            
            wait_per_segment = (ramp_time / segments_per_second) / 1000 # convert milliseconds to seconds
            speed_segment_deg = accel_sp_dps / segments_per_second 

            return (wait_per_segment, speed_segment_deg)

        # TODO what about going reverse
        def ramp_up(wait_per_segment, speed_segment_deg):
            '''
                ``ramp_up_sp`` (sp = set point - i.e. user's target value)
                    Writing sets the 'ramp up setpoint'. Reading returns the current value.
                    Units are in milliseconds and must be positive. When set to a non-zero
                    value, the motor speed will increase from 0 to 100% of ``max_speed``
                    over the span of this setpoint. 
                    
                    The actual ramp time is the ratio of the difference between the ``speed_sp`` 
                    and the current ``speed`` and max_speed[;] multiplied by ``ramp_up_sp``. 
                    
                    Values must not be negative.

                    see: https://github.com/ev3dev/lego-linux-drivers/blob/ev3dev-buster/motors/tacho_motor_class.c
            '''
            start_distance = self.distance()            
 
            max_speed = self.left_motor.max_speed
            speed = speed_segment_deg   
            while robot_speed() <= speed_sp_dps and speed < max_speed and self.distance() <= distance:
                self.left_motor.run(speed)
                self.right_motor.run(speed)
                time.sleep(wait_per_segment)                 
                speed = speed + speed_segment_deg

            return self.distance() - start_distance # ramp_distance

        def ramp_down(wait_per_segment, speed_segment_deg):
            '''
                ``ramp_down_sp``
                    Writing sets the ramp down setpoint. Reading returns the current
                    value. Units are in milliseconds and must be positive. When set to a
                    non-zero value, the motor speed will decrease from 0 to 100% of
                    ``max_speed`` over the span of this setpoint. The actual ramp time is
                    the ratio of the difference between the ``speed_sp`` and the current
                    ``speed`` and ``max_speed`` multiplied by ``ramp_down_sp``. Values
                    must not be negative.

                    see: https://github.com/ev3dev/lego-linux-drivers/blob/ev3dev-buster/motors/tacho_motor_class.c 
            '''
            speed = speed_sp_dps - speed_segment_deg
            while robot_speed() >= 0 and speed >= 0 and self.distance() <= distance:
                self.left_motor.run(speed)
                self.right_motor.run(speed)
                speed = speed - speed_segment_deg
                time.sleep(wait_per_segment) 

            if speed < 0:
                self.left_motor.run(0)
                self.right_motor.run(0)
                time.sleep(wait_per_segment) 

            self.left_motor.stop()
            self.right_motor.stop()

        (wait_per_segment, speed_segment_deg) = rampTime()
        ramp_up_distance = ramp_up(wait_per_segment, speed_segment_deg)

        # not an exact approach but good enough for a simulation; ramp_down_distances 
        # are always a bit shorter than ramp_up_distance because of distance measurement lag,

        # note: self.distance() already includes the distance travelled by ramp_up        
        remaining_distance = distance - ramp_up_distance

        if remaining_distance > ramp_up_distance:
            while self.distance() < remaining_distance:
                self.left_motor.run(speed_sp_dps)
                self.right_motor.run(speed_sp_dps)

                time.sleep(wait_per_segment) 
                
        ramp_down(wait_per_segment, speed_segment_deg)

        self.reset()

    def turn(self, angle): # pivot turn
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
                theta / 360degrees = dCT / Cturn

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
        def get_speed_steering(steering, speed):
            if steering > 100 or steering < -100:
                raise ValueError("Invalid Steering Value. Between -100 and 100 (inclusive).")

            # Assumes left motor's speed stats are the same as the right motor's
            if not isinstance(speed, ev3dev2.motor.SpeedValue):
                if -100 <= speed <= 100:
                    speed_obj = ev3dev2.motor.SpeedPercent(speed)
                    speed_var = speed_obj.to_native_units(self.tank_drive.left_motor)
                else:
                    raise Exception("Invalid Speed Percentage. Speed must be between -100 and 100)")
            else:
                speed_var = speed.to_native_units(self.tank_drive.left_motor)

            left_speed = speed_var
            right_speed = speed_var
            speed_factor = (50 - abs(float(steering))) / 50.0

            if steering >= 0:
                right_speed *= speed_factor
            else:
                left_speed *= speed_factor

            return(left_speed, right_speed)        
            
        def getSpeedDPSObj(): # return settings_turn_rate (i.e. speed of robot turn) in degrees-per-second
            rotations = self.settings_turn_rate / self.wheel_circumference
            degrees = rotations * 360
            return ev3dev2.motor.SpeedDPS(degrees)  
      
        def getTurnInDegrees():
            robot_circumference_of_turn = self.axle_track * math.pi
            angleInDegrees = angle * (robot_circumference_of_turn / self.wheel_circumference)
            return angleInDegrees

        self.pivot_turn_angle = angle

        (left_speed, right_speed) = get_speed_steering(steering=100, speed=getSpeedDPSObj())
        self.tank_drive.on_for_degrees(ev3dev2.motor.SpeedNativeUnits(left_speed), ev3dev2.motor.SpeedNativeUnits(right_speed), degrees=getTurnInDegrees(), brake=True, block=True)

    def drive(self, drive_speed, turn_rate):
        '''
            This method converts the Pybricks drive() command to the on() method of 
            the EV3DEV2 MoveTank class, whose speed parameters can be instantiated as 
            a SpeedDPS (Degrees per Second) class.  This translates the robot drive 
            command into wheel speeds.

            Pybricks drive command parameters:
                drive_speed: millimetres-per-second
                turn_rate: rotational speed in deg/s
            Gears uses the EV3DEV2 class MoveTank() on() methods, with the parameters:
                left_speed
                right_speed
            which are instances of the SpeedValue base class, which can be instantiated as a
            SpeedDPS class, with the parameter:
                degrees_per_second

            Background:

            We are using two models:
                unicycle model - which is a simplified view that assumes the robot
                    only has one wheel, 
                differential drive model - which models the rotation of the two wheels 
                    of the robot

            We are taking the simplfied view of the unicycle model and translating it to
            the implementation view of differential drive model.

            A. Unicycle Model

            The robot's motion is described using a Unicycle Model where a robot is defined 
            using 3 states:
                x-position on the x-axis (in millimetres)
                y-position on the y-axis (in millimetres)
                phi - angle of the unicycle, counter clockwise from the x-axis (in radians)

            There are 2 unicycle inputs that affect these 3 states:
                v = robot forward velocity (in metres per second)
                w = robot angular velocity (in radians per second)

            B. Differential drive model

            The 2 unicycle inputs need to be converted into differential drive inputs 
            in order for the robot to move:
                v_r = clockwise angular velocity of right wheel (in radians per sec)
                v_l = counter-clockwise angular velocity of left wheel (in radians per sec)

            C. Equations

            Required constants in order to make calculations:
                L = wheelbase (in metres per radian of a robot swing turn), where radian
                    corresponds to radius of robot turning in a circle with one fixed wheel)
                R = wheel radius (in metres per radian of wheel)

            Therefore, we can use the kinematics for a unicycle model and kinematics 
            for a differential drive model to come up with the following equations to 
            convert unicycle v and w inputs into v_r and v_l differential drive inputs 
            for our robot:
                v_r = ((2 * v) + (w * L)) / (2 * R)
                v_l = ((2 * v) - (w * L)) / (2 * R)

            where v_r and v_l are in radians per second

            (see footnote 1)

            D. Radians

            Why use radians?
                easier to work with radians in trig calculations

            A radian is the arc length that corresponds to length of radius; therefore:
                radius = wheel_diameter / 2
                radian = radius 

            In this case, there are two different radian measures: 
                1. Wheelbase radians
                    Is the radius of the circular motion of the robot turning a circle with
                    one fixed wheel - so basically the radius of this circle corresponds to
                    the wheelbase or axle length, in metres.

                2. Wheel radians
                    Is the radius of the actual wheel, since a wheel forms a circle, in metres.

            Conversions:
                degrees = radian * (180 / pi)
                radians = degrees * (pi / 180)
                
            E. Example

            Problem statement
                We have a robot with an wheel base (axle width) of 119mm and a wheel diameter 
                of 94.2mm. We want our robot to perform a continuous turn of 30 degrees/sec 
                while travelling at a speed of 200 mm/sec. 

            Unit conversions
                wheelbase = 110mm = 0.11metres; which is the radius of a robot turn with one 
                                                wheel fixed.
                wheelbase_radian = 0.11metres
        
                wheel_radius = 94.2mm / 2 = 47.1mm = 0.0471metres
                wheel_radians = 0.0471metres

                v = 200 mm/sec = 0.2 metres per sec
                w = 30 deg/sec = 0.5236 radians per sec
                L = .011 metres per radian
                R = 0.0471 metres per radian
                    
            Solve equations

            right wheel velocity
                v_r = ((2 * v) + (w * L)) / (2 * R)

                v_r = ((2 * 0.2 m/sec) + (0.5236 rad/sec * 0.11 m/rad)) / (2 * 0.0471 m/rad)

                multiply and cancel out rads
                    v_r = ((0.4 m/sec) + (0.057596 m/sec)) / (0.0942 m/rad)
                add, invert fraction in denominator and multiply
                    v_r = ((0.457596 m/sec)) * (rad / 0.0942 m)
                divide and cancel out m:
                    v_r = 4.8577 rad/sec

            left wheel velocity

                v_l = (0.4 m/sec) - (0.057596 m/sec)) / (0.0942 m/rad)
                ...

                v_l = (0.342404 m/sec) * (rad / 0.0942 m)
                v_l = 3.6349 rad/sec

            Convert these to degrees per second so can be used by on() method of the 
            EV3DEV2 MoveTank class:
                v_r = 4.8577 rad/sec * (180 / pi) = 278.3260 degrees/sec
                v_l = 3.6349 rad/sec * (180 / pi) = 208.2623 degrees/sec

            footnotes/references:
                1. hackernoon: https://hackernoon.com/unicycle-to-differential-drive-courseras-control-of-mobile-robots-with-ros-and-rosbots-part-2-6d27d15f2010
                2. mouhknowsbest: https://www.youtube.com/watch?v=aE7RQNhwnPQ&list=PLp8ijpvp8iCvFDYdcXqqYU5Ibl_aOqwjr&index=10
        '''
        self.drive_speed = drive_speed
        self.drive_turn_rate = turn_rate

        v = drive_speed / 1000 # forward velocity (metres per sec)
        w = math.radians(turn_rate) # angular velocity (radians per sec)
        L = self.axle_track / 1000 # wheelbase (metres per one_wheel_robot_turn in radians)
        R = self.wheel_radius / 1000 # radius (metres per wheel in radians)
     
        def getLeftSpeedDPSObj(): 
            v_r = ((2 * v) + (w * L)) / (2 * R) # in radians
            degrees = v_r * (180 / math.pi) # convert to degrees
            return ev3dev2.motor.SpeedDPS(degrees)            
        def getRightSpeedDPSObj(): 
            v_l = ((2 * v) - (w * L)) / (2 * R) # in radians
            degrees = v_l * (180 / math.pi) # convert to degrees
            return ev3dev2.motor.SpeedDPS(degrees)    

        self.tank_drive.on(getLeftSpeedDPSObj(), getRightSpeedDPSObj())        

    def stop(self):
        self.tank_drive.off(motors=None, brake=True)

    def distance(self):   
        '''
            Gets the estimated driven distance in mm since last reset
        '''
        def distanceWheel(wheel_position):
            '''
                # this is an expensive calculation to start with... adding 
                # a wait stime slows it down too much
                #time.sleep(SENSOR_DELAY)
            '''
            quotient_rotations = wheel_position // 360
            remainder_degrees = wheel_position % 360
            fractionOfrotations = remainder_degrees / 360
            distance_rotations = quotient_rotations + fractionOfrotations
            distance_mm = distance_rotations * self.wheel_circumference
            return distance_mm

        def distanceLeftWheel():
            return distanceWheel(self.left_motor.wheel.position())

        def distanceRightWheel():
            return distanceWheel(self.right_motor.wheel.position()) 

        return ( distanceLeftWheel() + distanceRightWheel() ) / 2

    def speed(self):
        speed_dps = self.tank_drive.left_motor.speed()  # dps = degrees per second
        speed_rps = speed_dps / 360  # rotations per second 
        speed_mms = speed_rps * self.wheel_circumference # millimetres per second (where wheel_circumference = wheel_diameter * pi)
        return speed_mms

    def angle(self):
        return int(self.robot.angle())

    def state(self):
        '''
            returns the current distance(), drive speed, angle(), and turn rate of the robot
        '''
        #return self.distance(), self.speed(), self.angle(), self.drive_turn_rate # tuple
        print("Warning: robot turn_rate not implemented")
        return self.distance(), self.speed(), self.angle() # tuple        

    def reset(self):
        # resets distance
        # TODO does robot.reset() actually reset angle?
        self.tank_drive.left_motor.reset() 
        self.tank_drive.right_motor.reset()         
