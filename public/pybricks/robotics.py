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
                self.wheel_diameter = wheel_diameter # in mm; robotTemplates uses cm
                self.wheel_radius = left_motor.wheelRadius # in mm; robotTemplates uses cm
            else:
                raise ValueError("Wheel circumference must be between " + 
                str(DriveBase.SMALLEST_TIRE_DIAMETER) + "mm and " + str(DriveBase.LARGEST_TIRE_DIAMETER) + "mm")
        def check_axle_track():
            # !!!!!! makes no sense for a wheel to have an axletrack property???
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
        self.tank_drive = MoveTank(left_motor.port, right_motor.port)          
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
        def getSpeedDPSObj(): # straight_speed in degrees-per-second
            rotations = self.straight_speed / self.wheel_circumference
            degrees = rotations * 360
            return SpeedDPS(degrees)
        def getDistanceInDegrees():
            dist_in_rotations = distance / self.wheel_circumference
            return dist_in_rotations * 360

        speedDPS_obj = getSpeedDPSObj()
        self.tank_drive.on_for_degrees(speedDPS_obj, speedDPS_obj, getDistanceInDegrees(), brake=True, block=True)

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
            if not isinstance(speed, SpeedValue):
                if -100 <= speed <= 100:
                    speed_obj = SpeedPercent(speed)
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
        def getSpeedDPSObj(): # turn_rate (speed) in degrees-per-second
            rotations = self.turn_rate / self.wheel_circumference
            degrees = rotations * 360
            return SpeedDPS(degrees)        
        def getTurnInDegrees():
            robot_circumference_of_turn = self.axle_track * math.pi
            return angle * (robot_circumference_of_turn / self.wheel_circumference)

        #steering_drive = MoveSteering(self.left_motor.port, self.right_motor.port)
        #steering_drive.on_for_degrees(steering=100, speed=getSpeedDPSObj(), degrees=getTurnInDegrees(), brake=True, block=True)

        (left_speed, right_speed) = get_speed_steering(steering=100, speed=getSpeedDPSObj())
        self.tank_drive.on_for_degrees(SpeedNativeUnits(left_speed), SpeedNativeUnits(right_speed), degrees=getTurnInDegrees(), brake=True, block=True)

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
                v = forward velocity (in metres per second)
                w = angular velocity (in radians per second)

            B. Differential drive model

            The 2 unicycle inputs need to be converted into differential drive inputs 
            in order for the robot to move:
                v_r = clockwise angular velocity of right wheel (in radians per sec)
                v_l = counter-clockwise angular velocity of left wheel (in radians per sec)

            C. Equations

            Required constants in order to make calculations:
                L = wheelbase (in metres per radian of one_wheel robot turn; where radian corresponds to radius of 
                    robot turning in a circle with one fixed wheel)
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

            Problem stattement
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
                    v_r = ((0.4 m/sec) + (0.90514 m/sec)) / (0.056 m/rad)
                invert fraction in denominator and multiply
                    v_r = ((0.4  m/sec) + (0.057596 m/sec)) * (rad / 0.056m)
                distributive multiply and cancel out m:
                    v_r = (0.4  / 0.056 rad/sec) + (0.057596 / 0.056 rad/sec)
                divisions:
                    v_r = (4.2463 rad/sec) + (0.6114 rad/sec)
                addition:
                    v_r = 4.8577 rad/sec

            left wheel velocity

                v_l = ((2 * 0.2 m/sec) - (0.5236 rad/sec * 0.11 m/rad)) / (2 * 0.0471 m/rad)
                ...

                v_l = (4.2463 rad/sec) - (0.6114 rad/sec)
                v_l = 3.6349 rad/sec

            Convert these to degrees per second so can be used by on() method of the 
            EV3DEV2 MoveTank class:
                v_r = 4.8577 rad/sec * (180 / pi) = 278.3260 degrees/sec
                v_l = 3.6349 rad/sec * (180 / pi) = 208.2623 degrees/sec

            footnotes/references:
                1. hackernoon: https://hackernoon.com/unicycle-to-differential-drive-courseras-control-of-mobile-robots-with-ros-and-rosbots-part-2-6d27d15f2010
                2. mouhknowsbest: https://www.youtube.com/watch?v=aE7RQNhwnPQ&list=PLp8ijpvp8iCvFDYdcXqqYU5Ibl_aOqwjr&index=10
        '''
        def getDriveSpeedDPSObj(): # convert drive_speed from millimetres-per-second to degrees-per-second
            rotations = drive_speed / self.wheel_circumference
            degrees = rotations * 360
            return SpeedDPS(degrees)

        v = drive_speed/1000 # forward velocity (metres per sec)
        w = math.radians(turn_rate) # angular velocity (radians per sec)
        # !!!!!! makes no sense for axleTrack to be attached to motor property
        L = self.left_motor.axleTrack / 1000 # wheelbase (metres per one_wheel_robot_turn radian)
        R = self.wheel_radius / 1000 # radius (metres per wheel radian)
        '''
            print("axleTrack: " + str(self.left_motor.axleTrack) + "mm") # should not be a motor property; should be a robot property
            print("wheel_radius: " + str(self.wheel_radius) + "mm")           
            print("v: " + str(v))
            print("w: " + str(w))        
            print("L: " + str(L))
            print("R: " + str(R)) 
        '''           
        def getLeftSpeedDPSObj(): 
            v_r = ((2 * v) + (w * L)) / (2 * R) # in radians
            degrees = v_r * (180 / math.pi) # convert to degrees
            #print("left: " + str(degrees))
            return SpeedDPS(degrees)            
        def getRightSpeedDPSObj(): 
            v_l = ((2 * v) - (w * L)) / (2 * R) # in radians
            degrees = v_l * (180 / math.pi) # convert to degrees
            return SpeedDPS(degrees)    

        if turn_rate == 0:
            speedDPS_obj = getDriveSpeedDPSObj()
            self.tank_drive.on(left_speed=speedDPS_obj, right_speed=speedDPS_obj)  
        else:
            self.tank_drive.on(getLeftSpeedDPSObj(), getRightSpeedDPSObj())  

    # rename to stop
    def off(self, motors=None, brake=True):
        self.tank_drive.off(motors, brake)

    def distance(self):
        print("startin distance")
        time.sleep(SENSOR_DELAY)
        average_wheel_position = (self.left_motor.wheel.position() + self.left_motor.wheel.position()) / 2
        int_rotations = average_wheel_position // 360
        remainder_degrees = average_wheel_position % 360
        remainder_rotations = remainder_degrees / 360
        distance_rotations = int_rotations + remainder_rotations
        distance_mm = distance_rotations * self.wheel_circumference
        print("finish distance")        
        return distance_mm

    ###########################################################################

    def angle(self):
        print("not implemented")      
        return self.angle        
      
    def state(self):
        print("not implemented")       
        return (self.distance, self.drive_speed, self.angle, self.turn_rate)

    def reset(self):
        print("not implemented") 

