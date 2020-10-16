# Needed to prevent loops from locking up the javascript thread
SENSOR_DELAY = 0.001

# Import the necessary libraries
import simPython, time
import math
import ev3dev2.motor
from pybricks.parameters import *

class Motor:
    MAX_SPEED = 300    
    MAX_DURATION = 1000    
    # TODO not a ful implementation

    def __init__(self, port, positive_direction=Direction.CLOCKWISE, gears=None):
        self.port = port
        self.positive_direction = positive_direction
        if (gears is not None):
            raise ValueError("gears not implemented")

        self.motor = ev3dev2.motor.Motor(address=port)
        self.wheel = self.motor.wheel

        # TODO should these be robot attributes, not wheel attributes?
        self.wheelDiameter = self.wheel.wheelDiameter()
        self.wheelRadius = self.wheel.wheelRadius()        
        
        # copied from ev3dev2/motor.py
        self.max_speed = 1050
        self.count_per_rot = 360
        self.max_rps = float(self.max_speed/self.count_per_rot)
        self.max_rpm = self.max_rps * 60
        self.max_dps = self.max_rps * 360
        self.max_dpm = self.max_rpm * 360

    def __str__(self):
        return "Port: " + str(self.port) + ";\n robotTemplates.js wheelDiameter: " + str(self.wheelDiameter) 

    # Measuring

    def speed(self):
        # TODO how to convert speed in degrees/sec to equivalent 
        # robot.drive() drive_speed parameter

        '''
            Gets the speed of the motor.
            Return type:	rotational speed: deg/s

            Note: robot.drive() drive_speed parm != motor.speed()
                robot drive_speed = mm/s # Speed of the robot.
                motor.speed       = deg/s # wheel rotational speed:

            =============================================================================

            speed¶
            Returns the current motor speed in tacho counts per second. Note, this is not 
            necessarily degrees (although it is for LEGO motors). Use the count_per_rot attribute
            to convert this value to RPM or deg/sec.

            speed_sp
            Writing sets the target speed in tacho counts per second used for all run-* commands 
            except run-direct. Reading returns the current value. A negative value causes the motor 
            to rotate in reverse with the exception of run-to-*-pos commands where the sign is 
            ignored. Use the count_per_rot attribute to convert RPM or deg/sec to tacho counts per 
            second. Use the count_per_m attribute to convert m/s to tacho counts per second.

            see: https://ev3dev-lang.readthedocs.io/projects/python-ev3dev/en/stable/motors.html#ev3dev2.motor.Motor.speed

            for EV3:
                self.count_per_rot = 360 = one rotation of a circle = 360degrees
            therefore            
                tacho counts per sec = degrees per sec
        '''
        time.sleep(SENSOR_DELAY)
        return int(self.wheel.speed_sp())  # degrees per sec

    # position is same as angle for EV3
    def angle(self):
        '''
            position
            Returns the current position of the motor in pulses of the rotary encoder. 
            When the motor rotates clockwise, the position will increase. Likewise, 
            rotating counter-clockwise causes the position to decrease. Writing will set 
            the position to that value.

            position_sp
            Writing specifies the target position for the run-to-abs-pos and run-to-rel-pos 
            commands. Reading returns the current value. Units are in tacho counts. 
            You can use the value returned by count_per_rot to convert tacho counts 
            to/from rotations or degrees.

            see: https://ev3dev-lang.readthedocs.io/projects/python-ev3dev/en/stable/motors.html#ev3dev2.motor.Motor.position

            tacho counts = degrees
        '''
        time.sleep(SENSOR_DELAY)
        return int(self.wheel.position()) # in degrees

    def reset_angle(self, angle):
        time.sleep(SENSOR_DELAY)        
        self.wheel.position(int(angle))

    # Stopping

    def stop(self):
        """
            Stop any of the run commands before they are complete using the
            action specified by stop_action.
        
        """
        self.wheel.stop_action(ev3dev2.motor.Motor.STOP_ACTION_COAST)
        self.wheel.command('')

    def brake(self):
        self.wheel.stop_action(ev3dev2.motor.Motor.STOP_ACTION_BRAKE)
        self.wheel.command('')

    def hold(self):
        self.wheel.stop_action(ev3dev2.motor.Motor.STOP_ACTION_HOLD)
        self.wheel.command('')

    # speed (rotational speed: deg/s) – Speed of the motor.
    def run(self, speed):
        # speed_sp -> sp = set point ( a setpoint is the desired or target value 
        # for an essential variable, or process value of a system. )
        speedValue = ev3dev2.motor.SpeedDPS(speed)   
        speed_sp = int(round(speedValue.to_native_units(self.motor)))             
        self.wheel.speed_sp(speed_sp)
        self.wheel.command('run-forever')

    def wait(self, cond, timeout=None):
        """
            Blocks until ``cond(self.state)`` is ``True``.  The condition is
            checked when there is an I/O event related to the ``state`` attribute.
            Exits early when ``timeout`` (in milliseconds) is reached.

            Returns ``True`` if the condition is met, and ``False`` if the timeout
            is reached.

            Valid flags for state attribute: running, ramping, holding,
            overloaded and stalled
        """
        if timeout != None:
            timeout = time.clock() + timeout
        while True:
            time.sleep(0.01)
            if cond(str(self.wheel.state())):
                return True
            if timeout != None and time.clock() >= timeout:
                return False

    def wait_until(self, s, timeout=None):
        """
            Blocks until ``s`` is in ``self.state``.  The condition is checked when
            there is an I/O event related to the ``state`` attribute.  Exits early
            when ``timeout`` (in milliseconds) is reached.

            Returns ``True`` if the condition is met, and ``False`` if the timeout
            is reached.

            Example::
                m.wait_until('stalled')
        """
        return self.wait(lambda state: s in state, timeout)

    def wait_until_not_moving(self, timeout=None):
        """
            Blocks until one of the following conditions are met:
            - ``running`` is not in ``self.state``
            - ``stalled`` is in ``self.state``
            - ``holding`` is in ``self.state``
            The condition is checked when there is an I/O event related to
            the ``state`` attribute.  Exits early when ``timeout`` (in
            milliseconds) is reached.

            Returns ``True`` if the condition is met, and ``False`` if the timeout
            is reached.

            Example::

                m.wait_until_not_moving()
        """
        return self.wait(lambda state: ev3dev2.motor.Motor.STATE_RUNNING not in state or ev3dev2.motor.Motor.STATE_STALLED in state, timeout)

    def run_time(self, speed, time, then=Stop.HOLD, wait=True):
        speedValue = ev3dev2.motor.SpeedDPS(speed)   
        speed_sp = int(round(speedValue.to_native_units(self.motor)))             
        self.wheel.speed_sp(speed_sp)
        self.wheel.time_sp(time)  # in milliseconds   

        if then == Stop.HOLD:
            self.wheel.stop_action = ev3dev2.motor.Motor.STOP_ACTION_HOLD
        else:
            self.wheel.stop_action = ev3dev2.motor.Motor.STOP_ACTION_COAST             
            
        self.wheel.command('run-timed')

        if wait:
            self.wait_until('running', timeout=ev3dev2.motor.WAIT_RUNNING_TIMEOUT)
            self.wait_until_not_moving(timeout=ev3dev2.motor.WAIT_RUNNING_TIMEOUT)

    def _set_rel_position_degrees_and_speed_sp(self, degrees, speed):
        degrees = degrees if speed >= 0 else -degrees
        speed = abs(speed)

        position_delta = int(round((degrees * self.motor.count_per_rot)/360))
        speed_sp = int(round(speed))

        self.wheel.position_sp(position_delta)
        self.wheel.speed_sp(speed_sp)        

    def run_angle(self, speed, rotation_angle, then=Stop.HOLD, wait=True):
        speedValue = ev3dev2.motor.SpeedDPS(speed)   
        speed_sp = int(round(speedValue.to_native_units(self.motor))) 

        self._set_rel_position_degrees_and_speed_sp(rotation_angle, speed_sp)

        if then == Stop.HOLD:
            self.wheel.stop_action = ev3dev2.motor.Motor.STOP_ACTION_HOLD
        else:
            self.wheel.stop_action = ev3dev2.motor.Motor.STOP_ACTION_COAST             
            
        self.wheel.command('run-to-rel-pos')

        if wait:
            self.wait_until('running', timeout=ev3dev2.motor.WAIT_RUNNING_TIMEOUT)
            self.wait_until_not_moving(timeout=ev3dev2.motor.WAIT_RUNNING_TIMEOUT)

    def run_target(self, speed, target_angle, then=Stop.HOLD, wait=True):
        '''
            when running run-to-abs-pos, note that the robot is reset with each run of the 
            simulator, thereby resetting the absolute location of the motor angle each time 
            it is reset

            It will run to absolute position only within a given run when you give multiple
            run_target commands in the same run


            position_sp¶
            - Writing specifies the target position for the run-to-abs-pos and run-to-rel-pos 
            commands. Reading returns the current value. 
            - Units are in tacho counts, *BUT* EV3 tacho counts = degrees, so no conversions required
            on EV3!
        '''        
        speedValue = ev3dev2.motor.SpeedDPS(speed)   
        speed_sp = int(round(speedValue.to_native_units(self.motor))) 
        self.wheel.speed_sp(speed_sp)     
        self.wheel.position_sp(target_angle)

        if then == Stop.HOLD:
            self.wheel.stop_action = ev3dev2.motor.Motor.STOP_ACTION_HOLD
        else:
            self.wheel.stop_action = ev3dev2.motor.Motor.STOP_ACTION_COAST             
            
        self.wheel.command('run-to-abs-pos')

        if wait:
            self.wait_until('running', timeout=ev3dev2.motor.WAIT_RUNNING_TIMEOUT)
            self.wait_until_not_moving(timeout=ev3dev2.motor.WAIT_RUNNING_TIMEOUT)

    def wait_until_stalled(self, timeout=None):
        """
            Blocks until one of the following conditions are met:
            - ``stalled`` is in ``self.state``
            The condition is checked when there is an I/O event related to
            the ``state`` attribute.  Exits early when ``timeout`` (in
            milliseconds) is reached.

            Returns ``True`` if the condition is met, and ``False`` if the timeout
            is reached.

        """
        return self.wait(lambda state:  ev3dev2.motor.Motor.STATE_STALLED in state, timeout)

    def run_until_stalled(self, speed, then=Stop.COAST, duty_limit=None):
        '''
            TODO run_until_stalled not sure how to test this..
            TODO run_until_stalled not implemented: return angle, duty_limit

            Notes:
            runs as a blocking command... this is how Gearsbot implements it, but not sure if EV3 is
            supposed to be non-blocking, so can run on two motors for example...
        '''        
        speedValue = ev3dev2.motor.SpeedDPS(speed)   
        speed_sp = int(round(speedValue.to_native_units(self.motor)))             
        self.wheel.speed_sp(speed_sp)

        if then == Stop.HOLD:
            self.wheel.stop_action = ev3dev2.motor.Motor.STOP_ACTION_HOLD
        else:
            self.wheel.stop_action = ev3dev2.motor.Motor.STOP_ACTION_COAST             

        self.wheel.command('run-forever')
        self.wait_until_stalled(timeout=ev3dev2.motor.WAIT_RUNNING_TIMEOUT)        

    def dc(self, power):
        '''
        dc = duty cycle
        '''
        if -100 <= power <= 100:
            speedValue = ev3dev2.motor.SpeedPercent(power)
            speed_sp = int(round(speedValue.to_native_units(self.motor)))       
        else:
            raise Exception("Invalid Speed Percentage. Duty (power) must be between -100 and 100)")      
          
        self.wheel.speed_sp(speed_sp)
        self.wheel.command('run-forever')

class UltrasonicSensor:
    def __init__(self, address=None):
        self.sensor = simPython.UltrasonicSensor(address)

    def distance(self): # in cm not mm
        time.sleep(SENSOR_DELAY)
        return float(self.sensor.dist())

    @property
    def presence(self):
        print("Error UltrasonicSensor presence not implemented")
        return False

class ColorSensor:
    '''
    EV3 color detector should optimally be 1.6cm from surface... not sure about this environment

    '''
    def __init__(self, address=None):
        self.sensor = simPython.ColorSensor(address)

    def reflection(self):
        time.sleep(SENSOR_DELAY)
        return int(list(self.sensor.value())[0] / 2.55)

    def color(self):
        time.sleep(SENSOR_DELAY)
        hsv = self.hsv

        if hsv[1] < 20:
            if hsv[2] < 30:
                return Color.BLACK
            else:
                return Color.WHITE

        elif hsv[0] < 30:
            return Color.RED

        elif hsv[0] < 90:
            return Color.YELLOW

        elif hsv[0] < 163:
            return Color.GREEN

        elif hsv[0] < 283:
            return Color.BLUE

        else:
            return Color.RED

    def rgb(self):
        time.sleep(SENSOR_DELAY)
        rgb = list(self.sensor.value())
        for i in range(3):
            rgb[i] = int(rgb[i])
        return rgb

    @property
    def hsv(self):
        time.sleep(SENSOR_DELAY)
        hsv = list(self.sensor.valueHSV())
        for i in range(3):
            hsv[i] = int(hsv[i])
        return hsv

# TODO how to test to see if Gyro actually in the port selected???
class GyroSensor:
    def __init__(self, port, positive_direction=Direction.CLOCKWISE):
        if positive_direction is not Direction.CLOCKWISE:
            print("ERROR: cannot set positive_direction in this virtual environment")        
        self.port = port
        self.positive_direction = positive_direction 
        self.sensor = simPython.GyroSensor(port)

    def speed(self):
        # rate is actually: angularVelocity
        time.sleep(SENSOR_DELAY)
        rate = self.angle_and_rate()
        return rate[1]

    def angle(self):
        # angle() is in degrees
        time.sleep(SENSOR_DELAY)        
        angle = self.angle_and_rate()
        return angle[0]   

    def angle_and_rate(self):
        time.sleep(SENSOR_DELAY)
        angle_and_rate = list(self.sensor.angleAndRate())
        for i in range(2):
            angle_and_rate[i] = int(angle_and_rate[i])
        return angle_and_rate

    def reset_angle(self, angle=0):
        print("WARNING: cannot set Gyro to specified angle in this virtual environment")
        self.sensor.reset()

#############################################
### not implemented

class TouchSensor:
    def __init__(self, port):
        self.port = port

    def pressed(self):
        print("TouchSensor not implemented")
