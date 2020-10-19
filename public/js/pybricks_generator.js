var pybricks_generator = new function() {
  var self = this;

  self.straight_speed = 200;
  self.straight_acceleration = 100;
  self.turn_rate = 100;
  self.turn_acceleration = 100;

  // Load Python generators
  this.load = function() {
    Blockly.Python['when_started'] = self.when_started;
    // Robot move and stop
    Blockly.Python['py_robot_settings'] = self.py_robot_settings;      
    Blockly.Python['py_robot_straight'] = self.py_robot_straight; 
    Blockly.Python['py_robot_turn'] = self.py_robot_turn;     
    // Robot continuous move
    Blockly.Python['py_robot_drive'] = self.py_robot_drive;       
    Blockly.Python['py_robot_stop'] = self.py_robot_stop;        
    Blockly.Python['py_robot_distance'] = self.py_robot_distance;    
    Blockly.Python['py_robot_angle'] = self.py_robot_angle;      
    Blockly.Python['py_robot_state'] = self.py_robot_state;     
    Blockly.Python['py_robot_reset'] = self.py_robot_reset;        
    // Motor run
    Blockly.Python['py_motor_run'] = self.py_motor_run;    
    Blockly.Python['py_motor_stop'] = self.py_motor_stop;      
    Blockly.Python['py_motor_speed'] = self.py_motor_speed;  
    Blockly.Python['py_motor_angle'] = self.py_motor_angle;  
    Blockly.Python['py_motor_reset_angle'] = self.py_motor_reset_angle;      
    // Motor run plus
    Blockly.Python['py_motor_run_time'] = self.py_motor_run_time;    
    Blockly.Python['py_motor_run_angle'] = self.py_motor_run_angle;    
    Blockly.Python['py_motor_run_target'] = self.py_motor_run_target;    
    Blockly.Python['py_motor_run_until_stalled'] = self.py_motor_run_until_stalled;    
    Blockly.Python['py_motor_dc'] = self.py_motor_dc;    
    // Sensors
     Blockly.Python['py_gyro'] = self.py_gyro;    
     Blockly.Python['py_reset_gyro'] = self.py_reset_gyro;        
     Blockly.Python['py_color'] = self.py_color;    
     Blockly.Python['py_color_name'] = self.py_color_name;    
     Blockly.Python['py_ultrasonic'] = self.py_ultrasonic;       
    // Logic - Blockly builtin
    // Loops - Blockly builtin
    // Math - Blockly builtin
    // Text - Blockly builtin
    // Control
    Blockly.Python['py_wait'] = self.py_wait;    
    // Variables - Blockly builtin
    // Functions - Blockly builtin
  };

  // Generate python code
  this.genCode = function() {
    let code =
      '#!/usr/bin/env python3\n' +
      `\n` +
      '# Import the necessary libraries\n' +
      'import math\n' +
      'import time\n' +      
      'from pybricks.ev3devices import *\n' +      
      'from pybricks.parameters import *\n' +    
      'from pybricks.robotics import *\n' + 
      'from pybricks.tools import wait\n' +
      'from pybricks.hubs import EV3Brick\n' +

      '\n' +
      'ev3 = EV3Brick()\n' +
      'motorA = Motor(Port.A)\n' +          
      'motorB = Motor(Port.B)\n' +      
      'left_motor = motorA\n' + 
      'right_motor = motorB\n' + 
       
      'robot = DriveBase(left_motor, right_motor, wheel_diameter=56, axle_track=152)\n' +   
      // TODO only required if user using straight or turn commands...
      'robot.settings(straight_speed=' + self.straight_speed + ', ' +
                     'straight_acceleration=' + self.straight_acceleration +  ', ' +
                     'turn_rate=' + self.turn_rate + ')\n' +                                     
      '\n';
    var sensorsCode = '';
    var i = 1;
    var sensor = null;

    while (sensor = robot.getComponentByPort('in' + i)) {
      if (sensor.type == 'ColorSensor') {
        sensorsCode += 'color_sensor_in' + i + ' = ColorSensor(Port.S' + i + ')\n';
      } else if (sensor.type == 'UltrasonicSensor') {
        sensorsCode += 'obstacle_sensor = UltrasonicSensor(Port.S' + i + ')\n';
      } else if (sensor.type == 'GyroSensor') {
        sensorsCode += 'gyro_sensor= GyroSensor(Port.S' + i + ')\n';
      } 
      i++;
    }

    let PORT_LETTERS = ' ABCDEFGHIJKLMNOPQRSTUVWXYZ';
    var motorsCode = '';
    i = 3;
    var motor = null;

    while (motor = robot.getComponentByPort('out' + PORT_LETTERS[i])) {
      if (motor.type == 'MagnetActuator') {
        motorsCode += 'motor' + PORT_LETTERS[i] + ' = Motor(Port.' + PORT_LETTERS[i] + ') # Magnet\n';
      } else if (motor.type == 'ArmActuator') {
        motorsCode += 'motor' + PORT_LETTERS[i] + ' = Motor(Port.' + PORT_LETTERS[i] + ') # Arm\n';
      } else if (motor.type == 'SwivelActuator') {
        motorsCode += 'motor' + PORT_LETTERS[i] + ' = Motor(Port.' + PORT_LETTERS[i] + ') # Swivel\n';
      } else if (motor.type == 'PaintballLauncherActuator') {
        motorsCode += 'motor' + PORT_LETTERS[i] + ' = Motor(Port.' + PORT_LETTERS[i] + ') # Paintball Launcher\n';
      }
      i++;
    }
    code += sensorsCode + '\n';
    code += motorsCode + '\n';

    code += '# Here is where your code starts\n\n';

    code += Blockly.Python.workspaceToCode(blockly.workspace);
    return code
  };

  //
  // Python Generators
  //

  // Start
  this.when_started = function(block) {
    var code = '';
    return code;
  };

  this.py_robot_straight = function(block) {
    var distance = Blockly.Python.valueToCode(block, 'distance', Blockly.Python.ORDER_ATOMIC);
    if (distance === undefined) { distance = 0;  }

    var code = 'robot.straight(' + distance + ')\n';

    return code;    
  }

  this.py_robot_turn = function(block) {
    var angle = Blockly.Python.valueToCode(block, 'angle', Blockly.Python.ORDER_ATOMIC);
    if (angle === undefined) { angle = 0; }

    var code = 'robot.turn(' + angle + ')\n';

    return code;    
  }  
  
  this.py_robot_settings = function(block) {
    straight_speed = Blockly.Python.valueToCode(block, 'straight_speed', Blockly.Python.ORDER_ATOMIC);
    if (straight_speed === undefined) { straight_speed = 0;  }

    straight_acceleration = Blockly.Python.valueToCode(block, 'straight_acceleration', Blockly.Python.ORDER_ATOMIC);
    if (straight_acceleration === undefined) { straight_acceleration = 0;  }

    turn_rate = Blockly.Python.valueToCode(block, 'turn_rate', Blockly.Python.ORDER_ATOMIC);
    if (turn_rate === undefined) { turn_rate = 0;  }

    var code = 'robot.settings(straight_speed=' + straight_speed + ', straight_acceleration=' + straight_acceleration + ', turn_rate=' + turn_rate + ')\n';

    return code;        
  }

  this.py_robot_drive = function(block) {
    var drive_speed = Blockly.Python.valueToCode(block, 'drive_speed', Blockly.Python.ORDER_ATOMIC);
    var turn_rate = Blockly.Python.valueToCode(block, 'turn_rate', Blockly.Python.ORDER_ATOMIC);  
    if (drive_speed === undefined) { drive_speed = 0;  }
    if (turn_rate === undefined) { turn_rate = 0;  }

    var code = 'robot.drive(' + drive_speed + ', ' + turn_rate + ')\n';

    return code;    
  }  

  this.py_robot_stop = function(block) {
    var code = 'robot.stop()\n';
    return code;
  };

  this.py_robot_distance = function(block) {
    var code = 'robot.distance()';
    return [code, Blockly.Python.ORDER_NONE];
  };

  this.py_robot_angle = function(block) {
    var code = 'robot.angle()';
    return [code, Blockly.Python.ORDER_NONE];
  };

  this.py_robot_state = function(block) {
    var code = 'robot.state()';
    return [code, Blockly.Python.ORDER_NONE];
  };  

  this.py_robot_reset = function(block) {
    var code = 'robot.reset()\n';
    return code;
  };

  this.py_gyro = function(block) {
    var dropdown_type = block.getFieldValue('type');

    if (dropdown_type == 'ANGLE') {
      var typeStr = 'angle';
    } else if (dropdown_type == 'SPEED') {
      var typeStr = 'speed';
    }
    var code = 'gyro_sensor' + '.' + typeStr + '()';

    return [code, Blockly.Python.ORDER_ATOMIC];
  };  

  this.py_reset_gyro = function(block) {
    var dropdown_port = block.getFieldValue('port');
    // should be able to reset to a given angle... not implemented yet
    var code = 'gyro_sensor.reset_angle()\n';
    return code;
  };

  this.py_color = function(block) {
    var dropdown_type = block.getFieldValue('type');
    var dropdown_port = block.getFieldValue('port');
    var methodStr = '';

    if (dropdown_type == 'COLOR') {
      methodStr = 'color()';
    } else if (dropdown_type == 'REFLECTION') {
      methodStr = 'reflection()';
    } else if (dropdown_type == 'RGB') {
      methodStr = 'rgb()';
    }

    var code = 'color_sensor_in' + dropdown_port + '.' + methodStr;
    return [code, Blockly.Python.ORDER_ATOMIC];
  };  

  this.py_color_name = function(block) {
    var color_name = block.getFieldValue('color_name');

    var code = color_name;
    return [code, Blockly.Python.ORDER_ATOMIC];
  };  

  this.py_ultrasonic = function(block) {

    var code = 'obstacle_sensor.distance()';
    return [code, Blockly.Python.ORDER_NONE];
  };  

  this.py_wait = function(block) {
    var value_milliseconds = Blockly.Python.valueToCode(block, 'wait_time', Blockly.Python.ORDER_ATOMIC);

    var code = 'wait(' + value_milliseconds + ')\n';

    return code;
  };

  this.py_motor_run = function(block) {
    var dropdown_port = block.getFieldValue('port');
    var speed = Blockly.Python.valueToCode(block, 'speed', Blockly.Python.ORDER_ATOMIC);

    var code = 'motor' + dropdown_port + '.run(speed=' + speed + ')\n';

    return code;
  }

  this.py_motor_run_time = function(block) {
    var dropdown_port = block.getFieldValue('port');
    var speed = Blockly.Python.valueToCode(block, 'speed', Blockly.Python.ORDER_ATOMIC);
    var time = Blockly.Python.valueToCode(block, 'time', Blockly.Python.ORDER_ATOMIC);
    var then = block.getFieldValue('then');    
    var wait = Blockly.Python.valueToCode(block, 'wait', Blockly.Python.ORDER_ATOMIC);

    var code = 'motor' + dropdown_port + '.run_time(speed=' + speed + ', time=' + time + ', then=' + then + ', wait=' + wait +  ')\n';

    return code;
  }  

  this.py_motor_run_angle = function(block) {
    var dropdown_port = block.getFieldValue('port');
    var speed = Blockly.Python.valueToCode(block, 'speed', Blockly.Python.ORDER_ATOMIC);
    var rotation_angle = Blockly.Python.valueToCode(block, 'rotation_angle', Blockly.Python.ORDER_ATOMIC);
    var then = block.getFieldValue('then');    
    var wait = Blockly.Python.valueToCode(block, 'wait', Blockly.Python.ORDER_ATOMIC);

    var code = 'motor' + dropdown_port + '.run_angle(speed=' + speed + ', rotation_angle=' + rotation_angle + ', then=' + then + ', wait=' + wait +  ')\n';

    return code;
  }  

  this.py_motor_run_target = function(block) {
    var dropdown_port = block.getFieldValue('port');
    var speed = Blockly.Python.valueToCode(block, 'speed', Blockly.Python.ORDER_ATOMIC);
    var target_angle = Blockly.Python.valueToCode(block, 'target_angle', Blockly.Python.ORDER_ATOMIC);
    var then = block.getFieldValue('then');    
    var wait = Blockly.Python.valueToCode(block, 'wait', Blockly.Python.ORDER_ATOMIC);

    var code = 'motor' + dropdown_port + '.run_target(speed=' + speed + ', target_angle=' + target_angle + 
               ', then=' + then + ', wait=' + wait +  ')\n';

    return code;
  }  

  this.py_motor_run_until_stalled = function(block) {
    var dropdown_port = block.getFieldValue('port');
    var speed = Blockly.Python.valueToCode(block, 'speed', Blockly.Python.ORDER_ATOMIC);
    var then = block.getFieldValue('then');    
    var duty_limit = Blockly.Python.valueToCode(block, 'duty_limit', Blockly.Python.ORDER_ATOMIC);

    var code = 'motor' + dropdown_port + '.run_until_stalled(speed=' + speed + 
               ', then=' + then + ', duty_limit=' + duty_limit + ')\n';  

    return code;
  }  

  this.py_motor_dc = function(block) {
    var dropdown_port = block.getFieldValue('port');
    var power = Blockly.Python.valueToCode(block, 'power', Blockly.Python.ORDER_ATOMIC);

    if (power > Math.abs(100)) {
      console.log("Warning: py_motor_dc dc power outside range -100.0 to 100")
      if (power > 100) {
        power = 100
      } else if (power < -100) {
        power = -100
      } 
    }
    var code = 'motor' + dropdown_port + '.dc(' + power + ')\n';  

    return code;
  }  

 this.py_motor_speed = function(block) {
  var dropdown_port = block.getFieldValue('port');

  var code = 'motor' + dropdown_port + '.speed()';

  return [code, Blockly.Python.ORDER_ATOMIC];
  };

  this.py_motor_angle = function(block) {
    var dropdown_port = block.getFieldValue('port');

    var code = 'motor' + dropdown_port + '.angle()';

    return [code, Blockly.Python.ORDER_ATOMIC];
  };

  this.py_motor_reset_angle = function(block) {
    var dropdown_port = block.getFieldValue('port');
    var angle = Blockly.Python.valueToCode(block, 'angle', Blockly.Python.ORDER_ATOMIC);

    var code = 'motor' + dropdown_port + '.reset_angle(angle=' +angle +  ')\n';

    return code;
  };

  this.py_motor_stop = function(block) {
    var dropdown_port = block.getFieldValue('port');
    var dropdown_stop_action = block.getFieldValue('stop_action');

    if (dropdown_stop_action == 'BRAKE') {
      var stop_action = 'brake';
    } else if (dropdown_stop_action == 'COAST') {
      var stop_action = 'stop';
    } else if (dropdown_stop_action == 'HOLD') {
      var stop_action = 'hold';
    }

    var code = 'motor' + dropdown_port + '.' + stop_action + '()\n';

    return code;
  };

}

