var pybricks_generator = new function() {
  var self = this;

  // Load Python generators
  this.load = function() {
    Blockly.Python['when_started'] = self.when_started;

    Blockly.Python['py_straight'] = self.py_straight; // !!!!!!
    Blockly.Python['py_turn'] = self.py_turn; // !!!!!!    
    Blockly.Python['py_drive'] = self.py_drive; // !!!!!!      
    Blockly.Python['py_settings'] = self.py_settings; // !!!!!!          
    Blockly.Python['py_stop'] = self.py_stop; // !!!!!!       
    Blockly.Python['py_distance'] = self.py_distance; // !!!!!!   
    Blockly.Python['py_angle'] = self.py_angle; // !!!!!!   
    Blockly.Python['py_gyro'] = self.py_gyro; // !!!!!!   
    Blockly.Python['py_reset_gyro'] = self.py_reset_gyro; // !!!!!!       
    Blockly.Python['py_color'] = self.py_color; // !!!!!!   
    Blockly.Python['py_ultrasonic'] = self.py_ultrasonic; // !!!!!!   
    Blockly.Python['py_wait'] = self.py_wait; // !!!!!!   
    Blockly.Python['py_reset_robot'] = self.py_reset_robot; // !!!!!!   
    Blockly.Python['py_motor_run'] = self.py_motor_run; // !!!!!!   
    Blockly.Python['py_motor_run_time'] = self.py_motor_run_time; // !!!!!!   
    Blockly.Python['py_motor_run_angle'] = self.py_motor_run_angle; // !!!!!!   
    Blockly.Python['py_motor_run_target'] = self.py_motor_run_target; // !!!!!!   
    Blockly.Python['py_motor_run_until_stalled'] = self.py_motor_run_until_stalled; // !!!!!!   
    Blockly.Python['py_motor_dc'] = self.py_motor_dc; // !!!!!!   
    Blockly.Python['py_motor_speed'] = self.py_motor_speed;// !!!!!!  
    Blockly.Python['py_motor_angle'] = self.py_motor_angle;// !!!!!!  
    Blockly.Python['py_motor_reset_angle'] = self.py_motor_reset_angle;    // !!!!!!  
    Blockly.Python['py_motor_stop'] = self.py_motor_stop; // !!!!!!   

    Blockly.Python['move_tank'] = self.move_tank;
    Blockly.Python['move_tank_for'] = self.move_tank_for;
    Blockly.Python['move_steering'] = self.move_steering;
    Blockly.Python['move_steering_for'] = self.move_steering_for;
    Blockly.Python['stop'] = self.stop;
    Blockly.Python['run_motor'] = self.run_motor;
    Blockly.Python['run_motor_for'] = self.run_motor_for;
    Blockly.Python['run_motor_to'] = self.run_motor_to;
    Blockly.Python['stop_motor'] = self.stop_motor;
    Blockly.Python['speed'] = self.speed;
    Blockly.Python['position'] = self.position;
    Blockly.Python['reset_motor'] = self.reset_motor;
    Blockly.Python['color_sensor'] = self.color_sensor;
    Blockly.Python['ultrasonic_sensor'] = self.ultrasonic_sensor;
    Blockly.Python['gyro_sensor'] = self.gyro_sensor;
    Blockly.Python['reset_gyro'] = self.reset_gyro;
    Blockly.Python['say'] = self.say;
    Blockly.Python['beep'] = self.beep;
    Blockly.Python['play_tone'] = self.play_tone;
    Blockly.Python['sleep'] = self.sleep;
    Blockly.Python['exit'] = self.exit;
    Blockly.Python['time'] = self.time;
    Blockly.Python['gps_sensor'] = self.gps_sensor;
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
      'robot.settings(straight_speed=200, straight_acceleration=100, turn_rate=100, turn_acceleration=100)\n' +                  
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

  // !!!!!!
  this.py_straight = function(block) {
    var distance = Blockly.Python.valueToCode(block, 'distance', Blockly.Python.ORDER_ATOMIC);
    if (distance === undefined) { distance = 0;  }

    var code = 'robot.straight(' + distance + ')\n';

    return code;    
  }

  this.py_turn = function(block) {
    var angle = Blockly.Python.valueToCode(block, 'angle', Blockly.Python.ORDER_ATOMIC);
    if (angle === undefined) { angle = 0; }

    var code = 'robot.turn(' + angle + ')\n';

    return code;    
  }  

  this.py_settings = function(block) {
    var straight_speed = Blockly.Python.valueToCode(block, 'straight_speed', Blockly.Python.ORDER_ATOMIC);
    if (straight_speed === undefined) { straight_speed = 0;  }
    var straight_acceleration = Blockly.Python.valueToCode(block, 'straight_acceleration', Blockly.Python.ORDER_ATOMIC);
    if (straight_acceleration === undefined) { straight_acceleration = 0;  }

    var turn_rate = Blockly.Python.valueToCode(block, 'turn_rate', Blockly.Python.ORDER_ATOMIC);
    if (turn_rate === undefined) { turn_rate = 0;  }
    var turn_acceleration = Blockly.Python.valueToCode(block, 'turn_acceleration', Blockly.Python.ORDER_ATOMIC);
    if (turn_acceleration === undefined) { turn_acceleration = 0;  }

    var code = 'robot.settings(straight_speed=' + straight_speed + ', straight_acceleration=' + straight_acceleration + ', turn_rate=' + turn_rate + ', turn_acceleration=' + turn_acceleration + ')\n';

    return code;    
  }

  this.py_drive = function(block) {
    var drive_speed = Blockly.Python.valueToCode(block, 'drive_speed', Blockly.Python.ORDER_ATOMIC);
    var turn_rate = Blockly.Python.valueToCode(block, 'turn_rate', Blockly.Python.ORDER_ATOMIC);  
    if (drive_speed === undefined) { drive_speed = 0;  }
    if (turn_rate === undefined) { turn_rate = 0;  }

    var code = 'robot.drive(' + drive_speed + ', ' + turn_rate + ')\n';

    return code;    
  }  

  this.py_stop = function(block) {
    var code = 'robot.stop()\n';
    return code;
  };

  this.py_distance = function(block) {
    var code = 'robot.distance()';
    return [code, Blockly.Python.ORDER_NONE];
  };

  this.py_angle = function(block) {
    var code = 'robot.angle()';
    return [code, Blockly.Python.ORDER_NONE];
  };  

  this.py_reset_robot = function(block) {
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

    var code = 'motor' + dropdown_port + '.run(' + speed + ')\n';

    return code;
  }

  this.py_motor_run_time = function(block) {
    var dropdown_port = block.getFieldValue('port');
    var speed = Blockly.Python.valueToCode(block, 'speed', Blockly.Python.ORDER_ATOMIC);
    var time = Blockly.Python.valueToCode(block, 'time', Blockly.Python.ORDER_ATOMIC);
    var then = block.getFieldValue('then');    
    var wait = Blockly.Python.valueToCode(block, 'wait', Blockly.Python.ORDER_ATOMIC);

    var code = 'motor' + dropdown_port + '.run_time(' + speed + ', ' + time + ', then=' + then + ', wait=' + wait +  ')\n';

    return code;
  }  

  this.py_motor_run_angle = function(block) {
    var dropdown_port = block.getFieldValue('port');
    var speed = Blockly.Python.valueToCode(block, 'speed', Blockly.Python.ORDER_ATOMIC);
    var rotation_angle = Blockly.Python.valueToCode(block, 'rotation_angle', Blockly.Python.ORDER_ATOMIC);
    var then = block.getFieldValue('then');    
    var wait = Blockly.Python.valueToCode(block, 'wait', Blockly.Python.ORDER_ATOMIC);

    var code = 'motor' + dropdown_port + '.run_angle(' + speed + ', ' + rotation_angle + ', then=' + then + ', wait=' + wait +  ')\n';

    return code;
  }  

  this.py_motor_run_target = function(block) {
    var dropdown_port = block.getFieldValue('port');
    var speed = Blockly.Python.valueToCode(block, 'speed', Blockly.Python.ORDER_ATOMIC);
    var target_angle = Blockly.Python.valueToCode(block, 'target_angle', Blockly.Python.ORDER_ATOMIC);
    var then = block.getFieldValue('then');    
    var wait = Blockly.Python.valueToCode(block, 'wait', Blockly.Python.ORDER_ATOMIC);

    var code = 'motor' + dropdown_port + '.run_target(' + speed + ', ' + target_angle + 
               ', then=' + then + ', wait=' + wait +  ')\n';

    return code;
  }  

  this.py_motor_run_until_stalled = function(block) {
    var dropdown_port = block.getFieldValue('port');
    var speed = Blockly.Python.valueToCode(block, 'speed', Blockly.Python.ORDER_ATOMIC);
    var then = block.getFieldValue('then');    
    var duty_limit = Blockly.Python.valueToCode(block, 'duty_limit', Blockly.Python.ORDER_ATOMIC);

    var code = 'motor' + dropdown_port + '.run_until_stalled(' + speed + 
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

  var code = 'motor' + dropdown_port + '.reset_angle(angle)\n';

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



  // !!!!!!

  // move tank
  this.move_tank = function(block) {
    var value_left = Blockly.Python.valueToCode(block, 'left', Blockly.Python.ORDER_ATOMIC);
    var value_right = Blockly.Python.valueToCode(block, 'right', Blockly.Python.ORDER_ATOMIC);
    var dropdown_unit = block.getFieldValue('units');

    if (dropdown_unit == 'PERCENT') {
      var leftStr = value_left;
      var rightStr = value_right;
    } else if (dropdown_unit == 'DEGREES') {
      var leftStr = 'SpeedDPS(' + value_left + ')';
      var rightStr = 'SpeedDPS(' + value_right + ')';
    } else if (dropdown_unit == 'ROTATIONS') {
      var leftStr = 'SpeedRPS(' + value_left + ')';
      var rightStr = 'SpeedRPS(' + value_right + ')';
    }

    var code = 'tank_drive.on(' + leftStr + ', ' + rightStr + ')\n';

    return code;
  };

  // move tank for
  this.move_tank_for = function(block) {
    var value_left = Blockly.Python.valueToCode(block, 'left', Blockly.Python.ORDER_ATOMIC);
    var value_right = Blockly.Python.valueToCode(block, 'right', Blockly.Python.ORDER_ATOMIC);
    var dropdown_units = block.getFieldValue('units');
    var value_duration = Blockly.Python.valueToCode(block, 'duration', Blockly.Python.ORDER_ATOMIC);
    var dropdown_units2 = block.getFieldValue('units2');

    if (dropdown_units == 'PERCENT') {
      var leftStr = value_left;
      var rightStr = value_right;
    } else if (dropdown_units == 'DEGREES') {
      var leftStr = 'SpeedDPS(' + value_left + ')';
      var rightStr = 'SpeedDPS(' + value_right + ')';
    } else if (dropdown_units == 'ROTATIONS') {
      var leftStr = 'SpeedRPS(' + value_left + ')';
      var rightStr = 'SpeedRPS(' + value_right + ')';
    }

    if (dropdown_units2 == 'ROTATIONS') {
      var cmdStr = 'on_for_rotations';
      var durationStr = value_duration;
    } else if (dropdown_units2 == 'DEGREES') {
      var cmdStr = 'on_for_degrees';
      var durationStr = value_duration;
    } else if (dropdown_units2 == 'SECONDS') {
      var cmdStr = 'on_for_seconds';
      var durationStr = value_duration;
    } else if (dropdown_units2 == 'MILLISECONDS') {
      var cmdStr = 'on_for_seconds';
      var durationStr = value_duration + ' / 1000';
    }

    var code = 'tank_drive.' + cmdStr + '(' + leftStr + ', ' + rightStr + ', ' + durationStr + ')\n';

    return code;
  };

  // Move steering
  this.move_steering = function(block) {
    var value_steering = Blockly.Python.valueToCode(block, 'steering', Blockly.Python.ORDER_ATOMIC);
    var value_speed = Blockly.Python.valueToCode(block, 'speed', Blockly.Python.ORDER_ATOMIC);
    var dropdown_units = block.getFieldValue('units');

    if (dropdown_units == 'PERCENT') {
      var speedStr = value_speed;
    } else if (dropdown_units == 'DEGREES') {
      var speedStr = 'SpeedDPS(' + value_speed + ')';
    } else if (dropdown_units == 'ROTATIONS') {
      var speedStr = 'SpeedRPS(' + value_speed + ')';
    }

    var code = 'steering_drive.on(' + value_steering + ', ' + speedStr + ')\n';

    return code;
  };

  // Move steering for
  this.move_steering_for = function(block) {
    var value_steering = Blockly.Python.valueToCode(block, 'steering', Blockly.Python.ORDER_ATOMIC);
    var value_speed = Blockly.Python.valueToCode(block, 'speed', Blockly.Python.ORDER_ATOMIC);
    var dropdown_units = block.getFieldValue('units');
    var value_duration = Blockly.Python.valueToCode(block, 'duration', Blockly.Python.ORDER_ATOMIC);
    var dropdown_units2 = block.getFieldValue('units2');

    if (dropdown_units == 'PERCENT') {
      var speedStr = value_speed;
    } else if (dropdown_units == 'DEGREES') {
      var speedStr = 'SpeedDPS(' + value_speed + ')';
    } else if (dropdown_units == 'ROTATIONS') {
      var speedStr = 'SpeedRPS(' + value_speed + ')';
    }

    if (dropdown_units2 == 'ROTATIONS') {
      var cmdStr = 'on_for_rotations';
      var durationStr = value_duration;
    } else if (dropdown_units2 == 'DEGREES') {
      var cmdStr = 'on_for_degrees';
      var durationStr = value_duration;
    } else if (dropdown_units2 == 'SECONDS') {
      var cmdStr = 'on_for_seconds';
      var durationStr = value_duration;
    } else if (dropdown_units2 == 'MILLISECONDS') {
      var cmdStr = 'on_for_seconds';
      var durationStr = value_duration + ' / 1000';
    }

    var code = 'steering_drive.' + cmdStr + '(' + value_steering + ', ' + speedStr + ', ' + durationStr + ')\n';

    return code;
  };

  // Stop
  this.stop = function(block) {
    var dropdown_stop_action = block.getFieldValue('stop_action');

    if (dropdown_stop_action == 'BRAKE') {
      var brake = 'True';
    } else if (dropdown_stop_action == 'COAST') {
      var brake = 'False';
    } else if (dropdown_stop_action == 'HOLD') {
      var brake = 'True';
    }

    var code = 'tank_drive.off(brake=' + brake + ')\n';

    return code;
  };

  // Run motor
  this.run_motor = function(block) {
    var dropdown_port = block.getFieldValue('port');
    var value_speed = Blockly.Python.valueToCode(block, 'speed', Blockly.Python.ORDER_ATOMIC);
    var dropdown_unit = block.getFieldValue('unit');

    if (dropdown_unit == 'PERCENT') {
      var speedStr = value_speed;
    } else if (dropdown_unit == 'DEGREES') {
      var speedStr = 'SpeedDPS(' + value_speed + ')';
    } else if (dropdown_unit == 'ROTATIONS') {
      var speedStr = 'SpeedRPS(' + value_speed + ')';
    }

    var code = 'motor' + dropdown_port + '.on(' + speedStr + ')\n';

    return code;
  }

  // Run motor for
  this.run_motor_for = function(block) {
    var dropdown_port = block.getFieldValue('port');
    var value_speed = Blockly.Python.valueToCode(block, 'speed', Blockly.Python.ORDER_ATOMIC);
    var dropdown_unit = block.getFieldValue('unit');
    var value_duration = Blockly.Python.valueToCode(block, 'duration', Blockly.Python.ORDER_ATOMIC);
    var dropdown_unit2 = block.getFieldValue('unit2');

    if (dropdown_unit == 'PERCENT') {
      var speedStr = value_speed;
    } else if (dropdown_unit == 'DEGREES') {
      var speedStr = 'SpeedDPS(' + value_speed + ')';
    } else if (dropdown_unit == 'ROTATIONS') {
      var speedStr = 'SpeedRPS(' + value_speed + ')';
    }

    if (dropdown_unit2 == 'ROTATIONS') {
      var cmdStr = 'on_for_rotations';
      var durationStr = value_duration;
    } else if (dropdown_unit2 == 'DEGREES') {
      var cmdStr = 'on_for_degrees';
      var durationStr = value_duration;
    } else if (dropdown_unit2 == 'SECONDS') {
      var cmdStr = 'on_for_seconds';
      var durationStr = value_duration;
    } else if (dropdown_units2 == 'MILLISECONDS') {
      var cmdStr = 'on_for_seconds';
      var durationStr = value_duration + ' / 1000';
    }

    var code = 'motor' + dropdown_port + '.' + cmdStr + '(' + speedStr + ', ' + durationStr + ')\n';

    return code;
  }

  // Run motor to
  this.run_motor_to = function(block) {
    var dropdown_port = block.getFieldValue('port');
    var value_speed = Blockly.Python.valueToCode(block, 'speed', Blockly.Python.ORDER_ATOMIC);
    var dropdown_unit = block.getFieldValue('unit');
    var value_degrees = Blockly.Python.valueToCode(block, 'degrees', Blockly.Python.ORDER_ATOMIC);

    if (dropdown_unit == 'PERCENT') {
      var speedStr = value_speed;
    } else if (dropdown_unit == 'DEGREES') {
      var speedStr = 'SpeedDPS(' + value_speed + ')';
    } else if (dropdown_unit == 'ROTATIONS') {
      var speedStr = 'SpeedRPS(' + value_speed + ')';
    }

    var code = 'motor' + dropdown_port + '.on_to_position(' + speedStr + ', ' + value_degrees + ')\n';

    return code;
  }

  // Stop motor
  this.stop_motor = function(block) {
    var dropdown_port = block.getFieldValue('port');
    var dropdown_stop_action = block.getFieldValue('stop_action');

    if (dropdown_stop_action == 'BRAKE') {
      var brake = 'True';
    } else if (dropdown_stop_action == 'COAST') {
      var brake = 'False';
    } else if (dropdown_stop_action == 'HOLD') {
      var brake = 'True';
    }

    var code = 'motor' + dropdown_port + '.off(brake=' + brake + ')\n';

    return code;
  };

  // get speed
  this.speed = function(block) {
    var dropdown_port = block.getFieldValue('port');

    var code = 'motor' + dropdown_port + '.speed';

    return [code, Blockly.Python.ORDER_ATOMIC];
  };

  // get position
  this.position = function(block) {
    var dropdown_port = block.getFieldValue('port');

    var code = 'motor' + dropdown_port + '.position';

    return [code, Blockly.Python.ORDER_ATOMIC];
  };

  // reset position
  this.reset_motor = function(block) {
    var dropdown_port = block.getFieldValue('port');

    if (dropdown_port == 'BOTH') {
      var code =
        'left_motor.position = 0\n' +
        'right_motor.position = 0\n';
    } else {
      var code = 'motor' + dropdown_port + '.position = 0\n';
    }

    return code;
  };

  // color sensor value
  this.color_sensor = function(block) {
    var dropdown_type = block.getFieldValue('type');
    var dropdown_port = block.getFieldValue('port');
    var typeStr = '';

    if (dropdown_type == 'INTENSITY') {
      typeStr = 'reflected_light_intensity';
    } else if (dropdown_type == 'COLOR') {
      typeStr = 'color';
    } else if (dropdown_type == 'COLOR_NAME') {
      typeStr = 'color_name';
    } else if (dropdown_type == 'RED') {
      typeStr = 'rgb[0]';
    } else if (dropdown_type == 'GREEN') {
      typeStr = 'rgb[1]';
    } else if (dropdown_type == 'BLUE') {
      typeStr = 'rgb[2]';
    } else if (dropdown_type == 'RGB') {
      typeStr = 'rgb';
    }

    var code = 'color_sensor_in' + dropdown_port + '.' + typeStr;
    return [code, Blockly.Python.ORDER_ATOMIC];
  };

  // ultrasonic
  this.ultrasonic_sensor = function(block) {
    var dropdown_port = block.getFieldValue('port');
    var dropdown_units = block.getFieldValue('units');

    if (dropdown_units == 'CM') {
      var multiplier = '';
      var order = Blockly.Python.ORDER_ATOMIC;
    } else if (dropdown_units == 'MM') {
      var multiplier = ' * 10';
      var order = Blockly.Python.ORDER_MULTIPLICATIVE;
    }
    var code = 'ultrasonic_sensor_in' + dropdown_port + '.distance_centimeters' + multiplier;
    return [code, order];
  };

  // gyro
  this.gyro_sensor = function(block) {
    var dropdown_type = block.getFieldValue('type');
    var dropdown_port = block.getFieldValue('port')

    if (dropdown_type == 'ANGLE') {
      var typeStr = 'angle';
    } else if (dropdown_type == 'RATE') {
      var typeStr = 'rate';
    }
    var code = 'gyro_sensor_in' + dropdown_port + '.' + typeStr;

    return [code, Blockly.Python.ORDER_ATOMIC];
  };

  // gyro reset
  this.reset_gyro = function(block) {
    var dropdown_port = block.getFieldValue('port');

    var code = 'gyro_sensor_in' + dropdown_port + '.reset()\n';
    return code;
  };

  // say
  this.say = function(block) {
    var value_text = Blockly.Python.valueToCode(block, 'text', Blockly.Python.ORDER_ATOMIC);
    var dropdown_block = block.getFieldValue('block');

    if (dropdown_block == 'NO_BLOCK') {
      var play_type = ', play_type=Sound.PLAY_NO_WAIT_FOR_COMPLETE';
    } else {
      var play_type = '';
    }

    var code = 'spkr.speak("' + value_text + '"' + play_type + ')\n';
    return code;
  }

  // beep
  this.beep = function(block) {
    var dropdown_block = block.getFieldValue('block');

    if (dropdown_block == 'NO_BLOCK') {
      var play_type = 'play_type=Sound.PLAY_NO_WAIT_FOR_COMPLETE';
    } else {
      var play_type = '';
    }

    var code = 'spkr.beep(' + play_type + ')\n';
    return code;
  }

  // play tone
  this.play_tone = function(block) {
    var value_frequency = Blockly.Python.valueToCode(block, 'frequency', Blockly.Python.ORDER_ATOMIC);
    var value_duration = Blockly.Python.valueToCode(block, 'duration', Blockly.Python.ORDER_ATOMIC);
    var dropdown_block = block.getFieldValue('block');

    if (dropdown_block == 'NO_BLOCK') {
      var play_type = ', play_type=Sound.PLAY_NO_WAIT_FOR_COMPLETE';
    } else {
      var play_type = '';
    }

    var code = 'spkr.play_tone(' + value_frequency + ', ' + value_duration + play_type + ')\n';
    return code;
  }

  // Sleep
  this.sleep = function(block) {
    var value_seconds = Blockly.Python.valueToCode(block, 'seconds', Blockly.Python.ORDER_ATOMIC);
    var dropdown_units = block.getFieldValue('units');

    var code = 'time.sleep(' + value_seconds;
    if (dropdown_units == 'SECONDS') {
      code += ')\n';
    } else if (dropdown_units == 'MILLISECONDS') {
      code += ' / 1000)\n';
    }
    return code;
  };

  // Exit
  this.exit = function(block) {
    var code = 'exit()\n';
    return code;
  };

  // time
  this.time = function(block) {
    var code = 'time.time()';

    return [code, Blockly.Python.ORDER_ATOMIC];
  };

  // gps
  this.gps_sensor = function(block) {
    var dropdown_type = block.getFieldValue('type');
    var dropdown_port = block.getFieldValue('port');

    if (dropdown_type == 'X') {
      var typeStr = 'x';
    } else if (dropdown_type == 'Y') {
      var typeStr = 'y';
    } else if (dropdown_type == 'ALTITUDE') {
      var typeStr = 'altitude';
    } else if (dropdown_type == 'POSITION') {
      var typeStr = 'position';
    }

    var code = 'gps_sensor_in' + dropdown_port + '.' + typeStr;

    return [code, Blockly.Python.ORDER_ATOMIC];
  }

}

