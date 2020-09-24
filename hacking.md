# Intro

This is a fork of quirkycorts excellent EV3 virtual environment ([Gears](https://github.com/QuirkyCort/gears) - Generic Educational Robotics Simulator) which uses EV3DEV2's python to use Lego's new version of Micropython, and have the Blockly blocks more in lined with [Lego Education's EV3-micropython API](https://pybricks.github.io/ev3-micropython)

# How code is laid out:

(work in progress)

## Blockly 
* blocklyPanel.js - tab to access Block code menu
* uses Google's Blockly - JavaScript library for building visual programming editors.
* toolbox.xml - defines:
    * category menu item - logical groupings of block code
    * block definitions - corresponds to actual Python code
* customBlocks.json - defines variables that user fills in, that is then included in generated python code
* pybricks_generator.js - generates Python code from user selected blocks

## Python Code Editor
* pythonPanel.js - tab to access Python code editor
* ace.js - embeddable code editor written in JavaScript

## Python Interpreter
  * skultp.js - Skulpt is a Javascript implementation of Python 2.x. - Python that runs in your browser.

## 3D Simulator
* simPanel.js - tab to access 3D virtual environment
  * Python robotics support library (in public/js/pybricks folder - formerly ev3dev2 folder)
    * motor.py
    * sound.py
    * sensor/lego.py
  * simPython.js -  skulpt implementation of robot components in Javascript, 
    * uses SK.builtin.func to make Javascript funtions available in Python environment; this is the link from Python code to the Javascript virtual environment

* 3D environment
  * Babylon.js - Web-based 3D environment implemented in Javascript
  * Robot.js - Javascript robot representation used in Babylon.js
    * robotComponents.js - 
    * robotTemplates.js - robot and component dimensions


-----

## Howto create new command in its own Python file

(See also: https://blockly-demo.appspot.com/static/demos/blockfactory/index.html)

* update toolbox.xml
```
 <category name="Pybrick" colour="#5b67a5">
    <block type="move_straight">
      <field name="units">PERCENT</field>
      <value name="left">
        <shadow type="math_number">
          <field name="NUM">20</field>
        </shadow>
      </value>
    </block>
  </category>

```

* update customBlocks.json
```
{
  "type": "move_straight",
  "message0": "move straight for speed %1",
  "args0": [
    {
      "type": "input_value",
      "name": "speed",
      "check": "Number"
    }
  ],
  "inputsInline": true,
  "previousStatement": null,
  "nextStatement": null,
  "colour": 230,
  "tooltip": "",
  "helpUrl": ""
},
```

* create new Python file: ev3dev2/pybricks.py
```
# Needed to prevent loops from locking up the javascript thread
SENSOR_DELAY = 0.001

# Import the necessary libraries
import simPython, time
import math
from ev3dev2.motor import *

def straight(speed):
    tank_drive = MoveTank(OUTPUT_A, OUTPUT_B)
    tank_drive.on(speed, speed) 

```

* update skulpt.js
  * add pybricks.py to externalLibs
```
   var externalLibs = {
      ...
      './ev3dev2/pybricks.py': 'ev3dev2/pybricks.py?v=1596843175',          
      ...
    }

```
  * if creating new folder, need to include empty '__init__.py' in folder, and in any subfolders
```
    var externalLibs = {
      ...
      './pybricks/__init__.py': 'pybricks/__init__.py?v=1596843175',      
      './pybricks/robotics.py': 'pybricks/robotics.py?v=1596843175',   
      ...        
```

* update ev3dev_generator.js
  * assign new function to Python command
```
  // Load Python generators
  this.load = function() {
    Blockly.Python['move_straight'] = self.move_straight; // !!!!!!  
    ...  
```
  * update base code
```
  // Generate python code
  this.genCode = function() {
    let code =
      '#!/usr/bin/env python3\n' +
      `\n` +
      '# Import the necessary libraries\n' +
      'import math\n' +
      'import time\n' +      
      'from ev3dev2.pybricks import *\n' +      
      ...
```
  * add Python generator function for newly created Python command
```
 // move straight
  this.move_straight = function(block) {
    var speed = Blockly.Python.valueToCode(block, 'speed', Blockly.Python.ORDER_ATOMIC);
    if (distance === undefined) { speed = 0;  }

    var code = 'straight(' + speed + ')\n';

    return code;    
  }
```

