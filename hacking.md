# Intro

This is a fork of quirkycorts excellent EV3 virtual environment ([Gears](https://github.com/QuirkyCort/gears) - Generic Educational Robotics Simulator) which uses EV3DEV2's python to use Lego's new version of Micropython, and have the Blockly blocks more in lined with [Lego Education's EV3-micropython API](https://pybricks.github.io/ev3-micropython)

# How code is laid out:

(work in progress)

## Blockly 
* blocklyPanel.js - tab to access Block code menu
* toolbox.xml - defines:
    * category menu item - for logical groupings of block code
    * block definitions - which correspond to actual Python3 code
* customBlocks.json - define the user created variables that need to be included in generated python code
* pybricks_generator.js - generates Python3 code from blocks user selects

## Python
* pythonPanel.js - tab to access Python3 code editor
* ace.js - Python3 code editor

### Python Interpreter
* skultp.js - Javascript implementation of Python2/3
* Python3 robotics support library (in public/js/pybricks folder - formerly ev3dev2 folder)
   * motor.py
   * sound.py
   * sensor/lego.py
* simPython.js -  skulpt implementation of robot components in Javascript, 
   * uses SK.builtin.func to make python code accessible in skulpt Python3 environment; this is the link from Python3 code to javascript virtual environment

### Simulator
* simPanel.js - tab to access virtual environment
* Babylon.js - 3D virtual environment implemented in Javascript
* Robot.js - Javascript robot representation used in Babylon.js
  * robotComponents.js - 
  * robotTemplates.js - robot and component dimensions