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
* Python robotics support library (in public/js/pybricks folder - formerly ev3dev2 folder)
   * motor.py
   * sound.py
   * sensor/lego.py
* simPython.js -  skulpt implementation of robot components in Javascript, 
   * uses SK.builtin.func to make Javascript funtions available in Python environment; this is the link from Python code to the Javascript virtual environment

## 3D Simulator
* simPanel.js - tab to access 3D virtual environment
* Babylon.js - Web-based 3D environment implemented in Javascript
* Robot.js - Javascript robot representation used in Babylon.js
  * robotComponents.js - 
  * robotTemplates.js - robot and component dimensions