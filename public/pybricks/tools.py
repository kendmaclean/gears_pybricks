# Needed to prevent loops from locking up the javascript thread
SENSOR_DELAY = 0.001

# Import the necessary libraries
import simPython, time
import math

def wait(wait_time_ms):
    secs = wait_time_ms / 1000
    time.sleep(secs)

            