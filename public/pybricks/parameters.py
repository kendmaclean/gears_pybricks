# Needed to prevent loops from locking up the javascript thread
SENSOR_DELAY = 0.001

class PortP:
    # motor ports
    A = 'outA'
    B = 'outB'
    C = 'outC'
    D = 'outD'

    # sensor ports
    S1 = 'in1'
    S2 = 'in2'
    S3 = 'in3'
    S4 = 'in4'
    S5 = 'in5'

class DirectionP:
    CLOCKWISE = 1
    COUNTERCLOCKWISE = -1

class StopP:
    COAST = 'COAST'
    BRAKE = 'BRAKE'
    HOLD = 'HOLD'

class ColorP:
    BLACK = 'BLACK'
    BLUE = 'BLUE'
    GREEN = 'GREEN'
    YELLOW = 'YELLOW'
    RED = 'RED'
    WHITE = 'WHITE'    
    BROWN = 'BROWN'
    ORANGE = 'ORANGE'
    PURPLE = 'PURPLE'    

#class ButtonP:
#   print("Button not implemented") 