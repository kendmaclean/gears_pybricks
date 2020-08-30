# Needed to prevent loops from locking up the javascript thread
SENSOR_DELAY = 0.001

class PortP:
    A = 'outA'
    B = 'outB'
    C = 'outC'
    D = 'outD'

    S1 = 'inS1'
    S2 = 'inS2'
    S3 = 'inS3'
    S4 = 'inS4'

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