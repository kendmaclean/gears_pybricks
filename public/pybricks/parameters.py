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
    # COLOR_NOCOLOR = 0 # HSV values from https://lego.fandom.com/wiki/Colour_Palette
    BLACK = 1   # 0, 0, 0
    BLUE = 2    # 207, 64, 78
    GREEN = 3   # 120, 100, 60
    YELLOW = 4  # 60, 100, 100
    RED = 5     # 0, 100, 100
    WHITE = 6   # 0, 0, 100
    BROWN = 7   # 24, 79, 25

#class ButtonP:
#   print("Button not implemented") 