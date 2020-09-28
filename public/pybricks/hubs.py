# Needed to prevent loops from locking up the javascript thread
SENSOR_DELAY = 0.001

# Import the necessary libraries
import time
import math
from pybricks.parameters import *

class EV3Brick:
    def __init__(self):
        self.buttons = Buttons()        
        self.light = Light()                
        self.speaker = Speaker()  
        self.screen = Screen()          
        self.battery = Battery()    

class Buttons():
    def pressed(self):
        print("buttons pressed")

class Light():
    def on(self, color):
        print("light on " + str(color))

    def off(self):
        print("light off")        

class Speaker():
    def beep(self, frequency=500, duration=100):
        print("beep")

    def play_notes(self, notes, tempo=120):
        print("notes " + ','.join(notes))

    def play_file(self, file_name):
        print("play_file " + file_name)

    def say(self, text):
        print("say " + text)

    def set_speech_options(self, language=None, voice=None, speed=None, pitch=None):
        print("set_speech_options " + language)

    def set_volume(self, volume, which='_all_'):
        print("set_volume " + str(volume))

class Screen():
    def clear(self):
        print("clear screen")       

    def draw_text(self, x, y, text, text_color=Color.BLACK, background_color=None):
        print("text" + text)  

    def print(self, *args, sep=' ', end='\n'):
        output = ''

        for textobj in args:
            output = output + sep + textobj
        output = output + end    

        print(output)     

    def set_font(self, font):
        print("font" + str(font))    

    def load_image(self, source):
        print("load_image" + str(source))           

    def draw_image(self, x, y, source, transparent=None):
        print("draw_image" + str(source))         

    def draw_pixel(self, x, y, color=Color.BLACK):
        print("draw_pixel")      

    def draw_line(self, x1, y1, x2, y2, width=1, color=Color.BLACK):
        print("draw_line")       

    def draw_box(self, x1, y1, x2, y2, r=0, fill=False, color=Color.BLACK):
        print("draw_box")     

    def draw_circle(self, x, y, r, fill=False, color=Color.BLACK):
        print("draw_circle")          

    def width(self):
        print("width")            
        return 0             

    def height(self):
        print("height")               
        return 0      

    def save(self, filename):
         print("save " + filename)                    


class Battery():
    def voltage(self):
        print("voltage")     
        return 0

    def current(self):
        print("current")      
        return 0
