from machine import Pin, SPI
from st7565 import ST7565
import random, time

RST = Pin(27, Pin.OUT)
A0 = Pin(26, Pin.OUT)
CS = Pin(28, Pin.OUT)
spibus = SPI(0, baudrate=9600, polarity=1, phase=1,
             sck = Pin(2),
             miso = Pin(4),
             mosi = Pin(3))

# Méthodes:
# https://docs.micropython.org/en/latest/library/framebuf.html

def setText(display, text):
    current_idx = 0
    posx = 8
    posy = 5
    length  = 0
    words = text.split(' ')
    for word in words:
        length += len(word)*8 + 8
        if length>128:
            posy += 10
            posx = 8
            length = len(word)*8 + 8
        display.text(word, posx, posy)
        posx += length
        
shape1 = {
    1 : None,
    2 : None,
    3 : (17,18),
    4 : (18,19),
    5 : (18,19,20),
    6 : (11,12,13,19,20,21),
    7 : (10,11,12,13,14,15,20,21),
    8 : (9,10,11,12,13,14,20,21,22),
    9 : (7,8,9,10,11,12,13,21,22,23),
    10 : (7,8,9,10,11,12,13,21,22,23,24),
    11 : (6,7,8,9,10,11,12,13,14,15,22,23,24),
    12 : (7,8,9,10,12,13,14,15,16,22,23,24,25),
    13 : (9,13,14,15,16,17,22,23,24,25),
    14 : (14,15,16,17,22,23,24,25),
    15 : (15,16,17,18,19,22,23,24,25),
    16 : (16,17,18,19,20,22,23,24,25),
    17 : (17,18,19,20,22,23,24,25),
    18 : (18,19,20,21,22,23,24),
    19 : (10,11,19,20,21,22,23,24),
    20 : (8,9,10,11,12,13,14,17,18,19,20,21,22,23,24),
    21 : (7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24),
    22 : (5,6,7,8,9,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26),
    23 : (5,6,7,8,14,15,16,17,18,19,20,22,23,24,25,26,27),
    24 : (4,5,6,7,23,24,25,26,27),
    25 : (4,5,6,24,25),
    26 : None,
    27 : None,
    28 : None
    }

display = ST7565(spibus, A0, CS, RST)
setText(display, "hey")




display.show()
