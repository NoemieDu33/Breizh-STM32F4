import math
import utime
from machine import Pin, UART, SPI
from ws2812 import WS2812
from st7565 import ST7565
import random, time

power = machine.Pin(11,machine.Pin.OUT)
power.value(1)

RST = Pin(27, Pin.OUT)
A0 = Pin(26, Pin.OUT)
CS = Pin(28, Pin.OUT)
spibus = SPI(0, baudrate=9600, polarity=1, phase=1,
             sck = Pin(2),
             miso = Pin(4),
             mosi = Pin(3))

def setText(display, text):
    display.fill(0)
    current_idx = 0
    posx = 8
    posy = 5
    length  = 0
    words = text.split(' ')
    for word in words:
        length += len(word)*8 + 8
        if "\n" in word:
            sub_words = word.split('\n')
            for sbw in sub_words:
                length += len(sbw)*8 + 8
                if length>128:
                    posy += 10
                    posx = 8
                    length = len(sbw)*8 + 8
                display.text(sbw, posx, posy)
                posy += 10
                posx = 8
                length = len(sbw)*8+8
            continue

        if length>128:
            posy += 10
            posx = 8
            length = len(word)*8 + 8
        display.text(word, posx, posy)
        posx += length
        

display = ST7565(spibus, A0, CS, RST)
setText(display, 'init done')
display.show()

uart = UART(0,baudrate = 115200,bits = 8,parity = None,stop = 1 ,tx = Pin(0),rx = Pin(1))

def main():
    while True:
        #bytearr = chr(x)+chr(y)+chr(z)
        #uart.write(bytearr)
        if uart.any(): 
            data = uart.read() 
            if data is not None:
                led.pixels_fill(COLORS[i%len(COLORS)])
                led.pixels_show()
                i+=1
        utime.sleep_ms(100)


if __name__ == "__main__":
    main()
    