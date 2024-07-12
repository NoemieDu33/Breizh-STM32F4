import math
import utime
from machine import Pin, I2C, UART
from mpu6050 import MPU6050
from ws2812 import WS2812

power = machine.Pin(11,machine.Pin.OUT)
power.value(1)

# uart = UART(1, baudrate=9600, tx=Pin(0), rx=Pin(1))
# uart.init(bits=8, parity=None, stop=2)

uart = UART(0,baudrate = 115200,bits = 8,parity = None,stop = 1 ,tx = Pin(0),rx = Pin(1))

BLACK = (0, 0, 0)
RED = (255, 0, 0)
YELLOW = (255, 150, 0)
GREEN = (0, 255, 0)
CYAN = (0, 255, 255)
BLUE = (0, 0, 255)
PURPLE = (180, 0, 255)
WHITE = (255, 255, 255)
COLORS = (RED, YELLOW, GREEN, CYAN, BLUE, PURPLE, WHITE)
 
led = WS2812(12,1) #WS2812(pin_num,led_count)

mpu = MPU6050(bus=1, scl=Pin(7), sda=Pin(6))


def main():
    last_Gx, last_Gy = 64, 64
    i=0

    while True:
        g = mpu.readData()
        x = int(10*round(g.Gx, 1))+80
        y = int(10*round(g.Gy, 1))+80
        z = int(10*round(g.Gz, 1))+80
        print(f"X={x}\tY={y}\tZ={z}\n")
        bytearr = chr(x)+chr(y)+chr(z)
        uart.write(bytearr)
        if uart.any(): 
            data = uart.read() 
            if data is not None:
                led.pixels_fill(COLORS[i%len(COLORS)])
                led.pixels_show()
                i+=1
        
        print(f"{bytearr=}")
        utime.sleep_ms(100)


if __name__ == "__main__":
    main()