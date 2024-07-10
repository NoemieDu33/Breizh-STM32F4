import math
import utime
from machine import Pin, I2C
from mpu6050 import MPU6050
from ws2812 import WS2812

power = machine.Pin(11,machine.Pin.OUT)
power.value(1)
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

    while True:
        for color in COLORS:
            led.pixels_fill(color)
            led.pixels_show()
            
            g = mpu.readData()
            x = round(g.Gx, 1)
            y = round(g.Gy, 1)
            z = round(g.Gz, 1)
            
            print(f"X:{x}\tY:{y}\tZ:{z}")
            utime.sleep_ms(200)


if __name__ == "__main__":
    main()