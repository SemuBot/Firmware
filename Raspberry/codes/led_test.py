#!/usr/bin/env python3

#This code makes all LED-s on the strip blink red, then green and then blue
# NB! If running first time this code, run /Raspberry/scripts/setup.sh first. 

import time
import board
import neopixel

# Configure the setup
PIXEL_PIN = board.D10  # pin that the NeoPixel is connected to
ORDER = neopixel.RGB  # pixel color channel order
COLOR = (255, 0, 0)  # color to blink
COLOR2 = (0, 255, 0)  # color to blink
COLOR3 = (0, 0, 255)  # color to blink

CLEAR = (0, 0, 0)  # clear (or second color)
DELAY = 0.2  # blink rate in seconds

# Create the NeoPixel object
pixel = neopixel.NeoPixel(PIXEL_PIN, 3, pixel_order=(1, 0, 2),brightness=0.1)

# Loop forever and blink the color
while True:
    pixel[0] = COLOR
    time.sleep(DELAY)
    pixel[1] = COLOR2
    time.sleep(DELAY)
    pixel[2] = COLOR3
    time.sleep(DELAY)
