#!/usr/bin/env python3
'''
Displays 35x35 images from PAA3905 sent over serial port

Copyright (c) 2024 Simon D. Levy

MIT License
'''

import argparse
import serial
import cv2
import numpy as np
from sys import stdout

SCALEUP = 10
FONT = cv2.FONT_HERSHEY_SIMPLEX
FONT_SIZE = 0.5
FONT_COLOR = (255,255,255)
FONT_THICKNESS = 1

def debug(s):
    print(s)
    stdout.flush()

def new_image():
    return np.zeros((35,35), dtype='uint8'), 0

parser = argparse.ArgumentParser()

parser.add_argument(dest='port', help='COM port')

args = parser.parse_args()

port = serial.Serial(args.port, 115200)

count = 0

image, count = new_image()

while True:

    b = ord(port.read(1))

    if b == 0xFF:

        resized = cv2.resize(image, (35*SCALEUP,35*SCALEUP), 
                interpolation= cv2.INTER_NEAREST)

        '''
        for j in range(35):
            for k in range(35):
                cv2.putText(resized,
                            '%d' % image[k,j],
                            (j*SCALEUP+SCALEUP//4,k*SCALEUP+SCALEUP//2),
                            FONT, FONT_SIZE, FONT_COLOR, FONT_THICKNESS, cv2.LINE_AA)

        '''

        cv2.imshow(      'PAA3905 [ESC to quit]', resized)

        if cv2.waitKey(1) == 27:
            break

        image, count = new_image()

    else:

        image[35-(count//35)-1, 35-(count%35)-1] = b
        count += 1

