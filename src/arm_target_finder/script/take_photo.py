#!/usr/bin/python

import cv2
from sys import argv

if len(argv) != 3:
    print 'usage: take_photo camera_id filename'
else:
    cap = cv2.VideoCapture(int(argv[1]))
    if(cap.isOpened()):
        ret, frame = cap.read()
        cv2.imwrite(argv[2], frame)
    else:
        print 'error: cannot open camera'



