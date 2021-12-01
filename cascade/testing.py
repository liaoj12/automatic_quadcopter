# -*- coding: utf-8 -*-
"""
Created on Thu Oct 01 02:17:12 2015

@author: JunjieLiao
"""

import cv2

"""
Just make sure that you put the xml file and the picture in the same
folder as the codes or provide the absolute path.
"""

balloon_cascade = cv2.CascadeClassifier('balloon-0-12.xml')
#balloon_cascade = cv2.CascadeClassifier('lbp-0-9.xml')


img = cv2.imread('image14-496.bmp', 1)
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

balloons = balloon_cascade.detectMultiScale(gray, 1.3, 5)

for (x, y, w, h) in balloons:
    cv2.rectangle(img, (x, y), (x+w, y+h), (255, 0, 0), thickness=2)


cv2.imshow('img', img)
cv2.waitKey(0)
cv2.destroyAllWindows()
