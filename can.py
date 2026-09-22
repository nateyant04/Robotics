import cv2
import numpy as np
import scipy

cam = cv2.VideoCapture(0)
r, bg = cam.read()
input()
r, current=cam.read()

bg = scipy.ndimage.gaussian_filter(bg, sigma=5)
current = scipy.ndimage.gaussian_filter(current, sigma=5)

cv2.imwrite("a.jpg", bg-current)
    
    
