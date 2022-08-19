import cv2 as cv
import numpy as np
import roslib
import sys
import math
from geometry_msgs.msg import Twist

def draw_lines(img, houghLines):

    if houghLines is not None:
        for line in houghLines:
            for rho,theta in line:
                a = np.cos(theta)
                b = np.sin(theta)
                x0 = a*rho
                y0 = b*rho
                x1 = int(x0 + 1000*(-b))
                y1 = int(y0 + 1000*(a))
                x2 = int(x0 + 1000*(b))
                y2 = int(y0 + 1000*(-a))

                cv.line(img,(x1,y1),(x2,y2),(0,255,0),2)
    else:
        print("No line detected in image")

def main():
    cap = cv.VideoCapture('lane01.mp4')

    while (cap.isOpened()):
        ret, frame = cap.read()
        #frame_new = cv.resize(frame, (720,480))
        frame_new = frame[560:700,550:850]
        if ret==True:
            gray_image = cv.cvtColor(frame_new, cv.COLOR_RGB2GRAY)
            canny_image = cv.Canny(gray_image,200,300)
            lines = cv.HoughLines(canny_image,rho=6,theta=np.pi/60,threshold=200)
            hough_image = np.zeros_like(frame_new)
            draw_lines(hough_image,lines)
            hough_image = cv.addWeighted(frame_new,0.8,hough_image,1.0,0.0)

            cv.imshow('Frame', canny_image)
            cv.imshow('Frame2', hough_image)

            if cv.waitKey(25) & 0xFF == ord('q'):
                #cv.imwrite('exemplo',hough_image)
	            break
            
        else: break

if __name__ == '__main__':
    main()