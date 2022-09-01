#!/usr/bin/env python 
import cv2 as cv
import numpy as np
import rospy
import sys
import math
from geometry_msgs.msg import Twist

def draw_lines(img, houghLines):

    if houghLines is not None:
        #for line in houghLines:
            for rho,theta in houghLines[0]:
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

def get_theta(img, houghLines):

    if houghLines is not None:
        #for line in houghLines:
            for theta in houghLines[0]:
                return theta
    else:
        return 0

def main():
    rospy.init_node('test', anonymous=True )
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()

    vel_msg.linear.y=0
    vel_msg.linear.z=0
    vel_msg.angular.x=0
    vel_msg.angular.y=0

    cap = cv.VideoCapture('lane01.mp4')

    while (cap.isOpened()):
        ret, frame = cap.read()

        vel_msg.linear.x=1.0

        if ret==True:
            frame_new = frame[560:700,550:850]

            src = np.float32([[0,239],[200,239],[0,0],[200,0]])
            dst = np.float32([[130,239],[170,239],[0,0],[299,0]])
            M = cv.getPerspectiveTransform(src,dst)

            warp_image = cv.warpPerspective(frame_new, M, (300,220))

            white_filtro = cv.inRange(warp_image, (230,230,230), (255,255,255))

            result_image = cv.bitwise_and(warp_image,warp_image, mask = white_filtro)

            gray_image = cv.cvtColor(result_image, cv.COLOR_RGB2GRAY)

            canny_image = cv.Canny(gray_image,200,300)
            lines = cv.HoughLines(canny_image,rho=2,theta=np.pi/90,threshold=150)
            hough_image = np.zeros_like(warp_image)
            draw_lines(hough_image,lines)
            hough_image = cv.addWeighted(warp_image,0.8,hough_image,1.0,0.0)

            if (get_theta!=0):
                vel_msg.linear.z=(get_theta/180)*math.pi
                
            pub.publish(vel_msg)

            cv.imshow ('Normal', white_filtro)
            cv.imshow('Canny', canny_image)
            cv.imshow('Lines', hough_image)
            cv.imshow ('Bird', warp_image)

            if cv.waitKey(25) & 0xFF == ord('q'):
                #cv.imwrite('exemplo',hough_image)
                break
           
        else: break

if __name__ == '__main__':
    main()