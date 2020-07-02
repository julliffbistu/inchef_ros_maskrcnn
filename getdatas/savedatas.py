#!/usr/bin/env python

import roslib
import sys
import rospy
import cv2
import math
from std_msgs.msg import String, Float32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import skimage.io
from skimage.util import img_as_float, img_as_ubyte
from skimage.segmentation import felzenszwalb
from skimage.segmentation import mark_boundaries
import skimage.color as color
import numpy as np
import time
import matplotlib.pyplot as plt
import scipy.io as scio
from numpy import *
from numpy.linalg import inv, qr,det
from math import sqrt
import datetime
import os

count_rgb_left=0
count_depth_left=0
count_rgb_right=0
count_depth_right=0
time_rgb_left=0
time_depth_left=0
time_rgb_right=0
time_depth_right=0
cout_rgb_left_dismatch=0
cout_depth_left_dismatch=0
count_rgb_depth_left_dismatch=0


cout_rgb_right_dismatch=0
cout_depth_right_dismatch=0
count_rgb_depth_right_dismatch=0    
two_camera_timediff=0
count_two_camera_dismatch=0
'''
count=2
savename='./left_camera/'+'left_rgb_'+str(count)+'.avi'
out = cv2.VideoWriter(savename,cv2.VideoWriter_fourcc('M','J','P','G'),20,(960,540))
'''
def callback_rgb_left(image):

   
    global count_rgb_left
    count_rgb_left=count_rgb_left+1
    print("count_rgb_left:",count_rgb_left)
    now = rospy.get_rostime()
    global time_rgb_left
    deltatime = abs((image.header.stamp.to_sec()-time_rgb_left)*1000)
    #print("timestamp doesn't match", deltatime)
    time_rgb_left=image.header.stamp.to_sec()
    print("time_rgb_left:",time_rgb_left)


 ### two rgb frame dismatch time and count
    global cout_rgb_left_dismatch
    if(deltatime > 100):
        print("timestamp doesn't match", deltatime)
        cout_rgb_left_dismatch=cout_rgb_left_dismatch+1
       # raise Exception,"Invalid level!"    
    print("cout_rgb_left_dismatch",cout_rgb_left_dismatch)
    bridge=CvBridge()
    cv_image =bridge.imgmsg_to_cv2(image,"bgr8")
    vis = cv_image.copy()
    print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~`")

    #count_rgb_left=1
    rgb_savename='./left_camera/'+'left_rgb_'+str(count_rgb_left)+'.jpg'
    cv2.imwrite(rgb_savename,vis)
    #out.write(vis)
def callback_depth_left(image):

    now = rospy.get_rostime()
    global count_depth_left
    count_depth_left=count_depth_left+1
    print("count_depth_left:",count_depth_left)
    now = rospy.get_rostime()
    global time_depth_left
    deltatime = abs((image.header.stamp.to_sec()-time_depth_left)*1000)
    time_depth_left=image.header.stamp.to_sec()
    print("time_depth_left:",time_depth_left)

#### rgb and depth in same camera
    global count_rgb_depth_left_dismatch
    rgb_depth_timediff=abs(time_depth_left-time_rgb_left)*1000
    if rgb_depth_timediff>100:
        count_rgb_depth_left_dismatch=count_rgb_depth_left_dismatch+1
    print("rgb_depth_timediff",rgb_depth_timediff)
    print("count_rgb_depth_left_dismatch",count_rgb_depth_left_dismatch)
### two depth frame dismatch time and count
    global cout_depth_left_dismatch
    if(deltatime > 100):
        print("timestamp doesn't match", deltatime)
        cout_depth_left_dismatch=cout_depth_left_dismatch+1
      #  raise Exception,"Invalid level!"
    print("cout_depth_left_dismatch",cout_depth_left_dismatch)
    bridge=CvBridge()
    cv_image_depth = bridge.imgmsg_to_cv2(image,"passthrough")
    cv_image_depth_left = cv_image_depth.copy()
    #out.write(vis)
    depth_savename='./left_camera/''left_depth_'+str(count_rgb_left)+'.jpg'
    print("+++++++++++++++++++++++++++++++++++++++++++")
    scio.savemat(depth_savename, {'cv_image_depth_left':cv_image_depth_left})


def callback_rgb_right(image):
   


    global count_rgb_right
    count_rgb_right=count_rgb_right+1
    print("count_rgb_right:",count_rgb_right)
    now = rospy.get_rostime()
    global time_rgb_right
    deltatime = abs((image.header.stamp.to_sec()-time_rgb_right)*1000)
    #print("timestamp doesn't match", deltatime)
    time_rgb_right=image.header.stamp.to_sec()
    print("time_rgb_right:",time_rgb_right)


### two rgb frame dismatch time and count
    global cout_rgb_right_dismatch
    if(deltatime > 100):
        print("timestamp doesn't match", deltatime)
        cout_rgb_right_dismatch=cout_rgb_right_dismatch+1
       # raise Exception,"Invalid level!"    
    print("cout_rgb_right_dismatch",cout_rgb_right_dismatch)
    bridge=CvBridge()
    cv_image =bridge.imgmsg_to_cv2(image,"bgr8")
    vis = cv_image.copy()
   # cv2.imshow("vis",vis)
   
    print("rrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrr`")
    #out.write(vis)
    
    rgb_savename='./right_camera/'+'right_rgb_'+str(count_rgb_right)+'.jpg'
    #cv2.imwrite(rgb_savename,vis)


def callback_depth_right(image):
#counts of depth image processing
    now = rospy.get_rostime()
    global count_depth_right
    count_depth_right=count_depth_right+1
    print("count_depth_right:",count_depth_right)
    now = rospy.get_rostime()
#time of depth image processing
    global time_depth_right
    deltatime = abs((image.header.stamp.to_sec()-time_depth_right)*1000)
    time_depth_right=image.header.stamp.to_sec()
    print("time_depth_right:",time_depth_right)

#### rgb and depth of same camera time dismatch
    global count_rgb_depth_right_dismatch
    rgb_depth_timediff=abs(time_depth_right-time_rgb_right)*1000
    if rgb_depth_timediff>100:
        count_rgb_depth_right_dismatch=count_rgb_depth_right_dismatch+1
    print("rgb_depth_timediff",rgb_depth_timediff)
    print("count_rgb_depth_right_dismatch",count_rgb_depth_right_dismatch)
 ### two depth frame dismatch time and count
    global cout_depth_right_dismatch
    if(deltatime > 100):
        print("timestamp doesn't match", deltatime)
        cout_depth_right_dismatch=cout_depth_right_dismatch+1
      #  raise Exception,"Invalid level!"
    print("cout_depth_right_dismatch",cout_depth_right_dismatch)
 ### two cameras time diff
    global two_camera_timediff
    global count_two_camera_dismatch
    two_camera_timediff=abs(time_depth_right-time_depth_left)*1000
    print("two_camera_timediff",two_camera_timediff)
    if(two_camera_timediff>100):
        count_two_camera_dismatch=count_two_camera_dismatch+1
    print("count_two_camera_dismatch",count_two_camera_dismatch)    



    
    bridge=CvBridge()
    cv_image_depth = bridge.imgmsg_to_cv2(image,"passthrough")
    cv_image_depth_right= cv_image_depth.copy()
    #out.write(vis)
    depth_savename='./right_camera/''right_depth_'+str(count_rgb_right)+'.jpg'
    print("kkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkk")
   # scio.savemat(depth_savename, {'cv_image_depth_right':cv_image_depth_right})




    
def listener():
    rospy.init_node("getdatas_node")
    rospy.Subscriber("/kinectleft/qhd/image_depth_rect",Image,callback_depth_left,queue_size=1)
    rospy.Subscriber("/kinectleft/qhd/image_color_rect",Image,callback_rgb_left,queue_size=1)
    #rospy.Subscriber("/kinectright/qhd/image_depth_rect",Image,callback_depth_right,queue_size=1)
   # rospy.Subscriber("/kinectright/qhd/image_color_rect",Image,callback_rgb_right,queue_size=1)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    #
    
if __name__ == '__main__':
    
    listener()
    
