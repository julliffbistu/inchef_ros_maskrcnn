# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""

import cv2
import threading
import numpy as np
from time import ctime,sleep
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
b=0
def callback_rgb_left(image):
   
    global count_rgb_left
    count_rgb_left=count_rgb_left+1
    print("count_rgb_left:",count_rgb_left)
    now = rospy.get_rostime()
    print("now:",now.to_sec())
    print("b:",b)



    global time_rgb_left
    time_rgb_left=image.header.stamp.to_sec()
    print("time_rgb_left:",time_rgb_left)


    deltatime = (now.to_sec()-image.header.stamp.to_sec())*1000
    
    if(deltatime > 30):
        print("timestamp doesn't match", deltatime)
        raise Exception,"Invalid level!"
    bridge=CvBridge()
    cv_image =bridge.imgmsg_to_cv2(image,"bgr8")
    vis = cv_image.copy()
    
    #out.write(vis)
    rgb_savename='./left_camera/'+'left_rgb_'+str(count_rgb_left)+'.jpg'
    #cv2.imwrite(rgb_savename,vis)

def callback_depth_left(image):

    now = rospy.get_rostime()
    global count_depth_left
    count_depth_left=count_depth_left+1
    print("count_depth_left:",count_depth_left)
    now = rospy.get_rostime()
    global time_depth_left
    time_depth_left=image.header.stamp.to_sec()
    print("time_depth_left:",time_depth_left)

    bridge=CvBridge()
    cv_image_depth = bridge.imgmsg_to_cv2(image,"passthrough")
    cv_image_depth_left = cv_image_depth.copy()
    #out.write(vis)
    depth_savename='./left_camera/''left_depth_'+str(count_rgb_left)+'.jpg'

   # scio.savemat(depth_savename, {'cv_image_depth_left':cv_image_depth_left})


def callback_rgb_right(image):
   
    global count_rgb_right
    count_rgb_right=count_rgb_right+1
    print("save count_rgb_right:",count_rgb_right)
    now = rospy.get_rostime()
    deltatime = (now.to_sec()-image.header.stamp.to_sec())*1000

    
    beginning = time.time()
    bridge=CvBridge()
    cv_image =bridge.imgmsg_to_cv2(image,"bgr8")
    vis = cv_image.copy()
    #out.write(vis)
    rgb_savename='./right_camera/'+'right_rgb_'+str(count_rgb_right)+'.jpg'
   # cv2.imwrite(rgb_savename,vis)

def callback_depth_right(image):
    a=image.header.stamp.to_sec()
    print("~~~~~~~~~~~~~",a)
    now = rospy.get_rostime()
    deltatime = (now.to_sec()-image.header.stamp.to_sec())*1000

    
    beginning = time.time()
    bridge=CvBridge()
    cv_image_depth = bridge.imgmsg_to_cv2(image,"passthrough")
    cv_image_depth1 = cv_image_depth.copy()
    #out.write(vis)
    depth_savename='./right_camera/''right_depth_'+str(count_rgb_right)+'.jpg'
    
    #scio.savemat(depth_savename, {'cv_image_depth_1':cv_image_depth1})

def thread_1():
    
    while(1):
        pass
        #print("The thread_1 time is %s \n" %ctime())
     
def thread_2():
    while(1):
        b=100
        #print("The thread_2 time is %s \n" %ctime())

    
threads = []
t1 = threading.Thread(target = thread_1)
threads.append(t1)
t2 = threading.Thread(target = thread_2)
threads.append(t2)

if __name__ == '__main__':
    
    for t in threads:
        t.setDaemon(True)
        t.start()
    #t.join()  ##等待子进程是否结束
    #rospy.init_node("getdatas_node")
    #rospy.Subscriber("/kinectleft/hd/image_color",Image,callback_rgb_left)
    #rospy.spin()
    while(1):
        rospy.init_node("getdatas_node")
        rospy.Subscriber("/kinectleft/hd/image_color",Image,callback_rgb_left)
        rospy.spin()
        print("The main thread time is %s \n" %ctime())


    cv2.waitKey(1)
    
cv2.destroyAllWindows()

