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
import tensorflow as tf



ROOT_DIR = os.path.abspath("./")

sys.path.append(ROOT_DIR)
print "niubibibibi:::::",ROOT_DIR

import maskrcnn

import keras
vis_global=np.zeros([960,540,3],dtype=np.uint8)
depth_global=np.zeros([960,540,1],dtype=np.uint16)


count_rgb_left=0
count_depth_left=0
count_rgb_right=0
count_depth_right=0

def spinOnce():
    r = rospy.Rate(1)
    r.sleep()

def callback_rgb_left(image):
   


    global count_rgb_left
    count_rgb_left=count_rgb_left+1

    now = rospy.get_rostime()
    imgaehead = image.header.stamp.to_sec()
    deltatime = (now.to_sec()-image.header.stamp.to_sec())*1000
    print("imgaehead", imgaehead)
    if(deltatime > 100):
        print("rgb timestamp doesn't match", deltatime)
        return
    
    beginning = time.time()
    bridge=CvBridge()
    cv_image =bridge.imgmsg_to_cv2(image,"passthrough")
    vis = cv_image.copy()
    global vis_global
    vis_global=cv_image.copy()
    print("~~~~~~~~~~~",vis.shape)
    vis_temp=cv2.resize(vis,(640,480))
    print("~~+++++++++++~~",vis_temp.shape)


def callback_depth_left(image):

    now = rospy.get_rostime()
    a=image.header.stamp.to_sec()
    print("~~~~~~~~~~~~~",a)
    deltatime = (now.to_sec()-image.header.stamp.to_sec())*1000
    if(deltatime > 100):
        print("depth timestamp doesn't match", deltatime)
        return
    
    beginning = time.time()
    bridge=CvBridge()
    cv_image_depth = bridge.imgmsg_to_cv2(image,"passthrough")
    global depth_global
    depth_global = cv_image_depth.copy()
    #out.write(vis)
    #depth_savename='./left_camera/''left_depth_'+str(count_rgb_left)+'.jpg'

   # scio.savemat(depth_savename, {'cv_image_depth_left':cv_image_depth_left})


def callback_rgb_right(image):
    pubtemp = rospy.Publisher('test', String, queue_size=10)
    pubtemp.publish("test")
    global count_rgb_right
    count_rgb_right=count_rgb_right+1
    print("save count_rgb_right:",count_rgb_right)
    now = rospy.get_rostime()
    deltatime = (now.to_sec()-image.header.stamp.to_sec())*1000
    if(deltatime > 100):
        print("rgb timestamp doesn't match", deltatime)
        return
    
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
    if(deltatime > 100):
        print("depth timestamp doesn't match", deltatime)
        return
    
    beginning = time.time()
    bridge=CvBridge()
    cv_image_depth = bridge.imgmsg_to_cv2(image,"passthrough")
    cv_image_depth1 = cv_image_depth.copy()
    #out.write(vis)
    depth_savename='./right_camera/''right_depth_'+str(count_rgb_right)+'.jpg'
    
    #scio.savemat(depth_savename, {'cv_image_depth_1':cv_image_depth1})



    
def listener():
    rospy.init_node("getdatas_node")
    rospy.Subscriber("/kinectleft_new/qhd/image_color",Image,callback_rgb_left)
    rospy.Subscriber("/kinectleft_new/qhd/image_depth_rect",Image,callback_depth_right,queue_size=1)
    pub = rospy.Publisher('topic_name', String, queue_size=10)
    r = rospy.Rate(1)
    while not rospy.is_shutdown():
        #rospy.Subscriber("/kinectleft/hd/image_depth_rect",Image,callback_depth_left)
       
        print("111111111111111111111")
        #rospy.Subscriber("/kinectright/hd/image_depth_rect",Image,callback_depth_right)
        #rospy.Subscriber("/kinectright/hd/image_color",Image,callback_rgb_right)
        # spin() simply keeps python from exiting until this node is stopped
        
 
        print("save count_rgb_left:",count_rgb_left)
        vis_global_now=vis_global.copy()
        depth_global_now=depth_global.copy()
        maskrcnn.model_detection(vis_global_now,depth_global_now)
        pub.publish("hello")
        #r.sleep()
       # keras.backend.clear_session() 
        

if __name__ == '__main__':  
    listener()

    cv2.destroyAllWindows()
#    while True:
 

    
