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

ISOTIMEFORMAT_F = '%s'
theTime = datetime.datetime.now().strftime(ISOTIMEFORMAT_F)
print("theTime",theTime)
savename=theTime+'.avi'
print("savename",savename)
cv_image_depth=np.zeros([640,480])
CX_GLOBAL=320
CY_GLOBAL=240
DETECTION_MIN_SIZE = 500#1500#500#200
DETECTION_MAX_SIZE = 4000#6000#3000
DETECTION_HUE_LOW = 0
DETECTION_HUE_HIGH = 10

PlateROI = (115, 117, 159, 151)
#DetectROI = (0, 0, 640, 240)
DetectROI = (0, 0, 640, 480)
RationalROI = (303, 34, 173, 188)

WRITE_TO_FILE = False
SHOW_DEBUG_IMAGE = True
SHOW_DEBUG_INFO = True

if(WRITE_TO_FILE == True):
    out = cv2.VideoWriter(savename,cv2.VideoWriter_fourcc('M','J','P','G'),20,(640,480))
    
resultRect = np.zeros(12,dtype=float)
resultCenterPose = np.zeros(6,dtype=float)
resultGraspPose = np.zeros(6,dtype=float)

frameC = 0

class point_transformation:

    def __init__(self):
        self.depth_img = np.zeros((480,640),dtype = np.uint16)
    
    def quat_to_Rot(self,quat):
        qx=quat[0]
        qy=quat[1]
        qz=quat[2]
        qw=quat[3]
        rot=np.zeros([3,3])
        rot[0,0]=1-2*qy*qy-2*qz*qz
        rot[0,1]=2*qx*qy - 2*qz*qw
        rot[0,2]=2*qx*qz + 2*qy*qw
        rot[1,0]=2*qx*qy + 2*qz*qw
        rot[1,1]=1 - 2*qx*qx - 2*qz*qz
        rot[1,2]=2*qy*qz - 2*qx*qw
        rot[2,0]=2*qx*qz - 2*qy*qw
        rot[2,1]=2*qy*qz + 2*qx*qw
        rot[2,2]=1 - 2*qx*qx - 2*qy*qy
        
        return rot

    def load_img(self,img):
        self.depth_img = img
       # print(self.depth_img)

    def Pix2baselink(self,x,y):
       # print("x,y",x,y)
        x=int(x)
        y=int(y)
        if x<0:
            x=0
        if x>639:
            x=639
        if y<0:
            y=0
        if y>479:
            y=479
        
        #print(self.depth_img)
        pixposition=np.ones((3,1))
        pixposition[0]=x
        pixposition[1]=y

        Camera_Internal_Mat =np.array([[609.5519409179688, 0.0, 322.4856262207031],[0.0, 608.9664916992188, 243.48764038085938]
                                ,[0.000000, 0.000000, 1.000000]])



        Quat=np.array([-0.0123091237996,0.999700449573,-0.0197228154856, 0.00764899626697])
        #RotMat=quaternion_to_rotation_matrix(Quat)       

        Camera_External_RotMat  = self.quat_to_Rot(Quat)
        Translation=np.array([[0.239349133545],[-0.246647413719],[1.19341411133]])
        baselink2base=np.array([[-1,0,0],[0,-1,0],[0,0,1]])
       # print("!!!!!!",x)
        #s = self.depth_img[x,y]
        global cv_image_depth
        s = cv_image_depth[y,x]*0.001
      #  print("!!",s)
        
        zConst=0#only compute point on base
        Camera_Mat_inv=np.linalg.inv(Camera_Internal_Mat)
        rightMatrix = Camera_External_RotMat.dot(Camera_Mat_inv.dot(pixposition))
        leftMatrix = Camera_External_RotMat.dot(Translation)
        #s = (zConst + leftMatrix[2])/rightMatrix[2]
        
        cameraPoint=(s*Camera_Mat_inv).dot(pixposition)
        worldPoint_3d = Camera_External_RotMat.dot(cameraPoint)+Translation
        worldPoint_3d=baselink2base.dot(worldPoint_3d)
       # print(",,,,,,,,,,,",worldPoint_3d)
        return worldPoint_3d[0],worldPoint_3d[1],worldPoint_3d[2]
        #print(cameraPoint)
        
        #worldPoint_2d=worldPoint_3d[0:2]

       # return Point2D(worldPoint_2d[0],worldPoint_2d[1])

def rpy2rotvec(Rx,Ry,Rz):
    #Rx--roll Ry---pitch，Rz--yawrate
	Txyz=np.array([[math.cos(Rz)*math.cos(Ry),math.cos(Rz)*math.sin(Ry)*math.sin(Rx)-math.sin(Rz)*math.cos(Rx),math.cos(Rz)*math.sin(Ry)*math.cos(Rx)+math.sin(Rz)*math.sin(Rx)],[math.sin(Rz)*math.cos(Ry),math.sin(Rz)*math.sin(Ry)*math.sin(Rx)+math.cos(Rz)*math.cos(Rx),math.sin(Rz)*math.sin(Ry)*math.cos(Rx)-math.cos(Rz)*math.sin(Rx)],[-math.sin(Ry),math.cos(Ry)*math.sin(Rx),math.cos(Ry)*math.cos(Rx)]])
	p=math.acos((Txyz[0,0]+Txyz[1,1]+Txyz[2,2]-1)/2)
	kx=(1/(2*math.sin(p)))*(Txyz[2,1]-Txyz[1,2])*p
	ky=(1/(2*math.sin(p)))*(Txyz[0,2]-Txyz[2,0])*p
	kz=(1/(2*math.sin(p)))*(Txyz[1,0]-Txyz[0,1])*p

	return kx,ky,kz

class camshift_tracking:
    def __init__(self):
        self.TrackInitStatus = False
        self.TrackNormalStatus = False
        self.show_backproj = False
        self.matrix_caculate = point_transformation()

    def initTrack(self, cv_image, detect_roi):
        
        self.TrackInitStatus = True
        self.TrackNormalStatus = True
        x0, y0, w, h = detect_roi
        self.track_window = (x0,y0,w,h)
      
        vis = cv_image.copy()
        hsv = cv2.cvtColor(vis, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array((0., 60., 32.)), np.array((10., 255., 255.)))
        mask = cv2.inRange(hsv, np.array((140., 60., 32.)), np.array((180., 255., 255.)))
 
        mask_roi = mask[y0:y0+h, x0:x0+w]
        hsv_roi = hsv[y0:y0+h, x0:x0+w]
        hist = cv2.calcHist( [hsv_roi], [0], mask_roi, [16], [0, 180] )
        cv2.normalize(hist, hist, 0, 255, cv2.NORM_MINMAX)
        self.hist = hist.reshape(-1)
        #self.show_hist()     
        #vis_roi = vis[y0:y0+h, x0:x0+h]
        #cv2.bitwise_not(vis_roi, vis_roi) 
        #vis[mask == 0] = 0

        self.kalman = cv2.KalmanFilter(4,2)
        self.kalman.statePre = np.zeros((4,1),np.float32)
        self.kalman.statePre[0] = x0+w/2
        self.kalman.statePre[1] = y0+h/2
        self.kalman.measurementMatrix = np.array([[1,0,0,0],[0,1,0,0]],np.float32)
        self.kalman.transitionMatrix = np.array([[1,0,1,0],[0,1,0,1],[0,0,1,0],[0,0,0,1]],np.float32)
        self.kalman.processNoiseCov = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]],np.float32) * 0.03
        self.measurement = np.array((2,1), np.float32) 
        self.prediction = np.zeros((2,1), np.float32)
        self.term_crit = ( cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 1 )
        self.center = (x0+w/2,y0+h/2)
        
    def show_hist(self):
        bin_count = self.hist.shape[0]
        bin_w = 24
        img = np.zeros((256, bin_count*bin_w, 3), np.uint8)
        for i in xrange(bin_count):
            h = int(self.hist[i])
            cv2.rectangle(img, (i*bin_w+2, 255), ((i+1)*bin_w-2, 255-h), (int(180.0*i/bin_count), 255, 255), -1)
        img = cv2.cvtColor(img, cv2.COLOR_HSV2BGR)
        #cv2.imshow('hist', img)

    def calcenter(self,points):
        x = (points[0][0] + points[1][0] + points[2][0] + points[3][0]) / 4.0
        y = (points[0][1] + points[1][1] + points[2][1] + points[3][1]) / 4.0
        return np.array([np.float32(x), np.float32(y)], np.float32)
    
    def getGraspPoints(self, vis, pts):

        xSorted = pts[np.argsort(pts[:, 0]), :]
        print("-----",xSorted[3,:])

        rightMost = xSorted[2:,:]
        rightcenterPoint = np.array([(rightMost[0][0]+rightMost[1][0])/2.0, (rightMost[0][1]+rightMost[1][1])/2.0])
        rightcenterWorld = self.matrix_caculate.Pix2baselink(rightcenterPoint[0],rightcenterPoint[1])
        #print("rightcenterPoint",rightcenterPoint)
        #print("rightcenterWorld",rightcenterWorld)
        centerPoint = self.calcenter(pts)
        delta_long_x = xSorted[3,0] - centerPoint[0]+7
        delta_long_y =np.tan(np.pi*30/180)*delta_long_x

        #print("delta_long_x",delta_long_x)  
        #print("centerPoint",centerPoint)
        centerWorld = self.matrix_caculate.Pix2baselink(centerPoint[0],centerPoint[1])
        #print("centerWorld", centerWorld)

        #thetaOrig = np.arctan2(rightcenterWorld[1]-centerWorld[1],rightcenterWorld[0]-centerWorld[0])
        thetaOrig = np.arctan2(rightcenterPoint[1]-centerPoint[1],rightcenterPoint[0]-centerPoint[0])
        theta = -thetaOrig
        #print("theta",theta*180/np.pi)
       # if thetaOrig < 0:
       #     theta = thetaOrig + np.pi*2
      #  print("theta",theta*180/np.pi)
       # grasppt_x = rightcenterPoint[0]+1*np.cos(thetaOrig)
       # grasppt_y = rightcenterPoint[1]+1*np.sin(thetaOrig)
        grasppt_x = centerPoint[0]+1*delta_long_x
        grasppt_y = centerPoint[1]+1*delta_long_y
        graspWorld = self.matrix_caculate.Pix2baselink(grasppt_x,grasppt_y)
        cv2.circle(vis, (int(grasppt_x),int(grasppt_y)), 4, (0,0,255), 8)

        resultCenterPose[0] = centerWorld[0] #centerX
        resultCenterPose[1] = centerWorld[1] #centerY
        #resultCenterPose[2] = centerWorld[2] +0.02#centerZ
        resultCenterPose[2] = -0.005
        #resultCenterPose[3] = -30.0*np.pi/180.0 #rx
        #resultCenterPose[4] = 0.0*np.pi/180.0 #ry
        #resultCenterPose[5] = theta
        if centerPoint[0] < 320:
            Ry = -60*np.pi/180
        else:
            Ry = -45*np.pi/180
        theta=-np.pi*30/180
        Rz = np.pi+theta
        Rx = 180*np.pi/180
        #Rx = 90*np.pi/180
        resultCenterPose[3],resultCenterPose[4],resultCenterPose[5] = rpy2rotvec(Rx,Ry,Rz)
        #print("~~~~~~RXYZ~~~~~~~~~~~~:",Rx*180/np.pi,Ry*180/np.pi,Rz*180/np.pi)
        print("~~~~~~CenterPose~~~~~~~~~~~~:",resultCenterPose)

        resultGraspPose[0] = graspWorld[0] #graspX
        resultGraspPose[1] = graspWorld[1] #graspY
        #resultGraspPose[2] = graspWorld[2]+0.02 #graspZ
        resultGraspPose[2] = -0.005
        #resultGraspPose[3] = -45.0*np.pi/180.0 #rx
        #resultGraspPose[4] = 0.0*np.pi/180.0 #ry
        #resultGraspPose[5] = theta
        print("theta",theta)
        theta=-np.pi*30/180
        if centerPoint[0] < 320:
            Ry = -60*np.pi/180
            print("30")
        else:
            Ry = -45*np.pi/180
        Rz = np.pi+theta
        #Ry = -45*np.pi/180
        Rx = 180*np.pi/180
        #Rx = 90*np.pi/180
        resultGraspPose[3],resultGraspPose[4],resultGraspPose[5] = rpy2rotvec(Rx,Ry,Rz)
        print("~~~~~~GraspPose~~~~~~~~~~~~:",resultGraspPose)

        #if SHOW_DEBUG_INFO is True:
            #print("centerWorld, graspWorld, theta", centerWorld, graspWorld, theta*180.0/np.pi)

        if SHOW_DEBUG_IMAGE is True:
            #cv2.circle(vis, (int(rightcenterPoint[0]),int(rightcenterPoint[1])), 4, (0,255,0), 4)
            endpt_x = int(rightcenterPoint[0]-20*np.cos(theta))
            endpt_y = int(rightcenterPoint[1]+20*np.sin(theta))
            arrow_x = int(endpt_x + 8*np.cos(theta+np.pi*45/180))
            arrow_y = int(endpt_y - 8*np.sin(theta+np.pi*45/180))
            cv2.line(vis,(int(rightcenterPoint[0]),int(rightcenterPoint[1])),(endpt_x,endpt_y),255,2)
            cv2.line(vis,(endpt_x,endpt_y),(arrow_x,arrow_y),255,2)
            arrow_x = int(endpt_x + 8*np.cos(theta-np.pi*45/180))
            arrow_y = int(endpt_y - 8*np.sin(theta-np.pi*45/180))
            cv2.line(vis,(endpt_x,endpt_y),(arrow_x,arrow_y),255,2)
            cv2.circle(vis, (int(grasppt_x),int(grasppt_y)), 4, (0,0,255), 4) 

    def bb_intersection_over_union(self, boxA, boxB):
        
        xA = max(boxA[0], boxB[0])
        yA = max(boxA[1], boxB[1])
        xB = min(boxA[2], boxB[2])
        yB = min(boxA[3], boxB[3])
        interArea = max(0, xB - xA + 1) * max(0, yB - yA + 1)
        boxAArea = (boxA[2] - boxA[0] + 1) * (boxA[3] - boxA[1] + 1)
        boxBArea = (boxB[2] - boxB[0] + 1) * (boxB[3] - boxB[1] + 1)
        iou = interArea / float(boxAArea + boxBArea - interArea)
        return iou
    
    def update(self, cv_image, detect_roi):
        global frameC 
        frameC= frameC + 1 
        if self.track_window is None or self.track_window[2]<=0 or self.track_window[3]<=0:
            self.TrackNormalStatus = False
            self.TrackInitStatus = False
            return
        
        vis = cv_image.copy()
        (rows,cols,channels) = vis.shape
        hsv = cv2.cvtColor(vis, cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(hsv, np.array((0., 60., 32.)), np.array((10., 255., 255.)))
        mask2 = cv2.inRange(hsv, np.array((140., 60., 32.)), np.array((180., 255., 255.)))
        mask = cv2.addWeighted(mask1, 1, mask2, 1, 0)
        prob = cv2.calcBackProject([hsv], [0], self.hist, [0, 180], 1)
        prob &= mask
        
        term_crit = ( cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 1 )
        track_box, self.track_window = cv2.CamShift(prob, self.track_window, term_crit)
        pts = cv2.boxPoints(track_box)
      
        #propose best grasp point
        self.getGraspPoints(vis, pts)
          
        pts = np.int0(pts)
        self.center = self.calcenter(pts)
        
        self.kalman.correct(self.center)
        prediction = self.kalman.predict()
        x,y,w,h = self.track_window

        xlnew = max(0, int(prediction[0]) - w/2)
        ylnew = max(0, int(prediction[1]) - h/2)
        xrnew = min(xlnew+w,cols-1)
        yrnew = min(ylnew+h,rows-1)
        self.track_window = xlnew, ylnew, xrnew-xlnew, yrnew-ylnew
        
        currTrackSize = track_box[1][0]*track_box[1][1]
        
        iou = 1
        if(detect_roi is not None):
            xd, yd, wd, hd = detect_roi
            iou = self.bb_intersection_over_union([x,y,x+w,y+h],[xd,yd,xd+wd,yd+hd])
        print("iou", iou)
        
        if currTrackSize < 100 or self.track_window is None or self.track_window[2]<= 0 or self.track_window[3] <= 0 or iou < 0.4:
            self.TrackNormalStatus = False
            self.TrackInitStatus = False
        else:
            cv2.ellipse(vis, track_box, (0, 0, 255), 2)
            cv2.circle(vis, (int(prediction[0]),int(prediction[1])),4,(255,0,0),-1)
            cv2.polylines(vis, [pts],True, 255,1)
            (posx,posy,posz) = self.matrix_caculate.Pix2baselink(track_box[0][0],track_box[0][1])
            (handlex, handley,handlez) = self.matrix_caculate.Pix2baselink(CX_GLOBAL,CY_GLOBAL)
            
            resultRect[0] = rospy.get_rostime().to_sec()#timestamp
            resultRect[1] = track_box[2]#angle
            resultRect[2] = posx#centerX
            resultRect[3] = posy#centerY
            resultRect[4] = posz#centerZ
            resultRect[5] = track_box[1][0] * 0.002#width
            resultRect[6] = track_box[1][1] * 0.002#height
            resultRect[11] = 1.0 #bBeefPresent
            # add center image_u,image_v
            #resultRect[12] = track_box[0][0]#image_u
            #resultRect[13] = track_box[0][1]#image_v
            #print("-----------",resultRect)
            temp_x, temp_y = int(track_box[0][1]),int(track_box[0][0])
            
            sum_data = 0
            k = 0
            for i in range(temp_x-1,temp_x+2):
                for j in range(temp_y-1,temp_y+2):
                    #print("point:",cv_image_depth[i,j])
                    if cv_image_depth[i,j] != 0:
                        k = k+1
                        sum_data = sum_data + cv_image_depth[i,j]
            
            result = 1000
            result = sum_data / k  ##917
            #print("result:",k,sum_data, result)
            drop = False
            if result > 1120:
                drop = True
            else:
                drop = False
            print("drop:",drop)
            #print("dorp is:",drop)
            resultRect[7] = drop #drop detection
            resultRect[8] = handlex#handleX
            resultRect[9] = handley#handleY
            resultRect[10] = handlez#handleZ

            #print('!!!!!!!!!!!!',resultRect)
            #print(" data is : ",int(result[1]))
            #result[1],result[2],result[3] = self.matrix_caculate.Pix2baselink(int(result[1]),int(result[2]))
            #print('asda',result)
            #print("---------1111------------",resultRect[7],resultRect[8],resultRect[9])
            #print('dadadad',result[6],result[7],result[8])
            #print('xxxxxxx',result[1],result[2],result[3])

            self.TrackNormalStatus = True
        
        if(WRITE_TO_FILE == True):
            #pass
            out.write(vis)
        if SHOW_DEBUG_IMAGE is True:
            vis = cv2.resize(vis,(int(1.4*640),int(1.4*480)),interpolation=cv2.INTER_CUBIC)
            if(frameC % 3 == 0):
                print("framc++++++++++",frameC)
                cv2.imshow("processed", vis)
                cv2.moveWindow("processed", 1000,800)  

class steak_detection:
    def __init__(self):
        self.detectSuccess = False
        self.detectROI = None
        self.bBeefPresent = False
    
    def isRational(self,vis):
        (roi_lx, roi_ly, roi_w, roi_h) = RationalROI
        tempframe = cv2.imread("rgb.jpg", 0)
        temproi = tempframe[roi_ly:(roi_ly+roi_h),roi_lx:(roi_lx+roi_w)]

        currframe = cv2.cvtColor(vis, cv2.COLOR_BGR2GRAY)
        currroi = currframe[roi_ly:(roi_ly+roi_h),roi_lx:(roi_lx+roi_w)]

        orb = cv2.ORB_create()
        kp1, des1 = orb.detectAndCompute(temproi, None)
        kp2, des2 = orb.detectAndCompute(currroi, None)
    
        bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck = True)
        matches = bf.match(des1,des2)
        matches = sorted(matches, key = lambda x:x.distance)

        if len(matches) < 8:
            return False
        
        matchimg = cv2.drawMatches(temproi, kp1, currroi, kp2, matches[:8], None, flags=2)
        src_pts = np.float32([kp1[m.queryIdx].pt for m in matches[:8]]).reshape(-1, 1, 2)
        dst_pts = np.float32([kp2[m.trainIdx].pt for m in matches[:8]]).reshape(-1, 1, 2)

        #cv2.imshow("match", matchimg)
        #diffframe = cv2.absdiff(currframe, tempframe)
        #cv2.imshow('diff', diffframe)
        #cv2.imshow("temp", tempframe)
        #cv2.imshow("curr", currframe)
        #cv2.waitKey(100) 
        
        if(len(src_pts) is not 0):
            error = 0
            for i in range(len(src_pts)):
                dist = math.sqrt((src_pts[i][0][0]-dst_pts[i][0][0])**2 + (src_pts[i][0][1]-dst_pts[i][0][1])**2) 
                error = error + dist
            error = error / len(src_pts)
            print("camera motion", error)
            if(error < 5):
                return True
            else:
                return False
        else:
            return False

    def gmmdetect(self, vis):
        print("==gmm==")
        (roi_lx, roi_ly, roi_w, roi_h) = DetectROI
        vis_roi = vis[roi_ly:(roi_ly+roi_h),roi_lx:(roi_lx+roi_w)]
        vis_display = vis.copy()
        
        Id = np.array(vis_roi)
        (h,w,d) = Id.shape
        
        b,g,r= cv2.split(Id)
        #r = r-82.4002
        #g = g-61.1831
        #b = b-81.5648

        #sig = [[405.308270122134,	330.889432140537,	350.578254502307],
        #        [330.889432140537,	316.988263196021,326.244912405013],
        #        [350.578254502307,	326.244912405013,	356.399178769789]]
        #r = r-103.6947
        #g = g-76.0862
        #b = b-100.8674

        #sig = [[799.0025,	595.9642,	733.3426],
        #        [595.9642,	543.0486,   610.7482],
        #        [733.3426,	610.7482,	741.5276]]
        
        r = r-104.5127
        g = g-70.0618
        b = b-79.0727

        sig = [[911.0353, 605.8751,	644.5027],
                [605.8751,	493.4596,   496.0905],
                [644.5027,	496.0905,	532.3943]]
        r1 = np.transpose(r)
        g1 = np.transpose(g)
        b1 = np.transpose(b)

        Id = cv2.merge([(r1), (g1), (b1)])
        Id = np.reshape(Id,((w*h),3))
        gcaa = exp(dot(- 0.5,np.sum(multiply(dot(Id,inv(sig)),Id),axis=1))) / (dot(2,pi)) ** 1.5 / det(sig) ** 0.5

        T =double( 8e-06)
        
        thresh = np.reshape(gcaa, (w,h))
        thresh=np.transpose(thresh)
    
        thresh[thresh >T] = 255
        thresh[thresh < 255] = 0
    
        thrd = thresh.astype(np.uint8)
        lab = skimage.measure.label(thresh,connectivity=2)
        #print(lab)
        props = skimage.measure.regionprops(lab)
        if(len(props) <= 0):
            return None,(0,0)
        npx = []
        for ia in range(len(props)):
            npx += [props[ia].area]

        maxnum = max(npx)
        index = npx.index(maxnum)

        eroded = cv2.erode(thrd, cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(3,3)),iterations=1)
        dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(5,5)),iterations=1)
        cv2.imshow("detectmask", thrd)
        cv2.imshow("dilated2", dilated)

        im, contours,hier = cv2.findContours(dilated, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        
        if(len(contours)>0):
            areas = [cv2.contourArea(c) for c in contours]
            max_index = np.argmax(areas)
            cnt = contours[max_index]
            maxarea = cv2.contourArea(cnt)
            print("maxarea",maxarea)
            if maxarea > DETECTION_MIN_SIZE and maxarea < DETECTION_MAX_SIZE:
                (x,y,w,h) = [int(v) for v in cv2.boundingRect(cnt)]
                self.detectROI = (x+roi_lx, y+roi_ly, w, h)
                cv2.drawContours(vis_display, [cnt], 0,(0,255,0),2, offset=(roi_lx,roi_ly))
        #cv2.imshow("dilated",dilated)
        cv2.imshow("detection",vis_display)

    def detect(self, vis):
        self.detectSuccess = False
        self.detectROI = None
        if(self.bBeefPresent == False):
            (roi_lx, roi_ly, roi_w, roi_h) = PlateROI
        else:
            (roi_lx, roi_ly, roi_w, roi_h) = DetectROI

        vis_roi = vis[roi_ly:(roi_ly+roi_h),roi_lx:(roi_lx+roi_w)]
        vis_display = vis.copy()
        cv2.rectangle(vis_display, (roi_lx, roi_ly), (roi_lx+roi_w, roi_ly+roi_h), (0, 255, 255), 2)
        hsvroi = cv2.cvtColor(vis_roi, cv2.COLOR_BGR2HSV)
        lower_hsv1 = np.array([DETECTION_HUE_LOW,60,32])
        upper_hsv1 = np.array([DETECTION_HUE_HIGH,255,255])
        colorMask1 = cv2.inRange(hsvroi, lower_hsv1, upper_hsv1)
        lower_hsv2 = np.array([140,60,32])#140
        upper_hsv2 = np.array([180,255,255])
        colorMask2 = cv2.inRange(hsvroi, lower_hsv2, upper_hsv2)
        colorMask = cv2.addWeighted(colorMask1, 1, colorMask2, 1, 0)
        eroded = cv2.erode(colorMask, cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(3,3)),iterations=1)
        dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(8,5)),iterations=2)
        
        im, contours,hier = cv2.findContours(dilated, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        
        if(len(contours)>0):
            areas = [cv2.contourArea(c) for c in contours]
            max_index = np.argmax(areas)
            cnt = contours[max_index]
            maxarea = cv2.contourArea(cnt)
            print("maxarea",maxarea)
            if maxarea > DETECTION_MIN_SIZE and maxarea < DETECTION_MAX_SIZE:
                (x,y,w,h) = [int(v) for v in cv2.boundingRect(cnt)]
                self.detectROI = (x+roi_lx, y+roi_ly, w, h)
                cv2.drawContours(vis_display, [cnt], 0,(255,255,0),2, offset=(roi_lx,roi_ly))
                self.detectSuccess = True
                self.bBeefPresent = True
            else:
                self.detectSuccess = False
        else:
            self.detectSuccess = False
        
        if SHOW_DEBUG_IMAGE is True:
            hsvroi = cv2.resize(hsvroi,(int(1.4*640),int(1.4*480)),interpolation=cv2.INTER_CUBIC)
            colorMask = cv2.resize(colorMask,(int(1.4*640),int(1.4*480)),interpolation=cv2.INTER_CUBIC)
            vis_display = cv2.resize(vis_display,(int(1.4*640),int(1.4*480)),interpolation=cv2.INTER_CUBIC)     
            cv2.imshow("hsv",hsvroi)
            cv2.imshow("msk",colorMask)
            #cv2.imshow("dilated",dilated)
            cv2.imshow("detection",vis_display)
            #cv2.imshow("panimg_color",self.pancolorimg)
            #cv2.imshow("self.pancontourimg",self.pancontourimg)
            cv2.moveWindow("detection",20,800)
            #cv2.moveWindow("dilated",1600,20)
            cv2.moveWindow("msk", 1000,20)
            cv2.moveWindow("hsv",20,20)

class steak_processor:
    def __init__(self):
        rospy.init_node("steak_detector_node")
        self.counter = 0
        self.sanityRational = False
        
        self.bridge = CvBridge()
        self.detector = steak_detection()
        self.tracker = camshift_tracking()
        self.detection_pan = pandetection()
        self.transformer = point_transformation()
        
        self.image_pub = rospy.Publisher("/detector/image_result",Image,queue_size=10)
        self.rect_pub = rospy.Publisher("/detector/rect",Float32MultiArray,queue_size=10)
        self.center_pose_pub = rospy.Publisher("/detector/centerpose",Float32MultiArray,queue_size=10)
        self.grasp_pose_pub = rospy.Publisher("/detector/grasppose",Float32MultiArray,queue_size=10)
         
        self.depth_sub = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw",Image,self.callback_depth)
        self.image_sub = rospy.Subscriber("/camera/color/image_raw",Image,self.callback,queue_size=1)
        #self.image_sub = rospy.Subscriber("/camera/image_raw",Image,self.callback,queue_size=1)
        #self.image_sub = rospy.Subscriber("/videofile/image_raw",Image,self.callback)
        rospy.spin()

    def callback(self,image):

        now = rospy.get_rostime()
        deltatime = (now.to_sec()-image.header.stamp.to_sec())*1000
        if(deltatime > 100):
            print("timestamp doesn't match", deltatime)
            return
        
        beginning = time.time()
        self.counter =  self.counter + 1
        cv_image = self.bridge.imgmsg_to_cv2(image,"bgr8")
        cv_image[179:187,415:425,0]=0
        cv_image[179:187,415:425,1]=0
        cv_image[179:187,415:425,2]=0
        vis = cv_image.copy()
        #out.write(vis)
        cv2.imwrite('rgb_temp1.jpg',vis)
        if(self.counter < 10):
            print("initializing")
            return
          
        #sanity check if first frame is rational
        #if(self.counter == 10):
        #    self.sanityRational = self.detector.isRational(vis)
        #if(self.sanityRational == False):
        #    print("Camera is moved or lighting is changed, need to do calibration AGAIN!")    
            
    
        self.detection_pan.BGR_detection(vis)
    
        #detection
        self.detector.detect(vis)   
        #self.detector.gmmdetect(vis)   
        if(self.detector.bBeefPresent == True):
            print("Found Beef!")
            resultRect[11] = 1.0 #bBeefPresent
        else:
            print("Empty Plate!")
            resultRect[11] = 0.0 #bBeefPresent
        
        #tracking
        if(self.detector.detectSuccess == True and self.tracker.TrackInitStatus == False):
            self.tracker.initTrack(vis,self.detector.detectROI)
        elif(self.tracker.TrackNormalStatus == True):
            self.tracker.update(vis, self.detector.detectROI)
        elif(self.tracker.TrackNormalStatus == False):
            self.tracker.TrackInitStatus = False
        
        #publish results
        tempResultRect = Float32MultiArray(data = resultRect)
        tempResultCenterPose = Float32MultiArray(data = resultCenterPose)
        tempResultGraspPose = Float32MultiArray(data = resultGraspPose)
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(vis,"bgr8"))
        self.rect_pub.publish(tempResultRect)
        self.center_pose_pub.publish(tempResultCenterPose)
        self.grasp_pose_pub.publish(tempResultGraspPose)
        
        end = time.time()
        #print("processing time in ms",(end-beginning)*1000)
        cv2.waitKey(1)

    def callback_depth(self,image):
       
        #self.counter =  self.counter + 1
        #print("deps_excute",3)
        global cv_image_depth
        cv_image_depth = self.bridge.imgmsg_to_cv2(image,"passthrough")
        #("pixel",cv_image_depth[320][240])
        '''
        if (self.detection_pan.counts==100):
            dataNew = 'cv_image_depth_1.mat'
            scio.savemat(dataNew, {'cv_image_depth_1':cv_image_depth})
            
        self.detection_pan.counts=self.detection_pan.counts+1
        '''
        self.transformer.load_img(cv_image_depth)
        self.detection_pan.depth_detection(cv_image_depth)

      #  print(cv_image_depth.shape)
        #print(cv_image_depth)
      #  plt.imshow(cv_image_depth,cmap="gray",vmin=0,vmax=1000)
     #   plt.show()
      #  cv2.waitKey(1)

class pandetection:
    def __init__(self):

        self.color_frame = None
        self.depth_frame = None
        self.CX = 0
        self.CY = 0

        self.count1 = 0
        self.count2 = 0
        self.counts = 0

    def depth_detection(self,cv_image_depth):

        self.depth_frame = cv_image_depth
        self.count1 = self.count1+1
        #print(self.depth_frame)

    def BGR_detection(self,vis):

        self.color_frame = vis
        self.count2 = self.count2+1
        #print('rgb',self.count2)
      #  print('dep',self.count1)
        if(self.count1!=0 and self.count2!=0):

            
            #print(self.depth_frame)
            img_color = self.color_frame
            img_depth = self.depth_frame

            #print(img_color.shape)
            #print(img_depth.shape)

            img_out = np.zeros((480,640),dtype=np.uint8)
            #img_out[:,] = np.zeros([480,640]) + 255

            beginning1 = time.time()
            gray = cv2.cvtColor(img_color, cv2.COLOR_BGR2GRAY)
            img_depth_rect=img_depth[170:300,290:500]
            index=(img_depth_rect>1080)&(img_depth_rect<1112)
            img_depth_rect=index*1

            img_out[170:300,290:500]=img_depth_rect.copy()
            img_out=img_out*gray
            #cv2.imshow("111",img_out)
            thresh2=(img_out<100)*img_out
            
            #cv2.imshow("gray",gray)
            ret,thresh2 = cv2.threshold(img_out,50,255,cv2.THRESH_BINARY)
            
            kernel = np.ones((3,3),np.uint8)
            thresh2 = cv2.dilate(thresh2,kernel,iterations = 1)
            #cv2.imshow("thresh2",thresh2)
            area = []
            max_idx = None
            binary,contours, hierarchy = cv2.findContours(thresh2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for c in range(len(contours)):

                area.append(cv2.contourArea(contours[c]))
                max_idx = np.argmax(area)

            if(max_idx!=None):
                rect = cv2.minAreaRect(contours[max_idx])
                cx, cy = rect[0]
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                cv2.drawContours(img_color,[box],0,(0,255,0),2)
                cv2.circle(img_color, (np.int32(cx), np.int32(cy)), 2, (255, 0, 0), 2, 8, 0)
                #cv2.drawContours(img_color, contours, max_idx, (0, 0, 255), 2, 8)
            
                #cv2.imshow("gray",gray)
                #cv2.imshow("img_color",img_color)
                end1 = time.time()
                
                self.CX = int(cx)
                self.CY = int(cy)
                global CX_GLOBAL
                global CY_GLOBAL
                CX_GLOBAL=self.CX
                CY_GLOBAL=self.CY
                #print("[c_x, c_y]:",self.CX,self.CY)
                #print("AAA:",(end1-beginning1)*1000)
                #cv2.waitKey(1)
               # return self.CX, self.CY


if __name__ == '__main__':
    
    detector = steak_processor()
    #out.release()
    cv2.destroyAllWindows()
