#!/usr/bin/env python
# coding: utf-8

import cv2
import numpy as np
import os
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'
os.environ['CUDA_VISIBLE_DEVICES'] = "0,1"
from tensorflow.compat.v1 import ConfigProto
from tensorflow.compat.v1 import InteractiveSession

from keras.backend.tensorflow_backend import set_session
import tensorflow as tf
config = tf.ConfigProto()
config.gpu_options.allow_growth = True  # dynamically grow the memory used on the GPU
config.log_device_placement = True  # to log device placement (on which device the operation ran)
sess = tf.Session(config=config)
set_session(sess)  # set this TensorFlow session as the default session for Keras

import sys
import random
import math
import skimage.io
import time
import mrcnn.config
from mrcnn import utils
#import model as modellib
import matplotlib.pyplot as plt
ROOT_DIR = os.path.abspath("./")
print("niubibibibi:::::",ROOT_DIR)
sys.path.append(ROOT_DIR)
from mrcnn import utils
import mrcnn.model as modellib
import coco
from mrcnn.config import Config
from mrcnn import visualize
from PIL import Image
def random_colors(N):
    np.random.seed(1)
    colors=[tuple(255*np.random.rand(3)) for _ in range(N)]
    return colors

def apply_mask(image, mask, color,label):
    """Apply the given mask to the image.
    """
    #if(mask[:][0]==True):
    #print("img::",mask[:][240])
    alpha=0.8
    for n, c in enumerate(color):
        image[:, :, n] = np.where(
            mask == 1,
            image[:, :, n] *(1 - alpha) + alpha * c,
            image[:, :, n]
        )
        cimgmask =np.uint8(mask)
        temp_mask=cimgmask*255

    area = []
    contours, hierarchy = cv2.findContours(temp_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for c in range(len(contours)):
        rect = cv2.minAreaRect(contours[c])
        cx, cy = rect[0]
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        #print("四个顶点坐标为;", box)
        #print("中心坐标：", rect[0])
        cv2.circle(image, (np.int32(cx), np.int32(cy)), 6, (0,0,255), -1)
        #print("宽度：", rect[1][0])
        #print("长度：", rect[1][1])

        length1=math.pow((box[1][1]-box[0][1]),2)+math.pow((box[1][0]-box[0][0]),2)
        length2=math.pow((box[2][1]-box[1][1]),2)+math.pow((box[2][0]-box[1][0]),2)
        grasp_pixcel = 60
        if(rect[1][1]>0):
            grasp_bit = grasp_pixcel/ rect[1][1]/2
        else:
            grasp_bit = 0.2
        if length1>length2:
            pointx_left,pointy_left=(box[2][0]-box[1][0])/2+box[1][0],(box[2][1]-box[1][1])/2+box[1][1]
            grasp_pointx_left,grasp_pointy_left = grasp_bit*(pointx_left-cx)+pointx_left,grasp_bit*(pointy_left-cy)+pointy_left
            grasp_pointx_right,grasp_pointy_right = 2*cx-grasp_pointx_left,2*cy-grasp_pointy_left
           
        else:
            pointx_left,pointy_left=(box[1][0]-box[0][0])/2+box[0][0],(box[1][1]-box[0][1])/2+box[0][1]
            grasp_pointx_left,grasp_pointy_left = grasp_bit*(pointx_left-cx)+pointx_left,grasp_bit*(pointy_left-cy)+pointy_left
            grasp_pointx_right,grasp_pointy_right = 2*cx-grasp_pointx_left,2*cy-grasp_pointy_left

        cv2.circle(image, (np.int32(grasp_pointx_left), np.int32(grasp_pointy_left)), 6, (0,0,255), -1)
        cv2.circle(image, (np.int32(grasp_pointx_right), np.int32(grasp_pointy_right)), 6, (0,0,255), -1)
        im = cv2.line(image, (np.int32(grasp_pointx_left),np.int32(grasp_pointy_left)), (np.int32(grasp_pointx_right), np.int32(grasp_pointy_right)), (0, 255, 255), 2)
        
        print("size:",contours[c].size)
        number = int(len(contours[c])/4)
        print("size:",number)
        cv2.circle(image, (contours[c][0][0][0], contours[c][0][0][1]), 6, (0,0,255), -1)
        cv2.circle(image, (contours[c][number*1][0][0], contours[c][number*1][0][1]), 6, (0,0,255), -1)
        cv2.circle(image, (contours[c][number*2][0][0], contours[c][number*2][0][1]), 6, (0,0,255), -1)
        cv2.circle(image, (contours[c][number*3][0][0], contours[c][number*3][0][1]), 6, (0,0,255), -1)
        #cv2.circle(image, (contours[c][number*4][0][0], contours[c][number*4][0][1]), 6, (0,0,255), -1)
        theta_left,theta_right=0,0
        if(cx-pointx_left!=0):
            theta_right = math.degrees(math.atan((pointy_left-cy)/(cx-pointx_left)))
            #print("旋转 right 角度：", theta_right)
            theta_left = theta_right + 180
        #print("旋转 left 角度：", theta_left)
        cv2.drawContours(image,[box],0,(0,255,0),2)
        cv2.drawContours(image, contours, c, (0, 0, 255), 2, 8)
        if(len(contours[c]) >= 5):
            ellipse = cv2.fitEllipse(contours[c])
            print("zuobiao:",contours[c][1][0], contours[c][1][0][0],contours[c][1][0][1])
            cv2.ellipse(image, ellipse, (255, 255, 0), 2)

    if(label=="cam"):
        print("cam is:",[int(cx), int(cy)])
        print("grasp_point_left:",(int(grasp_pointx_left),int(grasp_pointy_left)))
        print("grasp_point_right:",(int(grasp_pointx_right),int(grasp_pointy_right)))
        print("theta:[left,right]:",(int(theta_left),int(theta_right)))
    print("-------------------")
    if(label=="pingpang"):
        print("pingpang is:",[int(cx), int(cy)])
        print("grasp_point_left:",(int(grasp_pointx_left),int(grasp_pointy_left)))
        print("grasp_point_right:",(int(grasp_pointx_right),int(grasp_pointy_right)))
        print("theta:[left,right]:",(int(theta_left),(theta_right)))

    return image

def display_instances(image,boxes,masks,ids,names,scores):
    n_instances=boxes.shape[0]
    print('---'*30)
    if not n_instances:
        print('No instances to display')
    else:
        assert boxes.shape[0] == masks.shape[-1] == ids.shape[0]  
    colors=random_colors(n_instances)
    height, width = image.shape[:2]
    
    for i,color in enumerate(colors):
        if not np.any(boxes[i]):
            continue
        y1,x1,y2,x2=boxes[i]
        mask=masks[:,:,i]
        label=names[ids[i]]
        image=apply_mask(image,mask,color,label)
        image=cv2.rectangle(image,(x1,y1),(x2,y2),color,2)
        
        score=scores[i] if scores is not None else None
        caption='{}{:.2f}'.format(label,score) if score else label
        image=cv2.putText(
            image,caption,(x1,y1),cv2.FONT_HERSHEY_COMPLEX,0.7,(0,0,255),2
        )
    return image

if __name__=='__main__':

    sys.path.append(os.path.join(ROOT_DIR, "samples/coco/"))  # To find local version
    MODEL_DIR = os.path.join(ROOT_DIR, "logs")
    COCO_MODEL_PATH = os.path.join(ROOT_DIR, "mask_rcnn_coco_0012.h5")
    if not os.path.exists(COCO_MODEL_PATH):
        print('cannot find coco_model')

    class CocoConfig(Config):
        # 命名配置
        NAME = "coco"

        # 输入图像resing
        IMAGE_MIN_DIM = 480
        IMAGE_MAX_DIM = 640

        # 使用的GPU数量。 对于CPU训练，请使用1
        GPU_COUNT = 1

        IMAGES_PER_GPU = 1

        batch_size = GPU_COUNT * IMAGES_PER_GPU
        # STEPS_PER_EPOCH = int(train_images / batch_size * (3 / 4))
        STEPS_PER_EPOCH=1000

        VALIDATION_STEPS = STEPS_PER_EPOCH // (1000 // 50)

        NUM_CLASSES = 1 + 2  # 必须包含一个背景（背景作为一个类别） Pascal VOC 2007有20个类，前面加1 表示加上背景

        scale = 1024 // IMAGE_MAX_DIM
        RPN_ANCHOR_SCALES = (32 // scale, 64 // scale, 128 // scale, 256 // scale, 512 // scale)  

        RPN_NMS_THRESHOLD = 0.6  # 0.6

        RPN_TRAIN_ANCHORS_PER_IMAGE = 256 // scale

        MINI_MASK_SHAPE = (56 // scale, 56 // scale)

        TRAIN_ROIS_PER_IMAGE = 200 // scale

        DETECTION_MAX_INSTANCES = 100 * scale * 2 // 3

        DETECTION_MIN_CONFIDENCE = 0.6


    #class InferenceConfig(coco.CocoConfig):
    class InferenceConfig(CocoConfig):  ###zhu
        # Set batch size to 1 since we'll be running inference on
        # one image at a time. Batch size = GPU_COUNT * IMAGES_PER_GPU
        GPU_COUNT = 1
        IMAGES_PER_GPU = 1

    config = InferenceConfig()
    config.display()

    #model = modellib.MaskRCNN(mode="inference", model_dir=MODEL_DIR, config=config)
    model = modellib.MaskRCNN(mode="inference", model_dir=MODEL_DIR, config=config)
    # Load weights trained on MS-COCO
    model.load_weights(COCO_MODEL_PATH, by_name=True)
    '''
    class_names = ['BG', 'person', 'bicycle', 'car', 'motorcycle', 'airplane',
               'bus', 'train', 'truck', 'boat', 'traffic light',
               'fire hydrant', 'stop sign', 'parking meter', 'bench', 'bird',
               'cat', 'dog', 'horse', 'sheep', 'cow', 'elephant', 'bear',
               'zebra', 'giraffe', 'backpack', 'umbrella', 'handbag', 'tie',
               'suitcase', 'frisbee', 'skis', 'snowboard', 'sports ball',
               'kite', 'baseball bat', 'baseball glove', 'skateboard',
               'surfboard', 'tennis racket', 'bottle', 'wine glass', 'cup',
               'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple',
               'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza',
               'donut', 'cake', 'chair', 'couch', 'potted plant', 'bed',
               'dining table', 'toilet', 'tv', 'laptop', 'mouse', 'remote',
               'keyboard', 'cell phone', 'microwave', 'oven', 'toaster',
               'sink', 'refrigerator', 'book', 'clock', 'vase', 'scissors',
               'teddy bear', 'hair drier', 'toothbrush']'''
    class_names = ['BG', 'pingpang','cam'] 
    capture=cv2.VideoCapture(0)
    capture.set(cv2.CAP_PROP_FRAME_WIDTH,640)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT,480)

    while True:
        ret,frame=capture.read()
        results=model.detect([frame],verbose=1)
        r=results[0]

        frame=display_instances(
              frame,r['rois'], r['masks'], r['class_ids'], 
                            class_names, r['scores']
        )

        cv2.imshow('frame',frame)
        if cv2.waitKey(1)&0xFF==ord('q'):
            break

    capture.release()
    cv2.destroyAllWindows()
