import os
import numpy as np
import cv2
import maskrcnn


if __name__=='__main__':

    img=cv2.imread("/home/zhulifu/Desktop/new/coco-Mask_RCNN/samples/14.jpg")
    maskrcnn.model_detection(img)
    cv2.imshow('frame',img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
