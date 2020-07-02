import os
import numpy as np
import cv2
import maskrcnn

count=2
savename='test.avi'
out = cv2.VideoWriter(savename,cv2.VideoWriter_fourcc('M','J','P','G'),3,(960,540))

if __name__=='__main__':
    capture=cv2.VideoCapture('/home/zhulifu/Desktop/vd41.avi')
    capture.set(cv2.CAP_PROP_FRAME_WIDTH,960)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT,540)
    while(1):
        ret,img=capture.read()
        maskrcnn.model_detection(img)
        cv2.imshow('frame',img)
        out.write(img)
        if cv2.waitKey(1)&0xFF==ord('q'):
            break

    cv2.destroyAllWindows()
