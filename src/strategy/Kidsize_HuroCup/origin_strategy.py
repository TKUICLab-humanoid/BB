#!/usr/bin/env python
#coding=utf-8
from os import SEEK_CUR
from re import T
import rospy
import numpy as np
from Python_API import Sendmessage
import time

aa=False
obj_xmax=0
obj_xmin=0
imgdata=[[None for high in range(240)]for width in range(320)]

if __name__ == '__main__':
    send = Sendmessage()
    try:
        
        while not rospy.is_shutdown():
            if send.Web==True and aa==False:
                #  send.sendBodyAuto(0,0,0,0,1,0)
                #  send.sendContinuousValue(500,0,4,0)
                # for high in range(240):
                #     for width in range(320):
                #         imgdata[width][high]=send.Label_Model[high*320+width]
                # print(imgdata)        
                send.sendHeadMotor(1,1450,20)
                #  send.sendHeadMotor(1,1750,100)
                for yellow_cnt in range(send.color_mask_subject_cnts[0]):
                    obj_xmax=send.color_mask_subject_XMax[0][yellow_cnt]
                    obj_xmin=send.color_mask_subject_XMin[0][yellow_cnt]
                print(obj_xmax,obj_xmin)
                send.sendBodySector(1)
                send.drawImageFunction(1,0,160,160,0,240,255,255,255)
                send.drawImageFunction(2,0,0,320,120,120,255,255,255)
                 

                aa=True


    except rospy.ROSInterruptException:
        pass
