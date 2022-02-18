#!/usr/bin/env python
#coding=utf-8
import rospy
import numpy as np
from Python_API import Sendmessage
from function import image,trace
import time

bb = image()
cc = trace()
status = 1

if __name__ == '__main__':
    try:
        send = Sendmessage()
        while not rospy.is_shutdown():
            # print("aaaa")
            if send.is_start == True: #send.is_start

                if status == 1:
                    send.sendSensorReset()
                    cc.MoveWaist(2048,50)
                    cc.MoveHead(1,2750,200)
                    cc.MoveHead(2,2000,200)
                    time.sleep(1)
                    print("VerticalHeadPosition = ",cc.VerticalHeadPosition)
                    print("HorizontalHeadPosition = ",cc.HorizontalHeadPosition)
                    status = 2
                elif status == 2:
                    print("c")
                    bb.ball_info()
                    if bb.Ball_size > 400 :
                        cc.isStartContinuous("StartContinuous")
                        status = 3
                    else:
                        print("ballsize = ", bb.Ball_size)
                        cc.FindTargetHead(2000,1250,2750,1350)
                elif status == 3:
                    print("d")
                    bb.ball_info()
                    cc.TraceTargetHead(bb.Ball_X,bb.Ball_Y,160,120)
                    print("VerticalHeadPosition = ",cc.VerticalHeadPosition)
                    print("HorizontalHeadPosition = ",cc.HorizontalHeadPosition)
                    if abs(cc.HorizontalHeadPosition - 2048) < 80:
                        status = 4
                    else:
                        cc.TurnToTarget(bb.Ball_X,bb.Ball_Y,0,0,0,0)
                elif status == 4:
                    print("e")
                    bb.ball_info()
                    print("VerticalHeadPosition = ",cc.VerticalHeadPosition)
                    if cc.VerticalHeadPosition < 1650:
                        status = 5
                        print("!!!!!!!gooooooooood!!!!!!!")
                    else:
                        cc.TraceTargetWalk(bb.Ball_X,bb.Ball_Y,1650,1550,0,0,0,0)
                    
                elif status == 5:
                    cc.MoveHead(1,2048,200)
                    time.sleep(1)
                    cc.isStartContinuous("StopContinuous")
                    print("!!!!!!!STOOOOOOOOP!!!!!!!")
                    status = 6 

                elif status == 6:

                    bb.ball_info()
                    print("Ball_X = ",bb.Ball_X)
                    print(cc.WaistPosition)
                    if abs(bb.Ball_X - 160) < 3 :
                        print("finish")
                        status=7
                    elif bb.Ball_X != 0 :
                        cc.WaistFix(bb.Ball_X, bb.Ball_Y, 160, 120)
                
                elif status == 7:
                    pass

                else:
                    print('errrrrrrrrror')

            if send.is_start == False: #send.is_start
               
                print("123123")
                if status != 1:
                    print("reset")
                    send.sendBodySector(29)
                    bb = image()
                    cc = trace()
                    cc.isStartContinuous("StopContinuous")
                    status = 1


                #send.sendBodySector(29)
                # if cc.WaistPosition != 2048:
                #     cc.MoveWaist(2048,50)
                # if cc.HorizontalHeadPosition != 2048:
                #     cc.MoveHead(1,2048,200)
                # if cc.HorizontalHeadPosition != 2048:
                #     cc.MoveHead(2,2048,200)
                # status = 1
                # cc.isStartContinuous("StopContinuous")

    except rospy.ROSInterruptException:
        pass