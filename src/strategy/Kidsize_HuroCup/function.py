#!/usr/bin/env python
#coding=utf-8
import rospy
import numpy as np
from Python_API import Sendmessage
import time

send = Sendmessage()

class image():
    def __init__(self):
        self.Ball_size = 0
        self.Ball_XMin = 0
        self.Ball_XMax = 0
        self.Ball_YMin = 0
        self.Ball_YMax = 0
        self.Ball_X    = 0
        self.Ball_Y    = 0
        self.Basket_size = 0
        self.Basket_XMin = 0
        self.Basket_XMax = 0
        self.Basket_YMin = 0
        self.Basket_YMax = 0
        self.Basket_X    = 0
        self.Basket_Y    = 0
        self.BasketVerticalBaseLine50 = 200
        self.BasketVerticalBaseLine60 = 200
        self.BasketVerticalBaseLine70 = 200
        self.BasketVerticalBaseLine80 = 200
        self.BasketVerticalBaseLine90 = 200
        self.BasketVerticalBaseLine = 200

    def ball_info(self):
        for j in range(send.color_mask_subject_cnts[0]): # 0 = orange
            if ( send.color_mask_subject_size[0][j] > 400 and send.color_mask_subject_size[0][j]<40000 ):
                self.Ball_size = send.color_mask_subject_size[0][j]
                self.Ball_XMin = send.color_mask_subject_XMin[0][j]
                self.Ball_XMax = send.color_mask_subject_XMax[0][j]
                self.Ball_YMin = send.color_mask_subject_YMin[0][j]
                self.Ball_YMax = send.color_mask_subject_YMax[0][j]
                self.Ball_X    = send.color_mask_subject_X[0][j]
                self.Ball_Y    = send.color_mask_subject_Y[0][j]
            else :
                self.Ball_size = 0
                self.Ball_XMin = 0
                self.Ball_XMax = 0
                self.Ball_YMin = 0
                self.Ball_YMax = 0
                self.Ball_X    = 0
                self.Ball_Y    = 0



    def basket_info(self):
        for j in range(send.color_mask_subject_cnts[5]): # 5 = red
            if send.color_mask_subject_size[5][j].size > 400 :
                self.Basket_size = send.color_mask_subject_size[5][j]
                self.Basket_XMin = send.color_mask_subject_XMin[5][j]
                self.Basket_XMax = send.color_mask_subject_XMax[5][j]
                self.Basket_YMin = send.color_mask_subject_YMin[5][j]
                self.Basket_YMax = send.color_mask_subject_YMax[5][j]
                self.Basket_X    = send.color_mask_subject_X[5][j]
                self.Basket_Y    = send.color_mask_subject_Y[5][j]
            else :
                self.Basket_size = 0
                self.Basket_XMin = 0
                self.Basket_XMax = 0
                self.Basket_YMin = 0
                self.Basket_YMax = 0
                self.Basket_X    = 0
                self.Basket_Y    = 0

    def draw(self):
         
        send.drawImageFunction(1, 0, 0, 320, 120, 120, 152, 245, 255)                                       #用於逆透視法測距的VisionMiddle blue ? ?
        send.drawImageFunction(2, 0, self.BasketVerticalBaseLine50, self.BasketVerticalBaseLine50, 0, 240, 255, 0, 0) #50 red
        send.drawImageFunction(3, 0, self.BasketVerticalBaseLine60, self.BasketVerticalBaseLine60, 0, 240, 255, 0, 0) #60 red
        send.drawImageFunction(4, 0, self.BasketVerticalBaseLine70, self.BasketVerticalBaseLine70, 0, 240, 255, 0, 0) #70 red
        send.drawImageFunction(5, 0, self.BasketVerticalBaseLine80, self.BasketVerticalBaseLine80, 0, 240, 255, 0, 0) #80 red
        send.drawImageFunction(6, 0, self.BasketVerticalBaseLine90, self.BasketVerticalBaseLine90, 0, 240, 255, 0, 0) #90 red
        send.drawImageFunction(7, 0, self.BasketVerticalBaseLine, self.BasketVerticalBaseLine , 0, 240, 255, 0, 0)    #default red
        send.drawImageFunction(8, 1, self.Ball_XMin, self.Ball_XMax, self.Ball_YMin, self.Ball_YMax, 255, 0, 255)               #Ball Area pink
        send.drawImageFunction(9, 1, self.Basket_XMin, self.Basket_XMax, self.Basket_YMin, self.Basket_YMax, 255, 255, 0)       #Basket Area yellow
        send.drawImageFunction(10, 0, self.Basket_X, self.Basket_X, 0, 240, 255, 255, 0)                              #Basket Vertical Midline yellow

class trace():
    def __init__(self):
        self.HorizontalHeadPosition = 2048
        self.VerticalHeadPosition = 2048
        self.WaistPosition = 2048
        self.VerticalMaxAngle = 2048
        self.VerticalMinAngle = 2048
        self.HorizontalMaxAngle = 2048
        self.HorizontalMinAngle = 2048
        self.HeadHorizontalState = True
        self.HeadVerticalState = True
        self.HeadTurnSpeed = 40
        self.ImgHorizontalAngle = 70.42
        self.ImgVerticalAngle = 43.3
        self.RobotVisionWidth = 320
        self.RobotVisionHeight = 240
        self.TraceDegreePercent = 0.25
        self.Deg2Scale = 11.377777777 #4096/360
        self.MoveX = 0
        self.MoveY = 0
        self.MoveW = 0
        self.ErrorHorizontalAngle = 0
        self.ErrorVerticalAngle = 0
        self.send_theta = 0
        self.send_X = 0
        self.HorizontalHeadAngle = 0
        self.VerticalHeadAngle = 0
        self.NowX = 0
        self.NowY = 0
        self.NowTheta = 0
        self.ContinuousFlag = False
        self.StatesFlag = False
        # self.AddX = 0
        # self.AddY = 0
        # self.AddTheta = 0

    def MoveHead(self,ID, Position,Speed):#動頭馬達編號 1:水平 2:垂直，刻度，速度
        send.sendHeadMotor(ID,Position,Speed)
        time.sleep(0.05)
        if ID == 1 :
            self.HorizontalHeadPosition = Position
        else:
            self.VerticalHeadPosition = Position

    def MoveWaist(self, Position, Speed):#刻度，速度
        send.sendSingleMotor(9,(Position - self.WaistPosition),Speed)
        time.sleep(0.05)
        self.WaistPosition = Position
        
    def FindTargetHead(self,VerticalMaxAngle ,VerticalMinAngle ,HorizontalMaxAngle, HorizontalMinAngle):
        if self.HeadVerticalState:
            print('1')
            self.MoveHead(2,  VerticalMaxAngle, 200)
        else:
            print('2')
            self.MoveHead(2, VerticalMinAngle, 200)

        if self.HeadHorizontalState:
            print('3')
            if (self.HorizontalHeadPosition - self.HeadTurnSpeed) > HorizontalMinAngle:
                self.MoveHead(1, self.HorizontalHeadPosition - self.HeadTurnSpeed, 200)
            else:
                self.HeadVerticalState = False
                self.HeadHorizontalState = False
        else:
            print('4')
            if (self.HorizontalHeadPosition + self.HeadTurnSpeed) < HorizontalMaxAngle:
                self.MoveHead(1, self.HorizontalHeadPosition + self.HeadTurnSpeed, 200)
            else:
                self.HeadVerticalState = True
                self.HeadHorizontalState = True

    def TraceTargetHead(self,Target_X,Target_Y,TargetVerticalBaseLine,TargetHorizontalBaseLine):
        self.MoveX = Target_X - TargetVerticalBaseLine       #可以當作與球baseline的差
        self.MoveY = Target_Y - TargetHorizontalBaseLine
        self.ErrorHorizontalAngle = self.ImgHorizontalAngle * self.MoveX / self.RobotVisionWidth      #馬達轉攝影機320pixel時轉的角度*與球baseline的差/320pixel,算出會得到角度
        self.ErrorVerticalAngle = self.ImgVerticalAngle * self.MoveY / self.RobotVisionHeight         #馬達轉攝影機240pixel時轉的角度*與球baseline的差/240pixel,算出會得到角度
        self.MoveHead(1, self.HorizontalHeadPosition - round(self.ErrorHorizontalAngle * self.TraceDegreePercent * 1 * self.Deg2Scale), 200)#再利用上面得到的角度來換算成刻度，來call   MoveHead()
        self.MoveHead(2, self.VerticalHeadPosition - round(self.ErrorVerticalAngle * self.TraceDegreePercent * 1 * self.Deg2Scale), 200)

    def TurnToTarget(self,Target_X,Target_Y,x,y,z,theta):
        self.TraceTargetHead(Target_X,Target_Y,160,120)
        # self.HorizontalHeadAngle = self.HorizontalHeadPosition
        if (self.HorizontalHeadPosition - 2048) > 0 :
            self.send_theta = theta + 3
            self.MoveContinuous( x, y, self.send_theta, 50, 50, 1)
            print("turn left")
        else:
            self.send_theta = theta - 3
            self.MoveContinuous( x, y, self.send_theta, 50, 50, 1)
            print("turn right")

    def TraceTargetWalk(self,Target_X,Target_Y,Catch_Target_XMAX,Catch_Target_Xmin,x,y,z,theta):
        self.TraceTargetHead(Target_X,Target_Y,160,120)

        # self.VerticalHeadAngle = self.VerticalHeadPosition
        # self.HorizontalHeadAngle = self.HorizontalHeadPosition

        if (self.HorizontalHeadPosition - 2048) > 100 :
            self.send_theta = theta + 2
            self.send_X = x + 500
            print(self.send_theta , self.send_X)
            print("turn left")
        elif (self.HorizontalHeadPosition - 2048) < -100:
            self.send_theta = theta - 2
            self.send_X = x + 500
            print(self.send_theta , self.send_X)
            print("turn right")
        elif self.VerticalHeadPosition > Catch_Target_Xmin :
            self.send_theta = theta
            self.send_X = x + 800
            print(self.send_theta , self.send_X)
            print("gogogo")
        elif self.VerticalHeadPosition < Catch_Target_Xmin :
            self.send_theta = theta
            self.send_X = x - 800
            print(self.send_theta , self.send_X)
            print("backback")
        
        
            
        self.MoveContinuous( self.send_X, y, self.send_theta, 100, 100, 2)
        print("HorizontalHeadPosition = ", self.HorizontalHeadPosition)
        # print("send_X = ", self.send_X)

    def WaistFix(self, Target_X, Target_Y, TargetXCenter, TargeYCenter):#轉腰調整Basket.X與BasketVerticalBaseLine的誤差
        self.MoveW = TargetXCenter - Target_X
        self.MoveY = TargeYCenter - Target_Y
        self.MoveWaist((self.WaistPosition + self.MoveW), 20)
        time.sleep(0.5)
        self.MoveHead(2, (self.VerticalHeadPosition + self.MoveY), 20)
        time.sleep(0.2)
        # if (Target_X - BasketVerticalBaseLine) > 0:
        #     print("RIGHT")
        #     self.MoveWaist((-1)*(Target_X - BasketVerticalBaseLine), 60)
        # elif (Target_X - BasketVerticalBaseLine) < 0 :
        #     print("LEFT")
        #     self.MoveWaist(BasketVerticalBaseLine - Target_X, 60)
        # time.sleep(1)

    def isStartContinuous(self, ContinuousStates):
        if ContinuousStates == "StartContinuous" :
            self.ContinuousFlag = True

        if ContinuousStates == "StopContinuous" :
            self.ContinuousFlag = False
            
        if self.StatesFlag != self.ContinuousFlag :
            send.sendBodyAuto(0,0,0,0,1,0)
            self.StatesFlag = not(self.StatesFlag) 
        


    def MoveContinuous(self ,ExpX ,ExpY ,Theta ,AddX ,AddY ,AddTheta) :
        if abs(self.NowX - ExpX) < AddX:
            self.NowX = ExpX
        else:
            if self.NowX < ExpX :
                self.NowX += AddX
            elif self.NowX > ExpX :
                self.NowX -= AddX
            else:
                pass

        if abs(self.NowY - ExpY) < AddY:
            self.NowY = ExpY
        else:
            if self.NowY < ExpY :
                self.NowY += AddY
            elif self.NowY > ExpY :
                self.NowY -= AddY
            else:
                pass

        if abs(self.NowTheta - Theta) < AddTheta:
            self.NowTheta = Theta
        else:
            if self.NowTheta < Theta :
                self.NowTheta += AddTheta
            elif self.NowTheta > Theta :
                self.NowTheta -= AddTheta
            else:
                pass

        print("NowX, NowY, NowTheta = ", self.NowX, ", ", self.NowY, ", ", self.NowTheta)
        
        send.sendContinuousValue(self.NowX, self.NowY, 0, self.NowTheta , 0)
        time.sleep(1)


# class Walk() :
#     def __init__(self) :
#         self.NowX = 0
#         self.NowY = 0
#         self.NowTheta = 0
#         self.ContinuousFlag = False
#         self.StatesFlag = False
#         # self.AddX = 0
#         # self.AddY = 0
#         # self.AddTheta = 0

#     def isStartContinuous(self, ContinuousStates):
#         if ContinuousStates == "StartContinuous" :
#             self.ContinuousFlag = True

#         if ContinuousStates == "StopContinuous" :
#             self.ContinuousFlag = False
            
#         if self.StatesFlag != self.ContinuousFlag :
#             send.sendBodyAuto(0,0,0,0,1,0)
#             self.StatesFlag = not(self.StatesFlag) 
        


#     def MoveContinuous(self ,ExpX ,ExpY ,Theta ,AddX ,AddY ,AddTheta) :
#         if abs(self.NowX - ExpX) < AddX:
#             self.NowX = ExpX
#         else:
#             if self.NowX < ExpX :
#                 self.NowX += AddX
#             elif self.NowX > ExpX :
#                 self.NowX -= AddX
#             else:
#                 pass

#         if abs(self.NowY - ExpY) < AddY:
#             self.NowY = ExpY
#         else:
#             if self.NowY < ExpY :
#                 self.NowY += AddY
#             elif self.NowY > ExpY :
#                 self.NowY -= AddY
#             else:
#                 pass

#         if abs(self.NowTheta - Theta) < AddTheta:
#             self.NowTheta = Theta
#         else:
#             if self.NowTheta < Theta :
#                 self.NowTheta += AddTheta
#             elif self.NowTheta > Theta :
#                 self.NowTheta -= AddTheta
#             else:
#                 pass

#         print("NowX, NowY, NowTheta = ", self.NowX, ", ", self.NowY, ", ", self.NowTheta)
        
#         send.sendContinuousValue(self.NowX, self.NowY, 0, self.NowTheta , 0)
