#!/usr/bin/env python
#coding=utf-8
from traceback import print_tb
import rospy
import numpy as np
from Python_API import Sendmessage
import time

aa = 1

x_place=0
y_place=0
ball_size=0
send=Sendmessage()
first_ball_search = 63
first_ball_trace = 20
first_basket_search = 90
first_basket_trace = 25
catch = 1
found = 1
head_back = 1


class target_location():
    def __init__(self):
        self.x_ball_place = 0
        self.y_ball_place = 0
        self.x_basket_place = 0
        self.y_basket_place = 0
        self.ball_size = 0
        self.basket_size = 0
        self.color_mask_subject_red = 0
        self.color_mask_subject_orange = 0
        
    def ball_parameter(self):
        self.color_mask_subject_red = send.color_mask_subject_cnts[0]
        for j in range (self.color_mask_subject_red):
            if send.color_mask_subject_size [0][j] > 70 :
                self.x_ball_place =  send.color_mask_subject_X [0][j]
                self.y_ball_place = send.color_mask_subject_Y [0][j]
                self.ball_size = send.color_mask_subject_size [0][j]
            else :
                self.x_ball_place = 0
                self.y_ball_place = 0
                self.ball_size = 0

            
    
    def basket_parameter(self):
        self.color_mask_subject_orange = send.color_mask_subject_cnts[5]
        for j in range (self.color_mask_subject_orange):
            if send.color_mask_subject_size [5][j] > 500 :
                self.x_basket_place =  send.color_mask_subject_X [5][j]
                self.y_basket_place = send.color_mask_subject_Y [5][j]
                self.basket_size = send.color_mask_subject_size [5][j]
            else :
                self.x_basket_place= 0
                self.y_basket_place = 0
                self.basket_size = 0

   

class motor_move():
    cc = target_location()

    def __init__(self):
        self.head_pos = 0
        self.horizon = 2048
        self.vertical = 2048
        self.horizon_start = 0
        self.trace_start = 0
        self.x_differ = 0
        self.y_differ = 0
        self.x_degree = 0
        self.y_degree = 0
        self.x_body_rotate = 0
       


    def move_head(self,ID,Position,Speed):
        send.sendHeadMotor(ID,Position,Speed)
        if ID == 1 :
            self.horizon =  Position
        else :
            self.vertical = Position

    def view_move(self,right_place,left_place,up_place,down_place,speed,delay):

        if self.horizon_start == 1 :
            if self.horizon > left_place:
                self.move_head(1,self.horizon,speed)
                self.horizon = self.horizon - speed
                time.sleep(delay) 
            else :
                self.move_head(2,up_place,speed)
                self.horizon_start = 0                   
        else :                
            if  self.horizon < right_place:
                self.move_head(1,self.horizon,speed)
                self.horizon = self.horizon + speed
                time.sleep(delay) 
            else :
                self.move_head(2,down_place,speed)
                self.horizon_start = 1 
    
    def trace_revise(self,x_target,y_target,speed) :
        self.x_differ =  x_target - 160 
        self.y_differ =  y_target - 120 
        self.x_degree = self.x_differ * (70.42 / 320)
        self.y_degree = self.y_differ * (43.3 / 240)
        self.move_head(1, self.horizon - round(self.x_degree * 4096 / 360 *0.15) ,speed)
        self.move_head(2, self.vertical - round(self.y_degree * 4096 / 360 *0.15) ,speed)
        print("x=",self.horizon)
        time.sleep(0.05)

    def body_trace_rotate(self) :
        self.x_body_rotate = self.horizon - 2048
        if self.x_body_rotate > 30 :
            send.sendContinuousValue(0,0,0,4,0)
            print( "go left = ",self.x_body_rotate)
            time.sleep(0.05)
        elif self.x_body_rotate < -30 :
            send.sendContinuousValue(0,0,0,-4,0)
            print( "go right = ",self.x_body_rotate)
            time.sleep(0.05)
        else :
            pass

    def body_trace_straight(self,degree,found_ball) :
        
        if (self.vertical - degree) > 10:
            send.sendContinuousValue(2,0,0,0,0)
            print( "go ahead = ",self.vertical)
            time.sleep(0.05)
        elif (self.vertical - degree) < -10 :
            send.sendContinuousValue(-1,0,0,0,0)
            print( "go back = ",self.vertical)
            time.sleep(0.05)
        elif  abs(self.vertical - degree) < 10 and found_ball > 0:
            send.sendBodyAuto(0,0,0,0,1,0)
            found_ball = found_ball - 1
            time.sleep(2)
            send.sendSingleMotor(1,-50,20)
            print(".................................................")
            time.sleep(2)
        else :
            pass
   
# .........................................
    # def body_trace_straight(self,ball_size,y_target) :
        
    #     if ball_size - y_target  < -60:
    #         send.sendContinuousValue(1,0,0,0,0)
    #         print( "go ahead size= ",ball_size)
    #         time.sleep(0.3)
    #     elif abs(ball_size - y_target) <  60:
    #         send.sendContinuousValue(0,0,0,0,0)
    #         print( "stop = ",self.x_body_rotate)
    #         time.sleep(0.3)
    #     elif ball_size - y_target  > 60:
    #         send.sendContinuousValue(-1,0,0,0,0)
    #         print( "go back size= ",ball_size)
    #         time.sleep(0.3)
    #     else :
    #         pass
# ..............................................
        
        
    # def trace_revise(self,horzine_target,vertical_target,speed,delay) :
    #     if (120 - horzine_target) < -10 :
    #         self.move_head(2,self.horizon,speed)
    #         self.horizon = self.horizon - speed
    #         time.sleep(delay) 
    #     elif (120 - horzine_target)  > 10 :
    #         self.move_head(2,self.horizon,speed)
    #         self.horizon = self.horizon + speed
    #         time.sleep(delay)
    #     else :
    #         time.sleep(delay)


if __name__ == '__main__' :
    
    bb = target_location()
    cc = motor_move()
    
    try:
        
        
        while not rospy.is_shutdown():
            if send.Web==True :
                
                send.drawImageFunction(1,0,160,160,0,240,255,255,255)
                send.drawImageFunction(2,0,0,320,120,120,255,255,255)

                if found > 0 :

                    if   bb.ball_size < 70 or first_ball_search > 0 :
                        cc.view_move(2380,1780,1900,1600,25,0.05)                 
                        time.sleep(0.05)
                        bb.ball_parameter()  
                        print("  ball => x:",bb.x_ball_place," y:",bb.y_ball_place," size:",bb.ball_size)
                        
                        first_ball_search = first_ball_search - 1

                    elif first_ball_search == 0 and first_ball_trace > 0 :
                        bb.ball_parameter() 
                        cc.trace_revise(bb.x_ball_place,bb.y_ball_place,25)
                        first_ball_trace = first_ball_trace - 1
                        

                    else :
                        
                        bb.ball_parameter()   
                        print(" ball => x:",bb.x_ball_place," y:",bb.y_ball_place," size:",bb.ball_size)                   
                        cc.trace_revise(bb.x_ball_place,bb.y_ball_place,25)

                        if aa == 1 :
                            time.sleep(1)
                            send.sendBodyAuto(0,0,0,0,1,0)
                            aa = aa - 1
                        

                        elif abs(cc.x_differ) > 3 :                    
                            cc.body_trace_rotate()
                        
                        elif abs(cc.vertical - 1300) > 3 and abs(cc.x_differ) < 3:
                            cc.body_trace_straight(1300,found)

                    
                        else :
                            pass
# ............................................................................................................................
# ............................................................................................................................
# ............................................................................................................................
# ............................................................................................................................

                elif  found == 0 :

                    if   bb.basket_size < 500 or first_basket_search > 0 :
                        cc.view_move(2480,1680,1900,1600,25,0.05)                 
                        time.sleep(0.05)
                        bb.basket_parameter()  
                        print("  basket => x:",bb.x_basket_place," y:",bb.y_basket_place," size:",bb.basket_size)
                        
                        first_basket_search = first_basket_search - 1

                    elif first_basket_search == 0 and first_basket_trace > 0 :
                        bb.basket_parameter() 
                        cc.trace_revise(bb.x_basket_place,bb.y_basket_place,25)
                        first_basket_trace = first_basket_trace - 1
                        

                    else :
                        
                        bb.basket_parameter()   
                        print(" basket => x:",bb.x_basket_place," y:",bb.y_basket_place," size:",bb.basket_size)                   
                        cc.trace_revise(bb.x_basket_place,bb.y_basket_place,25)

                        if aa == 1 :
                            time.sleep(1)
                            send.sendBodyAuto(0,0,0,0,1,0)
                            aa = aa - 1
                        

                        elif abs(cc.x_differ) > 5 :                    
                            cc.body_trace_rotate()
                        
                        elif abs(cc.vertical - 1300) > 5:
                            cc.body_trace_straight(1300,catch)

                            

                        else :
                            pass

                else :
                    pass



            else :
                if head_back == 1 :
                    send.sendHeadMotor(1,2048,30)
                    send.sendHeadMotor(2,2048,30)
                    head_back = head_back - 1
                    time.sleep(1)





                # elif  abs(cc.x_differ) > 3 :
                #     time.sleep(0.3)
                #     bb.ball_parameter()   
                #     print(" ball => x:",bb.x_ball_place," y:",bb.y_ball_place," size:",bb.ball_size)
            
                #     send.sendBodyAuto(0,0,0,0,1,0)                     
                #     cc.trace_revise(bb.x_ball_place,bb.y_ball_place,25)
                #     cc.body_trace_rotate()

                # else :
                #     time.sleep(0.3)
                #     bb.ball_parameter()   
                #     print(" ball => x:",bb.x_ball_place," y:",bb.y_ball_place," size:",bb.ball_size)
            
                #     send.sendBodyAuto(0,0,0,0,1,0)                     
                #     cc.trace_revise(bb.x_ball_place,bb.y_ball_place,25)
                #     cc.body_trace_straight(bb.ball_size,500)
                        
                        # if  :
                        #     cc.trace_revise(bb.x_ball_place,bb.y_ball_place,25)
                        #     cc.body_trace_rotate()



                        # if ball_size < 500 or ball_size >1200:
                        #     cc.trace_revise(bb.x_ball_place,bb.y_ball_place,25)
                        #     cc.body_trace_straight(bb.ball_size,500)
                        # else :
                        #     pass

    except rospy.ROSInterruptException:
        pass