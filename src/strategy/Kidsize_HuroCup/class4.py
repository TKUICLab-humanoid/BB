#!/usr/bin/env python
#coding=utf-8
from traceback import print_tb
import rospy
import numpy as np
from Python_API import Sendmessage
import time

gobasket = 1
aa = 1
x = 0
x_place=0
y_place=0
ball_size=0
send=Sendmessage()
first_ball_search = 50
first_ball_trace = 20
first_basket_search = 50
first_basket_trace = 25
catch = False




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
            if send.color_mask_subject_size [5][j] > 250 :
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
        self.found = False
        self.catch = False
        


    def move_head(self,ID,Position,max_horizon_size,Speed):
        send.sendHeadMotor(ID,Position,Speed)
        if abs(Position - 2048) > max_horizon_size :
            if (Position - (2048 + max_horizon_size)) < 0 :
                Position = 2048 - max_horizon_size
            elif (Position - (2048 + max_horizon_size)) > 0 :
                Position = 2048 + max_horizon_size
            else :
                Position = 2048

        else :
            if ID == 1 :
                self.horizon =  Position
            else :
                self.vertical = Position

    def view_move(self,right_place,left_place,up_place,down_place,speed,delay):

        if self.horizon_start == 1 :
            if self.horizon > left_place:
                self.move_head(1,self.horizon,500,speed)
                self.horizon = self.horizon - speed
                time.sleep(delay) 
            else :
                self.move_head(2,up_place,1100,speed)
                self.horizon_start = 0                   
        else :                
            if  self.horizon < right_place:
                self.move_head(1,self.horizon,500,speed)
                self.horizon = self.horizon + speed
                time.sleep(delay) 
            else :
                self.move_head(2,down_place,1100,speed)
                self.horizon_start = 1 
    
    def trace_revise(self,x_target,y_target,speed) :
        self.x_differ =  x_target - 160 
        self.y_differ =  y_target - 120 
        self.x_degree = self.x_differ * (70.42 / 320)
        self.y_degree = self.y_differ * (43.3 / 240)
        self.move_head(1, self.horizon - round(self.x_degree * 4096 / 360 *0.15),1300 ,speed)
        self.move_head(2, self.vertical - round(self.y_degree * 4096 / 360 *0.15),1300 ,speed)
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
        

    def body_trace_straight(self,degree) :
        
        if (self.vertical - degree) > 50:
            send.sendContinuousValue(1000,0,0,0,0)
            print( "go ahead = ",self.vertical)
            time.sleep(0.05)
        elif (self.vertical - degree) < -50 :
            send.sendContinuousValue(-1,0,0,0,0)
            print( "go back = ",self.vertical)
            time.sleep(0.05)
        elif  abs(self.vertical - degree) < 50 :
            send.sendBodyAuto(0,0,0,0,1,0)
            self.catch = False
            cc.found = True
            time.sleep(3)
            self.waist_revise(1)
            time.sleep(3)           
            send.sendSingleMotor(1,-50,20)
            time.sleep(2) 
            print("stop to the ball")
            send.sendSingleMotor(9,-round(self.x_body_rotate ),30)
            print(".................................................")
            
            time.sleep(2)
    
    def waist_revise(self,waist):
            
            if waist ==  1 :
                send.sendSingleMotor(9,round(self.x_body_rotate ),30)
                print(self.x_body_rotate)
                print( "go waist = ")
                time.sleep(0.05)
                waist = waist - 1


    def body_trace_basket_straight(self,basket_size) :
        
        if bb.basket_size - basket_size < -30 :
            send.sendContinuousValue(3000,0,0,0,0)
            print( "--------------------go ahead to basket---------------------  ",self.vertical)
            time.sleep(0.05)
        elif bb.basket_size - basket_size > 30 :
            send.sendContinuousValue(-3500,0,0,0,0)
            print( "--------------------go back from basket-------------------- ",self.vertical)
            time.sleep(0.05)
        elif  abs(bb.basket_size - basket_size) < 30 :
            print( "--------------------stop at the basket----------------------")
            send.sendBodyAuto(0,0,0,0,1,0)
            self.catch = True
            time.sleep(2)
    
    

        
        
    def slam(self,score) :
        if score == 1 :
            send.sendSingleMotor(2,-100,30)
            send.sendSingleMotor(1,-950,30)
            send.sendSingleMotor(2,300,30)
            print("slam dunk-------------slam dunk--------------slam dunk---------")
            score = score + 1
            time.sleep(2)
   
# .........................................
    # def body_trace_straight(self,ball_size,y_target) :
        
    #     if ball_size - y_target  < -60:
    #         send.sendContinuousValue(1,0,0,0,0)
    #         print( "go ahead sizecc.found == False= ",ball_size)
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
    step = 0
    sw = 0
    head_back = 0

    throw_ball_point = [1100,570] #灌籃未寫
    #                   三分  五分
    #尚未有sw控制 ：：throw_ball_point[sw]
    try:
        
        
        while not rospy.is_shutdown():
            if send.is_start==True :
                
                send.drawImageFunction(1,0,160,160,0,240,255,255,255)
                send.drawImageFunction(2,0,0,320,120,120,255,255,255)
                head_back = head_back +1
                if cc.found == False  :

                    if step == 0 :
                        for i in range (0,20,1) :
                            cc.view_move(2248,1848,2088,1600,30,0.05)                 
                            time.sleep(0.05)
                            print("start to find the ball")
                            bb.ball_parameter()  
                            print("  ball => x:",bb.x_ball_place," y:",bb.y_ball_place," size:",bb.ball_size)
                        step = step + 1

                    elif step == 1 :
                        for i in range (0,20,1) :
                            bb.ball_parameter() 
                            cc.trace_revise(bb.x_ball_place,bb.y_ball_place,25)
                        step = step + 1
                        

                    elif  step == 2 :
                        
                        bb.ball_parameter()   
                        print(" ball => x:",bb.x_ball_place," y:",bb.y_ball_place," size:",bb.ball_size)                   
                        cc.trace_revise(bb.x_ball_place,bb.y_ball_place,25)

                        if aa == 1 :
                            time.sleep(1)
                            send.sendBodyAuto(0,0,0,0,1,0)
                            aa = aa - 1
                        

                        elif abs(cc.x_differ) > 3  or abs(cc.vertical - 1300) < 50 :                    
                            cc.body_trace_rotate()
                        
                        elif abs(cc.vertical - 1300) > 50 :
                            cc.body_trace_straight(1300)
                            
                             
                             
                            print("check")
# ............................................................................................................................
# ............................................................................................................................
# ............................................................................................................................
# ............................................................................................................................

                elif  cc.found == True and cc.catch == False : 
                    if step == 2 :
                        step = step + 1
                        print("start to find the basket")
                    elif  step == 3 :
                        send.sendHeadMotor(2,2048,30)
                        print("---------------reset the robot's head position-------------")
                        for j in range (0,25,1) :
                            cc.view_move(2548,1548,2248,1948,30,0.05)
                            time.sleep(0.05)
                            bb.basket_parameter()
                            print("  basket => x:",bb.x_basket_place," y:",bb.y_basket_place," size:",bb.basket_size)
                        step = step +1

                    elif step == 4 :
                        for i in range (0,20,1) :
                            bb.basket_parameter() 
                            cc.trace_revise(bb.x_basket_place,bb.y_basket_place,25)
                        step = step +1


                    elif  step == 5 :

                        bb.basket_parameter()
                        print(" basket => x:",bb.x_basket_place," y:",bb.y_basket_place," size:",bb.basket_size)
                        cc.trace_revise(bb.x_basket_place,bb.y_basket_place,25)

                        if gobasket == 1 :
                            time.sleep(1)
                            send.sendBodyAuto(0,0,0,0,1,0)
                            gobasket = gobasket - 1
                            time.sleep(1)


                        elif abs(cc.x_differ) > 2 or abs(bb.basket_size - throw_ball_point[sw]) < 10 :
                            cc.body_trace_rotate()

                        elif abs(bb.basket_size - throw_ball_point[sw]) > 10 :
                            cc.body_trace_basket_straight(throw_ball_point[sw])
                            print("-----------start the action of 3 point--------------")
                            if cc.catch == True :
                                step = step + 1

                elif step == 6 :
                    print("aaaaaaaaaaaaaaaaaaaaaaaaAAA")
                    # send.sendSingleMotor(9,300,30)
                    cc.waist_revise(1)
                    cc.slam(1)
                    step=step+1

            # elif send.Web==False and head_back = -1:
            #         head_back = 0 
            #         send.sendHeadMotor(1,2048,30)
            #         send.sendHeadMotor(2,2048,30)
            #         bb = target_location()
            #         cc = motor_move()
            #         send.sendBodySector(29)
                    
            #         print("-----------------------end--------------------------")
            #         time.sleep(1)





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