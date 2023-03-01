#!/usr/bin/env python
#coding=utf-8
from cmath import sqrt
from re import T
import time
from traceback import print_tb

import numpy as np
import rospy
from Python_API import Sendmessage
send=Sendmessage()

class target_location():
    def __init__(self):
        self.ball_x = 0
        self.ball_y = 0
        self.basket_x = 0
        self.basket_y = 0
        self.ball_size = 0
        self.basket_size = 0
        self.color_mask_subject_red = 0
        self.color_mask_subject_orange = 0
        self.ball_x_min = 0
        self.ball_y_min = 0
        self.ball_x_max = 0
        self.ball_y_max = 0
        self.basket_x_min = 0
        self.basket_y_min = 0
        self.basket_x_max = 0
        self.basket_y_max = 0
        
        
    def ball_parameter(self):
        self.color_mask_subject_red = send.color_mask_subject_cnts[0]
        self.ball_x = 0
        self.ball_y = 0
        self.ball_size = 0
        for j in range (self.color_mask_subject_red):
            if 310 > send.color_mask_subject_X [0][j] > 10 and 230 > send.color_mask_subject_Y [0][j] > 10:

                if send.color_mask_subject_size [0][j] > 350  and send.color_mask_subject_size [0][j] > self.ball_size:
                    self.ball_x =  send.color_mask_subject_X [0][j]
                    self.ball_y = send.color_mask_subject_Y [0][j]
                    self.ball_size = send.color_mask_subject_size [0][j]
                    self.ball_x_min = send.color_mask_subject_XMin[0][j] 
                    self.ball_y_min = send.color_mask_subject_YMin[0][j] 
                    self.ball_x_max = send.color_mask_subject_XMax[0][j] 
                    self.ball_y_max =send.color_mask_subject_YMax[0][j]
            
                

            
    
    def basket_parameter(self):
        self.color_mask_subject_orange = send.color_mask_subject_cnts[5] 
        self.basket_x= 0
        self.basket_y = 0
        self.basket_size = 0
        for j in range (self.color_mask_subject_orange):
            if send.color_mask_subject_size [5][j] > 500 and send.color_mask_subject_size [5][j] > self.basket_size:
                self.basket_x =  send.color_mask_subject_X [5][j]
                self.basket_y = send.color_mask_subject_Y [5][j]
                self.basket_size = send.color_mask_subject_size [5][j]
                self.basket_x_min = send.color_mask_subject_XMin[5][j] 
                self.basket_y_min = send.color_mask_subject_YMin[5][j] 
                self.basket_x_max = send.color_mask_subject_XMax[5][j] 
                self.basket_y_max =send.color_mask_subject_YMax[5][j]

        
               
   

class motor_move():
    #motor = motor_move()
    target = target_location()
    motor = target_location()
    def __init__(self):
        self.head_horizon = 2048                #頭部水平刻度
        self.head_vertical = 2048               #頭部垂直刻度
        self.waist_position = 2048              #腰當下的刻度
        self.head_horizon_flag = 1              #找目標物的旗標
        self.x_differ = 0                       #目標與中心x差距
        self.y_differ = 0                       #目標與中心y差距
        self.x_degree = 0                       #目標與中心x角度
        self.y_degree = 0                       #目標與中心y角度
        self.x_body_rotate = 0                  #身體要旋轉多少？？？
        self.found = False
        self.catch = False
        self.now_state = 0
        self.Move_waist = 0                     #修腰
        self.MoveY = 0                          #轉腰修正頭的高度
        self.NowX = 0                           #現在要移動的x量
        self.NowY = 0                           #現在要移動的y量
        self.NowTheta = 0                       #現在要旋轉的theta量
        self.throw_strength = 0                 #不知道
        self.sixty_distance = 0
        self.ninty_distance = 0
        self.detect = False
        self.distance_new = 0
        self.switch_flag = 2
        self.switch_reset_flag = 1
        self.in_tacties = 0
        self.start_get_point = False
        self.num = 3
        self.directly = False
        self.dir_num = 0
        self.clock = 3
        self.temp = 2048
        

    def switch_control(self):                                                       #need test
        target = target_location()
        motor = motor_move()
        
        if    send.DIOValue == 9 or send.DIOValue == 12 :  
            motor.test_distance() 
            print("Ball Y = ",target.basket_size , "Basket vert = ",motor.head_vertical)               
            time.sleep(0.2)                                     #籃框               上下下下 or 下下上下 or 上下上下
            self.switch_reset_flag = 1 #sector 111  
            self.in_tacties = 0

        elif    send.DIOValue == 13: 
            motor.trace_revise(target.ball_x,target.ball_y,25)
                                                             #球                 上下up下
            print("Ball vert = ",motor.head_vertical)
            self.switch_reset_flag = 1
            self.in_tacties = 0
            time.sleep(0.03)

        elif    (send.DIOValue == 27 or send.DIOValue == 11) and (self.in_tacties != 10 or self.switch_reset_flag == 1) :    
            #                   24+3                    8+3 #開啟三分策略  上上下下 
            self.switch_flag = 0
            print("SW = ", self.switch_flag)
            motor.bodyauto_close(0)
            target.ball_parameter()
            target.basket_parameter()
            step = 'begin'
            send.sendHeadMotor(1,2048,30)
            send.sendHeadMotor(2,2048,30)
            send.sendBodySector(29)

            self.switch_reset_flag = 0
            self.in_tacties = 10

        elif    (send.DIOValue == 31 or send.DIOValue == 15) and (self.in_tacties != 11 or self.switch_reset_flag == 1) :
            #                   24+7                    8+7 #開啟五分策略  上上上下
            self.switch_flag = 1
            print("SW = ", self.switch_flag)
            motor.bodyauto_close(0)
            target.ball_parameter()
            target.basket_parameter()
            step = 'begin'
            send.sendHeadMotor(1,2048,30)
            send.sendHeadMotor(2,2048,30)
            send.sendBodySector(29)
            self.switch_reset_flag = 0
            self.in_tacties = 11

        elif    (send.DIOValue == 24 or send.DIOValue ==  8) and (self.in_tacties != 12 or self.switch_reset_flag == 1) :
            #                   24                      8  #開啟二分策略  下下下下
            self.switch_flag = 2
            print("SW = ", self.switch_flag)
            motor.bodyauto_close(0)
            target.ball_parameter()
            target.basket_parameter()
            step = 'begin'
            send.sendHeadMotor(1,2048,30)
            send.sendHeadMotor(2,2048,30)
            send.sendBodySector(29)
            self.switch_reset_flag = 0
            self.in_tacties = 12

    def move_head(self,ID,Position,max_head_horizon_size,max_head_vertical_size,Speed):
        send.sendHeadMotor(ID,Position,Speed)
        target.ball_parameter()
        target.basket_parameter()
        if ID == 1 :
            self.head_horizon =  Position
            if abs(self.head_horizon - 2048) > max_head_horizon_size :
                if (self.head_horizon - 2048 ) > 0 :
                    self.head_horizon = 2048 + max_head_horizon_size
                elif (self.head_horizon - 2048 ) < 0 :
                    self.head_horizon = 2048 - max_head_horizon_size

        else :
            self.head_vertical = Position
            if abs(self.head_vertical - 2048) > max_head_vertical_size :
                    if (self.head_vertical - 2048 ) > 0 :
                        self.head_vertical = 2048 + max_head_vertical_size
                    elif (self.head_vertical - 2048) < 0 :
                        self.head_vertical = 2048 - max_head_vertical_size

    def waist_reset(self,waist_x,Speed):
        send.sendSingleMotor(9,waist_x-self.waist_position,Speed)
        self.waist_position =  waist_x 
    

    def view_move(self,right_place,left_place,up_place,down_place,speed,delay):

        if self.head_horizon_flag == 3 :
            if self.head_horizon >= left_place:
                self.move_head(1,self.head_horizon,880,880,speed)
                self.head_horizon = self.head_horizon - speed
                time.sleep(delay)
            else:
                self.head_horizon_flag = 4  
                time.sleep(delay*5) 


        elif self.head_horizon_flag == 4 :
            if self.head_vertical <= up_place:
                self.move_head(2,self.head_vertical,880,880,speed)
                self.head_vertical = self.head_vertical + speed
                time.sleep(delay)
            else:
                self.head_horizon_flag = 1  
                time.sleep(delay*5)

                    
        elif  self.head_horizon_flag == 1 :
            if  self.head_horizon <= right_place:
                self.move_head(1,self.head_horizon,880,880,speed)
                self.head_horizon = self.head_horizon + speed
                time.sleep(delay) 
            else:
                self.head_horizon_flag = 2 
                time.sleep(delay*5)      

        
        elif self.head_horizon_flag ==  2:
            if self.head_vertical >= down_place:
                self.move_head(2,self.head_vertical,880,880,speed)
                self.head_vertical = self.head_vertical - speed
                time.sleep(delay)
                
            else:
                self.head_horizon_flag = 3
                time.sleep(delay*5)
   
    
    def trace_revise(self,x_target,y_target,speed) :
        # if x_target != 0 and y_target != 0:
        self.x_differ =  x_target - 160 
        self.y_differ =  y_target - 120 
        self.x_degree = self.x_differ * (65 / 320)
        self.y_degree = self.y_differ * (38 / 240)
        self.move_head(1, self.head_horizon - round(self.x_degree * 4096 / 360 *0.15),880,880,speed)
        self.move_head(2, self.head_vertical - round(self.y_degree * 4096 / 360 *0.15),880,880,speed)
        time.sleep(0.05)

    def body_trace_rotate(self,degree) :
        self.x_body_rotate = self.head_horizon - 2048
        if self.x_body_rotate > degree :
            motor.MoveContinuous(left_correct[0],left_correct[1],left_correct[2],100,100,2)
            print( "go left = ",self.x_body_rotate)
            time.sleep(0.05)
        elif self.x_body_rotate < -degree :
            motor.MoveContinuous(right_correct[0],right_correct[1],right_correct[2],100,100,2)#!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            print( "go right = ",self.x_body_rotate)
            time.sleep(0.05)
        

    def body_trace_straight(self,degree,ball_degree) :
        
        if (self.head_vertical - degree) - 150 > ball_degree :
            motor.MoveContinuous(2400+correct[0],0+correct[1],0+correct[2],100,100,2) #!!!!!!!!!!!!!!!!!!!!!!
            print( "go ahead bigbigbigbigbigbigbig= ",self.head_vertical)
            time.sleep(0.05)
        elif (self.head_vertical - degree) - 150 < ball_degree and (self.head_vertical - degree)  > ball_degree:
            motor.MoveContinuous(1500+correct[0],0+correct[1],0+correct[2],100,100,2) #!!!!!!!!!!!!!!!!!!!!!!
            print( "go ahead smallsmallsmallsmall= ",self.head_vertical)
            time.sleep(0.05)
        elif (self.head_vertical - degree) < -ball_degree :
            motor.MoveContinuous(-2400+correct[0],0+correct[1],0+correct[2],100,100,2)
            print( "go back = ",self.head_vertical)
            time.sleep(0.05)
        elif  abs(self.head_vertical - degree) <= ball_degree :
            #send.sendBodyAuto(0,0,0,0,1,0)
            
            motor.bodyauto_close(0)
            self.catch = False
            self.found = True
            time.sleep(1.2)
            send.sendBodySector(5)    #2222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222
            time.sleep(1.2)
            motor.move_head(1,1800,880,880,50) #1748
            time.sleep(2.5)           #!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1
            target.ball_parameter()
            
    def WaistFix(self, Target_X, Target_Y, TargetXCenter, TargeYCenter):#轉腰調整Basket.X與Baskethead_verticalBaseLine的誤差
        
        self.MoveW = TargetXCenter - Target_X
        if self.MoveW > 15:
            self.MoveW = 15
        elif self.MoveW < -15:
            self.MoveW = -15
        
        

        self.waist_reset((self.waist_position + self.MoveW), 30)
        self.move_head(2, self.head_vertical ,880,880,20)
        
        time.sleep(0.15)
        # time.sleep(0.2)


     ####################################### basket degree version #######################################

    def body_trace_basket_straight_2(self,degree,basket_error) :
        
        if self.head_vertical - degree > basket_error  and self.head_vertical > 1965 :  #與籃匡距離（減速）
            motor.MoveContinuous(1200+correct[0],0+correct[1],0+correct[2],75,100,2)#!!!!!!!!!!!!!!!
            print( "--------------------go ahead bbbb to basket---------------------  ",self.head_vertical)
            time.sleep(0.05)

        elif self.head_vertical - degree > basket_error and  self.head_vertical < 1965:
            motor.MoveContinuous(1200+correct[0],0+correct[1],0+correct[2],150,100,2)#!!!!!!!!!!!!!!!
            print( "--------------------go ahead sss to basket---------------------  ",self.head_vertical)
            time.sleep(0.05)    
        elif self.head_vertical - degree < basket_error and abs(self.head_vertical - degree) > basket_error:
            motor.MoveContinuous(-800+correct[0],0+correct[1],0+correct[2],100,100,2)#!!!!!!!!!!!!!!
    
            print( "--------------------go back from basket-------------------- ",self.head_vertical)
            time.sleep(0.05)
        elif abs(self.head_vertical - degree) < basket_error : 
            print( "--------------------stop at the basket----------------------",self.head_vertical - 2048)
            #send.sendBodyAuto(0,0,0,0,1,0)
            motor.bodyauto_close(0)
            target.basket_parameter()
            
            time.sleep(1)
            motor.move_head(1,1823,880,880,30)
            time.sleep(0.8)
            send.sendBodySector(887)
            # send.sendBodySector(3)   #準備上籃之動作一（舉手...）111111111111111111111111111111111111111111111111111111111`
            print("-------------------------send.sendBodySector(3)------------------------------")
            time.sleep(1.5)
            self.found = True
            self.catch = True
    
     #####################################################################################################

    def body_trace_basket_straight_3(self,basket_size,basket_error) :
        
        if target.basket_size - basket_size < -basket_error and target.basket_size > 1718:
            send.sendContinuousValue(300+correct[0],0+correct[1],0,0+correct[2],0)#!!!!!!!!!!!!!!!
            print( "--------------------go ahead to basket---------------------  ",self.head_vertical)
            time.sleep(0.05)
        elif target.basket_size - basket_size < -basket_error and target.basket_size < 1718:
            send.sendContinuousValue(1200+correct[0],0+correct[1],0,0+correct[2],0)#!!!!!!!!!!!!!!!
            print( "--------------------go ahead to basket---------------------  ",self.head_vertical)
            time.sleep(0.05)

        elif target.basket_size - basket_size > basket_error and target.basket_size > 1718:
            send.sendContinuousValue(-1200+correct[0],0+correct[1],0,0+correct[2],0)#!!!!!!!!!!!!!!!!!
            print( "--------------------go back from basket-------------------- ",self.head_vertical)
            time.sleep(0.05)   
        elif target.basket_size - basket_size > basket_error and target.basket_size < 1718:
            send.sendContinuousValue(-800+correct[0],0+correct[1],0,0+correct[2],0)#!!!!!!!!!!!!!!!!!
            print( "--------------------go back from basket-------------------- ",self.head_vertical)
            time.sleep(0.05)
        elif abs(target.basket_size - basket_size) < basket_error :
            print( "--------------------stop at the basket----------------------",self.head_horizon - 2048)
            #send.sendBodyAuto(0,0,0,0,1,0)
            motor.bodyauto_close(0)
            target.basket_parameter()
            
            time.sleep(0.5)
            motor.move_head(1,2048,880,880,30)
            time.sleep(0.2)
            send.sendBodySector(5301)
            # send.sendBodySector(3)   #33333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333
            print("-------------------------send.sendBodySector(3)------------------------------")
            time.sleep(2)
            self.found = True
            self.catch = True
    
    # def body_trace_basket_straight_5(self,degree,basket_error) :
        
    #     if self.head_vertical - degree < basket_error  and abs(self.head_vertical - degree) > basket_error and self.head_vertical < 2150 :
    #         motor.MoveContinuous(-1300+correct[0],0+correct[1],0+correct[2],100,100,2)#!!!!!!!!!!!!!!!
    #         print( "--------------------go back bbbb to basket---------------------  ",self.head_vertical)
    #         time.sleep(0.05)

    #     elif (self.head_vertical - degree < basket_error  and abs(self.head_vertical - degree) > basket_error and self.head_vertical > 2150) or  self.head_vertical - degree < -basket_error:
    #         motor.MoveContinuous(-800+correct[0],0+correct[1],0+correct[2],100,100,2)#!!!!!!!!!!!!!!!
    #         print( "--------------------go back sss to basket---------------------  ",self.head_vertical)
    #         time.sleep(0.05)    
    #     elif self.head_vertical - degree > basket_error:
    #         motor.MoveContinuous(800+correct[0],0+correct[1],0+correct[2],100,100,2)#!!!!!!!!!!!!!!
    
    #         print( "--------------------go ahead from basket-------------------- ",self.head_vertical)
    #         time.sleep(0.05)
    #     elif self.head_vertical - degree < basket_error : 
    #         print( "--------------------stop at the basket----------------------",self.head_vertical - 2048)
    #         #send.sendBodyAuto(0,0,0,0,1,0)
    #         motor.bodyauto_close(0)
    #         target.basket_parameter()
            
    #         time.sleep(1)
    #         send.sendBodySector(5301)
    #         time.sleep(0.8)
    #         print("throw_ball_point ==                               ",motor.head_vertical)
    #         print("-------------------------send.sendBodySector(3)------------------------------")
    #         time.sleep(2.5)
    #         self.found = True
    #         self.catch = True

    def body_trace_basket_straight_5(self,basket_size,basket_error) :
        if target.basket_size - basket_size < basket_error  and (basket_size > target.basket_size > (basket_size - basket_error)) :
            self.num = self.num - 1
            if self.num <= 0 :
                print( "--------------------stop at the basket----------------------",target.basket_size)
                #send.sendBodyAuto(0,0,0,0,1,0)
                motor.bodyauto_close(0)
                target.basket_parameter()
                print("throw_ball_point ==                               ",target.basket_size ,  target.basket_size - basket_size )
                time.sleep(3)
                send.sendBodySector(5301)
                print("-------------------------send.sendBodySector(3)------------------------------")
                time.sleep(1)
                self.found = True
                self.catch = True
        
        elif target.basket_size - basket_size < -basket_error  :
            motor.MoveContinuous(800+correct[0],0+correct[1],0+correct[2],100,100,2)#!!!!!!!!!!!!!!
            print( "--------------------go ahead to basket---------------------  ",target.basket_size)
            self.num = 3
            time.sleep(0.05)

        elif (target.basket_size - basket_size >= 0 and target.basket_size > 1200):
            motor.MoveContinuous(-800+correct[0],0+correct[1],0+correct[2],100,100,2)
            print( "--------------------go back bbbbbb from basket-------------------- ",target.basket_size)
            self.num = 3
            time.sleep(0.05)   
        elif (target.basket_size - basket_size >= 0 and target.basket_size < 1200)  :
            motor.MoveContinuous(-500+correct[0],0+correct[1],0+correct[2],100,100,2)#!!!!!!!!!!!!!!!
            print( "--------------------go back ssssss from basket-------------------- ",target.basket_size)
            self.num = 3
            time.sleep(0.05) 
        
    def MoveContinuous(self ,ExpX ,ExpY ,ExpTheta ,AddX ,AddY ,AddTheta) :
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

        if abs(self.NowTheta - ExpTheta) < AddTheta:
            self.NowTheta = ExpTheta
        else:
            if self.NowTheta < ExpTheta :
                self.NowTheta += AddTheta
            elif self.NowTheta > ExpTheta :
                self.NowTheta -= AddTheta
            else:
                pass

        print("NowX, NowY, NowTheta = ", self.NowX, ", ", self.NowY, ", ", self.NowTheta)
        send.sendContinuousValue(self.NowX, self.NowY, 0, self.NowTheta , 0)



    def test_distance(self):
        send.drawImageFunction(4,1,target.basket_x_min ,target.basket_x_max ,target.basket_y_min ,target.basket_y_max,0,0,0)
        target.basket_parameter()
        # target.ball_parameter()
        #if target.basket_size >= 300 and abs(motor.head_horizon-2048) < 600:
        # print("Basket Y = ",target.basket_size , "Basket vert = ",motor.head_vertical ,target.basket_size)
        print("Basket Y = ",target.basket_size , "Basket vert = ",motor.head_vertical ,target.basket_size,target.basket_y_max)
        if abs(target.basket_x - 160) > 5  or abs(target.basket_y - 120) > 4 :
            motor.trace_revise(target.basket_x,target.basket_y,25)
            target.basket_parameter() 
            
            time.sleep(0.05) 
            
        else :
            print("Basket Y = ",target.basket_size , "Basket vert = ",motor.head_vertical)
            print(target.basket_size - 930)

    # def test_distance(self):
    #     send.drawImageFunction(4,1,target.ball_x_min ,target.ball_x_max ,target.ball_y_min ,target.ball_y_max,0,0,0)
    #     target.ball_parameter()
    #     #if target.basket_size >= 300 and abs(motor.head_horizon-2048) < 600:

    #     if abs(target.ball_x - 160) > 5  or abs(target.ball_y - 120) > 4 :
    #         motor.trace_revise(target.ball_x,target.ball_y,25)
    #         target.ball_parameter() 
            
    #         time.sleep(0.05) 
            
    #     else :
    #         print("Basket Y = ",target.basket_size , "Basket vert = ",motor.head_vertical)        
                


    def throw_ball_strength (self):
        target.basket_parameter()
        if self.distance_new >= 55 and self.distance_new < 60 : #55cm
            self.throw_strength = round(abs( 40 * (-(self.distance_new - 60)/5) + 45 * ((self.distance_new - 55)/5)))
        
        elif self.distance_new >= 60 and self.distance_new < 65 : #60cm
            self.throw_strength = round(abs( 50 * (-(self.distance_new - 65)/5) + 55 * ((self.distance_new - 60)/5)))
        
        elif self.distance_new >= 65 and self.distance_new < 70 : #65cm)
            self.throw_strength = round(abs( 50 * (-(self.distance_new - 70)/5) + 55 * ((self.distance_new - 65)/5)))
        
        elif self.distance_new >= 70 and self.distance_new < 75 : #70cm
            self.throw_strength = round(abs( 50 * (-(self.distance_new - 75)/5) + 302 * ((self.distance_new - 70)/5)))
        
        elif self.distance_new >= 75 and self.distance_new < 80 : #70cm
            self.throw_strength = round(abs( 302 * (-(self.distance_new - 80)/5) + 305 * ((self.distance_new - 75)/5)))
        
        elif self.distance_new >= 80 and self.distance_new < 85 : #70cm
            self.throw_strength = round(abs( 305 * (-(self.distance_new - 85)/5) + 311 * ((self.distance_new - 80)/5)))
        
        elif self.distance_new >= 85 and self.distance_new < 90 : #70cm
            self.throw_strength = round(abs( 318 * (-(self.distance_new - 90)/5) + 311 * ((self.distance_new - 85)/5)))
        
        elif self.distance_new >= 90 and self.distance_new < 95 : #70cm
            self.throw_strength = round(abs( 345 * (-(self.distance_new - 95)/5) + 319 * ((self.distance_new - 90)/5)))

        elif self.distance_new >= 95 and self.distance_new < 100 : #70cm
            self.throw_strength = round(abs( 360 * (-(self.distance_new - 100)/5) + 322 * ((self.distance_new - 95)/5))) #319   322

        elif self.distance_new >= 100 and self.distance_new < 105 : #70cm
            self.throw_strength = round(abs( 365 * (-(self.distance_new - 105)/5) + 324 * ((self.distance_new - 100)/5)))

        elif self.distance_new >= 105 and self.distance_new < 110 : #70cm
            self.throw_strength = round(abs( 365 * (-(self.distance_new - 110)/5) + 330 * ((self.distance_new - 105)/5)))

        self.start_get_point = True
        print("throw_strength",self.throw_strength)


    def basket_distance(self,six,nine):
        print("target.basket_y - 120",target.basket_y - 120)
        
        target.basket_parameter()
        if abs(target.basket_y - 120) > 3 :
            motor.trace_revise(target.basket_x,target.basket_y,30)
    
        elif abs(target.basket_y - 120) <= 3  :
            time.sleep(1.3)
            self.sixty_distance = sqrt(abs((3600*six)/target.basket_size))
            self.ninty_distance = sqrt(abs((8100*nine)/target.basket_size))
            # if ( six + nine ) / 2 > target.basket_size :
            #     self.distance_new = abs(self.sixty_distance)
            # else :
            self.distance_new = abs(self.ninty_distance)
            motor.throw_ball_strength()
            print("Basket vertical = ",motor.head_vertical)
            print("Basket size = ",target.basket_size)
            print("Distance_60 = ",self.sixty_distance)
            print("Distance_90 = ",self.ninty_distance)
            print("Distance_fin = ",self.distance_new)


    def bodyauto_close(self,next_state):
        #if send.is_start == True :   # 0 stop  1 go
        if self.now_state == next_state :
            
            pass
        elif self.now_state != next_state :
            send.sendBodyAuto(0,0,0,0,1,0)
            self.now_state = next_state
                

        
       
   
# .........................................
    # def body_trace_straight(self,ball_size,y_target) :
        
    #     if ball_size - y_target  < -60:
    #         send.sendContinuousValue(1,0,0,0,0)
    #         print( "go ahead sizemotor.found == False= ",ball_size)
    #         time.sleep(0.3)
    #     elif abs(ball_size - y_target) <  60:
    #         send.sendContinuousValue(0,0,0,0,0)
    #         print( "stop = ",self.x_body_rotate)
    #         time.sleep(0.3)
    #     elif ball_size - y_target  > 60:
    #         send.sendContinuousValue(-1,0,0,0,0)
    #         print( "go back size= ",ball_size)
    #         time.sleep(0.3)
    #     else :send.sendBodyAuto(0,0,0,0,1,0)
            
        
    # def trace_revise(self,horzine_target,head_vertical_target,speed,delay) :
    #     if (120 - horzine_target) < -10 :
    #         self.move_head(2,self.head_horizon,speed)
    #         self.head_horizon = self.head_horizon - speed
    #         time.sleep(delay) 
    #     elif (120 - horzine_target)  > 10 :
    #         self.move_head(2,self.head_horizon,speed)
    #         self.head_horizon = self.head_horizon + speed
    #         time.sleep(delay)
    #     else :
    #         time.sleep(delay)print("---------again to find the ball-----------------")


if __name__ == '__main__' :
    
    target = target_location()
    motor = motor_move()
    step ='begin'

    #step  = ['begin','find_ball','open_ball_trace','walk_to_ball','ball_trace','catch_ball','find_basekt','basket_trace','walk_to_basket','find','waisting','finish']
    
    #           0              1                2           3             4                5             6           7               8      9           10
    
    # sw = 2
    gazebo_robot = 1
    # 0 for gazebo 1 for robot
    stategy_or_test = 1
    # 0 for test 1 for stategy

    basket_size_60_90 =[2116, 725] #sector 111   left side 1978, 899 right side  2140, 961
    five_point_degree = [1900]# left side 1960 right side  1940   too left-big too right-small
    throw_plus = 1 #line  0   left side 0 right side  4

    throw_ball_point = [0,833,1550] #投籃點 #strength left 1054 right 1156
    #                    size,degree
    ball_catch_size =[1540] #line  1650 ＃球大小
    # # for size          三分  五分  灌籃
    # throw_ball_point = [0,0,1300] 
    # for degree          三分  五分  灌籃
    
    correct       = [0,0,0]
    left_correct  = [0,0,3]
    right_correct = [0,0,-5]
    #                  x , y , theta   

    basket_error = [70,100,60]
    #  for size    三分  五分  灌籃
    # basket_error = [0,0,100]
    # for degree          三分  五分  灌籃

    ball_correct = [50,80]

    trace_parameter =[80]#25
    too_big = True
    
    try:
        
        
        while not rospy.is_shutdown():
            motor.switch_control()
            sw = motor.switch_flag
            
            if send.is_start==True :
                send.drawImageFunction(1,0,160,160,0,240,255,255,255) 
                send.drawImageFunction(2,0,0,320,120,120,255,255,255)
                send.drawImageFunction(3,1,target.ball_x_min ,target.ball_x_max ,target.ball_y_min ,target.ball_y_max,255,0,255)
                send.drawImageFunction(4,1,target.basket_x_min ,target.basket_x_max ,target.basket_y_min ,target.basket_y_max,0,0,0)
                target.ball_parameter() 


                if motor.found == False  :
                    print("head head sdfghjkl;gfdsdfghjkl;",motor.head_vertical)
                    if step == 'begin':
                        send.sendBodySector(9) #讓手回歸自我們的初始手部位置,原是AR的
                        time.sleep(0.7)
                        #send.sendBodySector(8910)
                        time.sleep(0.2)
                        step = 'find_ball'

                    elif step == 'find_ball':#@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
                        
                        if target.ball_size <= 350 :
                            motor.view_move(2428,1668,1800,1200,40,0.05)                 
                            print("start to find the ball")
                            print("stop====\n")
                            target.ball_parameter()  
                            print("  ball => x:",target.ball_x," y:",target.ball_y," size:",target.ball_size)
                        elif target.ball_size > 350 :
                            
                            step = 'open_ball_trace'#@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
                            

                    elif step == 'open_ball_trace' :#@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
                        if abs(target.ball_x - 160) > 6  or abs(target.ball_y - 120) > 8 :
                            target.ball_parameter()
                            print("ball siz",target.ball_size) 
                            print("open_ball_trace is opening")
                            motor.trace_revise(target.ball_x,target.ball_y,25)
                            time.sleep(0.05) 
                        else :
                            print("motor.head_vertical",motor.head_vertical)
                            print("motor.head_horizon",motor.head_horizon)
                            # time.sleep(8)

                            # if (1740 <= motor.head_vertical < 1830) and  (1698 <= motor.head_horizon <=2398):
                            # if (ball_catch_size[0]+10 <= motor.head_vertical <ball_catch_size[0]+130) and  (1698 <= motor.head_horizon <=2398):
                            #     motor.temp = motor.head_horizon
                            #     motor.move_head(1,1850,880,880,50) #1748
                            #     time.sleep(0.1)
                            #     send.sendBodySector(55)    #2222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222
                            #     time.sleep(0.2)
                                
                            #     target.ball_parameter()
                            #     time.sleep(2)
                                
                            #     motor.directly = True
                            #     motor.dir_num = 2
                            #     step = 'walk_to_ball'

                            # # elif (1670 < motor.head_vertical < 1740) and  (1698 <= motor.head_horizon <=2398):
                            # elif (ball_catch_size[0]-30 < motor.head_vertical <ball_catch_size[0]+10) and  (1698 <= motor.head_horizon <=2398):
                            #     motor.temp = motor.head_horizon
                            #     motor.move_head(1,1850,880,880,50) #1748
                            #     time.sleep(0.1)
                            #     send.sendBodySector(5)    #2222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222
                            #     time.sleep(0.2)
                            #     print("sjkcnqnvklqmeveqvmqeibmovqpe,obinbqmo,pqeqbqoinkeob")
                            #     target.ball_parameter()
                            #     time.sleep(2)
                                
                            #     motor.directly = True
                            #     motor.dir_num = 1
                            #     step = 'walk_to_ball'
                            
                            # # elif (1645 <= motor.head_vertical <1670) and  (1698 >= motor.head_horizon or motor.head_horizon >=2398):
                            # elif (ball_catch_size[0]-55 <= motor.head_vertical <ball_catch_size[0]-30) and  (1698 >= motor.head_horizon or motor.head_horizon >=2398):
                            #     motor.temp = motor.head_horizon
                            #     motor.move_head(1,1850,880,880,50) #1748
                            #     time.sleep(0.1)
                            #     send.sendBodySector(5872)    #2222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222
                            #     time.sleep(0.2)
                            #     print("dfghjkl;dfghjkbhuwfneuinvieuqnviquenviequnveiqnviueqnvuenvie")
                            #     target.ball_parameter()
                            #     time.sleep(2)
                                
                            #     motor.directly = True
                            #     motor.dir_num = 1
                            #     step = 'walk_to_ball'

                                
                            if (motor.head_vertical <= ball_catch_size[0]) or ((motor.head_vertical < ball_catch_size[0]-55) and  (1698 >= motor.head_horizon or motor.head_horizon >=2398)):
                                too_big = True
                                print("bigbigbig")
                                time.sleep(0.3)
                                print("-------------start walk to the ball--------------")
                                motor.bodyauto_close(1)
                                step = 'walk_to_ball'#@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
                                
                            else:
                                too_big = False
                                print("smallsmallsmall")
                                time.sleep(0.3)
                                print("-------------start walk to the ball--------------")
                                print("^ↀᴥↀ^")
                                motor.bodyauto_close(1)
                                step = 'walk_to_ball'#@@@@@@@@@@@@@@@@@@@@@@@
                            
                                

                    elif  step == 'walk_to_ball' :#@@@@@@@@@@@@@@@@@@
                        print("motor.temp",motor.temp)

                        if motor.directly == True  and motor.temp > 2048 :
                            motor.waist_position = 2048+round((motor.temp-ball_catch_size[0])*0.4)
                            send.sendSingleMotor(9,round((motor.temp-ball_catch_size[0])*0.4),5)
                            time.sleep(4)
                            print("ubwuivbwuivb")
                            target.ball_parameter() 
                            motor.found = True
                            step = 'ball_trace'

                        # if motor.directly == True  and target.ball_size <= 200 :
                            # motor.clock = motor.clock - 1
                            # if motor.clock > 0 :
                            #     print("tcvyubhbgc")
                            #     time.sleep(1)
                            # elif motor.clock <= 0 and motor.waist_position < 2548:
                            #     print("hothothothto")
                            #     motor.waist_reset(motor.waist_position + 10,30)
                            #     time.sleep(0.3)

                        else :
                            if motor.directly == True :
                                
                                print("DFGHJDFGHDFGHJ")
                                target.ball_parameter() 
                                time.sleep(2)
                                motor.found = True#@@@@@@@@@@@@@@@@@@
                            step = 'ball_trace'

                    elif  step == 'ball_trace' :#@@@@@@@@@@@@@@@@@@
                        
                            
                        target.ball_parameter()   
                        print(" ball => x:",target.ball_x," y:",target.ball_y," size:",target.ball_size)                   
                        motor.trace_revise(target.ball_x,target.ball_y,25)
                        print("abs(motor.x_body_rotate)",abs(motor.x_body_rotate),motor.head_horizon-2048)

                        if too_big == True:
                            motor.MoveContinuous(-1200+correct[0],0+correct[1],0+correct[2],100,100,1)
                            print("meowmeowmeowmeowmeow")

                            if motor.head_vertical >=ball_catch_size[0]+85 :
                                too_big = False


                        
                        if too_big == False:
                            if abs(motor.head_horizon-2048) > trace_parameter[0]  :  
                                motor.body_trace_rotate(trace_parameter[0])    
                                print("motor.head_vertical=========",motor.head_vertical)
                            
                            elif abs(motor.head_vertical - ball_catch_size[0]) > 5 : #1320是條球的距離150是誤差
                                print(motor.head_horizon - 2048)
                                motor.body_trace_straight(ball_catch_size[0],ball_correct[gazebo_robot])#!!!!!!!!!!!!!!!!!!!!!!!!!!!球的距離夠motor.found = True
                                print("motor.head_vertical-1320 = ",motor.head_vertical-ball_catch_size[0])
# ............................................................................................................................
# ............................................................................................................................
# ............................................................................................................................
# ............................................................................................................................

                elif  motor.found == True and motor.catch == False : 
                    target.basket_parameter() 
                    target.ball_parameter() 
                    if step == 'ball_trace' :#@@@@@@@@@@@@@@@@@@
                        time.sleep(0.03)
                        target.ball_parameter()  
                        if abs(target.ball_x-160) < 2 :
                                
                                print("step======",step)
                                step = 'catch_ball'#@@@@@@@@@@@@@@@@@@

                        elif target.ball_x != 0 :
                            target.ball_parameter()
                            motor.WaistFix(target.ball_x,target.ball_y,160,120)
                            print("abs(target.ball_x-160)hhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhh",abs(target.ball_x-160))
                    

                    elif step == 'catch_ball' :   #@@@@@@@@@@@@@@@@@@
                        
                        time.sleep(1)   
                        # send.sendBodySector(2)    #1111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111
                       
                        if motor.directly == False :
                            send.sendBodySector(6)    #2222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222
                            print("fuckvbhubruiefe...")
                            time.sleep(1)
                        elif motor.directly == True and motor.dir_num == 2:
                            print("fuckefef...")
                            send.sendBodySector(6873)
                            time.sleep(1)  
                        elif motor.directly == True and motor.dir_num == 1:
                            print("fuckefef...")
                            send.sendBodySector(6)
                            time.sleep(1)       
                        print("stop to the ball")
                        print("----------------------------------ready to waist_reset---------------------------------------")
                        
                        
                        motor.waist_reset(2048,70)
                        time.sleep(3)  
                        if motor.directly == False :
                            send.sendBodySector(7)    #2222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222
                            print("fuckvbhubrui...")
                            time.sleep(2)
                        elif motor.directly == True and motor.dir_num == 2 :
                            print("fuck...")
                            send.sendBodySector(7873)
                            time.sleep(2)
                        elif motor.directly == True and motor.dir_num == 1 :
                            print("fuck...")
                            send.sendBodySector(7)
                            time.sleep(2)
                        time.sleep(3)
                        # target.basket_size = 0
                        print(".................................................")
                        step = 'find_basekt'#@@@@@@@@@@@@@@@@@@
                                                                        

                    elif step == 'find_basekt' :#@@@@@@@@@@@@@@@@@@
                        target.basket_parameter()
                        if target.basket_size < 800 :
                                
                                print("find_basket",target.basket_size)
                                motor.view_move(2548,1548,2048,1948,50,0.04)

                                target.basket_parameter()
                                print("  basket => x:",target.basket_x," y:",target.basket_y," size:",target.basket_size)
                        elif target.basket_size > 800 :
                                step = 'basket_trace'#@@@@@@@@@@@@@@@@@@
                                time.sleep(1)
                                target.basket_parameter()
                                print("jump to basket_trace   !!!!!!!!!")
                                

                    elif step == 'basket_trace' :
                        if abs(target.basket_x - 160) > 12  or abs(target.basket_y - 120) > 9 :
                            target.basket_parameter() 
                            motor.trace_revise(target.basket_x,target.basket_y,35)
                            print("-------------basekt_trace有進喔--------------")
                            time.sleep(0.09) 

                        elif motor.directly == True:

                            if  (abs(target.basket_x - 160) < 12  or abs(target.basket_y - 120) < 9) and sw == 1:
                                print( "--------------------stop at the basket----------------------",target.basket_size)
                                #send.sendBodyAuto(0,0,0,0,1,0)
                                target.basket_parameter()
                                
                                time.sleep(1)
                                send.sendBodySector(5301)
                                print("-------------------------send.sendBodySector(3)------------------------------")
                                time.sleep(2)
                                send.sendBodySector(5595)
                                time.sleep(1)
                                motor.catch = True
                                motor.move_head(1,five_point_degree[0],880,880,50)
                                motor.move_head(2,2048,880,880,50)
                                time.sleep(2)
                                step = 'walk_to_basket'
                            elif (abs(target.basket_x - 160) < 12  or abs(target.basket_y - 120) < 9) and sw != 1:
                                motor.bodyauto_close(1)                
                                step = 'walk_to_basket'

                        elif motor.directly == False and (abs(target.basket_x - 160) < 12  or abs(target.basket_y - 120) < 9):
                            print("SDFGHJKLERTYU")                        
                            motor.bodyauto_close(1)                
                            step = 'walk_to_basket' #@@@@@@@@@@@@@@@@@@


                    elif  step == 'walk_to_basket' :#@@@@@@@@@@@@@@@@@@
                        print("walk_to_basket")
                        target.basket_parameter()
                        print(" basket => x:",target.basket_x," y:",target.basket_y," size:",target.basket_size)
                        motor.trace_revise(target.basket_x,target.basket_y_max,35)

                        


                        if abs(motor.head_horizon-2048) > 80 :#!!!!
                            target.basket_parameter()
                            if sw == 0 or sw == 1:
                                motor.body_trace_rotate(40)
                                print("target.basket_size ==",target.basket_size)
                                print("throw_ball_point ==",throw_ball_point)
                                print("",abs(target.basket_size - throw_ball_point[sw]))
                            elif sw == 2 :
                                motor.body_trace_rotate(80)
                                print("target.basket_size ==",target.basket_size)
                                print("throw_ball_point ==",throw_ball_point)
                                print("",abs(target.basket_size - throw_ball_point[sw]))
                            

                        elif abs(motor.head_horizon-2048) <= 80 :#!!!!!!!!!!!!!!!要調整！！！！！！！！！！！！！！！！！！！！！！！
                            if sw == 2:
                                print("----------------start the slam slam slam --------------")
                                motor.body_trace_basket_straight_2(throw_ball_point[sw],basket_error[sw]) #接近匡catch=true
                                print("motor.catch = ",motor.catch)
                                if motor.catch == True :
                                    print("step is ",step)#@@@@@@@@@@@@@@@@@@@@@@@@

                            elif sw == 0:
                                print("----------------start the action of get point--------------")
                                motor.body_trace_basket_straight_3(throw_ball_point[sw],basket_error[sw])
                                print("motor.catch = ",motor.catch)
                                if motor.catch == True :
                                    print("step is ",step)#@@@@@@@@@@@@@@@@@@@@@@
                                    motor.move_head(1,five_point_degree[0],880,880,50)
                                    motor.move_head(2,2048,880,880,50)
                                    time.sleep(2)

                            elif sw == 1:
                                print("----------------start the action of get point--------------")
                                motor.body_trace_basket_straight_5(throw_ball_point[sw],basket_error[sw])
                                print("motor.catch = ",motor.catch)
                                if motor.catch == True :
                                    print("step is kkkk",step)#@@@@@@@@@@@@@@@@@@@@@@@
                                    motor.move_head(1,five_point_degree[0],880,880,50)
                                    motor.move_head(2,2048,880,880,50)
                                    time.sleep(2)
                                

                    
                    
                
                elif  motor.found == True and motor.catch == True :
                    if sw == 2 :
                        if step == 'walk_to_basket' :
                            target.basket_parameter()
                            if target.basket_x != 0 :
                                motor.WaistFix(target.basket_x,target.basket_y,160,120)
                                print("abs(target.basket_x-160)",abs(target.basket_x-160))
                                if abs(target.basket_x-160) < 3:
                                    step = 'find'#@@@@@@@@@@@@@@@@@@@@@
                                    print("step======",step)#@@@@@@@@@@@@@@@@@

                        elif step == 'find' :
                            time.sleep(1)
                            send.sendBodySector(987)
                            # send.sendBodySector(4) #上籃之動作二（把球放入籃框）1111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111
                            print("2222222222")
                            # if  abs(motor.head_horizon - 2048) <= 5  and abs(target.basket_x - 160) <= 1 :
                            #     print("---------------gogogogogogogogogogogogogogoro-------------")
                            step = 'waisting' #都一直線#@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@


                    elif sw == 0:
                        if step == 'walk_to_basket' :
                            time.sleep(0.03)
                            target.basket_parameter()
                            print("sdfghjkl;",target.basket_x,target.basket_size)
                                
                            if target.basket_x != 0 :
                                motor.WaistFix(target.basket_x,target.basket_y,160,120)
                                print("abs(target.basket_x-160)",abs(target.basket_x-160))
                                if abs(target.basket_x-160) < 1:
                                    step = 'find'#@@@@@@@@@@@@@@@@@@@@@
                                    print("step======",step)#@@@@@@@@@@@@@@@@@

                        elif step == 'find' :#@@@@@@@@@@@@@@@@@@@@@@@@
                            time.sleep(0.05)
                            motor.basket_distance(basket_size_60_90[0],basket_size_60_90[1])
                            print("throw_strength = ", motor.throw_strength)
                            if motor.start_get_point == True :
                                if motor.directly == False :
                                    send.sendSingleMotor(9,-round((motor.distance_new-90)*1.8),15)
                                    time.sleep(2)
                                step = "waisting"                           

                        elif step == 'waisting' :#@@@@@@@@@@@@@@@@@@@@@@@@
                            
                            time.sleep(0.1)
                            
                            send.sendBodySector(5502)
                            time.sleep(2)
                            motor.throw_strength = motor.throw_strength - throw_plus
                            send.sendHandSpeed(503,motor.throw_strength + throw_plus )
                            print("sdfghjkl;")
                            time.sleep(2)
                            send.sendBodySector(503)
                            print("tttttt",motor.throw_strength + throw_plus)


                            step ="finish"
                            
                            
                    

                    elif sw == 1:
                        
                        if step == 'walk_to_basket' :
                            time.sleep(0.03)
                            target.basket_parameter()
                            print('aaaaaa')
                            print("sdfghjkl;",target.basket_x,target.basket_size)
                                
                            if target.basket_x != 0 :
                                motor.WaistFix(target.basket_x,target.basket_y,160,120)
                                print("abs(target.basket_x-160)",abs(target.basket_x-160))
                                if abs(target.basket_x-160) < 2:
                                    step = 'find'#@@@@@@@@@@@@@@@@@@@@@
                                    print("step======",step)#@@@@@@@@@@@@@@@@@

                        elif step == 'find' :#@@@@@@@@@@@@@@@@@@@@@@@@
                            time.sleep(0.05)
                            motor.basket_distance(basket_size_60_90[0],basket_size_60_90[1])
                            print("throw_strength = ", motor.throw_strength)
                            if motor.start_get_point == True :
                                if motor.directly == False :
                                    send.sendSingleMotor(9,-round((motor.distance_new-90)*1.8),15)
                                    print("ASDFGHJ")
                                    time.sleep(2)
                                step = "waisting"                          

                        elif step == 'waisting' :#@@@@@@@@@@@@@@@@@@@@@@@@
                            
                            time.sleep(0.1)
                            
                            send.sendBodySector(5502)
                            time.sleep(2)
                            # motor.throw_strength = motor.throw_strength - throw_plus
                            send.sendHandSpeed(503,motor.throw_strength + throw_plus )
                            print("sdfghjkl;")
                            time.sleep(2)
                            send.sendBodySector(503)
                            print("tttttt",motor.throw_strength + throw_plus)


                            step ="finish"

                
                
            else :
                if step != 'begin' and (send.DIOValue == 8 or send.DIOValue == 9 or send.DIOValue ==10 or send.DIOValue ==12 or send.DIOValue ==13
                                     or send.DIOValue ==11 or send.DIOValue ==15 or send.DIOValue ==24 or send.DIOValue ==27 or send.DIOValue ==31):

                    motor.bodyauto_close(0)
                    target = target_location()
                    motor = motor_move()
                    step = 'begin'
                    situation = 0
                    send.sendHeadMotor(1,2048,30)
                    send.sendHeadMotor(2,2048,30)
                    time.sleep(0.05)
                    send.sendBodySector(29)
                    print("-------------------reset and stoping-------------------------")
                    print("-------------------reset and stoping-------------------------")
                    print("好棒棒")
                    # print("")
                    # print("◢███◣。。。。。。◢███◣" )
                    # print("▇▇□██。。。。。。██□██")
                    # print("  ◥███◤◢████◣◥███◤")
                    # print("◢█。。。。。。。。。。█◣")
                    # print("█。╔╗。。。。。。。╔╗。█")
                    # print("█。∥●。。。╭╮。。。∥●。█")
                    # print("█。╚╝。。。╰╯。。。╚╝。█")
                    # print("   ◥▆▆▆▆▆▆▆▆▆▆▆▆▆▆")
                    time.sleep(0.05)
                    print("\n")
                    print(" ┌─╮◆╭═┐╭═┐╭═┐◆╭─┐")

                    print("│┌╯◆║加║║油║║囉║◆╰┐│")

                    print(" ╰╯↘◆└═╯└═╯└═╯◆↙╰╯")
                    
            

                    print("\n")
                    print("..../\„./\...../\„./\ ")             
                    print("... (=';'=)....(=';'=) ♥♥")              
                    print("..../*♥♥**\ ♥  /*♥♥**\ ")
                    print(".(.| |..| |.)(.| |..| |.)♥")

                    motor.switch_control()

                    

                            
                
            
                

    except rospy.ROSInterruptException:
        pass