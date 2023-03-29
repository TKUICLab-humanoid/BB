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
            if send.color_mask_subject_size [5][j] > 400 and send.color_mask_subject_size [5][j] > self.basket_size:
                self.basket_x =  send.color_mask_subject_X [5][j]
                self.basket_y = send.color_mask_subject_Y [5][j]
                self.basket_size = send.color_mask_subject_size [5][j]
                self.basket_x_min = send.color_mask_subject_XMin[5][j] 
                self.basket_y_min = send.color_mask_subject_YMin[5][j] 
                self.basket_x_max = send.color_mask_subject_XMax[5][j] 
                self.basket_y_max =send.color_mask_subject_YMax[5][j]

        
               
   

class motor_move():
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
        self.stop = False
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
            time.sleep(0.2)                                     #籃框               上下下下 or 下下上下 or 上下上下
            self.switch_reset_flag = 1 #sector 111  
            self.in_tacties = 0

        elif    send.DIOValue == 13:                                                  #球                 上下up下
            print("Basket Y = ",target.basket_size , "Basket vert = ",motor.head_vertical)
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
        if x_target != 0 and y_target != 0:
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
            motor.MoveContinuous(1500+correct[0],0+correct[1],0+correct[2],100,100,2) #!!!!!!!!!!!!!!!!!!!!!!
            print( "go ahead bigbigbigbigbigbigbig= ",self.head_vertical)
            time.sleep(0.05)
        elif (self.head_vertical - degree) - 150 < ball_degree and (self.head_vertical - degree)  > ball_degree:
            motor.MoveContinuous(1000+correct[0],0+correct[1],0+correct[2],100,100,2) #!!!!!!!!!!!!!!!!!!!!!!
            print( "go ahead smallsmallsmallsmall= ",self.head_vertical)
            time.sleep(0.05)
        elif (self.head_vertical - degree) < -ball_degree :
            motor.MoveContinuous(-1500+correct[0],0+correct[1],0+correct[2],100,100,2)
            print( "go back = ",self.head_vertical)
            time.sleep(0.05)
        elif  abs(self.head_vertical - degree) <= ball_degree :
            motor.bodyauto_close(0)
            self.stop = True
            
            
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
        
        if self.head_vertical - degree > basket_error  and self.head_vertical > 1990 :
            motor.MoveContinuous(2500+correct[0],0+correct[1],0+correct[2],75,100,2)#!!!!!!!!!!!!!!!
            print( "--------------------go ahead bbbb to basket---------------------  ",self.head_vertical)
            time.sleep(0.05)

        elif self.head_vertical - degree > basket_error and  self.head_vertical < 1990:
            motor.MoveContinuous(600+correct[0],0+correct[1],0+correct[2],150,100,2)#!!!!!!!!!!!!!!!
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
            self.stop = True
            
            
    
     #####################################################################################################

    def body_trace_basket_straight_3(self,basket_size,basket_error) :
        
        if target.basket_size - basket_size < -basket_error and target.basket_size > 2100:
            send.sendContinuousValue(300+correct[0],0+correct[1],0,0+correct[2],0)#!!!!!!!!!!!!!!!
            print( "--------------------go ahead to basket---------------------  ",self.head_vertical)
            time.sleep(0.05)
        elif target.basket_size - basket_size < -basket_error and target.basket_size < 2100:
            send.sendContinuousValue(1200+correct[0],0+correct[1],0,0+correct[2],0)#!!!!!!!!!!!!!!!
            print( "--------------------go ahead to basket---------------------  ",self.head_vertical)
            time.sleep(0.05)

        elif target.basket_size - basket_size > basket_error and target.basket_size > 2400:
            send.sendContinuousValue(-1200+correct[0],0+correct[1],0,0+correct[2],0)#!!!!!!!!!!!!!!!!!
            print( "--------------------go back from basket-------------------- ",self.head_vertical)
            time.sleep(0.05)   
        elif target.basket_size - basket_size > basket_error and target.basket_size < 2400:
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
    
    

    def body_trace_basket_straight_5(self,basket_size,basket_error) :
        if target.basket_size - basket_size < basket_error  and (basket_size > target.basket_size > (basket_size - basket_error)) :
            self.num = self.num - 1
            if self.num <= 0 :
                motor.bodyauto_close(0)
                self.stop = True
                
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
            self.throw_strength = round(abs( 311 * (-(self.distance_new - 90)/5) + 313 * ((self.distance_new - 85)/5)))
        
        elif self.distance_new >= 90 and self.distance_new < 95 : #70cm
            self.throw_strength = round(abs( 313 * (-(self.distance_new - 95)/5) + 319 * ((self.distance_new - 90)/5)))

        elif self.distance_new >= 95 and self.distance_new < 100 : #70cm
            self.throw_strength = round(abs( 319 * (-(self.distance_new - 100)/5) + 322 * ((self.distance_new - 95)/5))) #319   322

        elif self.distance_new >= 100 and self.distance_new < 105 : #70cm
            self.throw_strength = round(abs( 322 * (-(self.distance_new - 105)/5) + 324 * ((self.distance_new - 100)/5)))

        elif self.distance_new >= 105 and self.distance_new < 110 : #70cm
            self.throw_strength = round(abs( 324 * (-(self.distance_new - 110)/5) + 330 * ((self.distance_new - 105)/5)))

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
                

