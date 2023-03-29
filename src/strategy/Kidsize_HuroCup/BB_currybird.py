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
        
        
    def ball_parameter(self):    #利用色模建籃球
        self.color_mask_subject_orange = send.color_mask_subject_cnts[0]
        self.ball_x = 0
        self.ball_y = 0
        self.ball_size = 0
        for j in range (self.color_mask_subject_orange):   #將所有看到的橘色物件編號
            if 310 > send.color_mask_subject_X [0][j] > 10 and 230 > send.color_mask_subject_Y [0][j] > 10:

                if send.color_mask_subject_size [0][j] > 350  and send.color_mask_subject_size [0][j] > self.ball_size: #用大小過濾物件
                    self.ball_x =  send.color_mask_subject_X [0][j]
                    self.ball_y = send.color_mask_subject_Y [0][j]
                    self.ball_size = send.color_mask_subject_size [0][j]
                    self.ball_x_min = send.color_mask_subject_XMin[0][j] 
                    self.ball_y_min = send.color_mask_subject_YMin[0][j] 
                    self.ball_x_max = send.color_mask_subject_XMax[0][j] 
                    self.ball_y_max =send.color_mask_subject_YMax[0][j]       

            
    
    def basket_parameter(self): #利用色模建籃框
        self.color_mask_subject_red = send.color_mask_subject_cnts[5] 
        self.basket_x= 0
        self.basket_y = 0
        self.basket_size = 0
        for j in range (self.color_mask_subject_red):     #將所有看到的紅色物件編號
            if send.color_mask_subject_size [5][j] > 500 and send.color_mask_subject_size [5][j] > self.basket_size:   #用大小過濾物件(濾雜訊)
                self.basket_x =  send.color_mask_subject_X [5][j]
                self.basket_y = send.color_mask_subject_Y [5][j]
                self.basket_size = send.color_mask_subject_size [5][j]
                self.basket_x_min = send.color_mask_subject_XMin[5][j] 
                self.basket_y_min = send.color_mask_subject_YMin[5][j] 
                self.basket_x_max = send.color_mask_subject_XMax[5][j] 
                self.basket_y_max =send.color_mask_subject_YMax[5][j]

        
               
   

class motor_move():

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
        self.reg = 2048
        self.ready_ball_catch = False
        self.ready_basket_dunk = False
        self.reg_use = False
        
    ######################################## switch #######################################

    # def switch_control(self):                                                       #need test
        
    #     if    send.DIOValue == 9 or send.DIOValue == 12 :  
    #         motor.test_distance()                
    #         time.sleep(0.2)                                     #籃框               上下下下 or 下下上下 or 上下上下
    #         self.switch_reset_flag = 1 #sector 111  
    #         self.in_tacties = 0

    #     elif    send.DIOValue == 13:                                                  #球                 上下up下
    #         print("Basket Y = ",target.basket_size , "Basket vert = ",motor.head_vertical)
    #         self.switch_reset_flag = 1
    #         self.in_tacties = 0
    #         time.sleep(0.03)

    #     elif    (send.DIOValue == 27 or send.DIOValue == 11) and (self.in_tacties != 10 or self.switch_reset_flag == 1) :    
    #         #                   24+3                    8+3 #開啟三分策略  上上下下 
    #         self.switch_flag = 0
    #         print("SW = ", self.switch_flag)
    #         # motor.bodyauto_close(0)
    #         target.ball_parameter()
    #         target.basket_parameter()
    #         send.sendHeadMotor(1,2048,30)
    #         send.sendHeadMotor(2,2048,30)
    #         send.sendBodySector(29)

    #         self.switch_reset_flag = 0
    #         self.in_tacties = 10

    #     elif    (send.DIOValue == 31 or send.DIOValue == 15) and (self.in_tacties != 11 or self.switch_reset_flag == 1) :
    #         #                   24+7                    8+7 #開啟五分策略  上上上下
    #         self.switch_flag = 1
    #         print("SW = ", self.switch_flag)
    #         # motor.bodyauto_close(0)
    #         target.ball_parameter()
    #         target.basket_parameter()
    #         send.sendHeadMotor(1,2048,30)
    #         send.sendHeadMotor(2,2048,30)
    #         send.sendBodySector(29)
    #         self.switch_reset_flag = 0
    #         self.in_tacties = 11
    

    #     elif    (send.DIOValue == 24 or send.DIOValue ==  8) and (self.in_tacties != 12 or self.switch_reset_flag == 1) :
    #         #                   24                      8  #開啟二分策略  下下下下
    #         self.switch_flag = 2
    #         print("SW = ", self.switch_flag)
    #         # motor.bodyauto_close(0)
    #         target.ball_parameter()
    #         target.basket_parameter()
    #         send.sendHeadMotor(1,2048,30)
    #         send.sendHeadMotor(2,2048,30)
    #         send.sendBodySector(29)
    #         self.switch_reset_flag = 0
    #         self.in_tacties = 12
        ######################################## switch #######################################

    def move_head(self,ID,Position,max_head_horizon_size,max_head_vertical_size,Speed):  #把相對頭部變化變絕對(call 2048就變2048)
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

    def waist_rotate(self,waist_x,Speed):
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
   
    
    def trace_revise(self,x_target,y_target,speed) :    #看誤差調整頭的角度(讓頭看向籃框或球)
        if x_target != 0 and y_target != 0:
            self.x_differ =  x_target - 160             
            self.y_differ =  y_target - 120 
            self.x_degree = self.x_differ * (65 / 320)
            self.y_degree = self.y_differ * (38 / 240)
            self.move_head(1, self.head_horizon - round(self.x_degree * 4096 / 360 *0.15),880,880,speed)
            self.move_head(2, self.head_vertical - round(self.y_degree * 4096 / 360 *0.15),880,880,speed)
            time.sleep(0.05)
        else :
            print("miss_target->需重新尋求")

    def body_trace_rotate(self,degree) : #步態旋轉到可拿球的角度
        self.x_body_rotate = self.head_horizon - 2048
        if self.x_body_rotate > degree :
            motor.MoveContinuous(left_correct[0],left_correct[1],left_correct[2],100,100,8)
            print( "rotate left = ",self.x_body_rotate)
            time.sleep(0.05)
        elif self.x_body_rotate < -degree :
            motor.MoveContinuous(right_correct[0],right_correct[1],right_correct[2],100,100,8)#!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            print( "rotate right = ",self.x_body_rotate)
            time.sleep(0.05)
            
        
    ######################################## body_trace_straight 副函式 #######################################

    # def body_trace_straight(self,catch_degree,gait_correct) :   #前進後退至可找拿球的距離
        
    #     if (self.head_vertical - catch_degree) - 150 > gait_correct :  #離遠時的前進速度
    #         motor.MoveContinuous(1500+correct[0],0+correct[1],0+correct[2],100,100,2) 
    #         print( "大前進,self.head_vertical= ",self.head_vertical)
    #         time.sleep(0.05)
    #     elif (self.head_vertical - catch_degree) - 150 < gait_correct and (self.head_vertical - catch_degree)  > gait_correct:    #離近時的前進速度
    #         motor.MoveContinuous(1000+correct[0],0+correct[1],0+correct[2],100,100,2) 
    #         print( "小前進,self.head_vertical= ",self.head_vertical)
    #         time.sleep(0.05)
    #     elif (self.head_vertical - catch_degree) < -gait_correct : #不小心走過頭，後退
    #         motor.MoveContinuous(-1500+correct[0],0+correct[1],0+correct[2],100,100,2)
    #         print( "倒退修正,self.head_vertical= ",self.head_vertical)
    #         time.sleep(0.05)
    #     elif  abs(self.head_vertical - catch_degree) <= gait_correct : #確認到達可夾球距離(改變旗標)
    #         motor.bodyauto_close(0) #步態停止
    #         time.sleep(1.2)
    #         self.ready_ball_catch = True

    ######################################## body_trace_straight_new 副函式 ########################################

    def body_trace_straight_new(self,forward_degree,slow_degree,backward_degree) :   #前進後退至可找拿球的距離
        #前進太多時forward_gait_correct改大
        #後退太多時back_gait_correct改小
            
        if self.head_vertical > forward_degree :  #大前進
                motor.MoveContinuous(1500+correct[0],0+correct[1],0+correct[2],100,100,2) 
                print( "大前進,self.head_vertical= ",self.head_vertical)
                time.sleep(0.05)

        elif slow_degree < self.head_vertical < forward_degree :  #進入減速範圍
            motor.MoveContinuous(1000+correct[0],0+correct[1],0+correct[2],100,100,2)
            print( "進入減速範圍,self.head_vertical= ",self.head_vertical)
            time.sleep(0.05)
    
                
        elif backward_degree <= self.head_vertical <= slow_degree :
            motor.bodyauto_close(0) #步態停止
            time.sleep(1.2)
            self.ready_ball_catch = True
            print( "到達夾球範圍STOP!!,self.head_vertical= ",self.head_vertical)                
            time.sleep(0.05)
    

        elif self.head_vertical < backward_degree: 
            motor.MoveContinuous(-500+correct[0],0+correct[1],0+correct[2],100,100,2)
            print( "大後退,self.head_vertical= ",self.head_vertical)                
            time.sleep(0.05)

    ######################################## body_trace_straight 副函式 ########################################

    def body_trace_straight(self,catch_degree,forward_gait_correct,back_gait_correct) :   #前進後退至可找拿球的距離
        #前進太多時forward_gait_correct改大
        #後退太多時back_gait_correct改小
            
        print( "catch_degree= ",catch_degree)
        if self.head_vertical > (catch_degree + forward_gait_correct) + 200 :  #未進入減速範圍 #前進太多時forward_gait_correct改大
                motor.MoveContinuous(1500+correct[0],0+correct[1],0+correct[2],100,100,2) 
                print( "大前進,self.head_vertical= ",self.head_vertical)
                time.sleep(0.05)

        elif catch_degree + forward_gait_correct < self.head_vertical < (catch_degree + forward_gait_correct) + 200 : #進入減速範圍
            motor.MoveContinuous(1000+correct[0],0+correct[1],0+correct[2],100,100,2)
            print( "進入減速範圍,self.head_vertical= ",self.head_vertical)
            time.sleep(0.05)
    
                
        elif (catch_degree - back_gait_correct) <= self.head_vertical <= catch_degree + forward_gait_correct :
            motor.bodyauto_close(0) #步態停止
            time.sleep(1.2)
            self.ready_ball_catch = True
            print( "到達夾球範圍STOP!!,self.head_vertical= ",self.head_vertical)                
            time.sleep(0.05)
    

        elif self.head_vertical < (catch_degree + back_gait_correct): 
            motor.MoveContinuous(-500+correct[0],0+correct[1],0+correct[2],100,100,2)
            print( "大後退,self.head_vertical= ",self.head_vertical)                
            time.sleep(0.05)

            
    def WaistFix(self, Target_X, TargetXCenter):#轉腰調整Basket.X與Baskethead_verticalBaseLine的誤差
        
        self.MoveW = TargetXCenter - Target_X
        if self.MoveW > 15:
            self.MoveW = 15
        elif self.MoveW < -15:
            self.MoveW = -15
        
        self.waist_rotate((self.waist_position + self.MoveW), 30)
        self.move_head(2, self.head_vertical ,880,880,20)
        
        time.sleep(0.15)
        # time.sleep(0.2)

    def Null_WaistFix(self, turn_final):#轉腰調整Basket.X與Baskethead_verticalBaseLine的誤差

        if (self.waist_position - 10) < turn_final:
            self.waist_position -= 10
            self.waist_rotate(self.waist_position, 30)
            #self.move_head(2, self.head_vertical ,880,880,20)
            time.sleep(0.15)
        else:
            print("fail")
      

    ######################################## walk_to_basket_dunk 副函式 ######################################## 

    # def walk_to_basket_dunk(self,stop_degree,gait_correct):   

    #     print("walk_to_basket")
    #     target.basket_parameter()
    #     print(" basket => x:",target.basket_x," y:",target.basket_y," size:",target.basket_size)
    #     self.trace_revise(target.basket_x,target.basket_y,35)
        
    #     if abs(self.head_horizon-2048) > 80 :  #80為步態誤差
    #         print("rotate調整")
    #         self.body_trace_rotate(80) #步態走路時的誤差跟籃框遠近的關係  #80
    #         print("target.basket_size = ",target.basket_size)
    #         print("throw_ball_point = ",throw_ball_point)
    #         print("motor.head_horizon-2048 = ",motor.head_horizon-2048)
    #     elif abs(self.head_horizon-2048) <= 80 :
    #         print("motor.head_horizon-2048 = ",motor.head_horizon-2048)
    #         print("motor.head_vertical = ",self.head_vertical)
    #         print("straight調整")

    #         #stop_degree=1200
    #         #gait_correct=100

    #         if  self.head_vertical > 1600 + 100 :  #未進入減速範圍
    #             motor.MoveContinuous(2500+correct[0],0+correct[1],0+correct[2],75,100,2)#!!!!!!!!!!!!!!!
    #             print("未進入減速範圍->大前進 ",self.head_vertical)
    #             time.sleep(0.05)

    #         elif self.head_vertical < 1600 + 100 :  #進入減速範圍
    #             motor.MoveContinuous(600+correct[0],0+correct[1],0+correct[2],150,100,2)#!!!!!!!!!!!!!!!
    #             print("已進入減速範圍->小前進 ",self.head_vertical)
    #             time.sleep(0.05)    

    #         elif self.head_vertical < 1200 + 100 :
    #             motor.MoveContinuous(-800+correct[0],0+correct[1],0+correct[2],100,100,2)#!!!!!!!!!!!!!!
    #             print( "--------------------go back from basket-------------------- ",self.head_vertical)
    #             time.sleep(0.05)
    #         elif 1220 + 100 < self.head_vertical < 1240 + 100 : 
    #             print( "--------------------stop at the basket----------------------",self.head_vertical - 2048)
    #             motor.bodyauto_close(0)
    #             target.basket_parameter()

    #             time.sleep(1)
    #             motor.move_head(1,1823,880,880,30)
    #             time.sleep(0.8)
    #             print("伸手準備投籃")
    #             #send.sendBodySector(887)
    #             time.sleep(1.5)
    #             self.ready_basket_dunk = True

    ######################################## degree_straight_new 副函式 ########################################

    def degree_straight_new(self,forward_degree,slow_degree,backward_degree) :   #前進後退至可找拿球的距離
 
        print("walk_to_basket")
        target.basket_parameter()
        print(" basket => x:",target.basket_x," y:",target.basket_y," size:",target.basket_size)
        self.trace_revise(target.basket_x,target.basket_y,35)
            
        if self.head_vertical > forward_degree :  #大前進
            motor.MoveContinuous(1500+correct[0],0+correct[1],0+correct[2],100,100,2) 
            print( "大前進,self.head_vertical= ",self.head_vertical)
            time.sleep(0.05)

        elif slow_degree < self.head_vertical < forward_degree :  #進入減速範圍
            motor.MoveContinuous(1000+correct[0],0+correct[1],0+correct[2],100,100,2)
            print( "進入減速範圍,self.head_vertical= ",self.head_vertical)
            time.sleep(0.05)
    
                
        elif backward_degree <= self.head_vertical <= slow_degree :
            print( "--------------------stop at the basket----------------------",self.head_vertical - 2048)
            motor.bodyauto_close(0)
            target.basket_parameter()
            time.sleep(1)
            motor.move_head(1,1580,880,880,30)
            time.sleep(0.8)
            print("伸手準備投籃")
            send.sendBodySector(887)
            time.sleep(1.5)
            self.ready_basket_dunk = True
    

        elif self.head_vertical < backward_degree: 
            motor.MoveContinuous(-500+correct[0],0+correct[1],0+correct[2],100,100,2)
            print( "大後退,self.head_vertical= ",self.head_vertical)                
            time.sleep(0.05)

    ######################################## degree_straight 副函式 ########################################

    def degree_straight(self,stop_degree,gait_correct):

        print("walk_to_basket")
        print("stop_degree= ",stop_degree)
        target.basket_parameter()
        print(" basket => x:",target.basket_x," y:",target.basket_y," size:",target.basket_size)
        self.trace_revise(target.basket_x,target.basket_y,35)

        if self.head_vertical > (stop_degree + gait_correct) + 300 :  #未進入減速範圍 #前進太多時forward_gait_correct改大
            motor.MoveContinuous(1500+correct[0],0+correct[1],0+correct[2],100,100,2) 
            print( "大前進,self.head_vertical= ",self.head_vertical)
            time.sleep(0.05)

        elif stop_degree < self.head_vertical < (stop_degree + gait_correct) + 300 : #進入減速範圍
            motor.MoveContinuous(1000+correct[0],0+correct[1],0+correct[2],100,100,2)
            print( "進入減速範圍,self.head_vertical= ",self.head_vertical)
            time.sleep(0.05)
   
            
        elif self.head_vertical <= stop_degree :
            print( "--------------------stop at the basket----------------------",self.head_vertical - 2048)
            motor.bodyauto_close(0)
            target.basket_parameter()
            time.sleep(1)
            motor.move_head(1,1823,880,880,30)
            time.sleep(0.8)
            print("伸手準備投籃")
            send.sendBodySector(887)
            time.sleep(1.5)
            self.ready_basket_dunk = True
  

    ######################################## walk_to_basket_dunk 副函式 ########################################  

    def walk_to_basket_dunk(self,stop_degree,gait_correct):   
        print("walk_to_basket")

        self.x_body_rotate = self.head_horizon - 2048 
        target.basket_parameter()
        motor.trace_revise(target.basket_x,target.basket_y,35)

        print(" basket => x:",target.basket_x," y:",target.basket_y," size:",target.basket_size)
        

        if 1600 < self.head_vertical < 1700 : 
            print( "--------------------stop at the basket----------------------",self.head_vertical - 2048)
            motor.bodyauto_close(0)
            target.basket_parameter()
            time.sleep(1)
            motor.move_head(1,1823,880,880,30)
            time.sleep(0.8)
            print("伸手準備投籃")
            send.sendBodySector(887)
            time.sleep(1.5)
            self.ready_basket_dunk = True

        elif self.x_body_rotate > 80 :
            motor.MoveContinuous(left_correct[0],left_correct[1],left_correct[2],100,100,2)
            print( "rotate left = ",self.x_body_rotate)
            time.sleep(0.08)

        elif self.x_body_rotate < -80 :
            motor.MoveContinuous(right_correct[0],right_correct[1],right_correct[2],100,100,2)#!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            print( "rotate right = ",self.x_body_rotate)
            time.sleep(0.08)

        elif self.head_vertical > 1800 :  #未進入減速範圍
            motor.MoveContinuous(2500+correct[0],0+correct[1],0+correct[2],75,100,2)#!!!!!!!!!!!!!!!
            print("未進入減速範圍->大前進 ",self.head_vertical)
            time.sleep(0.05)

        elif 1700 < self.head_vertical < 1800: #進入減速範圍
            motor.MoveContinuous(800+correct[0],0+correct[1],0+correct[2],150,100,2)#!!!!!!!!!!!!!!!
            print("已進入減速範圍->小前進 ",self.head_vertical)
            time.sleep(0.05)    
            
        elif self.head_vertical > 1600 :
            motor.MoveContinuous(-800+correct[0],0+correct[1],0+correct[2],100,100,2)#!!!!!!!!!!!!!!
            print( "--------------------go back from basket-------------------- ",self.head_vertical)
            time.sleep(0.05)

    ######################################## walk_to_basket_shoot 副函式 ########################################


    def walk_to_basket_shoot(self,stop_size,gait_correct) :

        print("walk_to_basket")
        target.basket_parameter()
        print(" basket => x:",target.basket_x," y:",target.basket_y," size:",target.basket_size)
        motor.trace_revise(target.basket_x,target.basket_y,35)

        if abs(motor.head_horizon-2048) > 80 :
            print("rotate調整")
            motor.body_trace_rotate(40)
        elif abs(motor.head_horizon-2048) <= 80 :
            print("straight調整")

            if (target.basket_size - stop_size > gait_correct and target.basket_size > 1200):
                motor.MoveContinuous(-800+correct[0],0+correct[1],0+correct[2],100,100,2)
                print( "未進入減速範圍->大後退 ",target.basket_size)
                self.num = 3
                time.sleep(0.05)   
            elif (target.basket_size - stop_size > gait_correct and target.basket_size < 1200)  :
                motor.MoveContinuous(-500+correct[0],0+correct[1],0+correct[2],100,100,2)
                print( "已進入減速範圍->小後退 ",target.basket_size)
                self.num = 3
                time.sleep(0.05)

            elif target.basket_size - stop_size < gait_correct  and (stop_size > target.basket_size > (stop_size - gait_correct)) :
                self.num = self.num - 1
                if self.num <= 0 :
                    print( "--------------------stop at the basket----------------------",target.basket_size)
                    #send.sendBodyAuto(0,0,0,0,1,0)
                    motor.bodyauto_close(0)
                    target.basket_parameter()
                    print("throw_ball_point ==                               ",target.basket_size ,  target.basket_size - stop_size)
                    time.sleep(3)
                    send.sendBodySector(5301)
                    print("-------------------------send.sendBodySector(3)------------------------------")
                    time.sleep(1)
        
            elif target.basket_size - stop_size < -gait_correct  :
                motor.MoveContinuous(800+correct[0],0+correct[1],0+correct[2],100,100,2)#!!!!!!!!!!!!!!
                print( "--------------------go ahead to basket---------------------  ",target.basket_size)
                self.num = 3
                time.sleep(0.05)

    
    ######################################## walk_to_basket_shoot 副函式 ########################################                    


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
        
    def MoveContinuous(self ,ExpX ,ExpY ,ExpTheta ,AddX ,AddY ,AddTheta) :  #步態移動的馬達緩衝(調整距離:Now_X與Now_Y為現在要移動的x量與現在要移動的y量)
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
        print("Basket Y = ",target.basket_size , "Basket vert = ",motor.head_vertical ,target.basket_size,target.basket_y_max)
        if abs(target.basket_x - 160) > 5  or abs(target.basket_y - 120) > 4 :
            motor.trace_revise(target.basket_x,target.basket_y,25)
            target.basket_parameter() 
            
            time.sleep(0.05) 
            
        else :
            print("Basket Y = ",target.basket_size , "Basket vert = ",motor.head_vertical)
            print(target.basket_size - 930)   
                


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
            self.throw_strength = round(abs( 311 * (-(self.distance_new - 90)/5) + 311 * ((self.distance_new - 85)/5)))
        
        elif self.distance_new >= 90 and self.distance_new < 95 : #70cm
            self.throw_strength = round(abs( 311 * (-(self.distance_new - 95)/5) + 319 * ((self.distance_new - 90)/5)))

        elif self.distance_new >= 95 and self.distance_new < 100 : #70cm
            self.throw_strength = round(abs( 319 * (-(self.distance_new - 100)/5) + 322 * ((self.distance_new - 95)/5))) #319   322

        elif self.distance_new >= 100 and self.distance_new < 105 : #70cm
            self.throw_strength = round(abs( 322 * (-(self.distance_new - 105)/5) + 324 * ((self.distance_new - 100)/5)))

        elif self.distance_new >= 105 and self.distance_new < 110 : #70cm
            self.throw_strength = round(abs( 324 * (-(self.distance_new - 110)/5) + 330 * ((self.distance_new - 105)/5)))

        self.start_get_point = True
        print("throw_strength",self.throw_strength)


    def basket_distance(self,six,nine):   #利用60與90公分距離測出的籃框面積推得當前機器人與籃框的距離->self.distance_new
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


    def bodyauto_close(self,next_state):    #步態移動的開關控制(原地踏步)
        #if send.is_start == True :   # 0 stop  1 go
        if self.now_state == next_state :    
            pass

        elif self.now_state != next_state :
            send.sendBodyAuto(0,0,0,0,1,0)
            self.now_state = next_state
                

        
       



if __name__ == '__main__' :
    
    target = target_location()
    motor = motor_move()
    step ='begin'

    #step  = ['begin','find_ball','walk_to_ball','catch_ball','find_basekt','basket_trace','walk_to_basket','find','waisting','finish']
    
    #           0              1                2           3             4                5             6           7               8      9           10
    
    # sw = 2
    gazebo_robot = 1
    # 0 for gazebo 1 for robot
    stategy_or_test = 1
    # 0 for test 1 for stategy

    basket_size_60_90 =[2116, 899] #sector 111   left side 1978, 899 right side  2140, 961  #投籃時測量的籃框距離方法 #五分投籃時站姿高度看籃框size測距離
    five_point_degree = [1960]# left side 1960 right side  1940   too left-big too right-small #投籃前頭會固定一個角度，並扭腰
    throw_plus = 1 #line  0   left side 0 right side  4

    throw_ball_point = [910,1200,1700]   #[0,1,2]=[三分,五分,兩分] #二分走路時測量的籃框距離方法degree #五分走路時站姿高度看籃框size測距離
    #                  [size,size,degree] 
    ball_catch_size =[1500] #line  1650
    # # for size          三分  五分  灌籃
    # throw_ball_point = [0,0,1300] 
    # for degree          三分  五分  灌籃
    
    correct       = [-200,0,-1] #原地踏步修正
    left_correct  = [-100,-200,2]  #左旋修正
    right_correct = [-200,100,-4] #右旋修正
    #                  x , y , theta   

    basket_error = [70,100,60]
    #  for size    三分  五分  灌籃
    # basket_error = [0,0,100]
    # for degree          三分  五分  灌籃

    ball_correct = [300,10]  #forward,backward

    trace_parameter =[80]#25
    too_big = True

    catch_ball_line = [1700,1680,1530] #forward_degree,slow_degree,backward_degree
    two_point_line  = [1800,1800,1690] #forward_degree,slow_degree,backward_degree
    
    try:
           #step為紀錄目前進行到的步驟
        
        while not rospy.is_shutdown():
            #motor.switch_control()                              #warning    這東東把它閹了，改到is_start判斷之後
            if send.is_start==True: #send.Web
        
                send.drawImageFunction(1,0,160,160,0,240,255,255,255) 
                send.drawImageFunction(2,0,0,320,120,120,255,255,255)
                send.drawImageFunction(3,1,target.ball_x_min ,target.ball_x_max ,target.ball_y_min ,target.ball_y_max,255,0,255)
                send.drawImageFunction(4,1,target.basket_x_min ,target.basket_x_max ,target.basket_y_min ,target.basket_y_max,0,0,0)
                target.ball_parameter() 

                if step =='begin':

                ####################################### switch #######################################
            
                    if send.DIOValue == 24:  #開啟二分策略  下下下下
                        sw = 2
                        print("SW = ", sw)

                    elif send.DIOValue == 25: #開啟三分策略  上下下下                                 
                        sw = 0
                        print("SW = ", sw)
   
                    elif send.DIOValue == 31: #開啟五分策略  上上上(上)                                 
                        sw = 1
                        print("SW = ", sw)
 
                    else :
                        sw = 2
                        print("SW = ", sw)
 

                      #[sw==2]->2分球;[sw==0]->3分球;[sw==1]->5分球

                ######################################## switch #######################################

                    print("開始執行初始化")
                    send.sendBodySector(9) #讓手回歸自我們的初始手部位置,原是AR的
                    time.sleep(0.7)
                    send.sendBodySector(8910)   #步態調整
                    time.sleep(0.2)
                    step = 'find_ball'

                elif step == 'find_ball' :

                    if target.ball_size <= 350 :   #球在視野中太小
                        print("球在視野中太小->大範圍尋球")
                        motor.view_move(2428,1668,1800,1200,40,0.05)
                        target.ball_parameter()   
                    elif target.ball_size >350 :   #球在視野中夠大
                        if abs(target.ball_x - 160) > 6  or abs(target.ball_y - 120) > 8 :  #讓球在畫面中心
                            print("球在視野中夠大->鎖定球")
                            target.ball_parameter()
                            motor.trace_revise(target.ball_x,target.ball_y,25) 
                            time.sleep(0.05)
                        else : 
                            #motor.reg = motor.head_horizon
                            motor.reg = 2048 - motor.head_horizon

                            step = 'start_gait'       
                        
                elif step == 'start_gait' :

                    target.ball_parameter()
                    motor.trace_revise(target.ball_x,target.ball_y,25) 
                    motor.bodyauto_close(1)
                    time.sleep(0.05)
                    if (motor.head_vertical <= ball_catch_size[0]) or ((motor.head_vertical < ball_catch_size[0]-55) and  (1698 >= motor.head_horizon or motor.head_horizon >=2398)):
                        print("球太大->大倒退")
                        motor.MoveContinuous(-1200+correct[0],0+correct[1],0+correct[2],100,100,1)
                        step = 'walk_to_ball'
                    else:
                        print("可進行微小修正")
                        step = 'walk_to_ball'

                elif step == 'walk_to_ball' :
                    
                    if motor.ready_ball_catch == True :    #到達夾球位置

                        print("到達可夾球位置")
                        print("蹲下")
                        time.sleep(1.0)
                        send.sendBodySector(5) 
                        time.sleep(3)
                        print("頭往右轉")
                        motor.move_head(1,1820,880,880,50)
                        time.sleep(2.5) 
                        step = 'waist_fix'

                    else :

                        target.ball_parameter()
                        motor.trace_revise(target.ball_x,target.ball_y,25) 

                        if abs(motor.head_horizon-2048) > 100  :
                            print("頭部馬達水平刻度偏差>步態影響的")
                            print("rotate調整")
                            time.sleep(0.05)
                            motor.body_trace_rotate(40)

                        else:
                        #elif abs(motor.head_vertical - ball_catch_size[0]) > 10 :
                            print("頭部馬達垂直刻度與抓球角度差太多")
                            print( "head_vertical= ",motor.head_vertical)
                            print("straight調整")
                            time.sleep(0.05)
                            #motor.body_trace_straight(ball_catch_size[0],ball_correct[0],ball_correct[1])
                            motor.body_trace_straight_new(catch_ball_line[0],catch_ball_line[1],catch_ball_line[2])
                            

                elif step == 'waist_fix' :
                    
                    target.ball_parameter() 
                    if target.ball_x != 0 :
                        if abs(target.ball_x-160) < 2 :
                            print("球水平位置在中間")
                            step = 'catch_ball' 
                        elif abs(target.ball_x-160) > 2 : 
                            print("球水平位置不在在中間->waist_fix")
                            target.ball_parameter()
                            motor.WaistFix(target.ball_x,160)
                    else:
                        
                        print("球不在視野中->往左邊轉腰")
                        #motor.waist_rotate(2148,70)
                        
                elif step == 'catch_ball' :

                    if motor.directly == True :
                        print("執行直接夾球副函式")
                    elif motor.directly == False :
                        print("正常夾球動作")
                        time.sleep(1.5)
                        send.sendBodySector(6)
                        time.sleep(1) 
                    print("腰部回正")
                    motor.waist_rotate(2048,70)
                    time.sleep(3) 
                    if motor.directly == True :
                        print("根據各自夾球動作回復站姿")
                    elif motor.directly == False :
                        print("回復站姿")
                        send.sendBodySector(7) 
                        time.sleep(2)
                    time.sleep(3)
                    step = 'find_basket' 
                
                elif step == 'find_basket' :
                    
                    target.basket_parameter()
                    if target.basket_size < 500 :
                        print("籃框在視野裡太小->尋框")
                        print("basket_size = ",target.basket_size)

                        if motor.reg_use == False:
                            print("頭部左右找框調整")
                            #motor.move_head(1,motor.reg,880,880,50) 
                            print("motor.reg = ",motor.reg)
                            time.sleep(1)
                            print("頭部抬起尋框") 
                            motor.move_head(2,1900,880,880,50)                
                            motor.reg_use = True
                            time.sleep(1)

                        else:    
                            print("開始尋框")
                            motor.view_move(2548,1548,2048,1948,50,0.04)
                            print("  basket_x = ",target.basket_x,"  basket_y =",target.basket_y,"  basket_size = ",target.basket_size)
                    else:                                 #warning    為啥不用else就好
                        print("籃框在視野裡夠大->判斷策略所需前往的位置")
                        if sw == 0:
                            print("3分球")
                            step = 'stratagy_3'
                        elif sw == 1:
                            print("5分球")
                            step = 'stratagy_5'
                        elif sw == 2:
                            print("2分球")
                            motor.bodyauto_close(1)
                            time.sleep(1)
                            step = 'stratagy_2'

    ######################################## 二分球仿造catch_ball ######################################## 

                elif step == 'stratagy_2':

                    target.basket_parameter()
                    motor.trace_revise(target.basket_x,target.basket_y,35)
                        
                    if motor.ready_basket_dunk == False :
 

                        if abs(motor.head_horizon-2048) > 200  :
                            print("頭部馬達水平刻度偏差>步態影響的")
                            print("rotate調整")
                            time.sleep(0.05)
                            motor.body_trace_rotate(40)

                        else :
                        #elif abs(motor.head_vertical - ball_catch_size[0]) > 10 :
                            print("頭部馬達垂直刻度與抓球角度差太多")
                            print("straight調整")
                            time.sleep(0.05)
                            #motor.degree_straight(throw_ball_point[sw],basket_error[sw])
                            motor.degree_straight_new(two_point_line[0],two_point_line[1],two_point_line[2])

                    elif motor.ready_basket_dunk == True :

                        if target.basket_x != 0 :

                            if abs(target.basket_x-160) > 3:
                                target.basket_parameter()
                                print("腰部修正")
                                motor.WaistFix(target.basket_x,160)
                                print("abs(target.basket_x-160)",abs(target.basket_x-160))
                            elif abs(target.basket_x-160) < 3:
                                time.sleep(1)
                                print("執行2分球投籃")
                                send.sendBodySector(987)
                                step ="finish"
                        else:
                            print("框不在視野中->往左邊轉腰")
                            #motor.waist_rotate(1900,70) #?????????
                            motor.Null_WaistFix(1900)
  

    ######################################## 二分球舊 ######################################## 

                # elif step == 'stratagy_2':

                #     if motor.ready_basket_dunk == False :

                #         print("執行 walk_to_basket_dunk 副函式")
                #         motor.walk_to_basket_dunk(throw_ball_point[sw],basket_error[sw]) # walk to basket 副函式

                #     elif motor.ready_basket_dunk == True :

                #         if target.basket_x != 0 :

                #             if abs(target.basket_x-160) > 3:
                #                 target.basket_parameter()
                #                 print("腰部修正")
                #                 motor.WaistFix(target.basket_x,160)
                #                 print("abs(target.basket_x-160)",abs(target.basket_x-160))
                #             elif abs(target.basket_x-160) < 3:
                #                 time.sleep(1)
                #                 print("執行2分球投籃")
                #                 send.sendBodySector(987)
                #                 step ="finish"
                #         else:
                #             print("框不在視野中->往左邊轉腰")
                #             motor.waist_rotate(1900,70) #?????????


                #elif step == 'stratagy_3':

                    #print("執行 walk_to_basket_shoot 副函式")
                    #motor.walk_to_basket_shoot(throw_ball_point[sw],basket_error[sw])
                    #pass

######################################## 三分球用degree判斷 ########################################
                elif step == 'stratagy_3':

                    if motor.ready_basket_dunk == False :

                            print("執行 walk_to_basket_dunk 副函式")
                            motor.walk_to_basket_dunk(throw_ball_point[sw],basket_error[sw]) # walk to basket 副函式

                    elif motor.ready_basket_dunk == True :

                        if target.basket_x != 0 :
                                
                            if abs(target.basket_x-160) > 3:
                                    target.basket_parameter()
                                    print("腰部修正")
                                    motor.WaistFix(target.basket_x,target.basket_y,160,120)
                                    print("abs(target.basket_x-160)",abs(target.basket_x-160))
                            if abs(target.basket_x-160) < 3:
                                    time.sleep(1)
                                    print("執行3分球投籃")
                                    #send.sendBodysector(987)
                                    step ="finish"
                        else:
                            print("框不在視野中->往左邊轉腰")
                            motor.waist_rotate(1900,70) #?????????

######################################## 三分球用degree判斷 ########################################

                elif step == 'stratagy_5':

                    print("執行 walk_to_basket_shoot 副函式")
                    motor.walk_to_basket_shoot(throw_ball_point[sw],basket_error[sw])
                    pass
 
            else :
                if step != 'begin':
                    send.sendHeadMotor(1,2048,30)
                    send.sendHeadMotor(2,2048,30)
                    time.sleep(0.05)
                    motor.bodyauto_close(0)
                    time.sleep(1)
                    send.sendBodySector(29)
                    time.sleep(0.05)
                    target = target_location()
                    motor = motor_move()
                    step = 'begin'
                    print("-------------------reset and stoping-------------------------")
                    print("主策略指撥關閉->機器人回復初始狀態")
                    time.sleep(0.05)

                elif send.DIOValue == 11:           #catch ball point  上上下下 
                    print("Basket vert = ",motor.head_vertical)
                    time.sleep(0.2) 
                    step = 'test'

                elif send.DIOValue == 13:       #basket size   上下上下
                    print("Basket Y = ",target.basket_size , "Basket vert = ",motor.head_vertical)
                    time.sleep(0.2) 
                    step = 'test'

                else:
                    pass

    except rospy.ROSInterruptException:
        pass