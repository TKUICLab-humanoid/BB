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

BASKET_SIZE_100_150 =[2116, 899]        #投籃時測量的籃框距離方法 #五分投籃時站姿高度看籃框size測距離
FIVE_POINT_HEAD_DEGREE = [1930]         #投籃前頭會固定一個角度，並扭腰
THROW_PLUS = 1 
BALL_CATCH_SIZE =[1850] 


#======================================================================================

CORRECT       = [-1500,0,0]          #原地踏步修正
LEFT_CORRECT  = [-1400,-500,3]          #左旋修正
RIGHT_CORRECT = [-1400,100,-3]         #右旋修正
#                  x , y , theta 

CATCH_BALL_LINE = [2180,2340,2440] #forward_degree,slow_degree,backward_degree
TWO_POINT_LINE  = [2180,2350,2450] #forward_degree,slow_degree,backward_degree
THREE_POINT_LINE = [900,700,500] #forward_degree,slow_degree,backward_degree
#three_point_line = [2000,1930,1900] #forward_degree,slow_degree,backward_degree
FIVE_POINT_LINE  = [2600,2700,2750] #forward_degree,slow_degree,backward_degree


class Target_Location():
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
        
    def initial(self):
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

                if send.color_mask_subject_size [0][j] > 300  and send.color_mask_subject_size [0][j] > self.ball_size: #用大小過濾物件
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

        
               
   

class Motor_Move():

    def __init__(self):
        self.head_horizon = 2048                #頭部水平刻度
        self.head_vertical = 2048               #頭部垂直刻度
        self.waist_position = 2048              #腰當下的刻度
        self.head_horizon_flag1 = 1             #找目標物的旗標1
        self.head_horizon_flag2 = 1             #找目標物的旗標2
        self.now_state = 0
        self.nowx = 0                           #現在要移動的x量
        self.nowy = 0                           #現在要移動的y量
        self.nowtheta = 0                       #現在要旋轉的theta量
        self.throw_strength = 0                 #不知道
        self.distance_new = 0
        self.directly = False

    def initial(self):
        self.head_horizon = 2048                #頭部水平刻度
        self.head_vertical = 2048               #頭部垂直刻度
        self.waist_position = 2048              #腰當下的刻度
        self.head_horizon_flag1 = 1             #找目標物的旗標1
        self.head_horizon_flag2 = 1             #找目標物的旗標2
        self.now_state = 0
        self.nowx = 0                           #現在要移動的x量
        self.nowy = 0                           #現在要移動的y量
        self.nowtheta = 0                       #現在要旋轉的theta量
        self.throw_strength = 0                 #不知道
        self.distance_new = 0
        self.directly = False


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
        send.sendSingleMotor(9,-(waist_x-self.waist_position),Speed)
        self.waist_position =  waist_x 
    

    def view_move_left(self,right_place,left_place,up_place,down_place,speed,delay):    #先往左轉

        if self.head_horizon_flag1 == 3 :
            if self.head_horizon >= left_place:
                self.move_head(1,self.head_horizon,880,880,speed)
                self.head_horizon = self.head_horizon - speed
                time.sleep(delay)
            else:
                self.head_horizon_flag1 = 4  
                time.sleep(delay*5) 


        elif self.head_horizon_flag1 == 4 :
            if self.head_vertical <= up_place:
                self.move_head(2,self.head_vertical,880,880,speed)
                self.head_vertical = self.head_vertical + speed
                time.sleep(delay)
            else:
                self.head_horizon_flag1 = 1  
                time.sleep(delay*5)

                    
        elif  self.head_horizon_flag1 == 1 :
            if  self.head_horizon <= right_place:
                self.move_head(1,self.head_horizon,880,880,speed)
                self.head_horizon = self.head_horizon + speed
                time.sleep(delay) 
            else:
                self.head_horizon_flag1 = 2 
                time.sleep(delay*5)      

        
        elif self.head_horizon_flag1 ==  2:
            if self.head_vertical >= down_place:
                self.move_head(2,self.head_vertical,880,880,speed)      #頭向下的極限
                self.head_vertical = self.head_vertical - speed
                time.sleep(delay)   
            else:
                self.head_horizon_flag1 = 3
                time.sleep(delay*5)


    def view_move_right(self,right_place,left_place,up_place,down_place,speed,delay):    #先往右轉

            if self.head_horizon_flag2 == 1 :
                if self.head_horizon >= left_place:
                    self.move_head(1,self.head_horizon,880,880,speed)
                    self.head_horizon = self.head_horizon - speed
                    time.sleep(delay) 
                else:
                    self.head_horizon_flag2 = 2  
                    time.sleep(delay*5) 


            elif self.head_horizon_flag2 == 4 :
                if self.head_vertical <= up_place:
                    self.move_head(2,self.head_vertical,880,880,speed)
                    self.head_vertical = self.head_vertical + speed
                    time.sleep(delay)
                else:
                    self.head_horizon_flag2 = 1  
                    time.sleep(delay*5)

                        
            elif  self.head_horizon_flag2 == 3 :
                if  self.head_horizon <= right_place:
                    self.move_head(1,self.head_horizon,880,880,speed)
                    self.head_horizon = self.head_horizon + speed
                    time.sleep(delay) 
                else:
                    self.head_horizon_flag2 = 4 
                    time.sleep(delay*5)      

            
            elif self.head_horizon_flag2 ==  2 :
                if self.head_vertical >= down_place:
                    self.move_head(2,self.head_vertical,880,880,speed)      #頭向下的極限
                    self.head_vertical = self.head_vertical - speed
                    time.sleep(delay)   
                else:
                    self.head_horizon_flag2 = 3
                    time.sleep(delay*5)
   
    
    def trace_revise(self,x_target,y_target,speed) :    #看誤差調整頭的角度(讓頭看向籃框或球)
        if x_target != 0 and y_target != 0:
            x_differ =  x_target - 160             
            y_differ =  y_target - 120 
            x_degree = x_differ * (65 / 320)
            y_degree = y_differ * (38 / 240)
            self.move_head(1, self.head_horizon - round(x_degree * 4096 / 360 *0.15),880,880,speed)
            self.move_head(2, self.head_vertical + round(y_degree * 4096 / 360 *0.15),880,880,speed)
            time.sleep(0.05)
        else :
            rospy.logdebug(f"miss_target->需重新尋求")

    def body_trace_rotate(self,degree) : #步態旋轉到可拿球的角度
        x_body_rotate = self.head_horizon - 2048
        if x_body_rotate > degree :
            motor.MoveContinuous(LEFT_CORRECT[0],LEFT_CORRECT[1],LEFT_CORRECT[2],100,100,8)
            rospy.loginfo(f"rotate left = {x_body_rotate}")
            time.sleep(0.05)
        elif x_body_rotate < -degree :
            motor.MoveContinuous(RIGHT_CORRECT[0],RIGHT_CORRECT[1],RIGHT_CORRECT[2],100,100,8)#!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            rospy.loginfo(f"rotate right = {x_body_rotate}")
            time.sleep(0.05)
            

    ######################################## ball_trace_straight 副函式 ########################################

    def ball_trace_straight(self,slow_degree,stop_degree,backward_degree) :   #前進後退至可找拿球的距離
        #前進太多時forward_gait_correct改大
        #後退太多時back_gait_correct改小
            
        if self.head_vertical < slow_degree :  #大前進
                motor.MoveContinuous(3000+CORRECT[0],0+CORRECT[1],0+CORRECT[2],100,100,2) 
                rospy.loginfo(f"大前進,self.head_vertical = {self.head_vertical}")
                time.sleep(0.05)

        elif slow_degree < self.head_vertical < stop_degree :  #進入減速範圍
            motor.MoveContinuous(1000+CORRECT[0],0+CORRECT[1],0+CORRECT[2],100,100,2)
            rospy.loginfo(f"進入減速範圍,self.head_vertical = {self.head_vertical}")
            time.sleep(0.05)
    
        elif self.head_vertical > backward_degree: 
            motor.MoveContinuous(-1500+CORRECT[0],0+CORRECT[1],0+CORRECT[2],100,100,2)
            rospy.loginfo(f"大後退,self.head_vertical = {self.head_vertical}")                
            time.sleep(0.05)

    def WaistFix(self, Target_X, TargetXCenter):#轉腰調整Basket.X與Baskethead_verticalBaseLine的誤差
        
        self.MoveW = TargetXCenter - Target_X
        if self.MoveW > 5:
            self.MoveW = 5
        elif self.MoveW < -5:
            self.MoveW = -5
        
        self.waist_rotate((self.waist_position - self.MoveW), 30)
        self.move_head(2, self.head_vertical ,880,880,20)    
        
        time.sleep(0.15)
        # time.sleep(0.2)
            
    def Null_WaistFix(self, turn_final):#轉腰調整Basket.X與Baskethead_verticalBaseLine的誤差

        if (self.waist_position - 10) < turn_final:
            self.waist_position -= 10
            self.waist_rotate(self.waist_position, 30)
            time.sleep(0.15)
        else:
            rospy.logdebug(f"fail")

    ######################################## degree_straight 副函式 ########################################

    def degree_straight(self,forward_degree,slow_degree,backward_degree) :   #前進後退至可找拿球的距離
 
        rospy.logdebug(f"walk_to_basket")
        target.basket_parameter()
        rospy.logdebug(f" basket => x : {target.basket_x}, y : {target.basket_y}, size : {target.basket_size}")
        self.trace_revise(target.basket_x,target.basket_y,35)
            
        if self.head_vertical < forward_degree :  #大前進
            motor.MoveContinuous(3000+CORRECT[0],0+CORRECT[1],0+CORRECT[2],100,100,2) 
            rospy.loginfo(f"大前進,self.head_vertical = {self.head_vertical}")
            time.sleep(0.05)

        elif forward_degree < self.head_vertical < slow_degree :  #進入減速範圍
            motor.MoveContinuous(1000+CORRECT[0],0+CORRECT[1],0+CORRECT[2],100,100,2)
            rospy.loginfo(f"進入減速範圍,self.head_vertical = {self.head_vertical}")
            time.sleep(0.05)

        elif self.head_vertical > backward_degree: 
            motor.MoveContinuous(-500+CORRECT[0],0+CORRECT[1],0+CORRECT[2],100,100,2)
            rospy.logdebug(f"大後退,self.head_vertical = {self.head_vertical}")                
            time.sleep(0.05)

    ########################################  size_straight副函式 ########################################

    def size_straight(self,forward_size, stop_size, slow_size) :   #前進後退至可找拿球的距離

        rospy.logdebug(f"walk to five point line")
        target.basket_parameter()
        rospy.logdebug(f" basket => x : {target.basket_x}, y : {target.basket_y}, size : {target.basket_size}")
        self.trace_revise(target.basket_x,target.basket_y,35)

        if slow_size < target.basket_size :  #大後退
            motor.MoveContinuous(-1200+CORRECT[0],0+CORRECT[1],0+CORRECT[2],100,100,2)
            rospy.logdebug(f"大後退,target.basket_size = {target.basket_size}")
            time.sleep(0.05)

        elif stop_size < target.basket_size < slow_size :  #進入減速範圍
            motor.MoveContinuous(-500+CORRECT[0],0+CORRECT[1],0+CORRECT[2],100,100,2)
            rospy.logdebug(f"進入減速範圍,target.basket_size = {target.basket_size}")
            time.sleep(0.05)

        elif target.basket_size < forward_size: 
            motor.MoveContinuous(800+CORRECT[0],0+CORRECT[1],0+CORRECT[2],100,100,2)
            rospy.logdebug(f"大前進,target.basket_size = {target.basket_size}")                
            time.sleep(0.05)

 
    def MoveContinuous(self ,ExpX ,ExpY ,ExpTheta ,AddX ,AddY ,AddTheta) :  #步態移動的馬達緩衝(調整距離:Now_X與Now_Y為現在要移動的x量與現在要移動的y量)
        if abs(self.nowx - ExpX) < AddX:
            self.nowx = ExpX
        else:
            if self.nowx < ExpX :
                self.nowx += AddX
            elif self.nowx > ExpX :
                self.nowx -= AddX
            else:
                pass

        if abs(self.nowy - ExpY) < AddY:
            self.nowy = ExpY
        else:
            if self.nowy < ExpY :
                self.nowy += AddY
            elif self.nowy > ExpY :
                self.nowy -= AddY
            else:
                pass

        if abs(self.nowtheta - ExpTheta) < AddTheta:
            self.nowtheta = ExpTheta
        else:
            if self.nowtheta < ExpTheta :
                self.nowtheta += AddTheta
            elif self.nowtheta > ExpTheta :
                self.nowtheta -= AddTheta
            else:
                pass

        rospy.loginfo(f"NowX = {self.nowx}, NowY = {self.nowy}, NowTheta = {self.nowtheta}")
        send.sendContinuousValue(self.nowx, self.nowy, 0, self.nowtheta , 0)

    def test_distance(self):
        send.drawImageFunction(4,1,target.basket_x_min ,target.basket_x_max ,target.basket_y_min ,target.basket_y_max,0,0,0)
        target.basket_parameter()
        rospy.logdebug(f"Basket Y = {target.basket_size}, Basket vert = {motor.head_vertical}")
        if abs(target.basket_x - 160) > 5  or abs(target.basket_y - 120) > 4 :
            motor.trace_revise(target.basket_x,target.basket_y,25)
            target.basket_parameter() 
            time.sleep(0.05) 
            
        else :
            rospy.logdebug(f"Basket Y = {target.basket_size}, Basket vert = {motor.head_vertical}")   


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

        # self.start_get_point = True
        rospy.logdebug(f"throw_strength = {self.throw_strength}")


    def basket_distance(self,six,nine):   #利用60與90公分距離測出的籃框面積推得當前機器人與籃框的距離->self.distance_new

            sixty_distance = sqrt(abs((3600*six)/target.basket_size))
            ninty_distance = sqrt(abs((8100*nine)/target.basket_size))
            # if ( six + nine ) / 2 > target.basket_size :
            #     self.distance_new = abs(self.sixty_distance)
            # else :
            self.distance_new = abs(ninty_distance)
            motor.throw_ball_strength()
            rospy.logdebug(f"Basket vertical = {motor.head_vertical}")
            rospy.logdebug(f"Basket size = {target.basket_size}")
            rospy.logdebug(f"Distance_60 = {sixty_distance}")
            rospy.logdebug(f"Distance_90 = {ninty_distance}")
            rospy.logdebug(f"Distance_fin = {self.distance_new}")


    def bodyauto_close(self,next_state):    #步態移動的開關控制(原地踏步)
        #if send.is_start == True :   # 0 stop  1 go
        if self.now_state == next_state :    
            pass

        elif self.now_state != next_state :
            send.sendBodyAuto(0,0,0,0,1,0)
            self.now_state = next_state
                
class BasketBall():
    def __init__(self):
        self.step = 'begin'
        self.reg = 2048
        self.sw = -1
        self.throw_ball_action = True


    def initial(self):
        self.step = 'begin'
        self.reg = 2048
        self.sw = -1
        self.throw_ball_action = True


    def main(self):
        if send.is_start == True:
            self.drawline()
            if self.step =='begin' :
                self.begin()
            elif self.step == 'find_ball' :
                self.find_ball()
            elif self.step == 'start_gait' :
                self.start_gait()
            elif self.step == 'walk_to_ball' :
                self.walk_to_ball()
            elif self.step == 'waist_fix' :
                self.waist_fix()
            elif self.step == 'catch_ball' :
                self.catch_ball()
            elif self.step == 'find_basket' :
                self.find_basket()
            elif self.step == 'stratagy_2':
                self.stratagy_2()
            elif self.step == 'stratagy_3':
                self.stratagy_3()
            elif self.step == 'stratagy_5':
                self.stratagy_5()  
        else:
            self.stand()
        
    def begin(self):
        ####################################### switch #######################################

        if send.DIOValue == 24:  #開啟二分策略  下下下下
            self.sw = 2
            rospy.logdebug(f"SW = {self.sw}")

        elif send.DIOValue == 25: #開啟三分策略  上下下下                                 
            self.sw = 0
            rospy.logdebug(f"SW = {self.sw}")

        elif send.DIOValue == 31: #開啟五分策略  上上上(上)                                 
            self.sw = 1
            rospy.logdebug(f"SW = {self.sw}")

        else :
            self.sw = 2
            rospy.logdebug(f"SW = {self.sw}")


            #[sw==2]->2分球;[sw==0]->3分球;[sw==1]->5分球

    ######################################## switch #######################################

        rospy.logdebug(f"開始執行初始化")
        motor.move_head(1,2200,880,880,50) #請動動作(擺頭)
        send.sendBodySector(111)
        time.sleep(2)
        self.step = 'find_ball'

    def find_ball(self):

        rospy.logdebug(f"find_ball")
        if target.ball_size <= 300 :   #球在視野中太小
            rospy.logdebug(f"球在視野中太小->大範圍尋球")
            motor.view_move_left(2500,1650,2577,2048,40,0.05)
            target.ball_parameter()   
        elif target.ball_size > 300 :   #球在視野中夠大
            if abs(target.ball_x - 160) > 10  or abs(target.ball_y - 120) > 10 :  #讓球在畫面中心
                rospy.logdebug(f"球在視野中夠大->鎖定球")
                target.ball_parameter()
                motor.trace_revise(target.ball_x,target.ball_y,25) 
                time.sleep(0.05)
            else : 
                self.reg = 2048 - motor.head_horizon     

                self.step = 'start_gait'

    def start_gait(self):

        rospy.logdebug(f"start_gait")
        target.ball_parameter()
        motor.trace_revise(target.ball_x,target.ball_y,25) 
        motor.bodyauto_close(1)
        time.sleep(0.05)
        if (motor.head_vertical <= BALL_CATCH_SIZE[0]) or ((motor.head_vertical < BALL_CATCH_SIZE[0]-55) and  (1698 >= motor.head_horizon or motor.head_horizon >=2398)):
            rospy.loginfo(f"球太大->大倒退")
            motor.MoveContinuous(-500+CORRECT[0],0+CORRECT[1],0+CORRECT[2],100,100,1)
            self.step = 'walk_to_ball'
        else:
            rospy.loginfo(f"可進行微小修正")
            self.step = 'walk_to_ball'

    def walk_to_ball(self):

        rospy.logdebug(f"walk_to_ball")
        target.ball_parameter()
        motor.trace_revise(target.ball_x,target.ball_y,25) 

        if abs(motor.head_horizon-2048) > 100:
            rospy.logdebug(f"頭部馬達水平刻度偏差>步態影響的")
            rospy.logdebug(f"rotate調整")
            time.sleep(0.05)
            motor.body_trace_rotate(80)
        elif CATCH_BALL_LINE[1] <= motor.head_vertical <= CATCH_BALL_LINE[2] :    #到達夾球位置
            motor.bodyauto_close(0) #步態停止
            time.sleep(1.2)
            rospy.loginfo(f"到達夾球範圍STOP!!,self.head_vertical= {motor.head_vertical}")                
            time.sleep(0.05)

            rospy.logdebug(f"到達可夾球位置")
            rospy.logdebug(f"舉手手")
            time.sleep(1.0)
            send.sendBodySector(508) 
            time.sleep(2)
            rospy.logdebug(f"頭往右轉")
            motor.move_head(1,1960,880,880,50)
            time.sleep(4) 
            self.step = 'waist_fix'

        else:
            rospy.logdebug(f"頭部馬達垂直刻度與抓球角度差太多")
            rospy.logdebug(f"head_vertical= {motor.head_vertical}")
            rospy.logdebug(f"straight調整")
            time.sleep(0.05)
            motor.ball_trace_straight(CATCH_BALL_LINE[0],CATCH_BALL_LINE[1],CATCH_BALL_LINE[2])
        

            

    def waist_fix(self):

        rospy.logdebug(f"waist_fix")
        target.ball_parameter() 
        if target.ball_x != 0 :
            if abs(target.ball_x-160) < 3 :
                rospy.logdebug(f"球水平位置在中間")
                self.step = 'catch_ball' 
            elif abs(target.ball_x-160) > 3 : 
                rospy.logdebug(f"球水平位置不在在中間->waist_fix")
                target.ball_parameter()
                motor.WaistFix(target.ball_x,160)
        else:    
            rospy.logdebug(f"球不在視野中->往左邊轉腰")
            #motor.waist_rotate(2148,70)

    def catch_ball(self):

        rospy.logdebug(f"catch_ball")
        if motor.directly == True :
            rospy.logdebug(f"執行直接夾球副函式")
        elif motor.directly == False :
            rospy.logdebug(f"正常夾球動作")
            time.sleep(1.5)
            send.sendBodySector(854)
            time.sleep(2) 
        rospy.logdebug(f"腰部回正")
        motor.waist_rotate(2048,70)
        time.sleep(3) 
        if motor.directly == True :
            rospy.logdebug(f"根據各自夾球動作回復站姿")
        elif motor.directly == False :
            rospy.logdebug(f"回復站姿")
            send.sendBodySector(638) 
            time.sleep(2)
        time.sleep(4)
        motor.move_head(2,2200,880,880,50)
        time.sleep(5)
        self.step = 'find_basket' 

    def find_basket(self):

        rospy.logdebug(f"find_basket")            
        target.basket_parameter()
        if target.basket_size < 2000 :
            rospy.logdebug(f"籃框在視野裡太小->尋框")
            rospy.logdebug(f"basket_size = {target.basket_size}")

            # if motor.reg_use == False:
            rospy.logdebug(f"頭部左右找框調整")
            rospy.logdebug(f"self.reg = {self.reg}")
            if self.reg > 0:
                rospy.logdebug(f"reg大於0 =>頭往左轉修正")
                # motor.view_move = True
                rospy.logdebug(f"開始尋框")
                rospy.logdebug(f"  basket_x = {target.basket_x}, basket_y = {target.basket_y}, basket_size = {target.basket_size}")
                rospy.logdebug(f"左轉開始尋框 = view_move_left")
                motor.view_move_left(2457,1698,2577,2048,50,0.04)
            else:
                rospy.logdebug(f"開始尋框")
                rospy.logdebug(f"  basket_x = {target.basket_x}, basket_y = {target.basket_y}, basket_size = {target.basket_size}")
                rospy.logdebug(f"reg小於0=>頭往右轉修正")
                # motor.view_move = False
                rospy.logdebug(f"右轉開始尋框 = view_move_right")
                motor.view_move_right(2457,1698,2577,2048,50,0.04)

            # time.sleep(1)
            rospy.logdebug(f"頭部抬起尋框") 

        else:                                
            rospy.logdebug(f"籃框在視野裡夠大->判斷策略所需前往的位置")
            if self.sw == 0:
                rospy.logdebug(f"3分球")
                motor.bodyauto_close(1)
                time.sleep(1)
                self.step = 'stratagy_3'
            elif self.sw == 1:
                rospy.logdebug(f"5分球")
                # send.sendBodySector(111)
                motor.bodyauto_close(1)
                time.sleep(1)
                self.step = 'stratagy_5'
            elif self.sw == 2:
                rospy.logdebug(f"2分球")
                motor.bodyauto_close(1)
                time.sleep(1)
                self.step = 'stratagy_2'

    def stratagy_2(self):
        rospy.logdebug(f"stratagy_2")
        ######################################## 二分球仿造catch_ball ######################################## 

        target.basket_parameter()

        motor.trace_revise(target.basket_x,target.basket_y,35)

        if abs(motor.head_horizon-2048) > 200  :
            rospy.logdebug(f"頭部馬達水平刻度偏差>步態影響的")
            rospy.logdebug(f"rotate調整")
            time.sleep(0.05)
            motor.body_trace_rotate(80)

        elif TWO_POINT_LINE[1] <= motor.head_vertical <= TWO_POINT_LINE[2] :
            if self.throw_ball_action == True :
                rospy.logdebug(f"--------------------stop at the basket---------------------- {motor.head_vertical - 2048}")
                motor.bodyauto_close(0)
                target.basket_parameter()
                time.sleep(1)
                rospy.logdebug(f"頭部水平旋轉調整")
                motor.move_head(1,1740,880,880,30)
                time.sleep(0.8)
                rospy.logdebug(f"伸手準備投籃")
                send.sendBodySector(743)
                time.sleep(1.5)
                self.throw_ball_action = False

            else:
                if target.basket_x != 0 :
                    
                    if abs(target.basket_x-160) > 3:
                        target.basket_parameter()
                        rospy.logdebug(f"腰部修正")
                        motor.WaistFix(target.basket_x,160)
                        rospy.logdebug(f"abs(target.basket_x-160) = {abs(target.basket_x-160)}")
                    elif abs(target.basket_x-160) < 3:
                        time.sleep(1)
                        rospy.logdebug(f"執行2分球投籃")
                        send.sendBodySector(570)
                        self.step ="finish"
                else:
                    rospy.logdebug(f"框不在視野中->往左邊轉腰")
                    #motor.waist_rotate(1900,70) #?????????
                    motor.Null_WaistFix(1700)

        else :
            rospy.logdebug(f"頭部馬達垂直刻度與抓球角度差太多")
            rospy.logdebug(f"straight調整")
            time.sleep(0.05)
            motor.degree_straight(TWO_POINT_LINE[0],TWO_POINT_LINE[1],TWO_POINT_LINE[2])

        

    def stratagy_3(self):
        rospy.logdebug(f"stratagy_3")
        ######################################## 三分球用size判斷 ########################################
        
        target.basket_parameter()
            

        motor.trace_revise(target.basket_x,target.basket_y,35)

        if abs(motor.head_horizon-2048) > 300  :
            rospy.logdebug(f"頭部馬達水平刻度偏差>步態影響的")
            rospy.logdebug(f"rotate調整")
            time.sleep(0.05)
            motor.body_trace_rotate(40)

        if THREE_POINT_LINE[1] <= target.basket_size <= THREE_POINT_LINE[2] :
            if self.throw_ball_action == True :
                rospy.logdebug(f"--------------------stop at the basket---------------------- {target.basket_size}")
                motor.bodyauto_close(0)
                target.basket_parameter()
                rospy.logdebug(f"target.basket_size = {target.basket_size}")
                time.sleep(3)

                rospy.logdebug(f"三分球動作預備")
                rospy.logdebug(f"頭部水平旋轉調整")
                motor.move_head(1,2000,880,880,30)
                time.sleep(0.8)
                rospy.logdebug(f"伸手準備投籃")
                time.sleep(1.5)
                self.throw_ball_action = False

            else:
                if target.basket_x != 0 :

                    if abs(target.basket_x-160) > 3:
                        target.basket_parameter()
                        rospy.logdebug(f"腰部修正")
                        motor.WaistFix(target.basket_x,160)
                        rospy.logdebug(f"abs(target.basket_x-160) = {abs(target.basket_x-160)}")
                    elif abs(target.basket_x-160) < 3:
                        time.sleep(1)
                        rospy.logdebug(f"執行3分球投籃")
                        rospy.logdebug(f"手臂旋轉調整")  
                        rospy.logdebug(f"投籃")
                        self.step ="finish"
                else:
                    rospy.logdebug(f"框不在視野中->往左邊轉腰") 

        else :
            rospy.logdebug(f"頭部馬達垂直刻度與抓球角度差太多")
            rospy.logdebug(f"straight調整")
            time.sleep(0.05)
            motor.size_straight(THREE_POINT_LINE[0],THREE_POINT_LINE[1],THREE_POINT_LINE[2])

        

    def stratagy_5(self):
        rospy.logdebug(f"stratagy_5")
        ######################################## 五分球用size判斷 ########################################

        target.basket_parameter()  

        if self.throw_ball_action:
            motor.trace_revise(target.basket_x,target.basket_y,35)
            if abs(motor.head_horizon-2048) > 100 :
                rospy.logdebug(f"頭部馬達水平刻度偏差>步態影響的")
                rospy.logdebug(f"rotate調整")
                time.sleep(0.05)
                motor.body_trace_rotate(40)

            elif FIVE_POINT_LINE[1] <= target.basket_size <= FIVE_POINT_LINE[2] :
                
                    rospy.logdebug(f"--------------------stop at the basket---------------------- {target.basket_size}")
                    motor.bodyauto_close(0)
                    target.basket_parameter()
                    rospy.logdebug(f"target.basket_size = {target.basket_size}")
                    time.sleep(3)
                    rospy.logdebug(f"五分球動作預備")
                    rospy.logdebug(f"手臂往後舉")
                    send.sendBodySector(470)
                    time.sleep(6)

                    rospy.logdebug(f"頭部調整")    
                    rospy.logdebug(f"頭部水平旋轉調整")                                                
                    motor.move_head(1,FIVE_POINT_HEAD_DEGREE[0],880,880,50)
                    #rospy.logdebug(f"頭部垂直旋轉調整")
                    #motor.move_head(2,2048,880,880,50)
                    time.sleep(2)
                    self.throw_ball_action = False

            else :
                rospy.logdebug(f"頭部馬達垂直刻度與抓球角度差太多")
                rospy.logdebug(f"straight調整")
                time.sleep(0.05)
                motor.size_straight(FIVE_POINT_LINE[0],FIVE_POINT_LINE[1],FIVE_POINT_LINE[2])

        else:
            target.basket_parameter()
            if abs(target.basket_x-160) > 2 :

                # if abs(target.basket_x-160) > 2:
                rospy.logdebug(f"target.basket_size = {target.basket_size}")
                motor.WaistFix(target.basket_x,160)
                rospy.logdebug(f"abs(target.basket_x-160) = {abs(target.basket_x-160)}")

            else :
                rospy.logdebug(f"target.basket_y - 120 = {target.basket_y - 120}")
                # if abs(target.basket_y - 120) > 3 :
                #     motor.trace_revise(target.basket_x,target.basket_y,30)

                # elif abs(target.basket_y - 120) <= 3 :
                time.sleep(1.3)
                time.sleep(0.05)
                motor.basket_distance(BASKET_SIZE_100_150[0],BASKET_SIZE_100_150[1])
                rospy.logdebug(f"throw_strength = {motor.throw_strength}")
                # send.sendSingleMotor(9,-round((motor.distance_new-90)*1.8),15)
                rospy.logdebug(f"可以投射")
                # motor.curry_flag = True
                time.sleep(2)

                time.sleep(0.1)
                rospy.logdebug(f"開爪")
                time.sleep(2)
                # send.sendHandSpeed(503,motor.throw_strength + throw_plus )
                # time.sleep(2)
                rospy.logdebug(f"投射!!!")
                send.sendBodySector(467)
                rospy.logdebug(f"motor.throw_strength + THROW_PLUS = {motor.throw_strength + THROW_PLUS}")
                self.step ="finish"

        
                

        

    def drawline(self):
        send.drawImageFunction(1,0,160,160,0,240,255,255,255) 
        send.drawImageFunction(2,0,0,320,120,120,255,255,255)
        send.drawImageFunction(3,1,target.ball_x_min ,target.ball_x_max ,target.ball_y_min ,target.ball_y_max,255,0,255)
        send.drawImageFunction(4,1,target.basket_x_min ,target.basket_x_max ,target.basket_y_min ,target.basket_y_max,0,0,0)
        target.ball_parameter() 

    def stand(self):
        if send.DIOValue == 11:           #catch ball point  上上下下 
            self.drawline()
            motor.trace_revise(target.ball_x, target.ball_y, 35)
            rospy.loginfo(f'Head_vertical = {motor.head_vertical}')
            time.sleep(0.2) 
            self.step = 'test'

        elif send.DIOValue == 13:       #basket size   上下上下
            self.drawline()
            target.basket_parameter()
            motor.trace_revise(target.basket_x, target.basket_y, 35)
            rospy.loginfo(f'Basket_size = {target.basket_size}, Head_vertical = {motor.head_vertical}')
            time.sleep(0.2) 
            self.step = 'test'

        elif self.step != 'begin' :
            send.sendHeadMotor(1, 2048, 30)
            send.sendHeadMotor(2, 2048, 30)
            motor.bodyauto_close(0)
            time.sleep(1)
            send.sendBodySector(29)
            time.sleep(0.05)
            target.initial()
            motor.initial()
            self.initial()
            time.sleep(0.05)
            self.step = 'begin'
            rospy.logdebug(f'-------------------reset and stoping-------------------------')
            rospy.loginfo(f'主策略指撥關閉 -> 機器人回復初始狀態')
            time.sleep(0.05)

        else:
            pass
        

        



if __name__ == '__main__' :
    
    target = Target_Location()
    motor = Motor_Move()
    bb = BasketBall()

    try:
    
        while not rospy.is_shutdown():
            bb.main()


    except rospy.ROSInterruptException:
        pass

