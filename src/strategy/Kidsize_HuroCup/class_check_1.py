#!/usr/bin/env python
#coding=utf-8
from cmath import sqrt
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
        for j in range (self.color_mask_subject_red):
            if send.color_mask_subject_size [0][j] > 70 :
                self.ball_x =  send.color_mask_subject_X [0][j]
                self.ball_y = send.color_mask_subject_Y [0][j]
                self.ball_size = send.color_mask_subject_size [0][j]
                self.ball_x_min = send.color_mask_subject_XMin[0][j] 
                self.ball_y_min = send.color_mask_subject_YMin[0][j] 
                self.ball_x_max = send.color_mask_subject_XMax[0][j] 
                self.ball_y_max =send.color_mask_subject_YMax[0][j]
            else :
                self.ball_x = 0
                self.ball_y = 0
                self.ball_size = 0

            
    
    def basket_parameter(self):
        self.color_mask_subject_orange = send.color_mask_subject_cnts[5]
        for j in range (self.color_mask_subject_orange):
            if send.color_mask_subject_size [5][j] > 200 :
                self.basket_x =  send.color_mask_subject_X [5][j]
                self.basket_y = send.color_mask_subject_Y [5][j]
                self.basket_size = send.color_mask_subject_size [5][j]
                self.basket_x_min = send.color_mask_subject_XMin[5][j] 
                self.basket_y_min = send.color_mask_subject_YMin[5][j] 
                self.basket_x_max = send.color_mask_subject_XMax[5][j] 
                self.basket_y_max =send.color_mask_subject_YMax[5][j]
            else :
                self.basket_x= 0
                self.basket_y = 0
                self.basket_size = 0
   

class motor_move():
    #motor = motor_move()
    target = target_location()
    motor = target_location()
    def __init__(self):
        self.head_horizon = 2048                #頭部水平刻度
        self.head_vertical = 2048               #頭部垂直刻度
        self.waist_position = 2048              #腰當下的刻度
        self.head_horizon_flag = 0              #找目標物的旗標
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
        
    def switch_control(self):                                                       #need test
        time.sleep(0.1)
        target.ball_parameter()
        target.basket_parameter()
        if    send.DIOValue == 9:  
            motor.test_distance()                                                   #籃框               上下下下

        elif    send.DIOValue == 10:                                                  #球                 下上下下
            motor.basket_distance(basket_size_60_90[0],basket_size_60_90[1])
            #print Basket size  Distance_60 Distance_90 Distance_fin

        elif    send.DIOValue == 27 or send.DIOValue == 11:                           #開啟三分策略        上上下下     
            #               24+3                    8+3
            self.switch_flag = 0
            #print("SW = ", self.switch_flag)

        elif    send.DIOValue == 31 or send.DIOValue == 15:                           #開啟五分策略        上上上下
            #               24+7                    8+7
            self.switch_flag = 1
            #print("SW = ", self.switch_flag)

        else:
            self.switch_flag = 2


    def move_head(self,ID,Position,max_head_horizon_size,max_head_vertical_size,Speed):
        send.sendHeadMotor(ID,Position,Speed)
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

        if self.head_horizon_flag == 1 :
            if self.head_horizon > left_place:
                self.move_head(1,self.head_horizon,880,880,speed)
                self.head_horizon = self.head_horizon - speed
                time.sleep(delay) 
            else :
                self.move_head(2,up_place,880,880,100)
                self.head_horizon_flag = 0                   
        else :                
            if  self.head_horizon < right_place:
                self.move_head(1,self.head_horizon,880,880,speed)
                self.head_horizon = self.head_horizon + speed
                time.sleep(delay) 
            else :
                self.move_head(2,down_place,880,880,100)
                self.head_horizon_flag = 1
    
    def trace_revise(self,x_target,y_target,speed) :
        #if abs(x_target) > 0 and abs(y_target) > 0:
        self.x_differ =  x_target - 160 
        self.y_differ =  y_target - 120 
        self.x_degree = self.x_differ * (70.42 / 320)
        self.y_degree = self.y_differ * (43.3 / 240)
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
        
        if (self.head_vertical - degree) - 250 > ball_degree :
            motor.MoveContinuous(1900+correct[0],0+correct[1],0+correct[2],100,100,2) #!!!!!!!!!!!!!!!!!!!!!!
            print( "go ahead bigbigbigbigbigbigbig= ",self.head_vertical)
            time.sleep(0.05)
        elif (self.head_vertical - degree) - 250 < ball_degree and (self.head_vertical - degree)  > ball_degree:
            motor.MoveContinuous(1000+correct[0],0+correct[1],0+correct[2],100,100,2) #!!!!!!!!!!!!!!!!!!!!!!
            print( "go ahead smallsmallsmallsmall= ",self.head_vertical)
            time.sleep(0.05)
        elif (self.head_vertical - degree) < -ball_degree :
            motor.MoveContinuous(-1500+correct[0],0+correct[1],0+correct[2],100,100,2)
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
            motor.move_head(1,1848,880,880,50) #1748
            time.sleep(2.5)           #!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1
            target.ball_parameter()
            
            
    # def waist_revise(self):
            
    #             time.sleep(1)
    #             send.sendSingleMotor(9,round(self.x_body_rotate ),30)
    #             print("meowmeowmeoemeowmeow",self.x_body_rotate)
    #             time.sleep(0.05)

    def WaistFix(self, Target_X, Target_Y, TargetXCenter, TargeYCenter):#轉腰調整Basket.X與Baskethead_verticalBaseLine的誤差
        # self.MoveW = round((TargetXCenter - Target_X)*0.5)
        # self.MoveY = TargeYCenter - Target_Y

        self.MoveW = TargetXCenter - Target_X
        if self.MoveW > 15:
            self.MoveW = 15
        
        self.waist_reset((self.waist_position + self.MoveW), 20)
        self.move_head(2, self.head_vertical - round(self.y_degree * 4096 / 360 *0.15),880,880,20)
        time.sleep(0.15)
        
        # time.sleep(0.2)
                

    ####################################### basket size version #######################################

    # def body_trace_basket_straight_2(self,basket_size,basket_error) :
        
    #     if target.basket_size - basket_size < -basket_error and  target.basket_size <12000:
    #         motor.MoveContinuous(2500+correct[0],0+correct[1],0+correct[2],100,100,2)#!!!!!!!!!!!!!!!
    #         print( "--------------------go ahead to basket---------------------  ",self.head_vertical)
    #         time.sleep(0.05)

    #     elif target.basket_size - basket_size < -basket_error and  target.basket_size >12000:
    #         motor.MoveContinuous(500+correct[0],0+correct[1],0,0+correct[2],100,100,2)#!!!!!!!!!!!!!!!
    #         print( "--------------------go ahead to basket---------------------  ",self.head_vertical)
    #         time.sleep(0.05)    
    #     elif target.basket_size - basket_size > basket_error :
    #         motor.MoveContinuous(-800+correct[0],0+correct[1],0,0+correct[2],100,100,2)#!!!!!!!!!!!!!!!!!
    #         print( "--------------------go back from basket-------------------- ",self.head_vertical)
    #         time.sleep(0.05)
    #     elif abs(target.basket_size - basket_size) < basket_error :
    #         print( "--------------------stop at the basket----------------------",self.head_horizon - 2048)
    #         #send.sendBodyAuto(0,0,0,0,1,0)
    #         motor.bodyauto_close(0)
    #         target.basket_parameter()
            
    #         time.sleep(1)
    #         motor.move_head(1,1798,880,880,30)
    #         time.sleep(0.8)
    #         send.sendBodySector(3)   #11111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111`
    #         print("-------------------------send.sendBodySector(3)------------------------------")
    #         time.sleep(2)
    #         self.found = True
    #         self.catch = True

     #####################################################################################################

     ####################################### basket degree version #######################################

    def body_trace_basket_straight_2(self,degree,basket_error) :
        
        if self.head_vertical - degree > basket_error  and self.head_vertical > 1980 :
            motor.MoveContinuous(2600+correct[0],0+correct[1],0+correct[2],100,100,2)#!!!!!!!!!!!!!!!
            print( "--------------------go ahead bbbb to basket---------------------  ",self.head_vertical)
            time.sleep(0.05)

        elif self.head_vertical - degree > basket_error and  self.head_vertical < 1980:
            motor.MoveContinuous(1000+correct[0],0+correct[1],0+correct[2],100,100,2)#!!!!!!!!!!!!!!!
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
            motor.move_head(1,1798,880,880,30)
            time.sleep(0.8)
            send.sendBodySector(3)   #準備上籃之動作一（舉手...）111111111111111111111111111111111111111111111111111111111`
            print("-------------------------send.sendBodySector(3)------------------------------")
            time.sleep(2.5)
            self.found = True
            self.catch = True
    
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
            # send.sendBodySector(3)   #33333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333
            print("-------------------------send.sendBodySector(3)------------------------------")
            time.sleep(2)
            self.found = True
            self.catch = True

    def body_trace_basket_straight_5(self,basket_size,basket_error) :
        
        if target.basket_size - basket_size < -basket_error :
            send.sendContinuousValue(300+correct[0],0+correct[1],0,0+correct[2],0)#!!!!!!!!!!!!!!!
            print( "--------------------go ahead to basket---------------------  ",self.head_vertical)
            time.sleep(0.05)

        elif target.basket_size - basket_size > basket_error and target.basket_size > 1000:
            send.sendContinuousValue(-1200+correct[0],0+correct[1],0,0+correct[2],0)#!!!!!!!!!!!!!!!!!
            print( "--------------------go back from basket-------------------- ",self.head_vertical)
            time.sleep(0.05)   
        elif target.basket_size - basket_size > basket_error and target.basket_size < 1000:
            motor.MoveContinuous(-800+correct[0],0+correct[1],0+correct[2],100,100,2)#!!!!!!!!!!!!!!!!!
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
            # # send.sendBodySector(3)   #5555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555
            # print("-------------------------send.sendBodySector(3)------------------------------")
            # time.sleep(2)
            self.found = True
            self.catch = True
    
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
        #if target.basket_size >= 300 and abs(motor.head_horizon-2048) < 600:

        if abs(target.basket_x - 160) > 5  or abs(target.basket_y - 120) > 4 :
            motor.trace_revise(target.basket_x,target.basket_y,25)
            target.basket_parameter() 
            
            time.sleep(0.05) 
            
        else :
            print("Basket Y = ",target.basket_size)
                


    # def basket_distance(self):
    #     target.basket_parameter()
    #     if target.basket_size >= 2250 and target.basket_size < 2490 : #55cm
    #         self.throw_strength = abs( 40 * ((target.basket_size - 2490)/250) + 45 * ((target.basket_size - 2250)/250))
        
    #     elif target.basket_size >= 2010 and target.basket_size < 2250 : #60cm
    #         self.throw_strength = abs( 45 * ((target.basket_size - 2250)/250) + 50 * ((target.basket_size - 2010)/250))
        
    #     elif target.basket_size >= 1770 and target.basket_size < 2010 : #65cm
    #         self.throw_strength = abs( 50 * ((target.basket_size - 2010)/250) + 55 * ((target.basket_size - 1770)/250))
        
    #     elif target.basket_size >= 1530 and target.basket_size < 1770 : #70cm
    #         self.throw_strength = abs( 55 * ((target.basket_size - 1770)/250) + 60 * ((target.basket_size - 1530)/250))

    #     elif target.basket_size >= 1290 and target.basket_size < 1530 : #75cm
    #         self.throw_strength = abs( 60 * ((target.basket_size - 1530)/250) + 65 * ((target.basket_size - 1290)/250))    

    #     elif target.basket_size >= 1050 and target.basket_size < 1290 : #80cm
    #         self.throw_strength = abs( 65 * ((target.basket_size - 1290)/250) + 70 * ((target.basket_size - 1050)/250))

    #     elif target.basket_size >= 810 and target.basket_size < 1050 : #85cm
    #         self.throw_strength = abs( 70 * ((target.basket_size - 1050)/250) + 75 * ((target.basket_size - 810)/250))           

    #     elif target.basket_size >= 570 and target.basket_size < 810 : #90cm
    #         self.throw_strength = abs( 75 * ((target.basket_size - 810)/250) + 80 * ((target.basket_size - 570)/250)) 



    # def distance_test(self,sixty,ninty,sw):
    #         target.basket_parameter()
    #         if target.basket_y != 0:
    #             print("target.basket_y - 120=====",target.basket_y - 120)

    #             if abs(target.basket_y - 120 ) > 3 :

    #                 print("working------------------------------------")

    #                 time.sleep(1)
    #                 self.move_head (2, round(self.head_vertical + (120 - target.basket_y)0.7),880,880,20)
    #                 time.sleep(0.2)
    #                 target.basket_parameter()

    #             elif abs(target.basket_y - 120 ) <= 3  and sw == 2:
    #                 target.basket_parameter()
    #                 time.sleep(1)
    #                 self.distance = abs(sqrt(3600sixty/target.basket_size))
    #                 print("self.distance = ",self.distance)
    #             elif abs(target.basket_y - 120 ) <= 3  and sw == 1:
    #                 target.basket_parameter()
    #                 time.sleep(1)
    #                 self.distance = abs(sqrt(8100*ninty/target.basket_size))
    #                 print("self.distance = ",self.distance)

    def basket_distance(self,six,nine):
        print("target.basket_y - 120",target.basket_y - 120)
        self.move_head(1,2048,880,880,50)
        target.basket_parameter()
        if abs(target.basket_y - 120) > 3 :
            if target.basket_y == 0 :
                self.move_head(1,1850,880,880,50)
            elif target.basket_y - 120 > 0 :
                self.move_head(2,self.head_vertical - 1,880,880,50)
            elif target.basket_y - 120 < 0 :
                self.move_head(2,self.head_vertical + 1,880,880,50)
            target.basket_parameter()
            print("Basket Y = ",target.basket_y)
        
        else  :
            self.sixty_distance = sqrt(abs((3600*six)/target.basket_size))
            self.ninty_distance = sqrt(abs((8100*nine)/target.basket_size))
            if ( six + nine ) / 2 > target.basket_size :
                self.distance_new = self.sixty_distance
            else :
                self.distance_new = self.ninty_distance
            print("Basket size = ",target.basket_size)
            print("Distance_60 = ",self.sixty_distance)
            print("Distance_90 = ",self.ninty_distance)
            print("Distance_fin = ",self.distance_new)


        


#     BasketInfo->Headhead_verticalAngle = (double)(BasketInfo->head_verticalHeadPosition - 1024) * Scale2Deg + BasketInfo->RobotStandFeedBack + BasketInfo->FeedBackError;
#     BasketInfo->Distance_60 = sqrt( abs( ( BasketInfo->dis60_x * BasketInfo->dis60_x * BasketInfo->SizeOfDist[1]) / BasketInfo->Basket.size ) );
#     BasketInfo->Distance_90 = sqrt( abs( ( BasketInfo->dis90_x * BasketInfo->dis90_x * BasketInfo->SizeOfDist[4]) / BasketInfo->Basket.size ) );
#     if((BasketInfo->SizeOfDist[1] + BasketInfo->SizeOfDist[4])/2 > BasketInfo->Basket.size)
#     {
#         BasketInfo->Distancenew = BasketInfo->Distance_60 + BasketInfo->AreaDisError ;
#     }
#     else
#     {
#         BasketInfo->Distancenew = BasketInfo->Distance_90 + BasketInfo->AreaDisError ;
#     }

#     ROS_INFO("head_verticalHeadPosition = %d", BasketInfo->head_verticalHeadPosition);
#     ROS_INFO("Headhead_verticalAngle = %lf", BasketInfo->Headhead_verticalAngle);
#     ROS_INFO("Basket.size = %lf", BasketInfo->Basket.size);
#     ROS_INFO("Distance_60 = %lf", BasketInfo->Distance_60);
#     ROS_INFO("Distance_90 = %lf", BasketInfo->Distance_90);

#}
    
    def bodyauto_close(self,next_state):
        #if send.is_start == True :   # 0 stop  1 go
        if self.now_state == next_state :
            
            pass
        elif self.now_state != next_state :
            send.sendBodyAuto(0,0,0,0,1,0)
            self.now_state = next_state
                

        # elif send.is_start == False :
        #     if self.now_state == 0 and reset_close == 0:
        #         send.sendHeadMotor(1,2048,30)
        #         send.sendHeadMotor(2,2048,30)
        #         time.sleep(0.05)
        #         send.sendBodySector(29)
        #         print("-------------------reset and stoping-------------------------")
        #         print("-------------------reset and stoping-------------------------")
        #         print("好棒棒")
        #         print("")
        #         print("◢███◣。。。。。。◢███◣" )
        #         print("▇▇□██。。。。。。██□██")
        #         print("  ◥███◤◢████◣◥███◤")
        #         print("◢█。。。。。。。。。。█◣")
        #         print("█。╔╗。。。。。。。╔╗。█")
        #         print("█。∥●。。。╭╮。。。∥●。█")
        #         print("█。╚╝。。。╰╯。。。╚╝。█")
        #         print("   ◥▆▆▆▆▆▆▆▆▆▆▆▆▆▆")
        #         time.sleep(0.05)
        #         print("\n")
        #         print(" ┌─╮◆╭═┐╭═┐╭═┐◆╭─┐")

        #         print("│┌╯◆║加║║油║║囉║◆╰┐│")

        #         print(" ╰╯↘◆└═╯└═╯└═╯◆↙╰╯")
                
        

        #         print("\n")
        #         print("..../\„./\...../\„./\ ")             
        #         print("... (=';'=)....(=';'=) ♥♥")              
        #         print("..../*♥♥**\ ♥  /*♥♥**\ ")
        #         print(".(.| |..| |.)(.| |..| |.)♥")
    
        #     elif motor.now_state ==0 and reset_close == 1:
        #         reset_close = 0
        
        #     elif motor.now_state == 1 :
        #         send.sendHeadMotor(1,2048,30)
        #         send.sendHeadMotor(2,2048,30)
        #         time.sleep(0.5)
        #         send.sendBodySector(29)
        #         send.sendBodyAuto(0,0,0,0,1,0)
        #         print("-------------------reset and stoping-------------------------")
            
        
       
   
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
    
    stop=1
    target = target_location()
    motor = motor_move()
    step ='being'
    #step  = ['begin','find_ball','open_ball_trace','walk_to_ball','ball_trace','catch_ball','find_basekt','basket_trace','walk_to_basket','find','waisting','finish']
    
    #           0              1                2           3             4                5             6           7               8      9           10
    
    # sw = 2
    gazebo_robot = 1
    # 0 for gazebo 1 for robot
    stategy_or_test = 1
    # 0 for test 1 for stategy

    basket_size_60_90 =[2250,800]


    throw_ball_point = [2250,800,1800] #投籃未寫 #16500
    # # for size          三分  五分  灌籃
    # throw_ball_point = [0,0,1300] 
    # for degree          三分  五分  灌籃
    

    correct       = [-200,0,0]
    left_correct  = [-200,0,4]
    right_correct = [-200,0,-4]
    #                  x , y , theta   


    basket_error = [50,30,80]
    #  for size    三分  五分  灌籃
    # basket_error = [0,0,100]
    # for degree          三分  五分  灌籃

    ball_correct = [20,100]

    trace_parameter =[80]#25
    too_big = True
    reset_close = 0
    try:
        
        
        while not rospy.is_shutdown():
            motor.switch_control()
            sw = motor.switch_flag
            if send.is_start==True :
                reset_close = 0
                send.drawImageFunction(1,0,160,160,0,240,255,255,255) 
                send.drawImageFunction(2,0,0,320,120,120,255,255,255)
                send.drawImageFunction(3,1,target.ball_x_min ,target.ball_x_max ,target.ball_y_min ,target.ball_y_max,255,0,255)
                send.drawImageFunction(4,1,target.basket_x_min ,target.basket_x_max ,target.basket_y_min ,target.basket_y_max,0,0,0)
                target.ball_parameter() 
                

                if motor.found == False  :
                    if step == 'begin':
                        step = 'find_ball'

                    elif step == 'find_ball':#@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
                        
                        if target.ball_size < 70 :
                            motor.view_move(2698,1498,1800,1098,55,0.05)                 
                            time.sleep(0.05)
                            print("start to find the ball")
                            print("stop====\n",stop)
                            target.ball_parameter()  
                            print("  ball => x:",target.ball_x," y:",target.ball_y," size:",target.ball_size)
                        elif target.ball_size > 70 :
                            send.sendBodySector(9) #讓手回歸自我們的初始手部位置,原是AR的
                            
                            step = 'open_ball_trace'#@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
                            

                    elif step == 'open_ball_trace' :#@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
                        if abs(target.ball_x - 160) > 5  or abs(target.ball_y - 120) > 20 :
                            target.ball_parameter() 
                            print("open_ball_trace is opening")
                            motor.trace_revise(target.ball_x,target.ball_y,25)
                            time.sleep(0.05) 
                        else :
                            if motor.head_vertical <= 1550:
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
                        
                        step = 'ball_trace'#@@@@@@@@@@@@@@@@@@

                    elif  step == 'ball_trace' :#@@@@@@@@@@@@@@@@@@
                        
                        target.ball_parameter()   
                        print(" ball => x:",target.ball_x," y:",target.ball_y," size:",target.ball_size)                   
                        motor.trace_revise(target.ball_x,target.ball_y,25)
                        print("abs(motor.x_body_rotate)",abs(motor.x_body_rotate),motor.head_horizon-2048)

                        if too_big == True:
                            motor.MoveContinuous(-1200+correct[0],0+correct[1],0+correct[2],100,100,1)
                            print("meowmeowmeowmeowmeow")

                            if motor.head_vertical >=1660 :
                                too_big = False


                        
                        if too_big == False:
                            if abs(motor.head_horizon-2048) > trace_parameter[0]  :  
                                motor.body_trace_rotate(trace_parameter[0])    
                                print("motor.head_vertical=========",motor.head_vertical)
                            
                            elif abs(motor.head_vertical - 1590) > 5 : #1320是條球的距離150是誤差
                                print(motor.head_horizon - 2048)
                                motor.body_trace_straight(1610,ball_correct[gazebo_robot])#!!!!!!!!!!!!!!!!!!!!!!!!!!!球的距離夠motor.found = True
                                print("motor.head_vertical-1320 = ",motor.head_vertical-1590)
# ............................................................................................................................
# ............................................................................................................................
# ............................................................................................................................
# ............................................................................................................................

                elif  motor.found == True and motor.catch == False : 
                    target.basket_parameter() 
                    if step == 'ball_trace' :#@@@@@@@@@@@@@@@@@@
                        time.sleep(0.03)
                        target.ball_parameter()  
                        if abs(target.ball_x-160) < 3 :
                                
                                print("step======",step)
                                step = 'catch_ball'#@@@@@@@@@@@@@@@@@@

                        elif target.ball_x != 0 :
                            target.ball_parameter()
                            motor.WaistFix(target.ball_x,target.ball_y,160,120)
                            print("abs(target.ball_x-160)hhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhh",abs(target.ball_x-160))
                            


                    elif step == 'catch_ball' :   #@@@@@@@@@@@@@@@@@@
                        
                        
                        # send.sendBodySector(2)    #1111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111
                        send.sendBodySector(6)    #2222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222
                        time.sleep(2)     
                        print("stop to the ball")
                        print("----------------------------------ready to waist_reset---------------------------------------")
                        
                        
                        motor.waist_reset(2048,50)
                        time.sleep(2)  
                        send.sendBodySector(7)    #2222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222
                        #send.sendBodySector(3)
                        time.sleep(4)
                        print(".................................................")
                        step = 'find_basekt'#@@@@@@@@@@@@@@@@@@
                                                                        

                    if step == 'find_basekt' :#@@@@@@@@@@@@@@@@@@
                        target.basket_parameter()
                        if target.basket_size < 350 :
                                
                                print("find_basket")
                                motor.view_move(2448,1648,1948,1898,50,0.05)
                                time.sleep(0.04)
                                target.basket_parameter()
                                print("  basket => x:",target.basket_x," y:",target.basket_y," size:",target.basket_size)
                        elif target.basket_size > 350 :
                                step = 'basket_trace'#@@@@@@@@@@@@@@@@@@
                                target.basket_parameter()
                                print("jump to basket_trace   !!!!!!!!!")
                                

                    elif step == 'basket_trace' :
                        if abs(target.basket_x - 160) > 8  or abs(target.basket_y - 120) > 6 :
                            target.basket_parameter() 
                            motor.trace_revise(target.basket_x,target.basket_y,25)
                            print("-------------basekt_trace有進喔--------------")
                            time.sleep(0.05) 
                        else :
                                                    
                            motor.bodyauto_close(1)                         
                            step = 'walk_to_basket' #@@@@@@@@@@@@@@@@@@


                    elif  step == 'walk_to_basket' :#@@@@@@@@@@@@@@@@@@
                        print("walk_to_basket")
                        target.basket_parameter()
                        print(" basket => x:",target.basket_x," y:",target.basket_y," size:",target.basket_size)
                        motor.trace_revise(target.basket_x,target.basket_y,25)

                        


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

                            elif sw == 1:
                                print("----------------start the action of get point--------------")
                                motor.body_trace_basket_straight_5(throw_ball_point[sw],basket_error[sw])
                                print("motor.catch = ",motor.catch)
                                if motor.catch == True :
                                    print("step is ",step)#@@@@@@@@@@@@@@@@@@@@@@@

                    
                    
                
                elif  motor.found == True and motor.catch == True :
                    if step == 'walk_to_basket' :
                        target.basket_parameter()
                        if sw == 0:
                            if target.basket_x != 0 :
                                motor.WaistFix(target.basket_x,target.basket_y,160,120)
                                print("abs(target.basket_x-160)",abs(target.basket_x-160))
                                if abs(target.basket_x-160) < 3:
                                    step = 'find'#@@@@@@@@@@@@@@@@@@@@@
                                    print("step==========",step)
                        elif sw == 1:
                            if target.basket_x != 0 :
                                motor.WaistFix(target.basket_x,target.basket_y,160,120)
                                print("abs(target.basket_x-160)",abs(target.basket_x-160))
                                if abs(target.basket_x-160) < 3:
                                    step = 'find'#@@@@@@@@@@@@@@@@@@@@
                                    print("step==========",step)#@@@@@@@@@@@@@@@@@@@@
                        elif sw == 2 :
                            if target.basket_x != 0 :
                                motor.WaistFix(target.basket_x,target.basket_y,160,120)
                                print("abs(target.basket_x-160)",abs(target.basket_x-160))
                                if abs(target.basket_x-160) < 3:
                                    step = 'find'#@@@@@@@@@@@@@@@@@@@@@
                                    print("step======",step)#@@@@@@@@@@@@@@@@@
                    

                    elif step == 'find' :
                        if sw == 0:
                            time.sleep(0.5)
                            send.sendBodySector(5353)
                            print("3333333333")
                        elif sw == 1:
                            time.sleep(0.5)
                            send.sendBodySector(5353)
                            print("555555")

                        elif sw == 2 :
                            time.sleep(1)
                            send.sendBodySector(4) #上籃之動作二（把球放入籃框）1111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111
                            print("2222222222")
                            # if  abs(motor.head_horizon - 2048) <= 5  and abs(target.basket_x - 160) <= 1 :
                            #     print("---------------gogogogogogogogogogogogogogoro-------------")
                        step = 'waisting' #都一直線#@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

                    elif step == 'waisting' :
                        print("finish")

                            
                            
                
            else :
                if step != 'begin' and (send.DIOValue == 8 or send.DIOValue == 24 or send.DIOValue == 27 or send.DIOValue == 31):

                    motor.bodyauto_close(0)
                    target = target_location()
                    motor = motor_move()
                    step = 'begin'
                    send.sendHeadMotor(1,2048,30)
                    send.sendHeadMotor(2,2048,30)
                    time.sleep(0.05)
                    send.sendBodySector(29)
                    print("-------------------reset and stoping-------------------------")
                    print("-------------------reset and stoping-------------------------")
                    print("好棒棒")
                    print("")
                    print("◢███◣。。。。。。◢███◣" )
                    print("▇▇□██。。。。。。██□██")
                    print("  ◥███◤◢████◣◥███◤")
                    print("◢█。。。。。。。。。。█◣")
                    print("█。╔╗。。。。。。。╔╗。█")
                    print("█。∥●。。。╭╮。。。∥●。█")
                    print("█。╚╝。。。╰╯。。。╚╝。█")
                    print("   ◥▆▆▆▆▆▆▆▆▆▆▆▆▆▆")
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