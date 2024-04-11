#!/usr/bin/env python
#coding=utf-8
from cmath import sqrt
from re import T
import time
from traceback import print_tb
import numpy as np
import rospy
import math
from Python_API import Sendmessage

#======================================================================================

CORRECT       = [-800, -1400, 2]        #原地踏步修正
LEFT_CORRECT  = [-700, -1500, 3]        #左旋修正
RIGHT_CORRECT = [-400, -500, -3]       #右旋修正
#                 x , y , theta 

#======================================================================================

BASKET_SIZE_60_90 = [9870, 3224]       #sector 5301                                                     #投籃時測量的籃框距離方法 #五分投籃時站姿高度看籃框size測距離
FIVEPOINT_HEAD_Y_DEGREE = [1956]      #投出去偏向左邊＝>頭往左轉（大）-朝1960 ;  投出去偏向右邊＝>頭往右轉（小）-朝1940    #投籃前頭會固定一個角度，並扭腰
THROW_BALL_PLUS = 200                 #line  0   left side 0 right side  4
#15.9 - 150
#三用電表15.7以上ˇ
#======================================================================================

CATCH_BALL_LINE = [1850, 1750, 1680]            # slow_degree,stop_degree,backward_degree
TWO_POINT_LINE  = [1750, 1600, 1500]            # slow_degree,stop_degree,backward_degree
THREE_POINT_LINE = [1995, 1970, 1870]     # forward_stop_size < forward_slow_size < backward_slow_size < backward_stop_size #上下上下-30
FIVE_POINT_LINE  = [2700, 2850, 2900, 3000]         # forward_stop_size < forward_slow_size < backward_slow_size < backward_stop_size
#注意 size數值調越大會離籃框越近！！！

send = Sendmessage()

class BasketBall():
    def __init__(self):
        self.head_y_down_adjust = False
        self.head_y_up_adjust = False
        self.ready_dunk = False
        self.ready_shoot = False
        self.aiming_finish = False
        self.correct_left_flag = False
        self.correct_right_flag = False
        self.correct_back_flag = False
        self.correct_forward_flag = False
        self.step = 'begin'
        self.sw = 0
    
    def initial(self):
        self.head_y_down_adjust = False
        self.head_y_up_adjust = False
        self.ready_dunk = False
        self.ready_shoot = False
        self.aiming_finish = False
        self.correct_left_flag = False
        self.correct_right_flag = False
        self.correct_back_flag = False
        self.correct_forward_flag = False
        self.step = 'begin'
        self.sw = 0
    
    def main(self):
        target.ball_parameter()
        target.basket_parameter()
        motor.draw()
        if send.is_start: #send.Web 
            if self.step =='begin':
                self.begin()

            elif self.step == 'find_ball':
                self.find_ball()   

            elif self.step == 'start_gait':
                self.start_gait()

            elif self.step == 'walk_to_ball':
                self.walk_to_ball()

            elif self.step == 'correct_catch_ball':
                self.correct_catch_ball()

            elif self.step == 'catch_ball' :
                self.catch_ball() 
            
            elif self.step == 'find_basket':
                self.find_basket()

            elif self.step == 'stratagy_2':
                self.stratagy_2()

            elif self.step == 'stratagy_3':
                self.stratagy_3()

            elif self.step == 'stratagy_5':
                self.stratagy_5()

        else :
            
            if send.DIOValue == 11:           #catch ball point  上上下下 
                motor.trace_revise(target.ball_x, target.ball_y, 35)
                rospy.loginfo(f'Head_vertical = {motor.head_vertical}')
                rospy.loginfo(f'Ball_size = {target.ball_size}')
                time.sleep(0.2) 
                self.step = 'test'

            elif send.DIOValue == 13:       #basket size   上下上下
                motor.trace_revise(target.basket_x, target.basket_y, 35)
                rospy.loginfo(f'Basket_size = {target.basket_size}, Head_vertical = {motor.head_vertical}, basket_x ={target.basket_x}')
                rospy.loginfo(f'Head_vertical = {motor.head_vertical}')
                # rospy.loginfo(f'Head_horizon = {motor.head_horizon}')
                # motor.size_IMU_correct(target.basket_size)
                # rospy.loginfo(f'corrected_size = {motor.corrected_size}')
                time.sleep(0.2) 
                self.step = 'test'

            elif self.step != 'begin' :
                send.sendHeadMotor(1, 2048, 30)
                send.sendHeadMotor(2, 2048, 30)
                target.initial()
                motor.initial()
                self.initial()
                time.sleep(0.05)
                motor.bodyauto_close(0)
                time.sleep(1)
                send.sendBodySector(29)
                time.sleep(3)
                # send.sendBodySector(1)
                time.sleep(2)
                # send.sendBodySector(30)
                time.sleep(1)
                self.step = 'begin'
                rospy.logdebug(f'-------------------reset and stoping-------------------------')
                rospy.loginfo(f'主策略指撥關閉 -> 機器人回復初始狀態')
                time.sleep(0.05)

    def begin(self):
        ####################################### switch #######################################

        send.sendSensorReset(1, 1, 1)
        if send.DIOValue == 24:  #開啟二分策略  下下下下
            self.sw = 2
            rospy.loginfo(f'SW = {self.sw}')

        elif send.DIOValue == 25: #開啟三分策略  上下下下                                 
            self.sw = 0
            rospy.loginfo(f'SW = {self.sw}')

        elif send.DIOValue == 31: #開啟五分策略  上上上(上)                                 
            self.sw = 1
            rospy.loginfo(f'SW = {self.sw}')

        else :
            self.sw = 2
            rospy.loginfo(f'SW = {self.sw}')
        #[sw==2]->2分球;[sw==0]->3分球;[sw==1]->5分球

        ######################################## switch #######################################
        rospy.logdebug(f'開始執行初始化')
        #send.sendBodySector(9) #讓手回歸自我們的初始手部位置,原是AR的
        time.sleep(0.05)
        # send.sendBodySector(8910)   #步態調整
        time.sleep(0.05)
        self.step = 'find_ball'
        
    def find_ball(self):
        if not self.head_y_down_adjust:
            time.sleep(1)
            rospy.logdebug(f'頭部抬起尋框')
            motor.move_head(2,1700,880,880,50)                
            self.head_y_down_adjust = True
            time.sleep(1)
        else:
            if target.ball_size <= 350:   #球在視野中太小
                rospy.logdebug(f'球在視野中太小->大範圍尋球')
                #motor.view_search_left(2428, 1668, 1800, 1200, 40, 0.05)
                motor.view_search(2428, 1668, 1800, 1200, 40, 0.05)
            elif target.ball_size > 350:   #球在視野中夠大
                if abs(target.ball_x - 160) > 6  or abs(target.ball_y - 120) > 8:  #讓球在畫面中心
                    rospy.logdebug(f'球在視野中夠大->鎖定球')
                    motor.trace_revise(target.ball_x, target.ball_y, 25) 
                    time.sleep(0.05)
                # elif (CATCH_BALL_LINE[2] <= motor.head_vertical <= CATCH_BALL_LINE[1]-20) and (abs(motor.head_horizon-2048) <= 300):
                #     time.sleep(1.2)
                #     rospy.loginfo(f'到達夾球範圍 STOP!!, self.head_vertical = {motor.head_vertical}')                
                #     time.sleep(0.05)
                #     motor.trace_revise(target.ball_x, target.ball_y, 25) 
                #     rospy.logdebug(f'到達可夾球位置')
                #     rospy.logdebug(f'蹲下')
                #     time.sleep(1.0)
                #     send.sendBodySector(5) 
                #     time.sleep(2.0)
                #     rospy.logdebug(f'頭往右轉')
                #     motor.move_head(1, 1820, 880, 880, 50)
                #     time.sleep(2) 
                #     motor.reg = 2048 - motor.head_horizon
                #     motor.search_num = 0
                #     self.step = 'waist_fix'

                else: 
                    motor.reg = 2048 - motor.head_horizon
                    motor.search_num = 0
                    self.step = 'start_gait'   

    def start_gait(self):
        motor.trace_revise(target.ball_x, target.ball_y, 25) 
        motor.bodyauto_close(1)
        time.sleep(0.05)
        if (motor.head_vertical <= 1600): #????
            rospy.logdebug(f'球太大->大倒退')
            motor.MoveContinuous(-2000+CORRECT[0], 0+CORRECT[1], 0+CORRECT[2], 100, 100, 1) #??????
        else:
            rospy.logdebug(f'可進行微小修正')
            self.step = 'walk_to_ball'
    
    def walk_to_ball(self):
        motor.trace_revise(target.ball_x, target.ball_y, 25)
        rospy.loginfo(f"head_vertical = {motor.head_vertical}")

        if (CATCH_BALL_LINE[2] <= motor.head_vertical <= CATCH_BALL_LINE[1]) and (abs(motor.head_horizon-2048) <= 50):  #到達夾球位置
            motor.bodyauto_close(0) #步態停止
            time.sleep(1.2)
            rospy.loginfo(f'到達夾球範圍 STOP!!, self.head_vertical = {motor.head_vertical}')                
            time.sleep(0.05)
            rospy.logdebug(f'到達可夾球位置')
            time.sleep(0.05)
            self.step = 'correct_catch_ball'
        else:
            if abs(motor.head_horizon-2048) > 50:
                rospy.logdebug(f'頭部馬達水平刻度偏差>步態影響的')
                rospy.loginfo(f'rotate調整')
                motor.body_trace_rotate(40)

            else:
                rospy.logdebug(f'頭部馬達垂直刻度與抓球角度差太多')
                rospy.loginfo(f"head_vertical = {motor.head_vertical}")
                rospy.loginfo(f'straight調整')
                motor.ball_trace_straight(CATCH_BALL_LINE[0], CATCH_BALL_LINE[1], CATCH_BALL_LINE[2])                 

    def correct_catch_ball(self):
        if abs(target.ball_x - 160) > 2 or abs(target.ball_y - 120) > 2:
                motor.trace_revise(target.ball_x, target.ball_y, 25) 
                time.sleep(0.05)
        else:
            if (motor.head_horizon-2048) > 40:
                self.correct_left_flag = True
            elif (motor.head_horizon-2048) < -40:
                self.correct_right_flag = True
            if (motor.head_vertical - CATCH_BALL_LINE[1]) > 40:
                self.correct_forward_flag = True
            elif (motor.head_vertical - CATCH_BALL_LINE[1]) < -20:
                self.correct_back_flag = True
            send.sendBodySector(111) 
            time.sleep(3.0)
            if self.correct_left_flag:
                send.sendBodySector(123)#手往left修正
                time.sleep(1)
            elif self.correct_right_flag:
                send.sendBodySector(124)#手往right修正
                time.sleep(1)
            if self.correct_forward_flag:
                send.sendBodySector(121)#手往前修正
                time.sleep(1)
            elif self.correct_back_flag:
                send.sendBodySector(122)#手往後修正
                time.sleep(1)
            self.step = 'catch_ball'

    def catch_ball(self):
        rospy.loginfo(f'正常夾球動作')
        time.sleep(1.5)
        send.sendBodySector(1111)
        time.sleep(1) 
        rospy.logdebug(f'回復站姿')
        if self.correct_left_flag:
            send.sendBodySector(124)
            time.sleep(1)
            self.correct_left_flag = False
        elif self.correct_right_flag:
            send.sendBodySector(123)
            time.sleep(1)
            self.correct_right_flag = False
        if self.correct_forward_flag:
            send.sendBodySector(122)
            time.sleep(1)
            self.correct_forward_flag = False
        elif self.correct_back_flag:
            send.sendBodySector(121)
            time.sleep(1)
            self.correct_back_flag = False
        send.sendBodySector(11111) 
        time.sleep(5)
        self.step = 'find_basket' 
    
    def find_basket(self):
        if target.basket_size < 500:
            rospy.logdebug(f'籃框在視野裡太小->尋框')
            rospy.loginfo(f'basket_size =  {target.basket_size}')
            if not self.head_y_up_adjust:
                time.sleep(1)
                rospy.logdebug(f'頭部抬起尋框')
                motor.move_head(2,1900,880,880,50)                
                self.head_y_up_adjust = True
                time.sleep(3)
            else:                                   
                rospy.logdebug(f'開始尋框')
                rospy.loginfo(f'target.basket_x = {target.basket_x}, target.basket_y = {target.basket_y}, target.basket_size = {target.basket_size}')
                ####################################### view search #######################################
                motor.view_search(2548, 1548, 2048, 1948, 50, 0.04)
        else:                                
            rospy.logdebug(f'籃框在視野裡夠大->判斷策略所需前往的位置')
            time.sleep(2)
            motor.trace_revise(target.basket_x, target.basket_y, 35)
            if self.sw == 0:
                rospy.loginfo(f'3分球')
                motor.bodyauto_close(1)
                time.sleep(1)
                self.step = 'stratagy_3'

            elif self.sw == 1:
                rospy.loginfo(f'5分球')
                motor.bodyauto_close(1)
                time.sleep(1)
                self.step = 'stratagy_5'
                
            elif self.sw == 2:
                rospy.loginfo(f'2分球')
                motor.bodyauto_close(1)
                time.sleep(1)
                self.step = 'stratagy_2'

        ######################################## 二分球仿造catch_ball ######################################## 
    
    def stratagy_2(self):
        if not self.ready_dunk:
            motor.trace_revise(target.basket_x, target.basket_y, 35)
            if ((TWO_POINT_LINE[2]) <= motor.head_vertical <= TWO_POINT_LINE[1]) and (abs(motor.head_horizon-2048) <= 200):
                self.ready_dunk = True
                rospy.loginfo(f'到達可投籃角度 STOP!!, self.head_vertical =  {motor.head_vertical}')
                motor.bodyauto_close(0)
                time.sleep(1)
                rospy.logdebug(f'伸手準備投籃')  
            else:
                if abs(motor.head_horizon - 2048) > 50:
                    rospy.logdebug(f'頭部馬達水平刻度偏差 > 步態影響的')
                    rospy.logdebug(f'rotate調整')
                    time.sleep(0.05)
                    motor.body_trace_rotate(40)
                else :
                    rospy.logdebug(f'頭部馬達垂直刻度與抓球角度差太多')
                    rospy.logdebug(f'straight調整')
                    time.sleep(0.05)
                    motor.degree_straight(TWO_POINT_LINE[0], TWO_POINT_LINE[1], TWO_POINT_LINE[2])
        else:            
            if target.basket_x != 0 :
                if abs(target.basket_x - 160) > 2 or abs(target.basket_y - 120) > 2:
                    motor.trace_revise(target.basket_x, target.basket_y, 25) 
                    time.sleep(0.05)
                else:
                    if (motor.head_horizon-2048) > 20:
                        self.correct_left_flag = True
                    elif (motor.head_horizon-2048) < -20:
                        self.correct_right_flag = True
                    if (motor.head_vertical - TWO_POINT_LINE[1]) > 20:
                        self.correct_forward_flag = True
                    elif (motor.head_vertical - TWO_POINT_LINE[1]) < -20:
                        self.correct_back_flag = True
                    send.sendBodySector(222)
                    time.sleep(1.5)  
                    if self.correct_left_flag:
                        send.sendBodySector(123)#手往left修正
                        time.sleep(1)
                    elif self.correct_right_flag:
                        send.sendBodySector(124)#手往right修正
                        time.sleep(1)
                    if self.correct_forward_flag:
                        send.sendBodySector(121)#手往前修正
                        time.sleep(1)
                    elif self.correct_back_flag:
                        send.sendBodySector(122)#手往後修正
                        time.sleep(1)
                    rospy.logdebug(f'執行2分球投籃')
                    send.sendBodySector(2222)
                    self.step = "finish"

    def stratagy_3(self):
        ######################################## 三分球用size判斷 ########################################
        motor.trace_revise(target.basket_x, target.basket_y, 35)
        if not self.ready_shoot: 
            if (THREE_POINT_LINE[2] <= motor.head_vertical <= THREE_POINT_LINE[1]) and (abs(motor.head_horizon - 2048) <= 200): 
                if motor.head_horizon - 1940 > 50: 
                    rospy.loginfo(f'motor.head_horizon = {motor.head_horizon}')
                    motor.MoveContinuous(LEFT_CORRECT[0], LEFT_CORRECT[1], LEFT_CORRECT[2], 100, 100, 8)
                elif motor.head_horizon - 1940 < 50:
                    rospy.loginfo(f'motor.head_horizon = {motor.head_horizon}')
                    motor.MoveContinuous(RIGHT_CORRECT[0], RIGHT_CORRECT[1], RIGHT_CORRECT[2], 100, 100, 8)
                else:
                    self.ready_shoot = True
                    rospy.loginfo(f'到達可投籃大小 STOP!!, motor.head_vertical = {motor.head_vertical}')
                    motor.bodyauto_close(0)
                    time.sleep(3)
                    rospy.logdebug(f'三分球動作預備')
                    motor.move_head(1,1920, 880, 880, 30)
                    time.sleep(0.8)
                    rospy.logdebug(f'伸手準備投籃')
                    send.sendBodySector(333)
                    time.sleep(5)
            else:
                if abs(motor.head_horizon - 2048) > 200:
                    rospy.logdebug(f'頭部馬達水平刻度偏差 > 步態影響的')
                    rospy.logdebug(f'rotate調整')
                    time.sleep(0.05)
                    motor.body_trace_rotate(40)

                else :
                    rospy.logdebug(f'頭部馬達垂直刻度與抓球角度差太多')
                    rospy.logdebug(f'straight調整')
                    time.sleep(0.05)
                    motor.degree_straight(THREE_POINT_LINE[0], THREE_POINT_LINE[1], THREE_POINT_LINE[2])
            
        else:
            if target.basket_x != 0 :
                if abs(target.basket_x-160) > 5:
                    rospy.logdebug(f'腰部修正')
                    send.sendBodySector(131)
                    time.sleep(0.5)
                    rospy.loginfo(f'abs(target.basket_x - 160) = {abs(target.basket_x - 160)}')
                else:
                    time.sleep(2)
                    rospy.logdebug(f'執行3分球投籃')
                    send.sendBodySector(3333)
                    time.sleep(2)     
                    rospy.logdebug(f'投籃')
                    self.step = "finish"    

   
    def stratagy_5(self):
        ######################################## 五分球用size判斷 ########################################
        motor.size_IMU_correct(target.basket_size)
        rospy.loginfo(motor.corrected_size)  

        if not self.ready_shoot:
            motor.trace_revise(target.basket_x, target.basket_y, 35)
            if (FIVE_POINT_LINE[1] <= target.basket_size <= FIVE_POINT_LINE[2]) and (abs(motor.head_horizon - 2048) <= 50):
                self.ready_shoot = True
                rospy.loginfo(f'到達可投籃大小 STOP!!, target.basket_size = {target.basket_size}')
                motor.bodyauto_close(0)
                time.sleep(3)
                rospy.logdebug(f'五分球動作預備')
                rospy.logdebug(f'手臂往後舉')
                send.sendBodySector(555)
                time.sleep(2)   
                rospy.logdebug(f'頭部調整') 
                rospy.logdebug(f'頭部水平旋轉調整')                                              
                motor.move_head(1, FIVEPOINT_HEAD_Y_DEGREE[0], 880, 880, 50)
                rospy.logdebug(f'頭部垂直旋轉調整')
                motor.move_head(2, 2048, 880, 880, 50)
                time.sleep(5)

            else:

                if abs(motor.head_horizon-2048) > 50:
                    rospy.logdebug(f'頭部馬達水平刻度偏差>步態影響的')
                    rospy.logdebug(f'rotate調整')
                    time.sleep(0.05)
                    motor.body_trace_rotate(40)

                else:
                    rospy.logdebug(f'頭部馬達垂直刻度與抓球角度差太多')
                    rospy.logdebug(f'straight調整')
                    time.sleep(0.05)
                    motor.size_straight(FIVE_POINT_LINE[0], FIVE_POINT_LINE[1], FIVE_POINT_LINE[2], FIVE_POINT_LINE[3])
        else:

            if target.basket_x != 0 :
                if abs(target.basket_x-160) > 5:
                    rospy.logdebug(f'腰部修正')
                    send.sendBodySector(131)
                    time.sleep(0.5)
                    rospy.loginfo(f'abs(target.basket_x - 160) = {abs(target.basket_x - 160)}')
                else:
                    time.sleep(0.1)
                    rospy.logdebug(f'投射!!!')
                    send.sendBodySector(5555)
                    time.sleep(7)
                    self.step = "finish"  
            else:
                rospy.logdebug(f'框不在視野中 -> 五分球不會發生拉')

class TargetLocation():
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
            
    def ball_parameter(self):   #利用色模建籃球
        self.color_mask_subject_orange = send.color_mask_subject_cnts[0]
        self.ball_x = 0
        self.ball_y = 0
        self.ball_size = 0
        for j in range (self.color_mask_subject_orange):   #將所有看到的橘色物件編號
            if 310 > send.color_mask_subject_X [0][j] > 10 and 230 > send.color_mask_subject_Y [0][j] > 10 and send.color_mask_subject_size [0][j] > 350:

                if  send.color_mask_subject_size [0][j] > self.ball_size: #用大小過濾物件 #?????900待測試
                    self.ball_x =  send.color_mask_subject_X [0][j]
                    self.ball_y = send.color_mask_subject_Y [0][j]
                    self.ball_size = send.color_mask_subject_size [0][j]
                    self.ball_x_min = send.color_mask_subject_XMin[0][j] 
                    self.ball_y_min = send.color_mask_subject_YMin[0][j] 
                    self.ball_x_max = send.color_mask_subject_XMax[0][j] 
                    self.ball_y_max =send.color_mask_subject_YMax[0][j]       

    def basket_parameter(self): #利用色模建籃框
        self.color_mask_subject_red = send.color_mask_subject_cnts[5] 
        self.basket_x = 0
        self.basket_y = 0
        self.basket_size = 0
        
        for j in range (self.color_mask_subject_red):     #將所有看到的紅色物件編號
            if send.color_mask_subject_size [5][j] > 700:

                if  send.color_mask_subject_size [5][j] > self.basket_size:  #用大小過濾物件(濾雜訊)
                    self.basket_x =  send.color_mask_subject_X [5][j]
                    self.basket_y = send.color_mask_subject_Y [5][j]
                    self.basket_size = send.color_mask_subject_size [5][j]
                    self.basket_x_min = send.color_mask_subject_XMin[5][j] 
                    self.basket_y_min = send.color_mask_subject_YMin[5][j] 
                    self.basket_x_max = send.color_mask_subject_XMax[5][j] 
                    self.basket_y_max =send.color_mask_subject_YMax[5][j]
        

class MotorMove():

    def __init__(self):
        self.head_horizon = 2048                #頭部水平刻度
        self.head_vertical = 2048               #頭部垂直刻度
        self.waist_position = 2048              #腰當下的刻度
        self.search_num = 0
        self.now_x = 0                          #現在要移動的x量
        self.now_y = 0                          #現在要移動的y量
        self.now_theta = 0                      #現在要旋轉的theta量
        self.throw_strength = 0                 #不知道
        self.distance_new = 0
        self.now_state = 0
        self.directly = False
        self.reg = 2048
        self.desire_waist_degree = 2048
        self.size_correct = True
        self.corrected_size = 0

    def initial(self):
        self.head_horizon = 2048                #頭部水平刻度
        self.head_vertical = 2048               #頭部垂直刻度
        self.waist_position = 2048              #腰當下的刻度
        self.search_num = 0
        self.now_x = 0                          #現在要移動的x量
        self.now_y = 0                          #現在要移動的y量
        self.now_theta = 0                      #現在要旋轉的theta量
        self.throw_strength = 0                 #不知道
        self.distance_new = 0
        self.directly = False
        self.reg = 2048
        self.desire_waist_degree = 2048
        self.size_correct = True
        self.corrected_size = 0

    def draw(self):
        send.drawImageFunction(1, 0, 160, 160, 0, 240, 255, 255, 255) 
        send.drawImageFunction(2, 0, 0, 320, 120, 120, 255, 255, 255)
        send.drawImageFunction(3, 1, target.ball_x_min , target.ball_x_max , target.ball_y_min , target.ball_y_max, 255, 0, 255)
        send.drawImageFunction(4, 1, target.basket_x_min , target.basket_x_max , target.basket_y_min , target.basket_y_max, 0, 0, 0)

    def move_head(self, ID, Position,head_max_x, head_max_y, Speed):  #把相對頭部變化變絕對(call 2048就變2048)
        send.sendHeadMotor(ID,Position,Speed)
        if ID == 1:
            self.head_horizon =  Position
            if abs(self.head_horizon - 2048) > head_max_x:
                if (self.head_horizon - 2048 ) > 0:
                    self.head_horizon = 2048 + head_max_x
                elif (self.head_horizon - 2048 ) < 0:
                    self.head_horizon = 2048 - head_max_x

        else :
            self.head_vertical = Position
            if abs(self.head_vertical - 2048) > head_max_y :
                    if (self.head_vertical - 2048 ) > 0 :
                        self.head_vertical = 2048 + head_max_y
                    elif (self.head_vertical - 2048) < 0 :
                        self.head_vertical = 2048 - head_max_y

    def waist_rotate(self, waist_x, Speed):
        send.sendSingleMotor(9, waist_x-self.waist_position, Speed)
        self.waist_position =  waist_x 

    ####################################### view search #######################################
    def view_search(self, right_place, left_place, up_place, down_place, speed, delay):   
        rospy.loginfo(f'motor.reg =  {self.reg}')
        if self.reg > 0:
            turn_order = [3, 4, 1, 2]
        else:
            turn_order = [1, 4, 3, 2]

        self.search_flag = turn_order[self.search_num]

        if self.search_flag == 1:
            if self.head_horizon >= left_place:
                rospy.logdebug(f'左尋')
                rospy.loginfo(f'左尋')
                self.move_head(1, self.head_horizon, 880, 880, speed)
                self.head_horizon = self.head_horizon - speed
                time.sleep(delay)
            else:
                self.search_num += 1
                time.sleep(delay)

        elif self.search_flag == 4:
            if self.head_vertical <= up_place:
                rospy.logdebug(f'上尋')
                rospy.loginfo(f'上尋')
                self.move_head(2, self.head_vertical, 880, 880, speed)
                self.head_vertical = self.head_vertical + speed
                time.sleep(delay)
            else:
                self.search_num += 1  
                time.sleep(delay*5)
                    
        elif  self.search_flag == 3:
            rospy.logdebug(f'右尋')
            rospy.loginfo(f'右尋')
            if  self.head_horizon <= right_place:
                self.move_head(1, self.head_horizon, 880, 880, speed)
                self.head_horizon = self.head_horizon + speed
                time.sleep(delay) 
            else:
                self.search_num += 1
                time.sleep(delay*5)      
        
        elif self.search_flag ==  2:
            rospy.logdebug(f'下尋')
            rospy.loginfo(f'下尋')
            if self.head_vertical >= down_place:
                self.move_head(2, self.head_vertical, 880, 880, speed)      #頭向下的極限
                self.head_vertical = self.head_vertical - speed
                time.sleep(delay)   
            else:
                self.search_num = 0
                time.sleep(delay*5)

    ####################################### view search #######################################
      
    def trace_revise(self, x_target, y_target,speed):    #看誤差調整頭的角度(讓頭看向籃框或球)
        if x_target != 0 and y_target != 0:
            x_difference =  x_target - 160               #目標與中心x差距         
            y_difference =  y_target - 120               #目標與中心y差距
            x_degree = x_difference * (65 / 320)         #目標與中心x角度
            y_degree = y_difference * (38 / 240)         #目標與中心y角度
            self.move_head(1, self.head_horizon - round(x_degree * 4096 / 360 *0.15), 880, 880, speed)
            self.move_head(2, self.head_vertical - round(y_degree * 4096 / 360 *0.15), 880, 880, speed)
            time.sleep(0.05)
        else :
            rospy.logdebug(f'miss_target->需重新尋求')

    def body_trace_rotate(self, degree): #步態旋轉到可拿球的角度
        x_body_rotate = self.head_horizon - 2048 #身體需要旋轉多少
        if x_body_rotate > degree:
            self.MoveContinuous(LEFT_CORRECT[0], LEFT_CORRECT[1], LEFT_CORRECT[2], 100, 100, 8)
            rospy.loginfo(f'rotate left ={x_body_rotate}')
            time.sleep(0.05)
        elif x_body_rotate < -degree :
            self.MoveContinuous(RIGHT_CORRECT[0], RIGHT_CORRECT[1], RIGHT_CORRECT[2], 100, 100, 8)
            rospy.loginfo(f'rotate right = {x_body_rotate}')
            time.sleep(0.05)

    def ball_trace_straight(self, slow_degree, stop_degree, backward_degree):   #前進後退至可找拿球的距離
    ######################################## ball_trace_straight 副函式 ########################################
        if self.head_vertical > slow_degree:  #大前進
            self.MoveContinuous(2500+CORRECT[0], 0+CORRECT[1], 0+CORRECT[2], 100, 100, 2) 
            rospy.loginfo(f'大前進, self.head_vertical= {self.head_vertical}')

        elif stop_degree < self.head_vertical < slow_degree:  #進入減速範圍
            self.MoveContinuous(1500+CORRECT[0], 0+CORRECT[1], 0+CORRECT[2], 100, 100, 2)
            rospy.loginfo(f'進入減速範圍, self.head_vertical = {self.head_vertical}')

        elif self.head_vertical < backward_degree: 
            self.MoveContinuous(-1500+CORRECT[0],0+CORRECT[1],0+CORRECT[2],100,100,2)
            rospy.loginfo(f'大後退, self.head_vertical = {self.head_vertical}')                
            
    def degree_straight(self, slow_degree, stop_degree, backward_degree):   
    ######################################## degree_straight 副函式 ######################################## 
        rospy.logdebug(f'walk_to_basket')
        self.trace_revise(target.basket_x, target.basket_y, 35)
        if self.head_vertical > slow_degree:  #大前進
            self.MoveContinuous(2500+CORRECT[0], 0+CORRECT[1], 0+CORRECT[2], 100, 100, 2) 
            rospy.loginfo(f'大前進, self.head_vertical =  {self.head_vertical}')
            time.sleep(0.05)

        elif stop_degree < self.head_vertical < slow_degree :  #進入減速範圍
            self.MoveContinuous(1500+CORRECT[0], 0+CORRECT[1], 0+CORRECT[2], 100, 100, 2)
            rospy.loginfo(f'進入減速範圍, self.head_vertical =  {self.head_vertical}')
            time.sleep(0.05)
    
        elif self.head_vertical < backward_degree: 
            self.MoveContinuous(-1500+CORRECT[0], 0+CORRECT[1], 0+CORRECT[2], 100, 100, 2)
            rospy.loginfo(f'大後退, self.head_vertical =  {self.head_vertical}')               
            time.sleep(0.05)

    def size_straight(self,forward_slow_size, forward_stop_size, backward_stop_size, backward_slow_size):   #前進後退至可找拿球的距離
    ########################################  size_straight副函式 ########################################
        rospy.logdebug(f'walk to size line')
        rospy.loginfo(f'target.basket_x = {target.basket_x}, target.basket_y = {target.basket_y}, target.basket_size = {target.basket_size}')
        self.trace_revise(target.basket_x, target.basket_y,35)

        if motor.corrected_size > backward_slow_size:                         #大後退
            self.MoveContinuous(-1000+CORRECT[0], 0+CORRECT[1], 0+CORRECT[2], 100, 100, 2)
            rospy.loginfo(f'大後退, target.basket_size = {target.basket_size}')
            time.sleep(0.05)

        elif backward_stop_size < motor.corrected_size < backward_slow_size:  #進入後退減速範圍
            self.MoveContinuous(-500+CORRECT[0], 0+CORRECT[1], 0+CORRECT[2], 100, 100, 2)
            rospy.loginfo(f'進入後退減速範圍, target.basket_size = {target.basket_size}')
            time.sleep(0.05)

        elif forward_slow_size < motor.corrected_size < forward_stop_size:    #進入前進減速範圍
            self.MoveContinuous(1500+CORRECT[0], 0+CORRECT[1], 0+CORRECT[2], 100, 100, 2)
            rospy.loginfo(f'進入前進減速範圍, target.basket_size = {target.basket_size}')              
            time.sleep(0.05)

        elif motor.corrected_size < forward_slow_size:                        #大前進
            self.MoveContinuous(2000+CORRECT[0], 0+CORRECT[1], 0+CORRECT[2], 100, 100, 2)    
            rospy.loginfo(f'大前進, target.basket_size = {target.basket_size}')              
            time.sleep(0.05)
    
    def MoveContinuous(self ,expect_x ,expect_y ,expect_theta ,add_x ,add_y ,add_theta):  #步態移動的馬達緩衝(調整距離:Now_X與Now_Y為現在要移動的x量與現在要移動的y量)
        if abs(self.now_x - expect_x) < add_x:
            self.now_x = expect_x
        else:
            if self.now_x < expect_x:
                self.now_x += add_x
            elif self.now_x > expect_x:
                self.now_x -= add_x
            else:
                pass

        if abs(self.now_y - expect_y) < add_y:
            self.now_y = expect_y
        else:
            if self.now_y < expect_y:
                self.now_y += add_y
            elif self.now_y > expect_y:
                self.now_y -= add_y
            else:
                pass

        if abs(self.now_theta - expect_theta) < add_theta:
            self.now_theta = expect_theta
        else:
            if self.now_theta < expect_theta :
                self.now_theta += add_theta
            elif self.now_theta > expect_theta :
                self.now_theta -= add_theta
            else:
                pass

        rospy.loginfo(f'now_x = {self.now_x}, now_y = {self.now_y}, now_theta = {self.now_theta}') 
        send.sendContinuousValue(self.now_x, self.now_y, 0, self.now_theta , 0)

 ########################################  size_IMU_correct 副函式 ########################################

    def size_IMU_correct(self, current_size):
        if target.basket_size != 0:
            if self.size_correct:
                self.basket_distance(BASKET_SIZE_60_90[0],BASKET_SIZE_60_90[1])
                head_fix = 6 * math.cos(math.radians(90 + send.imu_value_Pitch - ((motor.head_vertical - 2086) * 360 / 4096)))
                rospy.loginfo(f"head_fix = {head_fix}")

                if send.imu_value_Pitch > 0:
                    self.corrected_size = ((self.distance_new)**2 * current_size) / (self.distance_new + 48*math.sin(math.radians(send.imu_value_Pitch)) - head_fix) ** 2
                else:
                    self.corrected_size = current_size

            else:
                self.corrected_size = current_size 
        else:
            self.corrected_size = current_size

 ########################################  size_IMU_correct 副函式 ########################################   


    def test_distance(self):
        send.drawImageFunction(4, 1, target.basket_x_min , target.basket_x_max , target.basket_y_min , target.basket_y_max, 0, 0, 0)
        rospy.loginfo(f'Basket_size = {target.basket_size}, Head_vertical = {self.head_vertical}, target.basket_y_max = {target.basket_y_max}') 

        if abs(target.basket_x - 160) > 5  or abs(target.basket_y - 120) > 4 :
            self.trace_revise(target.basket_x, target.basket_y, 25)
            time.sleep(0.05) 
            
        else :
            rospy.loginfo(f'Basket_size = {target.basket_size}, Head_vertical = {self.head_vertical}')
            rospy.loginfo(f'target.basket_size - 930 = {target.basket_size - 930}')  

    def throw_ball_strength (self):
        if self.distance_new >= 55 and self.distance_new < 60 : #55cm
            self.throw_strength = round(abs (40 * (- (self.distance_new - 60) /5 ) + 45 * (( self.distance_new - 55) /5 )))
        
        elif self.distance_new >= 60 and self.distance_new < 65 : #60cm
            self.throw_strength = round(abs (50 * (- (self.distance_new - 65) /5 ) + 55 * (( self.distance_new - 60) /5 )))
        
        elif self.distance_new >= 65 and self.distance_new < 70 : #65cm)
            self.throw_strength = round(abs (50 * (- (self.distance_new - 70) /5 ) + 55 * (( self.distance_new - 65) /5 )))
        
        elif self.distance_new >= 70 and self.distance_new < 75 : #70cm
            self.throw_strength = round(abs (50 * (- (self.distance_new - 75) /5 ) + 302 * (( self.distance_new - 70) /5 )))
        
        elif self.distance_new >= 75 and self.distance_new < 80 : #70cm
            self.throw_strength = round(abs (302 * (- (self.distance_new - 80) /5 ) + 305 * (( self.distance_new - 75) /5 )))
        
        elif self.distance_new >= 80 and self.distance_new < 85 : #70cm
            self.throw_strength = round(abs (305 * (- (self.distance_new - 85) /5 ) + 311 * (( self.distance_new - 80) /5 )))
        
        elif self.distance_new >= 85 and self.distance_new < 90 : #70cm
            self.throw_strength = round(abs (311 * (- (self.distance_new - 90) /5 ) + 311 * (( self.distance_new - 85) /5 )))
        
        elif self.distance_new >= 90 and self.distance_new < 95 : #70cm
            self.throw_strength = round(abs (311 * (- (self.distance_new - 95) /5 ) + 319 * (( self.distance_new - 90) /5 )))

        elif self.distance_new >= 95 and self.distance_new < 100 : #70cm
            self.throw_strength = round(abs (319 * (- (self.distance_new - 100) /5 ) + 322 * (( self.distance_new - 95) /5 ))) #319   322

        elif self.distance_new >= 100 and self.distance_new < 105 : #70cm
            self.throw_strength = round(abs (322 * (- (self.distance_new - 105) /5 ) + 324 * (( self.distance_new - 100) /5 )))

        elif self.distance_new >= 105 and self.distance_new < 110 : #70cm
            self.throw_strength = round(abs (324 * (- (self.distance_new - 110) /5 ) + 330 * (( self.distance_new - 105) /5 )))

        rospy.loginfo(f'throw_strength = {self.throw_strength}')

    def basket_distance(self, six, nine):   #利用60與90公分距離測出的籃框面積推得當前機器人與籃框的距離->self.distance_new
        if target.basket_size != 0:
            sixty_distance = sqrt(abs (( 3600 * six) /target.basket_size))
            ninty_distance = sqrt(abs(( 8100 * nine) /target.basket_size))
            self.distance_new = abs(ninty_distance)
            self.throw_ball_strength()
            rospy.loginfo(f'Head_vertical = {self.head_vertical}')
            rospy.loginfo(f'Basket size = {target.basket_size}')
            rospy.loginfo(f'Distance_60 = {sixty_distance}')
            rospy.loginfo(f'Distance_90 = {ninty_distance}')
            rospy.loginfo(f'Distance_fin = {self.distance_new}')

    def bodyauto_close(self,next_state):    #步態移動的開關控制(原地踏步)
        if self.now_state == next_state :    
            pass
        elif self.now_state != next_state :
            send.sendBodyAuto(0,0,0,0,1,0)
            self.now_state = next_state


if __name__ == '__main__' :
    target = TargetLocation()
    motor = MotorMove()
    strategy = BasketBall()
    rospy.init_node('BB_strategy', anonymous=True, log_level=rospy.INFO)   #初始化node
    try:
        while not rospy.is_shutdown():
            strategy.main()

    except rospy.ROSInterruptException:
        pass