#!/usr/bin/env python3
#coding=utf-8
from collections import Counter, deque
from cmath import sqrt
from re import T
import time
from traceback import print_tb
import numpy as np
import rospy
import math
from Python_API import Sendmessage

# 2025.8.1

#======================================================================================

CORRECT       = [-100, -50, 0]        # 原地踏步修正
LEFT_CORRECT  = [-300, -250, 2]        # 左旋修正
RIGHT_CORRECT = [-300, 100, -2]       # 右旋修正
#                 x , y , theta 

#=====================================================================================

BASKET_SIZE_60_90 = [3070, 1250]      #sector 5301                   # 投籃時測量的籃框距離方法 #五分投籃時站姿高度看籃框size測距離
FIVEPOINT_HEAD_Y_DEGREE = [2010]      #投出去偏向左邊＝>頭往左轉（大）-朝1960 ;  投出去偏向右邊＝>頭往右轉（小）-朝1940    #投籃前頭會固定一個角度，並扭腰

#=====================================================================================

CATCH_BALL_CORRECT = 2000

CATCH_BALL_LINE  = [1680, 1570, 1560]            # slow_degree, stop_degree, backward_degree
TWO_POINT_LINE   = [1800, 1770, 1760]            # slow_degree, stop_degree, backward_degree
THREE_POINT_LINE = [75, 64, 61, 55]             # forward_slow_distance > forward_stop_distance > backward_stop_distance > backward_slow_distance
FIVE_POINT_LINE  = [110, 96, 93, 88]           # srward_slow_distance > forward_stop_distance > backward_stop_distance > backward_slow_distance

# 計算焦距判斷距離
BASTET_LENGTH = 10   #增加以下全域變數
FOCAL_LENGTH  = 333   # 333 
TEST_DISTANCE = 90

send = Sendmessage()

class BasketBall():
    def __init__(self):
        rospy.init_node('bb', anonymous=True, log_level=rospy.INFO)
        self.head_y_down_adjust = False
        self.head_y_up_adjust = False
        self.ready_dunk = False
        self.ready_shoot = False
        self.aiming_finish = False
        self.step = 'begin'
        self.sw = 0
    
    def initial(self):
        self.head_y_down_adjust = False
        self.head_y_up_adjust = False
        self.ready_dunk = False
        self.ready_shoot = False
        self.aiming_finish = False
        self.step = 'begin'
        self.sw = 0
    
    def main(self):
        target.ball_parameter()
        target.basket_parameter()
        if send.is_start: #send.Web
            #rospy.loginfo(f'step = {self.step}')
            #rospy.loginfo(f'basket_size = {target.basket_size}') 
            motor.draw()
            
            if self.step == 'begin':
                self.begin()

            elif self.step == 'find_ball':
                self.find_ball()   

            elif self.step == 'start_gait':
                self.start_gait()

            elif self.step == 'walk_to_ball':
                self.walk_to_ball()

            elif self.step == 'waist_fix':
                self.waist_fix()

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
                
        elif send.DIOValue == 19:       # ball size   上上下下
            motor.trace_revise(target.ball_x, target.ball_y, 35)
            rospy.loginfo(f'Head_vertical = {motor.head_vertical}')
            rospy.loginfo(f'Ball_size = {target.ball_size}')
            time.sleep(0.2) 
            self.step = 'test'

        elif send.DIOValue == 21:       # basket size   上下上下
            motor.draw()
            motor.trace_revise(target.basket_x, target.basket_y, 35)
            motor.basket_distance() #增加
            rospy.loginfo(f'Head_vertical = {motor.head_vertical}') #增加
            rospy.loginfo(f'籃球框距離 = {motor.basket_distance_x}')
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
            time.sleep(0.05)
            self.step = 'begin'
            rospy.logdebug(f'-------------------reset and stoping-------------------------')
            rospy.loginfo(f'主策略指撥關閉 -> 機器人回復初始狀態')
            time.sleep(0.05)                        
            rospy.logdebug(f'投籃')


    def begin(self):
        ####################################### switch #######################################

        send.sendSensorReset(1, 1, 1) 
                
        if send.DIOValue == 48:   # 開啟二分策略  下下下下
            self.sw = 2
            rospy.loginfo(f'SW = {self.sw}')

        elif send.DIOValue == 49: # 開啟三分策略  上下下下                                 
            self.sw = 3
            rospy.loginfo(f'SW = {self.sw}')

        elif send.DIOValue == 55: # 開啟五分策略  上上上下                                 
            self.sw = 5
            rospy.loginfo(f'SW = {self.sw}')

        else :
            self.sw = 2
            rospy.loginfo(f'SW = {self.sw}')
        #[sw==2]->2分球;[sw==0]->3分球;[sw==1]->5分球

        ######################################## switch #######################################
        rospy.logdebug(f'開始執行初始化')

        #send.sendBodySector(6)      ############步態調整############
        #time.sleep(0.05)
        #send.sendBodySector(8) 
        #time.sleep(0.05)
        send.sendBodySector(100) 
        time.sleep(0.05)   


        self.step = 'find_ball'

        
    def find_ball(self):
        target.ball_parameter()

        if not self.head_y_down_adjust:
            time.sleep(1)
            rospy.logdebug(f'頭部抬起尋框')
            motor.move_head(2,1700,880,880,50)                
            self.head_y_down_adjust = True
            time.sleep(1)
        
        else:
            if target.ball_size <= 350:   # 球在視野中太小
                rospy.logdebug(f'球在視野中太小 -> 大範圍尋球')
                # motor.view_search_left(2428, 1668, 1800, 1200, 40, 0.05)
                motor.view_search(2428, 1668, 1800, 1200, 40, 0.05)
                target.ball_parameter() 

            elif target.ball_size > 350:   # 球在視野中夠大

                if abs(target.ball_x - 160) > 10  or abs(target.ball_y - 120) > 10:  # 讓球在畫面中心
                    rospy.logdebug(f'球在視野中夠大 -> 鎖定球')
                    target.ball_parameter()
                    motor.trace_revise(target.ball_x, target.ball_y, 65) 
                    time.sleep(0.05)

                elif (CATCH_BALL_LINE[2] <= motor.head_vertical <= CATCH_BALL_LINE[1]) and (abs(motor.head_horizon-2048) <= 270):
                    rospy.loginfo(f'到達夾球範圍 STOP!!, self.head_vertical = {motor.head_vertical}')                
                    time.sleep(0.05)
                    motor.trace_revise(target.ball_x, target.ball_y, 40) 
                    rospy.logdebug(f'到達可夾球位置')
                    rospy.loginfo(f'蹲下準備夾球')
                    time.sleep(1)
                    send.sendBodySector(101) 
                    time.sleep(0.5) 
                    send.sendBodySector(587) 
                    time.sleep(0.5)
                    motor.reg = 2048 - motor.head_horizon
                    motor.search_num = 0
                    motor.directly = True
                    self.step = 'waist_fix'

                else: 

                    motor.reg = 2048 - motor.head_horizon
                    motor.search_num = 0
                    self.step = 'start_gait'   


    def start_gait(self):
        target.ball_parameter()
        motor.trace_revise(target.ball_x, target.ball_y, 30) 
        motor.bodyauto_close(1)
        time.sleep(0.05)

        if (motor.head_vertical <= CATCH_BALL_LINE[2]-100): # 球太近，先後退一段距離
            rospy.loginfo(f'球太大 -> 大倒退')
            motor.trace_revise(target.ball_x, target.ball_y, 30) 
            motor.MoveContinuous(-1200+CORRECT[0], 0+CORRECT[1], 0+CORRECT[2], 70, 70, 1) # 超大後退

        else:
            rospy.logdebug(f'可進行微小修正')
            self.step = 'walk_to_ball'

    
    def walk_to_ball(self):
        target.ball_parameter()   
        motor.trace_revise(target.ball_x, target.ball_y, 60)
        rospy.loginfo(f"head_vertical = {motor.head_vertical}")

        if (CATCH_BALL_LINE[2] <= motor.head_vertical <= CATCH_BALL_LINE[1]) and (abs(motor.head_horizon-2048) <= 110):  # 到達夾球位置
            motor.bodyauto_close(0) # 步態停止
            rospy.loginfo(f'到達夾球範圍 STOP!!, self.head_vertical = {motor.head_vertical}')                
            time.sleep(0.05)
            motor.trace_revise(target.ball_x, target.ball_y, 25) 
            rospy.logdebug(f'到達可夾球位置')
            rospy.loginfo(f'蹲下準備夾球')
            time.sleep(1)
            send.sendBodySector(101) 
            time.sleep(0.5) 
            send.sendBodySector(587) 
            time.sleep(0.5)
            # rospy.logdebug(f'頭往右轉')
            # motor.move_head(1, 1820, 880, 880, 50)
            # time.sleep(2) 
            self.step = 'waist_fix'

        else:
            target.ball_parameter()
            motor.trace_revise(target.ball_x, target.ball_y, 35) 

            if abs(motor.head_horizon-2048) > 100:
                rospy.logdebug(f'頭部馬達水平刻度偏差 -> 步態影響')
                rospy.loginfo(f'rotate調整')
                motor.body_trace_rotate(60)

            else:
                rospy.logdebug(f'頭部馬達垂直刻度與抓球角度差太多')
                rospy.loginfo(f'straight調整')
                motor.ball_trace_straight(CATCH_BALL_LINE[0], CATCH_BALL_LINE[1], CATCH_BALL_LINE[2])        


    def waist_fix(self):
        target.ball_parameter()
        if abs(target.ball_x - 160) > 1  or abs(target.ball_y - 120) > 1:  # 讓球在畫面中心
            rospy.logdebug(f'球在視野中夠大 -> 鎖定球')
            motor.trace_revise(target.ball_x, target.ball_y, 35) 
            rospy.loginfo(f"motor.head_horizon = {motor.head_horizon}")
            time.sleep(0.05)
        else:
            if abs(motor.head_horizon-1860) > 8: 
                rospy.loginfo(f'球不在視野中間 -> 貓頭鷹修腰')
                # rospy.loginfo(f"motor.head_horizon = {motor.head_horizon}")
                motor.Owl_Rotate(1860)
            else :
                rospy.loginfo(f"motor.head_horizon = {motor.head_horizon}")
                rospy.loginfo(f'球水平位置在中間')
                self.step = 'catch_ball'

    
    def catch_ball(self):
        rospy.loginfo(f"target.ball_size = {target.ball_size}")

        if target.ball_size < CATCH_BALL_CORRECT:
            rospy.loginfo(f'夾球修正')
            time.sleep(0.5)
            send.sendBodySector(333)
            rospy.loginfo(f'正常夾球動作')
            time.sleep(2)
            send.sendBodySector(687)
            time.sleep(2) 
            motor.catch_correct = True

        else:
            rospy.loginfo(f'正常夾球動作')
            time.sleep(1)
            send.sendBodySector(687)
            time.sleep(2) 
            

        rospy.loginfo(f'腰部回正')
        motor.waist_rotate(2048,70)
        time.sleep(0.5) 

        if motor.catch_correct:
            rospy.loginfo(f'根據各自夾球動作回復站姿')
            send.sendBodySector(444)
            time.sleep(2)
            rospy.loginfo(f'回復站姿')
            send.sendBodySector(787) 
            time.sleep(2)

        else:
            rospy.loginfo(f'回復站姿')
            send.sendBodySector(787) 
            time.sleep(2)

        self.step = 'find_basket'  


    def find_basket(self):
        target.basket_parameter()
        if target.basket_size < 500:
            rospy.logdebug(f'籃框在視野裡太小 -> 尋框')
            rospy.loginfo(f'basket_size =  {target.basket_size}')

            if not self.head_y_up_adjust:
                time.sleep(0.5)
                rospy.logdebug(f'頭部抬起尋框')
                motor.move_head(2,1900,880,880,50)                
                self.head_y_up_adjust = True
                time.sleep(0.5)

            else:                                   
                rospy.logdebug(f'開始尋框')
                rospy.loginfo(f'target.basket_x = {target.basket_x}, target.basket_y = {target.basket_y}, target.basket_size = {target.basket_size}')
                ####################################### view search #######################################
                motor.view_search(2548, 1548, 2048, 1948, 50, 0.04)

        else:                                
            rospy.logdebug(f'籃框在視野裡夠大 -> 判斷策略所需前往的位置')

            if self.sw == 3:
                if abs(target.basket_x - 160) > 6  or abs(target.basket_y - 120) > 8:  #讓basket在畫面中心
                    rospy.logdebug(f'球在視野中夠大 -> 鎖定球')
                    target.ball_parameter()
                    motor.trace_revise(target.basket_x, target.basket_y, 25) 
                    time.sleep(0.05)
                else:
                    rospy.loginfo(f'3分球')
                    motor.bodyauto_close(1)
                    time.sleep(0.5)
                    self.step = 'stratagy_3'
                
            elif self.sw == 5:
                if abs(target.basket_x - 160) > 6  or abs(target.basket_y - 120) > 8:  #讓basket在畫面中心
                    rospy.logdebug(f'球在視野中夠大 -> 鎖定球')
                    target.ball_parameter()
                    motor.trace_revise(target.basket_x, target.basket_y, 25) 
                    time.sleep(0.05)
                else:
                    if motor.directly:
                        self.ready_shoot = True
                        self.step = 'stratagy_5'
                        target.basket_parameter()
                        time.sleep(2)
                        rospy.loginfo(f'五分球動作預備')
                        send.sendBodySector(5301)
                        time.sleep(4)   
                        rospy.logdebug(f'頭部調整') 
                        rospy.logdebug(f'頭部水平旋轉調整')                                              
                        motor.move_head(1, FIVEPOINT_HEAD_Y_DEGREE[0], 880, 880, 50)
                        time.sleep(1)
                        rospy.logdebug(f'頭部垂直旋轉調整')
                        motor.move_head(2, 2048, 880, 880, 50)

                    else:
                        rospy.loginfo(f'5分球')
                        motor.bodyauto_close(1)
                        time.sleep(0.5)
                        self.step = 'stratagy_5'
                
            elif self.sw == 2:
                if abs(target.basket_x - 160) > 8  or abs(target.basket_y - 120) > 12:  #讓basket在畫面中心
                    rospy.logdebug(f'球在視野中夠大 -> 鎖定球')
                    target.ball_parameter()
                    motor.trace_revise(target.basket_x, target.basket_y, 25) 
                    time.sleep(0.05)
                else:
                    rospy.loginfo(f'2分球')
                    motor.bodyauto_close(1)
                    time.sleep(1)
                    self.step = 'stratagy_2'


    ######################################## 二分球仿造catch_ball ######################################## 
    
    def stratagy_2(self):
        target.basket_parameter()

        if not self.ready_dunk:  
            motor.trace_revise(target.basket_x, target.basket_y, 35)

            if ((TWO_POINT_LINE[2]) <= motor.head_vertical <= TWO_POINT_LINE[1]) and (abs(motor.head_horizon - 2048) <= 280): 
                rospy.loginfo(f'到達可投籃角度 STOP!!, self.head_vertical =  {motor.head_vertical}')
                self.ready_dunk = True
                rospy.loginfo(f'到達可投籃大小 STOP!!, target.basket_size = {target.basket_size} ,corrected_size = {motor.corrected_size}')
                motor.bodyauto_close(0)
                time.sleep(1)
                target.basket_parameter()
                time.sleep(1)
                rospy.logdebug(f'伸手準備投籃')
                send.sendBodySector(887)
                time.sleep(1.5)
            else:

                if abs(motor.head_horizon - 2048) > 200:
                    rospy.logdebug(f'頭部馬達水平刻度偏差 -> 步態影響')
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
                if abs(target.basket_x- 160) > 1  or abs(target.basket_y - 120) > 2:  #讓匡在畫面中心
                    rospy.logdebug(f'球在視野中夠大 -> 鎖定球')
                    motor.trace_revise(target.basket_x, target.basket_y, 55) 
                    rospy.loginfo(f"motor.head_horizon = {motor.head_horizon}")
                    time.sleep(0.05)
                else:
                    if abs(motor.head_horizon-1850) > 5: 
                        rospy.loginfo(f'匡不在視野中間 -> 貓頭鷹修腰')
                        # rospy.loginfo(f"motor.head_horizon = {motor.head_horizon}")
                        motor.Owl_Rotate(1850)           
    
                    # if abs(target.basket_x-160) > 3:
                    #     target.basket_parameter()
                    #     rospy.logdebug(f'腰部修正')
                    #     motor.WaistFix(target.basket_x, 160)
                    #     rospy.loginfo(f'abs(target.basket_x - 160) = {abs(target.basket_x - 160)}')

                    else:
                        time.sleep(1)
                        rospy.logdebug(f'執行2分球投籃')
                        send.sendBodySector(987)
                        self.step = "finish"
            else:
                rospy.loginfo(f'框不在視野中 -> 往左邊轉腰')
                motor.Null_WaistFix(2100)     #2348


    ######################################## 三分球用size判斷 ########################################

    def stratagy_3(self):
        
        target.basket_parameter()

        if not self.ready_shoot:  
            motor.trace_revise(target.basket_x, target.basket_y, 65)

            if (THREE_POINT_LINE[1] >=  motor.basket_distance_x >= THREE_POINT_LINE[2]) and (abs(motor.head_horizon - 2048) <= 100) and not motor.turn_flag: 
                motor.line_flag += 1
                rospy.loginfo(f'line_flag = {motor.line_flag}')
                rospy.loginfo(f'籃球框距離 = {motor.basket_distance_x}')
                time.sleep(0.25)
                
                if (motor.line_flag >= 5):
                    self.ready_shoot = True
                    rospy.loginfo(f'到達可投籃大小 STOP!!, 籃球框距離 = {motor.basket_distance_x}')
                    motor.bodyauto_close(0)
                    target.basket_parameter()
                    time.sleep(1)
                    rospy.loginfo(f'3分球預備動作')
                    send.sendBodySector(887)
                    time.sleep(3)
                    rospy.logdebug(f'頭部水平旋轉調整')
                    #motor.move_head(1,1840, 880, 880, 30)
                    time.sleep(1)
            else:

                if abs(motor.head_horizon - 2048) > 100:
                    # rospy.loginfo(f'頭部馬達水平刻度偏差 > 步態影響的')
                    rospy.loginfo(f'rotate調整')
                    time.sleep(0.05)
                    motor.body_trace_rotate(20)
                    motor.turn_flag = True

                else :
                    rospy.logdebug(f'頭部馬達垂直刻度與抓球角度差太多')
                    rospy.loginfo(f'straight調整')
                    time.sleep(0.05)
                    motor.distance_straight(THREE_POINT_LINE[0], THREE_POINT_LINE[1], THREE_POINT_LINE[2], THREE_POINT_LINE[3])
                    motor.turn_flag = False
            
        else:
            if target.basket_x != 0 :
                if abs(target.basket_x- 160) > 1 or abs(target.basket_y - 120) > 1:  #讓匡在畫面中心
                    rospy.logdebug(f'球在視野中夠大 -> 鎖定球')
                    motor.trace_revise(target.basket_x, target.basket_y, 35) 
                    # rospy.loginfo(f"motor.head_horizon = {motor.head_horizon}")
                    time.sleep(0.05)
                else:
                    if abs(motor.head_horizon-1950) > 10: 
                        # rospy.loginfo(f'匡不在視野中間->貓頭鷹修腰')
                        # rospy.loginfo(f"motor.head_horizon = {motor.head_horizon}")
                        motor.Owl_Rotate(1950)  

                    else:
                        time.sleep(0.5)
                        rospy.loginfo(f'手臂旋轉調整')
                        send.sendBodySector(5) 
                        time.sleep(0.5) 
                        rospy.loginfo(f'開爪')
                        send.sendBodySector(886)
                        rospy.loginfo(f'投籃')
                        send.sendBodySector(988) 

                        self.step = "finish"    
            # else:
                # rospy.loginfo(f'框不在視野中 -> 往左邊轉腰')


    ######################################## 五分球用size判斷 ########################################

    def stratagy_5(self):

        target.basket_parameter()
        if not self.ready_shoot:
            motor.trace_revise(target.basket_x, target.basket_y, 65)
            if (FIVE_POINT_LINE[1] >= motor.basket_distance_x >= FIVE_POINT_LINE[2]) and (abs(motor.head_horizon - 2048) <= 70) and not motor.turn_flag:

                motor.line_flag += 1
                rospy.loginfo(f'line_flag = {motor.line_flag}')
                rospy.loginfo(f'籃球框距離 = {motor.basket_distance_x}')
                time.sleep(0.25)
                
                if (motor.line_flag >= 3):
                    self.ready_shoot = True
                    rospy.loginfo(f'到達可投籃大小 STOP!!, target.basket_distance = {motor.basket_distance_x}')
                    motor.bodyauto_close(0)
                    target.basket_parameter()
                    time.sleep(2)
                    rospy.loginfo(f'5分球預備動作')
                    send.sendBodySector(5301)
                    time.sleep(4)   
                    rospy.logdebug(f'頭部調整') 
                    rospy.logdebug(f'頭部水平旋轉調整')                                              
                    motor.move_head(1, FIVEPOINT_HEAD_Y_DEGREE[0], 880, 880, 50)
                    time.sleep(1)
                    # rospy.logdebug(f'頭部垂直旋轉調整')
                    # motor.move_head(2, 2048, 880, 880, 50)
                    # time.sleep(1)

            else:

                if abs(motor.head_horizon-2048) > 80:
                    rospy.logdebug(f'頭部馬達水平刻度偏差 -> 步態影響的')
                    rospy.loginfo(f'rotate調整')
                    time.sleep(0.05)
                    motor.body_trace_rotate(20)
                    motor.turn_flag = True

                else:
                    rospy.logdebug(f'頭部馬達垂直刻度與抓球角度差太多')
                    rospy.loginfo(f'straight調整')
                    time.sleep(0.05)
                    motor.distance_straight(FIVE_POINT_LINE[0], FIVE_POINT_LINE[1], FIVE_POINT_LINE[2], FIVE_POINT_LINE[3])
                    motor.turn_flag = False
        else:

            if target.basket_x != 0 :
                if abs(target.basket_x- 160) > 1  or abs(target.basket_y - 120) > 1:  #讓匡在畫面中心
                    rospy.logdebug(f'球在視野中夠大 -> 鎖定球')
                    motor.trace_revise(target.basket_x, target.basket_y, 40) 
                    # rospy.loginfo(f"motor.head_horizon = {motor.head_horizon}")
                    time.sleep(0.05)
                else:
                    if abs(motor.head_horizon-1980) > 5: 
                        # rospy.loginfo(f'匡不在視野中間->貓頭鷹修腰')
                        # rospy.loginfo(f"motor.head_horizon = {motor.head_horizon}")
                        motor.Owl_Rotate(1980)

                    else:
                        time.sleep(0.5)
                        rospy.loginfo(f'手臂旋轉調整')
                        #send.sendBodySector(5) 
                        #time.sleep(0.05) 
                        rospy.loginfo(f'開爪')
                        send.sendBodySector(5502)
                        time.sleep(3)
                        rospy.loginfo(f'投籃')
                        send.sendBodySector(503)
                        rospy.loginfo(f'motor.throw_strength  = {motor.throw_strength}')
                        self.step = "finish"
                
            #else:
                #rospy.logdebug(f'框不在視野中 -> 五分球不會發生拉')

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
        self.basket_y_max = 0
        self.basket_length = 0 #增加

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
        self.basket_length = 0 #增加
            
    def ball_parameter(self):   #利用色模建籃球
        self.color_mask_subject_orange = send.color_mask_subject_cnts[0]
        self.ball_x = 0
        self.ball_y = 0
        self.ball_size = 0
        for j in range (self.color_mask_subject_orange):   #將所有看到的橘色物件編號
            if 310 > send.color_mask_subject_X [0][j] > 10 and 230 > send.color_mask_subject_Y [0][j] > 10 and send.color_mask_subject_size [0][j] > 400:

                if  send.color_mask_subject_size [0][j] > self.ball_size: #用大小過濾物件 #?????900待測試
                    self.ball_x =  send.color_mask_subject_X [0][j]
                    self.ball_y = send.color_mask_subject_Y [0][j]
                    self.ball_size = send.color_mask_subject_size [0][j]
                    self.ball_x_min = send.color_mask_subject_XMin[0][j] 
                    self.ball_y_min = send.color_mask_subject_YMin[0][j] 
                    self.ball_x_max = send.color_mask_subject_XMax[0][j] 
                    self.ball_y_max = send.color_mask_subject_YMax[0][j]       

    def basket_parameter(self): #利用色模建籃框
        self.color_mask_subject_red = send.color_mask_subject_cnts[5] 
        self.basket_x = 0
        self.basket_y = 0
        self.basket_size = 0
        self.basket_length = 0 #增加
        
        for j in range (self.color_mask_subject_red):     #將所有看到的紅色物件編號
            if send.color_mask_subject_size [5][j] > 400:

                if  9500 > send.color_mask_subject_size [5][j] > self.basket_size:  #用大小過濾物件(濾雜訊)
                    self.basket_x =  send.color_mask_subject_X [5][j]
                    self.basket_y = send.color_mask_subject_Y [5][j]
                    self.basket_size = send.color_mask_subject_size [5][j]
                    self.basket_x_min = send.color_mask_subject_XMin[5][j] 
                    self.basket_y_min = send.color_mask_subject_YMin[5][j] 
                    self.basket_x_max = send.color_mask_subject_XMax[5][j] 
                    self.basket_y_max = send.color_mask_subject_YMax[5][j]
                    self.basket_length = send.color_mask_subject_YMax[5][j] - send.color_mask_subject_YMin[5][j]  #增加
        


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
        self.catch_correct = False
        self.reg = 2048
        self.desire_waist_degree = 2048
        self.size_correct = True
        self.corrected_size = 0
        self.turn_flag = True
        self.basket_distance_x = 0 #增加
        self.line_flag = 0

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
        self.catch_correct = False
        self.reg = 2048
        self.desire_waist_degree = 2048
        self.size_correct = True
        self.corrected_size = 0
        self.turn_flag = True
        self.basket_distance_x = 0 #增加
        self.line_flag = 0


    def draw(self):
        target.ball_parameter()
        target.basket_parameter()
        send.drawImageFunction(1, 0, 160, 160, 0, 240, 255, 255, 255) 
        send.drawImageFunction(2, 0, 0, 320, 120, 120, 255, 255, 255)
        send.drawImageFunction(3, 1, target.ball_x_min , target.ball_x_max , target.ball_y_min , target.ball_y_max, 255, 0, 255)
        send.drawImageFunction(4, 1, target.basket_x_min , target.basket_x_max , target.basket_y_min , target.basket_y_max, 0, 0, 0)
        # send.drawImageFunction(5, 1, send.yolo_XMin, send.yolo_XMax, send.yolo_YMin, send.yolo_YMax, 0, 255, 0)

    def move_head(self, ID, Position,head_max_x, head_max_y, Speed):  #把相對頭部變化變絕對(call 2048就變2048)
        send.sendHeadMotor(ID,Position,Speed)
        target.ball_parameter()
        target.basket_parameter()
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

        if self.search_num > len(turn_order):
            self.search_num = 0

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
      
    def trace_revise(self, x_target, y_target, speed):    #看誤差調整頭的角度(讓頭看向籃框或球)
        if x_target != 0 and y_target != 0:
            x_difference =  x_target - 160               #目標與中心x差距         
            y_difference =  y_target - 120               #目標與中心y差距
            x_degree = x_difference * (65 / 320)         #目標與中心x角度
            y_degree = y_difference * (38 / 240)         #目標與中心y角度
            self.move_head(1, self.head_horizon - round(x_degree * 4096 / 360 *0.15), 1000, 1000, speed)
            self.move_head(2, self.head_vertical - round(y_degree * 4096 / 360 *0.15), 1000, 1000, speed)
            time.sleep(0.05)
        else :
            rospy.logdebug(f'miss_target->需重新尋求')

    def body_trace_rotate(self, degree): #步態旋轉到可拿球的角度
        x_body_rotate = self.head_horizon - 2048 #身體需要旋轉多少
        if x_body_rotate > degree:
            self.MoveContinuous(LEFT_CORRECT[0], LEFT_CORRECT[1], LEFT_CORRECT[2], 100, 100, 8)
            # rospy.loginfo(f'右轉修正 = {x_body_rotate}')
            time.sleep(0.05)
        elif x_body_rotate < -degree :
            self.MoveContinuous(RIGHT_CORRECT[0], RIGHT_CORRECT[1], RIGHT_CORRECT[2], 100, 100, 8)
            # rospy.loginfo(f'左轉修正 = {x_body_rotate}')
            time.sleep(0.05)

    def ball_trace_straight(self, slow_degree, stop_degree, backward_degree):   #前進後退至可找拿球的距離
    ######################################## ball_trace_straight 副函式 ########################################
        if self.head_vertical > slow_degree:  #大前進
            self.MoveContinuous(1200+CORRECT[0], 0+CORRECT[1], 0+CORRECT[2], 100, 100, 2) 
            rospy.loginfo(f'大前進, self.head_vertical= {self.head_vertical}')

        elif stop_degree < self.head_vertical < slow_degree:  #進入減速範圍
            self.MoveContinuous(800+CORRECT[0], 0+CORRECT[1], 0+CORRECT[2], 100, 100, 2)
            rospy.loginfo(f'進入減速範圍, self.head_vertical = {self.head_vertical}')

        elif self.head_vertical < backward_degree: 
            self.MoveContinuous(-1000+CORRECT[0],0+CORRECT[1],0+CORRECT[2],100,100,2)
            rospy.loginfo(f'大後退, self.head_vertical = {self.head_vertical}')                

    def Owl_Rotate(self, turn_degree):

        rospy.loginfo(f"motor.head_horizon = {motor.head_horizon}")
        self.MoveW = motor.head_horizon - turn_degree
        if abs(self.MoveW) > 2:
            pass
        else:
            self.MoveW = -2 if self.MoveW < 0 else 2
            # elif self.MoveW < -3:
            #     self.MoveW = -3

        self.waist_rotate((self.waist_position + self.MoveW), 15)
        rospy.loginfo(f"貓頭鷹修腰 motor.head_horizon = {motor.head_horizon}")
        rospy.loginfo(f'self.MoveW = {self.MoveW}')
        time.sleep(0.5)

    def Null_WaistFix(self, turn_final): # 轉腰調整Basket.X與Baskethead_verticalBaseLine的誤差
 
        if (self.waist_position - 10) <= turn_final:
            self.desire_waist_degree -= 10
            self.waist_rotate(self.desire_waist_degree, 30)
            rospy.loginfo(f'waist_position = {self.waist_position}')
            time.sleep(0.15)
        else:
            rospy.loginfo(f'self.waist_position =  {self.waist_position}') 
            rospy.logerr(f'fail')

    def degree_straight(self, slow_degree, stop_degree, backward_degree):   # 前進後退至可投球球的距離 2分
    ######################################## degree_straight 副函式 ######################################## 
        rospy.logdebug(f'walk_to_basket')
        target.basket_parameter()
        rospy.loginfo(f'target.basket_x = {target.basket_x}, target.basket_y = {target.basket_y}, target.basket_size = {target.basket_size}')
        self.trace_revise(target.basket_x, target.basket_y, 35)
            
        if self.head_vertical > slow_degree:  #大前進
            self.MoveContinuous(1500+CORRECT[0], 0+CORRECT[1], 0+CORRECT[2], 100, 100, 2) 
            rospy.loginfo(f'大前進, self.head_vertical =  {self.head_vertical}')
            time.sleep(0.05)

        elif stop_degree < self.head_vertical < slow_degree :  #進入減速範圍
            self.MoveContinuous(1000+CORRECT[0], 0+CORRECT[1], 0+CORRECT[2], 100, 100, 2)
            rospy.loginfo(f'進入減速範圍, self.head_vertical =  {self.head_vertical}')
            time.sleep(0.05)
    
        elif self.head_vertical < backward_degree: 
            self.MoveContinuous(-500+CORRECT[0], 0+CORRECT[1], 0+CORRECT[2], 100, 100, 2)
            rospy.loginfo(f'大後退, self.head_vertical =  {self.head_vertical}')               
            time.sleep(0.05)

    def distance_straight(self,forward_slow_distance, forward_stop_distance, backward_stop_distance, backward_slow_distance):   # 前進後退至可投球球的距離 3分 5分
    ########################################  distance_straight副函式 ########################################
        rospy.logdebug(f'walk to line')
        target.basket_parameter()
        # rospy.loginfo(f'target.basket_yolo_x = {target.basket_yolo_x}, target.basket_yolo_y = {target.basket_yolo_y}, target.basket_size = {target.basket_size}')
        self.trace_revise(target.basket_x, target.basket_y,100)
        motor.basket_distance()

        if motor.basket_distance_x < backward_slow_distance:                         #大後退
            self.MoveContinuous(-1200+CORRECT[0], 0+CORRECT[1], 0+CORRECT[2], 100, 100, 2)
            # rospy.loginfo(f'大後退, target.basket_size = {target.basket_size}')
            rospy.loginfo(f'離籃球框很近, 大後退, 籃球框距離 = {motor.basket_distance_x}')
            
            time.sleep(0.05)

        elif backward_stop_distance > motor.basket_distance_x > backward_slow_distance:  #進入後退減速範圍
            self.MoveContinuous(-500+CORRECT[0], 0+CORRECT[1], 0+CORRECT[2], 100, 100, 2)
            # rospy.loginfo(f'進入後退減速範圍, target.basket_size = {target.basket_size}')
            rospy.loginfo(f'接近籃球框, 進入後退減速範圍, 籃球框距離 = {motor.basket_distance_x}')
            time.sleep(0.05)

        elif forward_slow_distance > motor.basket_distance_x > forward_stop_distance:    #進入前進減速範圍
            self.MoveContinuous(500+CORRECT[0], 0+CORRECT[1], 0+CORRECT[2], 100, 100, 2)
            # rospy.loginfo(f'進入前進減速範圍, target.basket_size = {target.basket_size}')      
            rospy.loginfo(f'接近籃球框, 進入前進減速範圍, 籃球框距離 = {motor.basket_distance_x}')          
            time.sleep(0.05)

        elif motor.basket_distance_x > forward_slow_distance:                        #大前進
            self.MoveContinuous(1200+CORRECT[0], 0+CORRECT[1], 0+CORRECT[2], 100, 100, 2)    
            # rospy.loginfo(f'大前進, target.basket_size = {target.basket_size}')        
            rospy.loginfo(f'離籃球框很遠, 大前進, 籃球框距離 = {motor.basket_distance_x}')          
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

        # rospy.loginfo(f'now_x = {self.now_x}, now_y = {self.now_y}, now_theta = {self.now_theta}') 
        send.sendContinuousValue(self.now_x, self.now_y, 0, self.now_theta , 0)


    def bodyauto_close(self,next_state):    #步態移動的開關控制(原地踏步)
        if self.now_state == next_state :    
            pass
        elif self.now_state != next_state :
            send.sendBodyAuto(0,0,0,0,1,0)
            self.now_state = next_state

    def basket_distance(self):      #增加整段
        focal_x = TEST_DISTANCE * target.basket_length / BASTET_LENGTH
        rospy.loginfo(f'focal = {focal_x}')
        self.basket_distance_x = FOCAL_LENGTH * BASTET_LENGTH / target.basket_length
        #rospy.loginfo(f'ball_distance = {self.basket_distance_x}')


class Coordinate:
    def __init__(self, x, y):
        self.x, self.y = x, y
    def __add__(self, other):
        return Coordinate((self.x + other.x), (self.y + other.y))
    def __sub__(self, other):
        return Coordinate((self.x - other.x), (self.y - other.y))
    def __floordiv__(self, other):
        return Coordinate((self.x // other), (self.y // other))


if __name__ == '__main__' :
    target = TargetLocation()
    motor = MotorMove()
    strategy = BasketBall()

    try:
        while not rospy.is_shutdown():
            strategy.main()

    except rospy.ROSInterruptException:
        pass