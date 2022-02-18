#!/usr/bin/env python
#coding=utf-8

import rospy
import numpy as np
from Python_API import Sendmessage
import time

send = Sendmessage()
class view_message:
    def __init__(self):
        #影像位置
        self.basket_X       =   0   #框
        self.basket_XMin    =   0
        self.basket_XMax    =   0
        
        self.basket_Y       =   0
        self.basket_YMin    =   0
        self.basket_YMax    =   0

        self.ball_X         =   0   #球
        self.ball_XMin      =   0
        self.ball_XMax      =   0
            
        self.ball_Y         =   0
        self.ball_YMin      =   0
        self.ball_YMax      =   0

        self.white_X        =   0
        self.white_XMin     =   0   #線
        self.white_XMax     =   0

        self.white_YMin     =   0
        self.white_YMax     =   0

        self.basket_size    =   0   #大小
        self.ball_size      =   0
        self.white_size     =   0

    def imageinfor(self):

        for basket_red in range (send.color_mask_subject_cnts[5]):
            
            if send.color_mask_subject_size[5][basket_red] > 300:
                self.basket_X       =   send.color_mask_subject_X[5][basket_red]
                self.basket_XMin    =   send.color_mask_subject_XMin[5][basket_red]
                self.basket_XMax    =   send.color_mask_subject_XMax[5][basket_red]
                
                self.basket_Y       =   send.color_mask_subject_Y[5][basket_red]
                self.basket_YMin    =   send.color_mask_subject_YMin[5][basket_red]
                self.basket_YMax    =   send.color_mask_subject_YMax[5][basket_red]

                self.basket_size    =   send.color_mask_subject_size[5][basket_red]
            else:
                self.basket_X       =   0
                self.basket_XMin    =   0
                self.basket_XMax    =   0
                
                self.basket_Y       =   0
                self.basket_YMin    =   0
                self.basket_YMax    =   0

                self.basket_size    =   0

        for ball_orange in range (send.color_mask_subject_cnts[0]):

            if send.color_mask_subject_size[0][ball_orange] > 300:
                self.ball_X         =   send.color_mask_subject_X[0][ball_orange]
                self.ball_XMin      =   send.color_mask_subject_XMin[0][ball_orange]
                self.ball_XMax      =   send.color_mask_subject_XMax[0][ball_orange]
                
                self.ball_Y         =   send.color_mask_subject_Y[0][ball_orange]
                self.ball_YMin      =   send.color_mask_subject_YMin[0][ball_orange]
                self.ball_YMax      =   send.color_mask_subject_YMax[0][ball_orange]

                self.ball_size      =   send.color_mask_subject_size[0][ball_orange]
            else:
                self.ball_X         =   0
                self.ball_XMin      =   0
                self.ball_XMax      =   0
                
                self.ball_Y         =   0
                self.ball_YMin      =   0
                self.ball_YMax      =   0

                self.ball_size      =   0

        for white in range (send.color_mask_subject_cnts[6]):

            if send.color_mask_subject_size[6][white] > 1000:   
                self.white_X        =   send.color_mask_subject_X[6][white]
                self.white_XMin     =   send.color_mask_subject_XMin[6][white]
                self.white_XMax     =   send.color_mask_subject_XMax[6][white]

                self.white_YMin     =   send.color_mask_subject_YMin[6][white]
                self.white_YMax     =   send.color_mask_subject_YMax[6][white]

                self.white_size     =   send.color_mask_subject_size[6][white]
            else:
                self.white_XMin     =   0
                self.white_XMax     =   0

                self.white_YMin     =   0
                self.white_YMax     =   0

                self.white_size     =   0

send = Sendmessage()
class move_message:

    def __init__(self):
        #動作位置
        self.now_RL_Position    =   2048
        self.now_UD_Position    =   2048

        self.now_waist_Position =   0
        
        self.robot_theta        =   0

        self.phase              =   0

        self.turn_head          =   0

        self.Record_Goal_X      =   None
        self.Record_Goal_Y      =   None

        self.walk_flag          =   False

        self.Move_X             =   0 
        self.Move_Y             =   0 
        self.Fallow_Goal_X      =   0 
        self.Fallow_Goal_Y      =   0

        self.change_point       =   0

    def now_HeadMotor(self,ID,Position,Speed):	    #頭部馬達 ID: 1左右 2上下

        if ID == 1:
            self.now_RL_Position = Position
            
        if ID == 2:
            self.now_UD_Position = Position
            
        send.sendHeadMotor(ID,Position,Speed)

    def Body_walk(self, walk_state):

        if walk_state == False and self.walk_flag == True:          #不要走 且 正在走
            send.sendBodyAuto(0,0,0,0,1,0)
            self.walk_flag = False
            print("\n'''''''''''''''''STOP'''''''''''''''''''\n")
            
        if walk_state == True  and self.walk_flag == False:         #要走  且 沒在走
            send.sendBodyAuto(0,0,0,0,1,0)
            self.walk_flag = True
            print("\n'''''''''''''''''MOVE'''''''''''''''''''\n")

    def now_SingleMotor(self,ID,Position,Speed):    #全身馬達 
        
        if ID == 9:                                 #9號 腰部馬達
            if Position > 0:
                self.now_waist_Position += Position

            if Position < 0:
                self.now_waist_Position += Position

        send.sendSingleMotor(ID,Position,Speed)

    def now_sendContinuousValue(self,x,y,z,theta,sensor):

        self.robot_theta = theta
        send.sendContinuousValue(x,y,z,theta,sensor)
    
    def find_goal(self, View_Goal_X, Head_DN_Position, move_Max, move_min, speed):   #初始階段 0 :找球

        if self.turn_head == 0:
            move.now_HeadMotor(2, Head_DN_Position, speed)          #低頭

            if move.now_RL_Position < move_Max:
                move.now_HeadMotor(1, move.now_RL_Position + 10, speed)
                # print("View_ball_size = ", view.ball_size)
                print("now_RL_Position = ", move.now_RL_Position)
                print("View_Goal_X = ", View_Goal_X)  
                time.sleep(0.05)   #延遲

            else:
                self.turn_head = 1
                print("head_right")

        if self.turn_head == 1:
            move.now_HeadMotor(2, 1800, speed)                      #抬頭

            if move.now_RL_Position > move_min:
                move.now_HeadMotor(1, move.now_RL_Position - 10, speed)
                # print("View_ball_size = ", view.ball_size)
                print("now_RL_Position = ", move.now_RL_Position)
                print("View_Goal_X = ", View_Goal_X)
                time.sleep(0.05)   #延遲

            else:
                self.turn_head = 0
                print("head_left")

    def Fallow_Goal(self, View_Goal_X, View_Goal_Y, speed):
        if View_Goal_X > 0:
            #追蹤
            self.Move_X = View_Goal_X - 160

            self.Fallow_Goal_X = round( ( 70.42 * ( self.Move_X / 320 ) ) * 0.25 * ( 4096 / 360 ) )
                                                                                #How much scale to move 

            move.now_HeadMotor(1, move.now_RL_Position - self.Fallow_Goal_X, speed)

        if View_Goal_Y > 0:
            #追蹤
            self.Move_Y = View_Goal_Y - 120

            self.Fallow_Goal_Y = round( ( 70.42 * ( self.Move_Y / 240 ) ) * 0.25 * ( 4096 / 360 ) )
                                                                                #How much scale to move 

            move.now_HeadMotor(2, move.now_UD_Position - self.Fallow_Goal_Y, speed)

    def turn_to_goal(self, View_Goal_X, head_move, speed, X, Y, left_big_theta, right_big_theta):

        if View_Goal_X > 2100:    #目標在左
            move.now_sendContinuousValue(X, Y, 0, left_big_theta, 0)            #旋轉量
            print("turn_left_big")
            print("now_theta = ", self.robot_theta)
            
        if View_Goal_X < 2000:    #目標在右
            move.now_sendContinuousValue(X, Y, 0, right_big_theta, 0)           #旋轉量                    
            print("turn_right_big")
            print("now_theta = ", self.robot_theta)

        print("now_RL_Position = ", move.now_RL_Position)

    def Go_to_Goal(self, x_Max, x_Min,   X, Y,   left_small_theta, right_small_theta,   forword_X, forword_Y, forword_theta):
        
        if move.now_RL_Position > x_Max:     #目標在左邊
            move.now_sendContinuousValue(X, Y, 0, left_small_theta, 0)          #旋轉量
            print("turn_left_small")
            print("now_theta = ", self.robot_theta)

        elif move.now_RL_Position < x_Min:   #目標在右邊
            move.now_sendContinuousValue(X, Y, 0, right_small_theta, 0)         #旋轉量
            print("turn_right_small")
            print("now_theta = ", self.robot_theta)

        else:                       #直走
            move.now_sendContinuousValue(forword_X, forword_Y, 0, forword_theta, 0)
            print("go straight")
            print("now_theta = ", self.robot_theta)

    def Turn_Waist_to_Goal(self, Goal_x, left_Waist_Amount, Right_Waist_Amount, Limit, speed):

        if Goal_x < Limit:
            move.now_SingleMotor(9, left_Waist_Amount, speed)

        if Goal_x > Limit:
            move.now_SingleMotor(9, Right_Waist_Amount, speed)

    def Go_Back_and_straight(self, Goal, Max, Min,   Back_X, Back_Y, Back_theta,   Straight_X, Straight_Y, Straight_theta,   left_X, left_Y, left_theta,   right_X, right_Y, right_theta):

        if view.white_size > Max:                      #退至線後
            move.now_sendContinuousValue( Back_X, Back_Y, 0, Back_theta, 0)
            print("go back")
            print("now_theta = ", move.robot_theta)

        if view.white_size < Min:                      #退過頭往前
            move.now_sendContinuousValue( Straight_X, Straight_Y, 0, Straight_theta, 0)
            print("go straight")
            print("now_theta = ", move.robot_theta)

        if view.white_X > 180:
            move.now_sendContinuousValue( left_X, left_Y, 0, right_theta, 0)
            print("Go Right")
            print("now_theta = ", move.robot_theta)
        
        if view.white_X < 140:
            move.now_sendContinuousValue( right_X, right_Y, 0, left_theta, 0)
            print("Go Left")
            print("now_theta = ", move.robot_theta)


if __name__ == '__main__':
    view = view_message()
    move = move_message()
    try:
        while not rospy.is_shutdown():
            # if send.Web == False :
                # print(send.Web)

            if send.is_start == False :
            #     print(send.is_start)

                view.imageinfor()
                
                send.drawImageFunction(1,0,160,160,0,240,0,0,0)
                send.drawImageFunction(2,0,0,320,120,120,0,0,0)
                send.drawImageFunction(3,1,120,200,80,160,0,0,0)
                send.drawImageFunction(4,1,80,240,40,200,0,0,0)

                send.drawImageFunction(5,0,0,320,view.white_YMin,view.white_YMin,255,0,0)
                send.drawImageFunction(6,0,0,320,view.white_YMax,view.white_YMax,255,0,0)

                print("\nwhite_YMin = ", view.white_YMin)
                print("\nwhite_YMax = ", view.white_YMax)
                print("\nwhite_Size = ", view.white_size)
                print("\nball_size = ", view.ball_size)

                move.Body_walk(False)

                send.sendBodySector(29)
                send.sendHeadMotor(1, 2048, 100)
                send.sendHeadMotor(2, 2048, 100)

                view.__init__()
                move.__init__()

                time.sleep(1)

            # if send.Web == True :
            
            if send.is_start == True :
                
                view.imageinfor()
                
                send.drawImageFunction(1,0,160,160,0,240,0,0,0)
                send.drawImageFunction(2,0,0,320,120,120,0,0,0)
                send.drawImageFunction(3,1,120,200,80,160,0,0,0)
                send.drawImageFunction(4,1,80,240,40,200,0,0,0)

                send.drawImageFunction(5,0,0,320,view.white_YMin,view.white_YMin,255,0,0)
                send.drawImageFunction(6,0,0,320,view.white_YMax,view.white_YMax,255,0,0)

                if move.phase == 0:                             #進入階段 0 :找球
                    view.imageinfor()
                    
                    if view.ball_X > 80 and view.ball_X < 240:
                    # if view.ball_size > 95:

                        print("\nstart turn to ball")
                        move.phase = 1                          #進入階段 1 :轉向球
                        print("phase = ", move.phase)           #找球結束

                        time.sleep(1)                                       #延遲

                    else:
                        move.find_goal( view.ball_X, 1600, 2600, 1400, 100)
                        # View_Goal_X, Head_DN_Position, move_Max, move_min, speed
                    
                if move.phase == 1:
                    view.imageinfor()
                    print("\nView_Ball_X = ", view.ball_X)  
                    print("View_Ball_Y = ", view.ball_Y)

                    print("\nFallow_Goal_X = ", move.Fallow_Goal_X)
                    print("Fallow_Goal_Y = ", move.Fallow_Goal_Y)

                    move.Body_walk(True)                                    #啟動連續步態

                    if move.now_RL_Position > 2000 and move.now_RL_Position < 2100:
                        move.now_HeadMotor(1, 2048, 100)                    #頭擺正
                        
                        move.phase = 2                          #進入階段 2 :走向球
                        print("phase = ", move.phase)           #轉向球結束
                    
                    else:
                        move.Fallow_Goal( view.ball_X, view.ball_Y, 200)
                        # View_Goal_X, View_Goal_Y, speed

                        move.turn_to_goal( move.now_RL_Position, 10, 200, 0, 0, 4, -4)
                        # View_Goal_X, head_move, speed, X, Y, left_big_theta, right_big_theta
                        time.sleep(0.5)

                if move.phase == 2:
                    view.imageinfor()
                    print("\nView_Ball_X = ", view.ball_X)  
                    print("\nView_Ball_Y = ", view.ball_Y)

                    if move.now_UD_Position > 1300 and move.now_UD_Position < 1500:
                        
                        move.now_sendContinuousValue(0, 0, 0, 0, 0)
                        print("No Headway")                                 #原地踏步
                        time.sleep(2)                                       #延遲
                        
                        move.Body_walk(False)                               #關閉連續步態
                        
                        move.now_HeadMotor(1, 2048, 100)
                        print("stop")
                        time.sleep(3)                                       #延遲

                        move.phase = 3                          #進入階段 3 :夾球
                        print("phase = ", move.phase)           #走向球結束
                    
                    else:
                        move.Fallow_Goal( view.ball_X, view.ball_Y, 200)
                        # View_Goal_X, View_Goal_Y, speed

                        move.Go_to_Goal( 2100, 2000,   0, 0, 3, -3,   1500, 0, 0)
                        # x_Max, x_Min,   X, Y, left_small_theta, right_small_theta, 
                        #                 forword_X, forword_Y, forword_theta
                        time.sleep(0.5)

                if move.phase == 3:
                    view.imageinfor()
                    print("\nView_Ball_X = ", view.ball_X)  
                    print("\nView_Ball_Y = ", view.ball_Y)

                    if view.ball_X > 158 and view.ball_X < 162:
                        print("start to chack ball")
                    
                        time.sleep(3)                                       #延遲

                        move.now_SingleMotor(9, - move.now_waist_Position, 100)
                        time.sleep(3)                                       #腰轉回來

                        move.phase = 4                          #進入階段 4 :找框
                        print("phase = ", move.phase)           #夾球結束
                    
                    else:                                                   #轉腰
                        move.Turn_Waist_to_Goal( view.ball_X, 1, -1, 160, 300)
                        # Goal_x, left_Waist_Amount, Right_Waist_Amount, Limit, speed
                        print("Now_Waist_Position = ", move.now_waist_Position)
                        time.sleep(0.05)

                if move.phase == 4:
                    view.imageinfor()

                    if view.basket_X > 80 and view.basket_X < 240:
                    # if view.ball_size > 95:

                        if move.change_point == 0:    
                            move.phase = 5                      #進入階段 5 :轉向框
                            print("start turn to basket")
                            print("phase = ", move.phase)       #找框結束

                        if move.change_point == 1 or move.change_point == 2:
                            move.phase = 9                      #進入階段 9 :腰轉向框
                            move.now_HeadMotor(1, 2048, 100)
                            print("phase = ", move.phase)       #找框結束

                        time.sleep(1)                                       #延遲

                    else:
                        move.find_goal( view.basket_X, 2048, 2800, 1200, 100)   
                        # View_Goal_X, Head_DN_Position, move_Max, move_min, speed

                if move.phase == 5:
                    view.imageinfor()
                    print("\nView_Basket_X = ", view.basket_X)  
                    print("\nView_Basket_Y = ", view.basket_Y)

                    move.Body_walk(True)                                    #啟動連續步態

                    if move.now_RL_Position > 2000 and move.now_RL_Position < 2100:
                        move.now_HeadMotor(1, 2048, 100)                    #頭擺正
                        
                        # move.phase = 6                          #進入階段 6 :走向框
                        # print("phase = ", move.phase)           #轉向框結束

                        move.now_HeadMotor(2, 2048, 100)                    #抬頭

                        move.phase = 8                          #進入階段 8 :退到線後
                        change_point = 1                        #到5分線後
                        # change_point = 2                        #到3分線後
                        print("phase = ", move.phase)           #轉向框結束
                        
                        time.sleep(1)
                    
                    else:
                        move.Fallow_Goal( view.basket_X, view.basket_Y, 200)
                        # View_Goal_X, View_Goal_Y, speed

                        move.turn_to_goal( move.now_RL_Position, 10, 100, 0, 0, 4, -4)
                        # View_Goal_X, head_move, speed, X, Y, left_big_theta, right_big_theta
                        time.sleep(0.5)

                if move.phase == 6:
                    view.imageinfor()
                    print("\nView_Basket_X = ", view.basket_X)  
                    print("\nView_Basket_YMax = ", view.basket_YMax)

                    if view.basket_YMax > 160:                  #basket_YMax 多少停下
                        
                        move.now_sendContinuousValue(0, 0, 0, 0, 0)
                        print("No Headway")                                 #原地踏步
                        time.sleep(2)                                       #延遲
                        
                        move.Body_walk(False)                               #關閉連續步態
                        
                        print("stop")
                        time.sleep(3)

                        move.phase = 7                          #進入階段 7 :灌籃
                        print("phase = ", move.phase)           #走向框結束
                        
                    else:
                        move.Fallow_Goal( view.ball_X, view.ball_Y, 200)
                        # View_Goal_X, View_Goal_Y, speed

                        move.Go_to_Goal( 2100, 2000,   0, 0, 3, -3,   1500, 0, 0)
                        # x_Max, x_Min,   X, Y, left_small_theta, right_small_theta, 
                        #                 forword_X, forword_Y, forword_theta
                        time.sleep(0.5)

                if move.phase == 7:
                    view.imageinfor()
                    print("\nView_Basket_X = ", view.basket_X)  
                    print("\nView_Basket_Y = ", view.basket_Y)

                    if view.basket_X > 158 and view.basket_X < 162:
                        print("start dunk")
                    
                        time.sleep(2)                                       #延遲

                        move.phase = 100                            
                        print("phase = ", move.phase)           #灌籃結束
                    
                    else:                                                   #轉腰
                        move.Turn_Waist_to_Goal( view.basket_X, 2, -2, 160, 300)
                        # Goal_x, left_Waist_Amount, Right_Waist_Amount, Limit, speed
                        print("Now_Waist_Position = ", move.now_waist_Position)
                        time.sleep(0.05)

                if move.phase == 8:
                    view.imageinfor()
                    print("\nwhite_YMin = ", view.white_YMin)
                    print("\nwhite_YMax = ", view.white_YMax)
                    print("\nwhite_Size = ", view.white_size)

                    if ((view.white_size < 3500 and view.white_size > 3400 and change_point == 1) or (view.white_size < 2950 and view.white_size > 2900 and change_point == 2)):
                        move.now_sendContinuousValue(0, 0, 0, 0, 0)
                        print("No Headway")                                 #原地踏步
                        time.sleep(2)                                       #延遲
                        
                        move.Body_walk(False)                               #關閉連續步態

                        print("stop")
                        time.sleep(3)

                        move.change_point = 2                   #改變進入階段

                        move.phase = 4                          #再次進入階段 4 :確認框
                        print("phase = ", move.phase)           #退至線後結束

                    else:
                        print("white_X = ", view.white_X)
                        
                        if change_point == 1:                               #退致5分線
                            move.Go_Back_and_straight( view.white_size, 3500, 3500,   -2500, 0, 0,   100, 0, 0,   0, 0, 0,   0, 0, -2)
                            # Goal, Max, Min,   Back_X, Back_Y, Back_theta,   Straight_X, Straight_Y, Straight_theta,   
                            #                   left_X, left_Y, left_theta,   right_X,    right_Y,    right_theta

                        if change_point == 2:                               #退致3分線
                            move.Go_Back_and_straight( view.white_size, 2950, 2900,   -2500, 0, 0,   100, 0, 0,   0, 0, 2,   0, 0, -2)
                            # Goal, Max, Min,   Back_X, Back_Y, Back_theta,   Straight_X, Straight_Y, Straight_theta,   
                            #                   left_X, left_Y, left_theta,   right_X,    right_Y,    right_theta

                        time.sleep(0.1)

                if move.phase == 9:
                    view.imageinfor()
                    print("\nView_Basket_X = ", view.basket_X)  

                    if view.basket_X > 158 and view.basket_X < 162:                        
                        
                        if view.white_size < 3500 +200 and view.white_size > 3500 -200: 
                            print("start shoot 5 point")
                        
                            time.sleep(2)                                   #延遲

                            move.phase = 100                            
                            print("phase = ", move.phase)       #投籃結束

                        if view.white_size < 2950 +200 and view.white_size > 2900 -200:
                            print("start shoot 3 point")
                        
                            time.sleep(2)                                   #延遲

                            move.phase = 100                            
                            print("phase = ", move.phase)       #投籃結束
                    
                    else:                                                   #轉腰
                        move.Turn_Waist_to_Goal( view.basket_X, 2, -2, 160, 300)
                        # Goal_x, left_Waist_Amount, Right_Waist_Amount, Limit, speed
                        print("Now_Waist_Position = ", move.now_waist_Position)
                        time.sleep(0.05)


    except rospy.ROSInterruptException:
        pass