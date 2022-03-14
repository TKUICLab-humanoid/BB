#!/usr/bin/env python
#coding=utf-8
import time
from traceback import print_tb
import numpy as np
import rospy
from Python_API import Sendmessage

gobasket = 1
aa = 1
x = 0
x_place=0
y_place=0
ball_size=0
send=Sendmessage()



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
        self.ball_minx = 0
        self.ball_miny = 0
        self.ball_maxx = 0
        self.ball_maxy = 0
        self.basket_minx = 0
        self.basket_miny = 0
        self.basket_maxx = 0
        self.basket_maxy = 0
        
        
    def ball_parameter(self):
        self.color_mask_subject_red = send.color_mask_subject_cnts[0]
        for j in range (self.color_mask_subject_red):
            if send.color_mask_subject_size [0][j] > 70 :
                self.x_ball_place =  send.color_mask_subject_X [0][j]
                self.y_ball_place = send.color_mask_subject_Y [0][j]
                self.ball_size = send.color_mask_subject_size [0][j]
                self.ball_minx = send.color_mask_subject_XMin[0][j] 
                self.ball_miny = send.color_mask_subject_YMin[0][j] 
                self.ball_maxx = send.color_mask_subject_XMax[0][j] 
                self.ball_maxy =send.color_mask_subject_YMax[0][j]
            else :
                self.x_ball_place = 0
                self.y_ball_place = 0
                self.ball_size = 0

            
    
    def basket_parameter(self):
        self.color_mask_subject_orange = send.color_mask_subject_cnts[5]
        for j in range (self.color_mask_subject_orange):
            if send.color_mask_subject_size [5][j] > 300 :
                self.x_basket_place =  send.color_mask_subject_X [5][j]
                self.y_basket_place = send.color_mask_subject_Y [5][j]
                self.basket_size = send.color_mask_subject_size [5][j]
                self.basket_minx = send.color_mask_subject_XMin[5][j] 
                self.basket_miny = send.color_mask_subject_YMin[5][j] 
                self.basket_maxx = send.color_mask_subject_XMax[5][j] 
                self.basket_maxy =send.color_mask_subject_YMax[5][j]
            else :
                self.x_basket_place= 0
                self.y_basket_place = 0
                self.basket_size = 0
   

class motor_move():
    #motor = motor_move()
    target = target_location()
    motor = target_location()
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
        self.now_state = 0
        


    def move_head(self,ID,Position,max_horizon_size,max_vertical_size,Speed):
        send.sendHeadMotor(ID,Position,Speed)
        if ID == 1 :
            self.horizon =  Position
            if abs(self.horizon - 2048) > max_horizon_size :
                if (self.horizon - 2048 ) > 0 :
                    self.horizon = 2048 + max_horizon_size
                elif (self.horizon - 2048 ) < 0 :
                    self.horizon = 2048 - max_horizon_size

        else :
            self.vertical = Position
            if abs(self.vertical - 2048) > max_vertical_size :
                    if (self.vertical - 2048 ) > 0 :
                        self.vertical = 2048 + max_vertical_size
                    elif (self.vertical - 2048) < 0 :
                        self.vertical = 2048 - max_vertical_size

    def view_move(self,right_place,left_place,up_place,down_place,speed,delay):

        if self.horizon_start == 1 :
            if self.horizon > left_place:
                self.move_head(1,self.horizon,880,880,speed)
                self.horizon = self.horizon - speed
                time.sleep(delay) 
            else :
                self.move_head(2,up_place,880,880,100)
                self.horizon_start = 0                   
        else :                
            if  self.horizon < right_place:
                self.move_head(1,self.horizon,880,880,speed)
                self.horizon = self.horizon + speed
                time.sleep(delay) 
            else :
                self.move_head(2,down_place,880,880,100)
                self.horizon_start = 1 
    
    def trace_revise(self,x_target,y_target,speed) :
        #if abs(x_target) > 0 and abs(y_target) > 0:
        self.x_differ =  x_target - 160 
        self.y_differ =  y_target - 120 
        self.x_degree = self.x_differ * (70.42 / 320)
        self.y_degree = self.y_differ * (43.3 / 240)
        self.move_head(1, self.horizon - round(self.x_degree * 4096 / 360 *0.15),880,880,speed)
        self.move_head(2, self.vertical - round(self.y_degree * 4096 / 360 *0.15),880,880,speed)
        print("x=",self.horizon)
        time.sleep(0.05)

    def body_trace_rotate(self,degree) :
        self.x_body_rotate = self.horizon - 2048
        if self.x_body_rotate > degree :
            send.sendContinuousValue(0+correct_left[0],0+correct_left[1],0,4+correct_left[2],0)
            print( "go left = ",self.x_body_rotate)
            time.sleep(0.05)
        elif self.x_body_rotate < -degree :
            send.sendContinuousValue(0+correct_right[0],0+correct_right[1],0,-4+correct_right[2],0)#!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            print( "go right = ",self.x_body_rotate)
            time.sleep(0.05)
        

    def body_trace_straight(self,degree,ball_degree) :
        
        if (self.vertical - degree) > ball_degree:
            send.sendContinuousValue(800+correct_ahead[0],0+correct_ahead[1],0,0+correct_ahead[2],0) #!!!!!!!!!!!!!!!!!!!!!!
            print( "go ahead = ",self.vertical)
            time.sleep(0.05)
        elif (self.vertical - degree) < -ball_degree :
            send.sendContinuousValue(-800+correct_back[0],0+correct_back[1],0,0+correct_back[2],0)
            print( "go back = ",self.vertical)
            time.sleep(0.05)
        elif  abs(self.vertical - degree) < ball_degree :
            #send.sendBodyAuto(0,0,0,0,1,0)
            
            motor.bodyauto_close(0)
            self.catch = False
            self.found = True
            time.sleep(1)
            send.sendBodySector(1)   #111111111111111111111111111111111111111111111111111111111111111
            time.sleep(1)
            # target.ball_parameter()
            
            
    
    def waist_revise(self):
            # if waist ==  1 :time.sleep(2) 
                # if abs(round(self.x_body_rotate) - 2048) > max_waist :
                #     if (round(self.x_body_rotate) - (2048 + max_waist)) < 0 :
                #         self.x_body_rotate = 2048 - max_waist
                #     elif (round(self.x_body_rotate) - (2048 + max_waist)) > 0 :
                #         self.x_body_rotate = 2048 + max_waist
                #     else :
                #         self.x_body_rotate = 2048
                # else :
                #     pass


                # def move_head(self,ID,Position,max_horizon_size,max_vertical_size,Speed):
                #  send.sendHeadMotor(ID,Position,Speed)
                # if ID == 1 :
                #    self.horizon =  Position
                # if abs(self.horizon - 2048) > max_horizon_size :
                # if (self.horizon - 2048 ) > 0 :
                #     self.horizon = 2048 + max_horizon_size
                # elif (self.horizon - 2048 ) < 0 :
                #     self.horizon = 2048 - max_horizon_size
                time.sleep(1)
                send.sendSingleMotor(9,round(self.x_body_rotate ),30)
                print("meowmeowmeoemeowmeow",self.x_body_rotate)
                time.sleep(0.05)
                


    def body_trace_basket_straight_2(self,basket_size,basket_error) :
        
        if target.basket_size - basket_size < -basket_error and  target.basket_size <12000:
            send.sendContinuousValue(1000+correct_ahead[0],0+correct_ahead[1],0,0+correct_ahead[2],0)#!!!!!!!!!!!!!!!
            print( "--------------------go ahead to basket---------------------  ",self.vertical)
            time.sleep(0.05)

        elif target.basket_size - basket_size < -basket_error and  target.basket_size >12000:
            send.sendContinuousValue(300+correct_ahead[0],0+correct_ahead[1],0,0+correct_ahead[2],0)#!!!!!!!!!!!!!!!
            print( "--------------------go ahead to basket---------------------  ",self.vertical)
            time.sleep(0.05)    
        elif target.basket_size - basket_size > basket_error :
            send.sendContinuousValue(-800+correct_back[0],0+correct_back[1],0,0+correct_back[2],0)#!!!!!!!!!!!!!!!!!
            print( "--------------------go back from basket-------------------- ",self.vertical)
            time.sleep(0.05)
        elif abs(target.basket_size - basket_size) < basket_error :
            print( "--------------------stop at the basket----------------------",self.horizon - 2048)
            #send.sendBodyAuto(0,0,0,0,1,0)
            motor.bodyauto_close(0)
            target.basket_parameter()
            
            time.sleep(1)
            send.sendBodySector(3)
            print("sdbasgyufbsauyhfbasuyfbsafubsa")
            time.sleep(2)
            self.found = True
            self.catch = True

    def body_trace_basket_straight_3(self,basket_size,basket_error) :
        
        if target.basket_size - basket_size < -basket_error and  target.basket_size <12000:
            send.sendContinuousValue(1000+correct_ahead[0],0+correct_ahead[1],0,0+correct_ahead[2],0)#!!!!!!!!!!!!!!!
            print( "--------------------go ahead to basket---------------------  ",self.vertical)
            time.sleep(0.05)

        elif target.basket_size - basket_size < -basket_error and  target.basket_size >12000:
            send.sendContinuousValue(300+correct_ahead[0],0+correct_ahead[1],0,0+correct_ahead[2],0)#!!!!!!!!!!!!!!!
            print( "--------------------go ahead to basket---------------------  ",self.vertical)
            time.sleep(0.05)    
        elif target.basket_size - basket_size > basket_error :
            send.sendContinuousValue(-800+correct_back[0],0+correct_back[1],0,0+correct_back[2],0)#!!!!!!!!!!!!!!!!!
            print( "--------------------go back from basket-------------------- ",self.vertical)
            time.sleep(0.05)
        elif abs(target.basket_size - basket_size) < basket_error :
            print( "--------------------stop at the basket----------------------",self.horizon - 2048)
            #send.sendBodyAuto(0,0,0,0,1,0)
            motor.bodyauto_close(0)
            target.basket_parameter()
            
            time.sleep(1)
            send.sendBodySector(3)
            print("sdbasgyufbsauyhfbasuyfbsafubsa")
            time.sleep(2)
            self.found = True
            self.catch = True

    def body_trace_basket_straight_5(self,basket_size,basket_error) :
        
        if target.basket_size - basket_size < -basket_error and  target.basket_size <12000:
            send.sendContinuousValue(1000+correct_ahead[0],0+correct_ahead[1],0,0+correct_ahead[2],0)#!!!!!!!!!!!!!!!
            print( "--------------------go ahead to basket---------------------  ",self.vertical)
            time.sleep(0.05)

        elif target.basket_size - basket_size < -basket_error and  target.basket_size >12000:
            send.sendContinuousValue(300+correct_ahead[0],0+correct_ahead[1],0,0+correct_ahead[2],0)#!!!!!!!!!!!!!!!
            print( "--------------------go ahead to basket---------------------  ",self.vertical)
            time.sleep(0.05)    
        elif target.basket_size - basket_size > basket_error :
            send.sendContinuousValue(-800+correct_back[0],0+correct_back[1],0,0+correct_back[2],0)#!!!!!!!!!!!!!!!!!
            print( "--------------------go back from basket-------------------- ",self.vertical)
            time.sleep(0.05)
        elif abs(target.basket_size - basket_size) < basket_error :
            print( "--------------------stop at the basket----------------------",self.horizon - 2048)
            #send.sendBodyAuto(0,0,0,0,1,0)
            motor.bodyauto_close(0)
            target.basket_parameter()
            
            time.sleep(1)
            send.sendBodySector(3)
            print("sdbasgyufbsauyhfbasuyfbsafubsa")
            time.sleep(2)
            self.found = True
            self.catch = True
    
    

        
        
    def slam(self,score) :
        if score == 1 :
            send.sendSingleMotor(2,-100,30)
            send.sendSingleMotor(1,-950,30)
            send.sendSingleMotor(2,300,30)
            print("---------------sw==%d-------------",sw)
            print("slam dunk-------------slam dunk--------------slam dunk---------")
            time.sleep(2)
            score = score - 1

    # def doubly_waist(self) :
        
        
    #     if  abs(motor.horizon - 2048) > 5  :
    #         if motor.horizon - 2048 < -5 :
    #             send.sendSingleMotor(9,-2,10)
    #             motor.horizon = motor.horizon + 2
    #             print("66666666666666666666666666666")
    #         elif motor.horizon - 2048 > 5 :
    #             send.sendSingleMotor(9,2,10)
    #             motor.horizon = motor.horizon - 2
    #             print("94994949949494949494949494949")

    #     elif  abs(target.x_ball_place - 160) > 1 :     
    #         motor.trace_revise(target.x_ball_place,target.y_ball_place,25)
    #         time.sleep(0.05)
    #         print("787878787887878787888788878787878787878")


    
    def bodyauto_close(self,next_state):
        if send.is_start == True :   # 0 stop  1 go
            if self.now_state == next_state :
                pass
            elif self.now_state != next_state :
                send.sendBodyAuto(0,0,0,0,1,0)
                self.now_state = next_state

        elif send.is_start == False :
            if self.now_state == 0 :
                send.sendHeadMotor(1,2048,30)
                send.sendHeadMotor(2,2048,30)
                time.sleep(1)
                send.sendBodySector(29)
                print("-------------------reset and stoping-------------------------")
                time.sleep(2)

            elif self.now_state == 1 :
                send.sendHeadMotor(1,2048,30)
                send.sendHeadMotor(2,2048,30)
                time.sleep(1)
                send.sendBodySector(29)
                send.sendBodyAuto(0,0,0,0,1,0)
                print("-------------------reset and stoping-------------------------")
            
        
       
   
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
    #         time.sleep(delay)print("---------again to find the ball-----------------")


if __name__ == '__main__' :
    
    step = 0
    head_back = 20000
    count = 0 
    stop=1
    target = target_location()
    motor = motor_move()
    jmp = 0
    sw = 2
    gazebo_robot = 1
    # 0 for gazebo 1 for robot
    throw_ball_point = [2250,800,17700] #灌籃未寫
    #                   三分  五分  灌籃
    step  = ['find_ball','open_ball_trace','walk_to_ball','ball_trace','catch_ball','open_basket_trace','basket_trace','walk_to_basket','find','waisting','finish']
    #           0              1                2           3             4                   5                  6           7      8
    correct_right = [0,0,-3]
    correct_left =  [0,0,-3]
    correct_ahead = [0,0,-3]
    correct_back =  [0,0,-3]

    # correct_right = [0,0,0]
    # correct_left =  [0,0,0]
    # correct_ahead = [0,0,0]
    # correct_back =  [0,0,0]
    #          x,y,theta    
    basket_error = [50,50,400]
    #尚未有sw控制 ：：throw_ball_point[sw] 3分跟5分一樣

    ball_correct = [20,150]

    trace_parameter =[25]
    straight =[motor.body_trace_basket_straight_3(throw_ball_point[sw],250),motor.body_trace_basket_straight_5(throw_ball_point[sw],250),motor.body_trace_basket_straight_2(throw_ball_point[sw],250)]
    #              旋轉大小—球
    try:
        
        
        while not rospy.is_shutdown():
            if send.is_start==True :
                count = 1
                send.drawImageFunction(1,0,160,160,0,240,255,255,255)
                send.drawImageFunction(2,0,0,320,120,120,255,255,255)
                send.drawImageFunction(3,1,target.ball_minx ,target.ball_maxx ,target.ball_miny ,target.ball_maxy,255,0,255)
                send.drawImageFunction(4,1,target.basket_minx ,target.basket_maxx ,target.basket_miny ,target.basket_maxy,0,0,0)
                target.ball_parameter() 
                

                if motor.found == False  :
                    print(target.ball_size)
                    if step[jmp] == 'find_ball':
                        if target.ball_size < 70 :
                            motor.view_move(2698,1498,1800,1098,30,0.05)                 
                            time.sleep(0.05)
                            print("start to find the ball")
                            print("stop====\n",stop)
                            target.ball_parameter()  
                            print("  ball => x:",target.x_ball_place," y:",target.y_ball_place," size:",target.ball_size)
                        elif target.ball_size > 70 :
                            jmp = jmp + 1

                    elif step[jmp] == 'open_ball_trace' :
                        if abs(target.x_ball_place - 160) > 4  or abs(target.y_ball_place - 120) > 3 :
                            target.ball_parameter() 
                            print("open_ball_trace is opening")
                            motor.trace_revise(target.x_ball_place,target.y_ball_place,25)
                            time.sleep(0.05) 
                        else :
                           jmp = jmp + 1
                           print("jmp======",jmp)

                    elif  step[jmp] == 'walk_to_ball' :
                        time.sleep(1)
                        print("-------------start walk to the ball--------------")
                        motor.bodyauto_close(1)
                        jmp = jmp + 1

                    elif  step[jmp] == 'ball_trace' :
                        
                        target.ball_parameter()   
                        print(" ball => x:",target.x_ball_place," y:",target.y_ball_place," size:",target.ball_size)                   
                        motor.trace_revise(target.x_ball_place,target.y_ball_place,25)
                        print("abs(motor.x_body_rotate)",abs(motor.x_body_rotate),motor.horizon-2048)

                        if abs(motor.horizon-2048) > trace_parameter[0]  :  
                            motor.body_trace_rotate(trace_parameter[0])    
                            print("motor.vertical=========",motor.vertical)
                        
                        elif abs(motor.vertical - 1320) > 5 : #1320是條球的距離150是誤差
                            print(motor.horizon - 2048)
                            motor.body_trace_straight(1320,ball_correct[gazebo_robot])#!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                            print("motor.vertical-1320 = ",motor.vertical-1320)
# ............................................................................................................................
# ............................................................................................................................
# ............................................................................................................................
# ............................................................................................................................

                elif  motor.found == True and motor.catch == False : 
                    target.basket_parameter() 
                    if step[jmp] == 'ball_trace' :
                        target.ball_parameter() 
                        # motor.doubly_waist()
                        if abs(target.x_ball_place - 160) > 4  or abs(target.y_ball_place - 120) > 3 :
                            target.ball_parameter() 
                            print("open_ball_trace is opening")
                            motor.trace_revise(target.x_ball_place,target.y_ball_place,25)
                            time.sleep(0.05) 
                        else :
                           jmp = jmp + 1
                           print("jmp======",jmp)



                    elif step[jmp] == 'catch_ball' :   
                        
                        time.sleep(3) 
                        motor.waist_revise()
                        send.sendBodySector(2)    #111111111111111111111111111111111111111111111111111111111111111
                        time.sleep(2)     
                        print("stop to the ball")
                        
                        
                        send.sendSingleMotor(9,-round(motor.x_body_rotate ),30)
                        time.sleep(1)
                        #send.sendBodySector(3)
                        time.sleep(1)
                        print(".................................................")
            
                        jmp = jmp + 1#!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                                                                        

                    if step[jmp] == 'open_basket_trace' :
                        target.basket_parameter()
                        if target.basket_size < 350 :
                                
                                print("open_basket_trace")
                                motor.view_move(2448,1648,2098,1848,30,0.05)
                                time.sleep(0.05)
                                target.basket_parameter()
                                print("  basket => x:",target.x_basket_place," y:",target.y_basket_place," size:",target.basket_size)
                        elif target.basket_size > 350 :
                                jmp = jmp + 1
                                print("jump to basket_trace   !!!!!!!!!")

                    elif step[jmp] == 'basket_trace' :
                        if abs(target.x_basket_place - 160) > 5  or abs(target.y_basket_place - 120) > 3 :
                            target.basket_parameter() 
                            motor.trace_revise(target.x_basket_place,target.y_basket_place,25)
                        else :
                            time.sleep(1)
                            
                            motor.bodyauto_close(1)
                            
                            jmp = jmp + 1


                    elif  step[jmp] == 'walk_to_basket' :
                        print("walk_to_basket")
                        target.basket_parameter()
                        print(" basket => x:",target.x_basket_place," y:",target.y_basket_place," size:",target.basket_size)
                        motor.trace_revise(target.x_basket_place,target.y_basket_place,25)

                        


                        if abs(motor.horizon-2048) > 25 :#!!!!
                            motor.body_trace_rotate(30)
                            print("target.basket_size ==",target.basket_size)
                            print("throw_ball_point ==",throw_ball_point)
                            print("",abs(target.basket_size - throw_ball_point[sw]))

                        elif abs(target.basket_size - throw_ball_point[sw]) > 10 :#!!!!!!!!!!!!!!!
                            print("----------------start the action of get point--------------")
                            motor.body_trace_basket_straight_2(throw_ball_point[sw],250)
                            
                            print("motor.catch = ",motor.catch)
                            if motor.catch == True :
                                
                                print("jmp,step[jmp] is ",jmp,step[jmp])

                elif  motor.found == True and motor.catch == True :
                    if step[jmp] == 'walk_to_basket' :
                        target.basket_parameter()
                        # if abs(target.x_basket_place - 160) > 5  or abs(target.y_basket_place - 120) > 3 :
                             
                        #     motor.trace_revise(target.x_basket_place,target.y_basket_place,25)
                        # else :
                        time.sleep(1)
                        jmp = jmp + 1

                    elif step[jmp] == 'find' :
                        # motor.doubly_waist()
                        target.basket_parameter()
                        time.sleep(2)
                        motor.waist_revise()
                        time.sleep(2)
                        send.sendBodySector(4)
                        print("5555555555")
                        # if  abs(motor.horizon - 2048) <= 5  and abs(target.x_basket_place - 160) <= 1 :
                        #     print("---------------gogogogogogogogogogogogogogoro-------------")
                        jmp = jmp + 1 #都一直線
                # elif step == 6 :
                #     print("-------------------waisting--------------------")
                #     # send.sendSingleMotor(9,300,30)
                #     motor.waist_revise()
                #     motor.slam(1)
                #     step = step + 1
                #     stop = 1 
                #     print("stop========",stop)

                    

                
                    



            elif send.is_start==False and jmp > -1:
                        
                        motor.bodyauto_close(motor.now_state)
                        aa= 1
                        target = target_location()
                        motor = motor_move()
                        jmp = 0
            # else :
                #   motor.found = False    
                #   motor.catch = False     
                #   aa= 1
                   




                # elif  abs(motor.x_differ) > 3 :
                #     time.sleep(0.3)
                #     target.ball_parameter()   
                #     print(" ball => x:",target.x_ball_place," y:",target.y_ball_place," size:",target.ball_size)
            
                #     send.sendBodyAuto(0,0,0,0,1,0)                     
                #     motor.trace_revise(target.x_ball_place,target.y_ball_place,25)
                #     motor.body_trace_rotate()

                # else :
                #     time.sleep(0.3)
                #     target.ball_parameter()   
                #     print(" ball => x:",target.x_ball_place," y:",target.y_ball_place," size:",target.ball_size)
            
                #     send.sendBodyAuto(0,0,0,0,1,0)                     
                #     motor.trace_revise(target.x_ball_place,target.y_ball_place,25)
                #     motor.body_trace_straight(target.ball_size,500)
                        
                        # if  :
                        #     motor.trace_revise(target.x_ball_place,target.y_ball_place,25)
                        #     motor.body_trace_rotate()



                        # if ball_size < 500 or ball_size >1200:
                        #     motor.trace_revise(target.x_ball_place,target.y_ball_place,25)
                        #     motor.body_trace_straight(target.ball_size,500)
                        # else :
                        #     pass

    except rospy.ROSInterruptException:
        pass