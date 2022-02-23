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
        self.time_body=0
        self.now_state = 0
        


    def move_head(self,ID,Position,max_horizon_size,max_vertical_size,Speed):
        send.sendHeadMotor(ID,Position,Speed)
        if ID == 1 :
            self.horizon =  Position
            if abs(Position - 2048) > max_horizon_size :
                if (Position - (2048 + max_horizon_size)) < 0 :
                    Position = 2048 - max_horizon_size
                elif (Position - (2048 + max_horizon_size)) > 0 :
                    Position = 2048 + max_horizon_size

        else :
            self.vertical = Position
            if abs(Position - 2048) > max_vertical_size :
                    if (Position - (2048 + max_vertical_size)) < 0 :
                        Position = 2048 - max_vertical_size
                    elif (Position - (2048 + max_vertical_size)) > 0 :
                        Position = 2048 + max_vertical_size

    def view_move(self,right_place,left_place,up_place,down_place,speed,delay):

        if self.horizon_start == 1 :
            if self.horizon > left_place:
                self.move_head(1,self.horizon,1300,500,speed)
                self.horizon = self.horizon - speed
                time.sleep(delay) 
            else :
                self.move_head(2,up_place,1300,500,speed)
                self.horizon_start = 0                   
        else :                
            if  self.horizon < right_place:
                self.move_head(1,self.horizon,1300,500,speed)
                self.horizon = self.horizon + speed
                time.sleep(delay) 
            else :
                self.move_head(2,down_place,1300,500,speed)
                self.horizon_start = 1 
    
    def trace_revise(self,x_target,y_target,speed) :
        #if abs(x_target) > 0 and abs(y_target) > 0:
        self.x_differ =  x_target - 160 
        self.y_differ =  y_target - 120 
        self.x_degree = self.x_differ * (70.42 / 320)
        self.y_degree = self.y_differ * (43.3 / 240)
        self.move_head(1, self.horizon - round(self.x_degree * 4096 / 360 *0.1),1300,500,speed)
        self.move_head(2, self.vertical - round(self.y_degree * 4096 / 360 *0.1),1300,500,speed)
        print("x=",self.horizon)
        time.sleep(0.05)

    def body_trace_rotate(self,degree) :
        self.x_body_rotate = self.horizon - 2048
        if self.x_body_rotate > degree :
            send.sendContinuousValue(0-correct_rot[0],0-correct_rot[1],0,5-correct_rot[2],0)
            print( "go left = ",self.x_body_rotate)
            time.sleep(0.05)
        elif self.x_body_rotate < -degree :
            send.sendContinuousValue(0-correct_rot[0],0-correct_rot[1],0,-5-correct_rot[2],0)
            print( "go right = ",self.x_body_rotate)
            time.sleep(0.05)
        

    def body_trace_straight(self,degree,ball_degree) :
        
        if (self.vertical - degree) > ball_degree:
            send.sendContinuousValue(700-correct_str[0],0-correct_str[1],0,0-correct_str[2],0)
            print( "go ahead = ",self.vertical)
            time.sleep(0.05)
        elif (self.vertical - degree) < -ball_degree :
            send.sendContinuousValue(-1-correct_str[0],0-correct_str[1],0,0-correct_str[2],0)
            print( "go back = ",self.vertical)
            time.sleep(0.05)
        elif  abs(self.vertical - degree) < ball_degree :
            #send.sendBodyAuto(0,0,0,0,1,0)
            
            motor.bodyauto_close(0)
            self.time_body = self.time_body + 1
            self.catch = False
            motor.found = True
            time.sleep(3)
            self.waist_revise()
            time.sleep(3)           
            send.sendSingleMotor(1,-50,20)
            time.sleep(2) 
            print("stop to the ball")
            send.sendSingleMotor(9,-round(self.x_body_rotate ),30)
            print(".................................................")
            
            time.sleep(2)
    
    def waist_revise(self):
            # if waist ==  1 :
                # if abs(round(self.x_body_rotate) - 2048) > max_waist :
                #     if (round(self.x_body_rotate) - (2048 + max_waist)) < 0 :
                #         self.x_body_rotate = 2048 - max_waist
                #     elif (round(self.x_body_rotate) - (2048 + max_waist)) > 0 :
                #         self.x_body_rotate = 2048 + max_waist
                #     else :
                #         self.x_body_rotate = 2048
                # else :
                #     pass
                send.sendSingleMotor(9,round(self.x_body_rotate ),30)
                print(self.x_body_rotate)
                time.sleep(0.05)
                


    def body_trace_basket_straight(self,basket_size,basket_error) :
        
        if target.basket_size - basket_size < -basket_error :
            send.sendContinuousValue(1100-correct_str[0],0-correct_str[1],0,0-correct_str[2],0)
            print( "--------------------go ahead to basket---------------------  ",self.vertical)
            time.sleep(0.05)
        elif target.basket_size - basket_size > basket_error :
            send.sendContinuousValue(-1500-correct_str[0],0-correct_str[1],0-correct_str[2],0,0)
            print( "--------------------go back from basket-------------------- ",self.vertical)
            time.sleep(0.05)
        elif abs(target.basket_size - basket_size) < basket_error :
            print( "--------------------stop at the basket----------------------")
            #send.sendBodyAuto(0,0,0,0,1,0)
            motor.bodyauto_close(0)
            self.found = False
            self.catch = True
            time.sleep(2)
            self.time_body = self.time_body + 1
            
    
    

        
        
    def slam(self,score) :
        if score == 1 :
            send.sendSingleMotor(2,-100,30)
            send.sendSingleMotor(1,-950,30)
            send.sendSingleMotor(2,300,30)
            print("---------------sw==%d-------------",sw)
            print("slam dunk-------------slam dunk--------------slam dunk---------")
            time.sleep(2)
            score = score - 1


    
    def bodyauto_close(self,next_state):
        if send.is_start == True :   # 0 stop  1 go
            if self.now_state == next_state :
                pass
            elif self.now_state != next_state :
                send.sendBodyAuto(0,0,0,0,1,0)
                self.now_state = next_state

        elif send.is_start== False :
            if self.now_state == 0 :
                send.sendHeadMotor(1,2048,30)
                send.sendHeadMotor(2,2048,30)
                time.sleep(1)
                send.sendBodySector(29)
                print("-------------------reset-------------------------")

            elif self.now_state == 1 :
                send.sendHeadMotor(1,2048,30)
                send.sendHeadMotor(2,2048,30)
                time.sleep(1)
                send.sendBodySector(29)
                send.sendBodyAuto(0,0,0,0,1,0)
                print("-------------------reset-------------------------")
            
        
       
   
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
            print("-------------------open walking----------------------")
        
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
    
    
    target = target_location()
    motor = motor_move()
    jmp = 0
    sw = 2
    throw_ball_point = [2250,800,13000] #灌籃未寫
    #                   三分  五分  灌籃
    step  = ['find_ball','open_ball_trace','walk_to_ball','ball_trace','open_basket_trace','basket_trace','walk_to_basket','find','waisting','finish']
    #           0              1                2           3             4                   5                  6           7      8
    correct_str = [0,0,0]
    correct_rot = [0,0,0]
    #          x,y,theta    
    basket_size_error = [50,50,300]
    #尚未有sw控制 ：：throw_ball_point[sw]
    trace_parameter =[25]
    #              旋轉大小—球
    try:
        
        
        while not rospy.is_shutdown():
            if send.is_start==True :
                count = 1
                send.drawImageFunction(1,0,160,160,0,240,255,255,255)
                send.drawImageFunction(2,0,0,320,120,120,255,255,255)
                send.drawImageFunction(3,1,target.ball_minx ,target.ball_maxx ,target.ball_miny ,target.ball_maxy,255,0,255)
                send.drawImageFunction(4,1,target.basket_minx ,target.basket_maxx ,target.basket_miny ,target.basket_maxy,0,0,0)

                if motor.found == False  :

                    if step[jmp] == 'find_ball':
                        if target.ball_size < 70 :
                            motor.view_move(2498,1698,1900,1400,30,0.05)                 
                            time.sleep(0.05)
                            print("start to find the ball")
                            
                            target.ball_parameter()  
                            print("  ball => x:",target.x_ball_place," y:",target.y_ball_place," size:",target.ball_size)
                        elif target.ball_size > 70 :
                            jmp = jmp + 1

                    elif step[jmp] == 'open_ball_trace' :
                        if abs(target.x_ball_place - 160) > 4  or abs(target.y_ball_place - 120) > 3 :
                            target.ball_parameter() 
                            print("fdfdfdfdfdfdfdfdfdfdf")
                            motor.trace_revise(target.x_ball_place,target.y_ball_place,25)
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
                            print("sdsdsdds",abs(motor.vertical - 1300))
                            print("motor.x_differ=========",motor.x_differ)
                            print("motor.vertical=========",motor.vertical)
                        
                        elif abs(motor.vertical - 1300) > 5 :
                            print(motor.horizon - 2048)
                            motor.body_trace_straight(1300,450)#!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                            print("check")
# ............................................................................................................................
# ............................................................................................................................
# ............................................................................................................................
# ............................................................................................................................

                elif  motor.found == True and motor.catch == False : 
                    if step[jmp] == 'ball_trace' :
                        time.sleep(0.5)
                        print("start to find the basket")
                        # send.sendHeadMotor(2,2048,30)
                        # send.sendHeadMotor(1,2048,30)
                        time.sleep(0.5)
                        jmp = jmp + 1
                        print("jmp=============",jmp)
                        print("---------------reset the robot's head position-------------")

                    if step[jmp] == 'open_basket_trace' :
                        if target.basket_size < 350 :
                                
                                print("open_basket_trace")
                                motor.view_move(2548,1548,2098,1848,30,0.05)
                                time.sleep(0.05)
                                target.basket_parameter()
                                print("  basket => x:",target.x_basket_place," y:",target.y_basket_place," size:",target.basket_size)
                        elif target.basket_size > 350 :
                                jmp = jmp + 1
                                print("jump!!!!!!!!!")

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
                            motor.body_trace_basket_straight(throw_ball_point[sw],basket_size_error[sw]) # change
                            print("----------------start the action of get point--------------")
                            print("motor.catch = ",motor.catch)
                            if motor.catch == True :
                                jmp = jmp + 1

                # elif step == 6 :
                #     print("-------------------waisting--------------------")
                #     # send.sendSingleMotor(9,300,30)
                #     motor.waist_revise()
                #     motor.slam(1)
                #     step = step + 1
                #     stop = 1 
                #     print("stop========",stop)

                    

                
                    



            elif send.is_start==False and jmp > -1:
                    print("-------------------start to stop------------------")
                    print("enter to stop function and stop  === ",motor.now_state)
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
