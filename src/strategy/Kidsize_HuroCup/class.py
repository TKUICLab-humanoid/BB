#!/usr/bin/env python
#coding=utf-8
from cmath import sqrt
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
        self.waist_position = 2048
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
        self.MoveW = 0
        self.MoveY = 0
        self.throw_strength = 0
        self.sixty_distance = 0
        self.ninty_distance = 0
        self.detect = False
        self.distance_new = 0
        


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

    def waist_move(self,waist_x,Speed):
        send.sendSingleMotor(9,waist_x-self.waist_position,Speed)
        self.waist_position =  waist_x 
    

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
            send.sendContinuousValue(0+correct[0],0+correct[1],0,7+correct[2],0)
            print( "go left = ",self.x_body_rotate)
            time.sleep(0.05)
        elif self.x_body_rotate < -degree :
            send.sendContinuousValue(0+correct[0],0+correct[1],0,-5+correct[2],0)#!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            print( "go right = ",self.x_body_rotate)
            time.sleep(0.05)
        

    def body_trace_straight(self,degree,ball_degree) :
        
        if (self.vertical - degree) - 200 > ball_degree :
            send.sendContinuousValue(1900+correct[0],0+correct[1],0,0+correct[2],0) #!!!!!!!!!!!!!!!!!!!!!!
            print( "go ahead bbbbb= ",self.vertical)
            time.sleep(0.05)
        elif (self.vertical - degree) - 200 < ball_degree and (self.vertical - degree)  > ball_degree:
            send.sendContinuousValue(1000+correct[0],0+correct[1],0,0+correct[2],0) #!!!!!!!!!!!!!!!!!!!!!!
            print( "go ahead ssssss= ",self.vertical)
            time.sleep(0.05)
        elif (self.vertical - degree) < -ball_degree :
            send.sendContinuousValue(-1000+correct[0],0+correct[1],0,0+correct[2],0)
            print( "go back = ",self.vertical)
            time.sleep(0.05)
        elif  abs(self.vertical - degree) <= ball_degree :
            #send.sendBodyAuto(0,0,0,0,1,0)
            
            motor.bodyauto_close(0)
            self.catch = False
            self.found = True
            time.sleep(1.2)
            # send.sendBodySector(1)   #1111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111
            send.sendBodySector(5)    #2222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222
            time.sleep(1.2)
            motor.move_head(1,1848,880,880,50) #1748

            time.sleep(1.5)
            target.ball_parameter()
            
            
            
    
    # def waist_revise(self):
            
    #             time.sleep(1)
    #             send.sendSingleMotor(9,round(self.x_body_rotate ),30)
    #             print("meowmeowmeoemeowmeow",self.x_body_rotate)
    #             time.sleep(0.05)

    def WaistFix(self, Target_X, Target_Y, TargetXCenter, TargeYCenter):#轉腰調整Basket.X與BasketVerticalBaseLine的誤差
        # self.MoveW = round((TargetXCenter - Target_X)*0.7)
        self.MoveW = TargetXCenter - Target_X
        self.MoveY = TargeYCenter - Target_Y
        
        self.waist_move((self.waist_position + self.MoveW), 20)
        time.sleep(0.5)
        self.move_head(2, (self.vertical + self.MoveY),880,880, 20)
        time.sleep(0.2)
                


    def body_trace_basket_straight_2(self,basket_size,basket_error) :
        
        if target.basket_size - basket_size < -basket_error and  target.basket_size <12000:
            send.sendContinuousValue(2000+correct[0],0+correct[1],0,0+correct[2],0)#!!!!!!!!!!!!!!!
            print( "--------------------go ahead to basket---------------------  ",self.vertical)
            time.sleep(0.05)

        elif target.basket_size - basket_size < -basket_error and  target.basket_size >12000:
            send.sendContinuousValue(500+correct[0],0+correct[1],0,0+correct[2],0)#!!!!!!!!!!!!!!!
            print( "--------------------go ahead to basket---------------------  ",self.vertical)
            time.sleep(0.05)    
        elif target.basket_size - basket_size > basket_error :
            send.sendContinuousValue(-800+correct[0],0+correct[1],0,0+correct[2],0)#!!!!!!!!!!!!!!!!!
            print( "--------------------go back from basket-------------------- ",self.vertical)
            time.sleep(0.05)
        elif abs(target.basket_size - basket_size) < basket_error :
            print( "--------------------stop at the basket----------------------",self.horizon - 2048)
            #send.sendBodyAuto(0,0,0,0,1,0)
            motor.bodyauto_close(0)
            target.basket_parameter()
            
            time.sleep(1)
            motor.move_head(1,1848,880,880,30)
            time.sleep(0.8)
            send.sendBodySector(3)   #11111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111`
            print("-------------------------send.sendBodySector(3)------------------------------")
            time.sleep(2)
            self.found = True
            self.catch = True

    def body_trace_basket_straight_3(self,basket_size,basket_error) :
        
        if target.basket_size - basket_size < -basket_error and target.basket_size > 2100:
            send.sendContinuousValue(300+correct[0],0+correct[1],0,0+correct[2],0)#!!!!!!!!!!!!!!!
            print( "--------------------go ahead to basket---------------------  ",self.vertical)
            time.sleep(0.05)
        elif target.basket_size - basket_size < -basket_error and target.basket_size < 2100:
            send.sendContinuousValue(1200+correct[0],0+correct[1],0,0+correct[2],0)#!!!!!!!!!!!!!!!
            print( "--------------------go ahead to basket---------------------  ",self.vertical)
            time.sleep(0.05)

        elif target.basket_size - basket_size > basket_error and target.basket_size > 2400:
            send.sendContinuousValue(-1200+correct[0],0+correct[1],0,0+correct[2],0)#!!!!!!!!!!!!!!!!!
            print( "--------------------go back from basket-------------------- ",self.vertical)
            time.sleep(0.05)   
        elif target.basket_size - basket_size > basket_error and target.basket_size < 2400:
            send.sendContinuousValue(-800+correct[0],0+correct[1],0,0+correct[2],0)#!!!!!!!!!!!!!!!!!
            print( "--------------------go back from basket-------------------- ",self.vertical)
            time.sleep(0.05)
        elif abs(target.basket_size - basket_size) < basket_error :
            print( "--------------------stop at the basket----------------------",self.horizon - 2048)
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
            print( "--------------------go ahead to basket---------------------  ",self.vertical)
            time.sleep(0.05)

        elif target.basket_size - basket_size > basket_error and target.basket_size > 1000:
            send.sendContinuousValue(-1200+correct[0],0+correct[1],0,0+correct[2],0)#!!!!!!!!!!!!!!!!!
            print( "--------------------go back from basket-------------------- ",self.vertical)
            time.sleep(0.05)   
        elif target.basket_size - basket_size > basket_error and target.basket_size < 1000:
            send.sendContinuousValue(-800+correct[0],0+correct[1],0,0+correct[2],0)#!!!!!!!!!!!!!!!!!
            print( "--------------------go back from basket-------------------- ",self.vertical)
            time.sleep(0.05)
        elif abs(target.basket_size - basket_size) < basket_error :
            print( "--------------------stop at the basket----------------------",self.horizon - 2048)
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
    

        
        
    def slam(self,score) :
        if score == 1 :
            send.sendSingleMotor(2,-100,30)
            send.sendSingleMotor(1,-950,30)
            send.sendSingleMotor(2,300,30)
            print("---------------sw==%d-------------",sw)
            print("slam dunk-------------slam dunk--------------slam dunk---------")
            time.sleep(0.5)
            score = score - 1

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
    #         if target.y_basket_place != 0:
    #             print("target.y_basket_place - 120=====",target.y_basket_place - 120)

    #             if abs(target.y_basket_place - 120 ) > 3 :

    #                 print("working------------------------------------")

    #                 time.sleep(1)
    #                 self.move_head (2, round(self.vertical + (120 - target.y_basket_place)0.7),880,880,20)
    #                 time.sleep(0.2)
    #                 target.basket_parameter()

    #             elif abs(target.y_basket_place - 120 ) <= 3  and sw == 2:
    #                 target.basket_parameter()
    #                 time.sleep(1)
    #                 self.distance = abs(sqrt(3600sixty/target.basket_size))
    #                 print("self.distance = ",self.distance)
    #             elif abs(target.y_basket_place - 120 ) <= 3  and sw == 1:
    #                 target.basket_parameter()
    #                 time.sleep(1)
    #                 self.distance = abs(sqrt(8100*ninty/target.basket_size))
    #                 print("self.distance = ",self.distance)

    def basket_distance(self,six,nine):
        print("target.y_basket_place - 120",target.y_basket_place - 120)
        self.move_head(1,2048,880,880,50)
        target.basket_parameter()
        if abs(target.y_basket_place - 120) > 3 :
            if target.y_basket_place == 0 :
                self.move_head(1,1850,880,880,50)
            elif target.y_basket_place - 120 > 0 :
                self.move_head(2,self.vertical - 1,880,880,50)
            elif target.y_basket_place - 120 < 0 :
                self.move_head(2,self.vertical + 1,880,880,50)
            target.basket_parameter()
            print("Basket Y = ",target.y_basket_place)
        
        elif abs(target.y_basket_place - 120) <= 3 and self.detect == False :
            self.sixty_distance = sqrt(abs((3600*six)/target.basket_size))
            self.ninty_distance = sqrt(abs((8100*nine)/target.basket_size))
            if ( six + nine ) / 2 > target.basket_size :
                self.distance_new = self.sixty_distance
            else :
                self.distance_new = self.ninty_distance

            self.detect = True
            print("Basket size = ",target.basket_size)
            print("Distance_60 = ",self.sixty_distance)
            print("Distance_90 = ",self.ninty_distance)
            print("Distance_fin = ",self.distance_new)


        


#     BasketInfo->HeadVerticalAngle = (double)(BasketInfo->VerticalHeadPosition - 1024) * Scale2Deg + BasketInfo->RobotStandFeedBack + BasketInfo->FeedBackError;
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

#     ROS_INFO("VerticalHeadPosition = %d", BasketInfo->VerticalHeadPosition);
#     ROS_INFO("HeadVerticalAngle = %lf", BasketInfo->HeadVerticalAngle);
#     ROS_INFO("Basket.size = %lf", BasketInfo->Basket.size);
#     ROS_INFO("Distance_60 = %lf", BasketInfo->Distance_60);
#     ROS_INFO("Distance_90 = %lf", BasketInfo->Distance_90);

#}
    
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
                time.sleep(0.5)
                send.sendBodySector(29)
                print("-------------------reset and stoping-------------------------")
                print("-------------------reset and stoping-------------------------")
                print("好棒棒")
                print("◢███◣。。。。。。◢███◣" )
                print("▇▇□██。。。。。。██□██")
                print("  ◥███◤◢████◣◥███◤")
                print("◢█。。。。。。。。。。█◣")
                print("█。╔╗。。。。。。。╔╗。█")
                print("█。∥●。。。╭╮。。。∥●。█")
                print("█。╚╝。。。╰╯。。。╚╝。█")
                print("   ◥▆▆▆▆▆▆▆▆▆▆▆▆▆▆")
                time.sleep(2)

            elif self.now_state == 1 :
                send.sendHeadMotor(1,2048,30)
                send.sendHeadMotor(2,2048,30)
                time.sleep(0.5)
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
    throw_ball_point = [2250,800,16500] #灌籃未寫
    #                   三分  五分  灌籃
    step  = ['find_ball','open_ball_trace','walk_to_ball','ball_trace','catch_ball','open_basket_trace','basket_trace','walk_to_basket','find','waisting','finish']
    #           0              1                2           3             4                   5                  6           7             8
    correct = [-400,0,-2]
    

    # correct = [0,0,0]
   
    #          x,y,theta    
    basket_error = [50,30,500]
    #尚未有sw控制 ：：throw_ball_point[sw] 3分跟5分一樣

    ball_correct = [20,100]

    trace_parameter =[60]#25
    too_big = True
    
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
                    
                    if step[jmp] == 'find_ball':
                        if target.ball_size < 70 :
                            motor.view_move(2698,1498,1800,1098,55,0.05)                 
                            time.sleep(0.05)
                            print("start to find the ball")
                            print("stop====\n",stop)
                            target.ball_parameter()  
                            print("  ball => x:",target.x_ball_place," y:",target.y_ball_place," size:",target.ball_size)
                        elif target.ball_size > 70 :
                            jmp = jmp + 1

                    elif step[jmp] == 'open_ball_trace' :
                        if abs(target.x_ball_place - 160) > 25  or abs(target.y_ball_place - 120) > 15 :
                            target.ball_parameter() 
                            print("open_ball_trace is opening")
                            motor.trace_revise(target.x_ball_place,target.y_ball_place,25)
                            time.sleep(0.05) 
                        else :
                            if motor.vertical <= 1300:
                              too_big = True
                              print("bigbigbig")
                            else:
                              too_big = False
                              print("smallsmallsmall")
                            jmp = jmp + 1
                            print("jmp======",jmp)

                    elif  step[jmp] == 'walk_to_ball' :
                        time.sleep(0.3)
                        print("-------------start walk to the ball--------------")
                        motor.bodyauto_close(1)
                        jmp = jmp + 1

                    elif  step[jmp] == 'ball_trace' :
                        
                        target.ball_parameter()   
                        print(" ball => x:",target.x_ball_place," y:",target.y_ball_place," size:",target.ball_size)                   
                        motor.trace_revise(target.x_ball_place,target.y_ball_place,25)
                        print("abs(motor.x_body_rotate)",abs(motor.x_body_rotate),motor.horizon-2048)

                        if too_big == True:
                            send.sendContinuousValue(-1200+correct[0],0+correct[1],0,0+correct[2],0)
                            print("meowmeowmeowmeowmeow")

                            if motor.vertical >=1370 :
                                too_big = False


                        
                        if too_big == False:
                            if abs(motor.horizon-2048) > trace_parameter[0]  :  
                                motor.body_trace_rotate(trace_parameter[0])    
                                print("motor.vertical=========",motor.vertical)
                            
                            elif abs(motor.vertical - 1410) > 5 : #1320是條球的距離150是誤差
                                print(motor.horizon - 2048)
                                motor.body_trace_straight(1410,ball_correct[gazebo_robot])#!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                                print("motor.vertical-1320 = ",motor.vertical-1410)
# ............................................................................................................................
# ............................................................................................................................
# ............................................................................................................................
# ............................................................................................................................

                elif  motor.found == True and motor.catch == False : 
                    target.basket_parameter() 
                    if step[jmp] == 'ball_trace' :
                        target.ball_parameter()  
                        if abs(target.x_ball_place-160) < 3 :
                                jmp = jmp + 1
                                print("jmp======",jmp)

                        elif target.x_ball_place != 0 :
                            target.ball_parameter()
                            motor.WaistFix(target.x_ball_place,target.y_ball_place,160,120)
                            print("abs(target.x_ball_place-160)hhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhh",abs(target.x_ball_place-160))
                            


                    elif step[jmp] == 'catch_ball' :   
                        
                        
                        # send.sendBodySector(2)    #1111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111
                        send.sendBodySector(6)    #2222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222
                        time.sleep(0.8)     
                        print("stop to the ball")
                        print("----------------------------------ready to waist_move---------------------------------------")
                        
                        
                        motor.waist_move(2048,50)
                        send.sendBodySector(7)    #2222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222
                        #send.sendBodySector(3)
                        time.sleep(0.8)
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
                        if abs(target.x_basket_place - 160) > 4  or abs(target.y_basket_place - 120) > 3 :
                            target.basket_parameter() 
                            motor.trace_revise(target.x_basket_place,target.y_basket_place,25)
                        else :
                                                    
                            motor.bodyauto_close(1)                         
                            jmp = jmp + 1


                    elif  step[jmp] == 'walk_to_basket' :
                        print("walk_to_basket")
                        target.basket_parameter()
                        print(" basket => x:",target.x_basket_place," y:",target.y_basket_place," size:",target.basket_size)
                        motor.trace_revise(target.x_basket_place,target.y_basket_place,25)

                        


                        if abs(motor.horizon-2048) > 25 :#!!!!
                            target.basket_parameter()
                            if sw == 0 or sw == 1:
                                motor.body_trace_rotate(30)
                                print("target.basket_size ==",target.basket_size)
                                print("throw_ball_point ==",throw_ball_point)
                                print("",abs(target.basket_size - throw_ball_point[sw]))
                            elif sw == 2 and target.basket_size < 12000:
                                motor.body_trace_rotate(50)
                                print("target.basket_size ==",target.basket_size)
                                print("throw_ball_point ==",throw_ball_point)
                                print("",abs(target.basket_size - throw_ball_point[sw]))
                            elif sw == 2 and target.basket_size >= 12000:
                                motor.body_trace_rotate(100)
                                print("target.basket_size ==",target.basket_size)
                                print("throw_ball_point ==",throw_ball_point)
                                print("",abs(target.basket_size - throw_ball_point[sw]))

                        elif abs(target.basket_size - throw_ball_point[sw]) > 10 :#!!!!!!!!!!!!!!!
                            if sw == 2:
                                print("----------------start the slam slam slam --------------")
                                motor.body_trace_basket_straight_2(throw_ball_point[sw],basket_error[sw])
                                print("motor.catch = ",motor.catch)
                                if motor.catch == True :
                                    print("jmp,step[jmp] is ",jmp,step[jmp])

                            elif sw == 0:
                                print("----------------start the action of get point--------------")
                                motor.body_trace_basket_straight_3(throw_ball_point[sw],basket_error[sw])
                                print("motor.catch = ",motor.catch)
                                if motor.catch == True :
                                    print("jmp,step[jmp] is ",jmp,step[jmp])

                            elif sw == 1:
                                print("----------------start the action of get point--------------")
                                motor.body_trace_basket_straight_5(throw_ball_point[sw],basket_error[sw])
                                print("motor.catch = ",motor.catch)
                                if motor.catch == True :
                                    print("jmp,step[jmp] is ",jmp,step[jmp])

                
                
                
                elif  motor.found == True and motor.catch == True :
                    if sw == 2 :
                        if step[jmp] == 'walk_to_basket' :
                            target.basket_parameter()
                            if target.x_basket_place != 0 :
                                motor.WaistFix(target.x_basket_place,target.y_basket_place,160,120)
                                print("abs(target.x_basket_place-160)",abs(target.x_basket_place-160))
                                if abs(target.x_basket_place-160) < 3:
                                    jmp = jmp + 1
                                    print("jmp======",jmp)

                        elif step[jmp] == 'find' :
                            time.sleep(1)
                            send.sendBodySector(4)#1111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111
                            print("2222222222")
                            # if  abs(motor.horizon - 2048) <= 5  and abs(target.x_basket_place - 160) <= 1 :
                            #     print("---------------gogogogogogogogogogogogogogoro-------------")
                            jmp = jmp + 1 #都一直線


                    elif sw == 0:
                        if step[jmp] == 'walk_to_basket' :
                            target.basket_parameter()
                            if target.x_basket_place != 0 :
                                motor.WaistFix(target.x_basket_place,target.y_basket_place,160,120)
                                print("abs(target.x_basket_place-160)",abs(target.x_basket_place-160))
                                if abs(target.x_basket_place-160) < 3:
                                    jmp = jmp + 1
                                    print("jmp======",jmp)

                        elif step[jmp] == 'find' :
                            time.sleep(0.5)
                            motor.basket_distance(2250,800)
                            print("3333333333")
                            
                            jmp = jmp + 1 #都一直線
                    

                    elif sw == 1:
                        if step[jmp] == 'walk_to_basket' :
                            target.basket_parameter()
                            if target.x_basket_place != 0 :
                                motor.WaistFix(target.x_basket_place,target.y_basket_place,160,120)
                                print("abs(target.x_basket_place-160)",abs(target.x_basket_place-160))
                                if abs(target.x_basket_place-160) < 3:
                                    jmp = jmp + 1
                                    print("jmp======",jmp)

                        elif step[jmp] == 'find' :
                            time.sleep(0.5)
                            motor.basket_distance(2250,800)
                            print("555555")
                            
                            jmp = jmp + 1 #都一直線
                

                
                    



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