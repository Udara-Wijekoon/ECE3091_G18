#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Aug 23 13:11:53 2021
@author: nikkihobman
"""

import gpiozero
import time
import numpy as np
from matplotlib import pyplot as plt


import ultrasonicFor4new
from ultrasonicFor4new import distance, distanceTrigger, GPIO_TRIGGER_FRONT, GPIO_TRIGGER_BACK
from ultrasonicFor4new import GPIO_TRIGGER_LEFT, GPIO_TRIGGER_RIGHT, GPIO_ECHO_FRONT, GPIO_ECHO_BACK, GPIO_ECHO_LEFT, GPIO_ECHO_RIGHT
from gpiozero import RotaryEncoder

#Shaft , Motor Control and Robot Navigation

#STEPS
# 1) SET UP SHAFT ENCODER DONE
#i) Set Microcontroller pins in code
#ii)  Write PWM to shaft encoder DONE
#iii)  Get speed from shaft encoder DONE


#2) Write Fucntions that take input arguments relative position
#3) Distance and Angle
#4) Calculate velocity of each tier to make turn
#5) rotate motor using PWM Signal



#INPUTS TO MOTOR CONTROLLER
#-PWM1 (Speed1) PIN 11
#-PWM2 (Speed2) PIN 15
#-DIR1 (Direction1) PIN 11
#-DIR2 (Direction 2) PIN 16

#OUTPUTS FROM SHAFT ENCODER: 
#A1
#A2
#B1
#B2
#C1 and C2 connects to commom ground

global DT
 
#Creating classes for PWM control and DIR Control
pwm1 = gpiozero.PWMOutputDevice(pin=12,active_high=True,initial_value=0,frequency=1000) #creates a class of PWM output
pwm2 = gpiozero.PWMOutputDevice(pin=13,active_high=True,initial_value=0,frequency=1000) #creates a class of PWM output


#Might need to change theese pin assignments
#in1 and in2 control the directions for the wheels
ain1= gpiozero.OutputDevice(pin=17) 
ain2= gpiozero.OutputDevice(pin=23) 
bin1= gpiozero.OutputDevice(pin=11) #Might change this pin
bin2= gpiozero.OutputDevice(pin=8) #Might change this pin

stby = gpiozero.OutputDevice(pin=10) #this signal was not changed in the example code/ Might change this pin
stby.value=1


encoder1 = gpiozero.RotaryEncoder(a=5, b=6,max_steps=100000)#cretates a class for a rotaty encoder
encoder2 = gpiozero.RotaryEncoder(a=26, b=16,max_steps=100000)
#TO DO! find apropriate max steps and figure out what to do with overflow
# Maybe: Refresh position and encoder values after we reach each goal. 

#We are using a 32-bit encoder (32 steps / revolution)

#These devices typically have three pins labelled “A”, “B”, and “C”. 
#Connect A and B directly to two GPIO pins, and C (“common”) to one of the ground pins on your Pi.
# Then simply specify the A and B pins as the arguments when constructing this classs


encoder1.steps = 0
encoder2.steps = 0


#Classes: ---------
class DiffDriveRobot: #estimates the absolute position of the robot
    
    def __init__(self,inertia=5, drag=0.2, wheel_radius=0.05, wheel_sep=0.15):
        
        self.x = 0.0 # y-position
        self.y = 0.0 # y-position 
        self.th = 0.0 # orientation
        
        self.wl = 0.0 #rotational velocity left wheel this updates during pose update
        self.wr = 0.0 #rotational velocity right wheel
        
        self.I = inertia
        self.d = drag
     
        
        self.r = wheel_radius
        self.l = wheel_sep
        
        self.pre_steps1 = 0
        self.pre_steps2 = 0
        self.s = 0
        self.e = 0.1
    
    
    # Veclocity motion model
    def base_velocity(self,wl,wr):
        
        v = (wl*self.r + wr*self.r)/2.0
        
        w = (wl - wr)/self.l
        
        return v, w
    
    # Kinematic motion model
    def pose_update(self):
        
        
        self.e = time.time()
        Dt = self.s - self.e  
        self.wl = 1.175*(encoder1.steps-self.pre_steps1)/(np.pi*32*Dt) #measured speed cm/s - shaft encoder is 32 bits
        self.wr = 1.175*(encoder2.steps-self.pre_steps2)/(np.pi*32*Dt)
        self.s = time.time()
        self.pre_steps1 = encoder1.steps
        self.pre_steps2 = encoder2.steps
        print('w:', self.wr, self.wl)
        print("Dt", Dt)
        
        
        vp, wp = self.base_velocity(self.wl,self.wr)#returns base linear and angular velocity
        
        self.x = self.x + Dt*vp*np.cos(self.th)
        self.y = self.y + Dt*vp*np.sin(self.th)
        self.th = self.th + wp*Dt
        
        return self.x, self.y, self.th #estimate current position
    
    

class TentaclePlanner: #plans where the robot is going finds the quickest path avoiding obstacles
    
    def __init__(self,obstacles,dt_test=0.05,steps=5,alpha=1,beta=0.1, turn = 0):
        
        self.dt = dt_test #use fake dt to plan next step
        self.steps = steps
        # Tentacles are possible trajectories to follow
        self.tentacles = [(0.1,1.0),(0.1,-1.0),(0.1,0.0), (0.0, -1.0), (0.0, 1.0)]#right left front back
        self.turn = turn
        
        for i,  (v, w) in enumerate(self.tentacles):
            self.tentacles[i] = 5*v, 5*w
        
        self.alpha = alpha
        self.beta = beta
        
        self.obstacles = obstacles
    
    # Play a trajectory and evaluate where you'd end up
    def roll_out(self,v,w,goal_x,goal_y,goal_th,x,y,th):
        
        for j in range(self.steps):
        
            x = x + self.dt*v*np.cos(th)
            y = y + self.dt*v*np.sin(th)
            th = (th + w*self.dt)
            
        
        # Wrap angle error -pi,pi
        e_th = goal_th-th
        e_th = np.arctan2(np.sin(e_th),np.cos(e_th))
        
        cost = self.alpha*((goal_x-x)**2 + (goal_y-y)**2) + self.beta*(e_th**2)
        
        return cost
        
    
    # Choose trajectory that will get you closest to the goal
    def plan(self,goal_x,goal_y,goal_th,x,y,th):
        
        costs =[]
        
        for v,w in self.tentacles:    
        #check for colision: Ignore tentacles that have object detected on sensor
            
            cost = self.roll_out(v,w,goal_x,goal_y,goal_th,x,y,th)
            
            #print('cost', cost)
            #print("obstacles:", self.obstacles)
            
            if (v > 0 and w == 0 and self.obstacles[0]): #front: can only turn left or right
               cost = np.inf
            if (w > 0 and self.obstacles[1]): #left
               cost = np.inf
            if (w < -0 and self.obstacles[2]): #right
               cost = np.inf
            if (v > 0 and w == 0 and self.obstacles[0]): #front
               cost = np.inf
            if (turn and v>0): #only allow for on the spot rotations
               cost = np.inf
                                        
    
            costs.append(cost)
            
        print(costs)
        best_idx = np.argmin(costs)
        
        #print("costs:", costs)
        #print("best route:", self.tentacles[best_idx])
        
        return self.tentacles[best_idx]
    


class RobotController: #calculates the required duty cycles to get wheels 
#turning at the required speed to go in the required direction
    
    def __init__(self,Kp=0.1,Ki=0.01,wheel_radius=0.02, wheel_sep=0.1):
        
        self.Kp = Kp
        self.Ki = Ki
        self.r = wheel_radius
        self.l = wheel_sep
        self.e_sum_l = 0
        self.e_sum_r = 0
        
    def p_control(self,w_desired,w_measured,e_sum):
        
        duty_cycle = min(max(0,np.abs(self.Kp*(w_desired-w_measured) + self.Ki*e_sum)),0.30)
        
        in1 = 1
        in2 = 0
        
        if w_desired < 0:
            
            in1 = 0
            in2 = 1
                                      
        
        e_sum = e_sum + (w_desired-w_measured)
        
       # print("w_desired", w_desired)
       # print("w_measured", w_measured)
        
        return duty_cycle, e_sum, in1, in2 #now returns wheel directional control
        
        
    def drive(self,v_desired,w_desired,wl,wr):
        
        wl_desired = v_desired + self.l*w_desired/2 
        wr_desired = v_desired - self.l*w_desired/2
         
        duty_cycle_l,self.e_sum_l, ain1, ain2 = self.p_control(wl_desired,wl,self.e_sum_l)
        duty_cycle_r,self.e_sum_r, bin1, bin2 = self.p_control(wr_desired,wr,self.e_sum_r)
        
        return duty_cycle_l, duty_cycle_r, ain1, ain2, bin1, bin2  #speed and direction control




#Mini Function Made By Audry
def distanceTF(GPIO_TRIGGER,GPIO_ECHO):
    
    a = distance(GPIO_TRIGGER,GPIO_ECHO)         #calculating distance
    b = distanceTrigger(a) 
                                                    #return true or false
    return b


        

    
    
#INITIAL TEST: ROTATE MOTOR USING PWM SIGNALS----------------------
#The code below turns on a pwm control, toggles the direction of the pwm drive,
# and reads from a rotary encoder. Parameters aren't tuned or properly set-up at all.
#to do! - Assign theese to correct pins


# Step through duty cycle values, slowly increasing the speed and changing the direction of motion



#Initialise direction as forward
ain1.value = 1; 
ain2.value = 0; 
bin1.value = 1; 
bin2.value = 0; 


pre_steps1 = encoder1.steps
pre_steps2 = encoder2.steps


# for j in range(5):
#     pwm1.value = j/10
#     pwm2.value = j/10
#     ain1.value = not ain1.value
#     ain2.value = not ain2.value
#     bin1.value = not bin1.value
#     bin2.value = not bin2.value
#     print('Duty cycle:',pwm1.value,'Direction:',ain1.value)
#     print('Duty cycle:',pwm2.value,'Direction:',bin2.value)
#     time.sleep(1.0) #*********
#     print('Counter:',encoder1.steps,'Speed:',(encoder1.steps-pre_steps1)/(32*5.0),'revs per second\n')
#     print('Counter:',encoder2.steps,'Speed:',(encoder2.steps-pre_steps2)/(32*5.0),'revs per second\n')
#     pre_steps1 = encoder1.steps
#     pre_steps2 = encoder2.steps
    
#NB, if steps keeps increasing, what about integer overflows?


    

#TEST 2: MOVE TOWARDS SPECIFIED GOAL----------------
#TO DO - PUT IN CORRECT ROBOT PARAMETERS

def move(goal_x, goal_y, goal_th, turn): 

 obstacles = [False,False,False,False] #Front, Left, Right, Back

 robot = DiffDriveRobot(inertia=5, drag=1, wheel_radius=2.65, wheel_sep=4.05) #INITIALISE ROBOT AND PARAMETERS
 controller = RobotController(Kp=1,Ki=0.25,wheel_radius=2.65,wheel_sep=4.05)
 planner = TentaclePlanner(obstacles = obstacles,steps=5,alpha=1,beta=0.01, turn = turn)


 #CODE TO CONTROL ROBOT MOTION BASED ON GOAL POSITION X, Y and THETA
 goal_x = 30
 goal_y = -30
 goal_th = 0

 print(goal_x, goal_y, goal_th)

 #Initialise direction as forward again. (prelim wheels dont change spin direction)
 ain1.value = 1
 ain2.value = 0
 bin1.value = 1 
 bin2.value = 0 

 pwm1.value = 0
 pwm2.value = 0
 time.sleep(2)#sleep for 2s

 pre_steps1 = 0
 pre_steps2 = 0
 encoder1.steps = 0
 encoder2.steps = 0

 print("start", encoder1.steps, encoder2.steps)

 x_logger = []
 y_logger = []

 for i in range(300): #goes to goal in 300 steps or less 1 step == 1 directional change ~0.1s
     print("")
     print("start")
     #print("time:", 0.1*i)

     costs = []

     #TO DO! check for obstacles and update detected obstacles
     Boolfront = distanceTF(GPIO_TRIGGER_FRONT,GPIO_ECHO_FRONT)
     Boolleft = distanceTF(GPIO_TRIGGER_LEFT,GPIO_ECHO_LEFT)
     Boolright = distanceTF(GPIO_TRIGGER_RIGHT,GPIO_ECHO_RIGHT)
     #Boolback = distanceTF(GPIO_TRIGGER_BACK,GPIO_ECHO_BACK)
     Boolback = False
     #--------------checking for obstacles

     obstacles = [Boolfront, Boolleft, Boolright, Boolback] #updating current obstacles
     planner.obstacles = obstacles

     #print("")
     print(obstacles)


     v,w = planner.plan(goal_x,goal_y,goal_th,robot.x,robot.y,robot.th) #Calculates required direction to get to goal v, w 

     end = time.time()

     if i>0: #update robot position after first iteration
         DT = end - start
         print("DT", DT)
         robot.x,robot.y,robot.th = robot.pose_update() #simulate robot movement, update position, no arguments as reads off encoders.
         x_logger.append(robot.x)
         y_logger.append(robot.y)
         print("Position")
         print([robot.x,robot.y,robot.th])
         #print("PWM:", pwm1.value, pwm2.value)

     #Calculates send velocities to pwm 
     pwm1.value, pwm2.value, ain1.value, ain2.value, bin1.value, bin2.value = controller.drive(v,w,robot.wl,robot.wr) 
     start = time.time() #START TIMER
     time.sleep(0.01)#move for 0.01 before calculating next route

     #ADD SOME MORE CODE TO MAKE THIS PART MORE ROBUST

     if(obstacles[0]):#force the robot to turn left 90 degrees if there is an obstacle then move forward 5cm
         robot.x,robot.y,robot.th = robot.pose_update()
         pwm1.value = 0
         pwm2.value = 0
         time.sleep(1)
         print("OBSTACLE")
         rad = 1.57
         if(obstacles[2]):rad = -rad
         stx = robot.x + 20*np.cos(rad)
         sty = robot.y + 20*np.sin(rad)
         while(np.sqrt((stx - robot.x)**2 + (sty- robot.y)**2)>2):
             print("IN LOOP")
             obstacles[0] = False
             v, w = planner.plan(stx,sty,0,robot.x,robot.y,robot.th)
             print("avoidance:",v, w)
             pwm1.value, pwm2.value, ain1.value, ain2.value, bin1.value, bin2.value = controller.drive(v,w,robot.wl,robot.wr)
             time.sleep(0.01)
             robot.x,robot.y,robot.th = robot.pose_update()
             x_logger.append(robot.x)
             y_logger.append(robot.y)              

     how_far = np.sqrt((goal_x-robot.x)**2 + (goal_y - robot.y)**2)  #returns how far off the final goal we are
     print("how_far", how_far)
     
     angle = np.abs(goal_th - robot.th) #how far off are we from our desired angle
     print("angle goal", angle)

     if ((how_far < 1) and turn ==0):
         print("reached goal")
         break #stop moving when we are close enough to our goal
        
     if (turn and (angle < 0.1) ): 
         print("reached angle") 
         break
       


 pwm1.value, pwm2.value = 0,0 #stop robot after 300 steps (30s) or if we reach out final goal

#plt.plot(y_logger, x_logger)
# plt.grid()
# plt.show()
#TO DO! find a way to calibrate robot posision using computer vision of US avoid error building up

move(30,0,0,0)


    
    
    
    
    
    


