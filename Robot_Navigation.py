#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Aug 23 13:11:53 2021

@author: nikkihobman
"""

import gpiozero
import time
import numpy as np
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


#Creating classes for PWM control and DIR Control
pwm1 = gpiozero.PWMOutputDevice(pin=13,active_high=True,initial_value=0,frequency=50000) #creates a class of PWM output
direction1= gpiozero.OutputDevice(pin=11)
pwm2 = gpiozero.PWMOutputDevice(pin=15,active_high=True,initial_value=0,frequency=50000) #creates a class of PWM output
direction2= gpiozero.OutputDevice(pin=16) 

encoder1 = gpiozero.RotaryEncoder(a=5, b=6,max_steps=100000)#cretates a class for a rotaty encoder
encoder2 = gpiozero.RotaryEncoder(a=5, b=6,max_steps=100000) 
#These devices typically have three pins labelled “A”, “B”, and “C”. 
#Connect A and B directly to two GPIO pins, and C (“common”) to one of the ground pins on your Pi.
# Then simply specify the A and B pins as the arguments when constructing this classs

direction1.value = 0; 
direction2.value = 0; 
pre_steps1 = encoder1.steps
pre_steps2 = encoder2.steps



#Classes: ---------
class DiffDriveRobot:
    
    def __init__(self,inertia=5, dt=0.1, drag=0.2, wheel_radius=0.05, wheel_sep=0.15):
        #TO DO ! ADJUST THEESE PARAMETRTS!!!!
        
        self.x = 0.0 # y-position
        self.y = 0.0 # y-position 
        self.th = 0.0 # orientation
        
        self.wl = (encoder1.steps-pre_steps1)/5.0 #absolute rotational velocity left wheel
        self.wr = (encoder1.steps-pre_steps2)/5.0 #absolute rotational velocity right wheel
        
        self.I = inertia
        self.d = drag
        self.dt = dt #TODO! put in code to find dt using actural time
        
        self.r = wheel_radius
        self.l = wheel_sep #wheel seperation
    
    # Should be replaced by motor encoder measurement which measures how fast wheel is turning
    # Here, we simulate the real system and measurement
    def motor(self,w,duty_cycle):
        
        torque = self.I*duty_cycle
        
        if (w > 0):
            w = min(w + self.dt*(torque - self.d*w),3)
        elif (w < 0):
            w = max(w + self.dt*(torque - self.d*w),-3)
        else:
            w = w + self.dt*(torque)
        
        return w
    
    # Veclocity motion model
    def base_velocity(self,wl,wr):
        
        v = (wl*self.r + wr*self.r)/2.0
        
        w = (wl - wr)/self.l
        
        return v, w
    
    # Kinematic motion model
    def pose_update(self,duty_cycle_l,duty_cycle_r):
        
        self.wl = self.motor(self.wl,duty_cycle_l)
        self.wr = self.motor(self.wr,duty_cycle_r)
        
        v, w = self.base_velocity(self.wl,self.wr)
        
        self.x = self.x + self.dt*v*np.cos(self.th)
        self.y = self.y + self.dt*v*np.sin(self.th)
        self.th = self.th + w*self.dt
        
        return self.x, self.y, self.th
    
    
class RobotController: #----------------
    
    def __init__(self,Kp=0.1,Ki=0.01,wheel_radius=0.02, wheel_sep=0.1):
        
        self.Kp = Kp
        self.Ki = Ki
        self.r = wheel_radius
        self.l = wheel_sep
        self.e_sum_l = 0
        self.e_sum_r = 0
        
    def p_control(self,w_desired,w_measured,e_sum): #PWM CONTROL USING FEEDBACK
        
        duty_cycle = min(max(-1,self.Kp*(w_desired-w_measured) + self.Ki*e_sum),1)
        
        e_sum = e_sum + (w_desired-w_measured)
        
        return duty_cycle, e_sum
        
        
    def drive(self,v_desired,w_desired,wl,wr):
        
        wl_desired = v_desired/self.r + self.l*w_desired/2 
        wr_desired = v_desired/self.r - self.l*w_desired/2
        
        duty_cycle_l,self.e_sum_l = self.p_control(wl_desired,wl,self.e_sum_l)
        duty_cycle_r,self.e_sum_r = self.p_control(wr_desired,wr,self.e_sum_r)
        
        return duty_cycle_l, duty_cycle_r  
    
    

class TentaclePlanner: #Allowes robot to move towards goal and avoids obstacles
    
    def __init__(self,obstacles,dt=0.1,steps=5,alpha=1,beta=0.1):
        
        self.dt = dt 
        self.steps = steps
        # Tentacles are possible trajectories to follow
        self.tentacles = [(0.0,1.0),(0.0,-1.0),(0.1,1.0),(0.1,-1.0),(0.1,0.5),(0.1,-0.5),(0.1,0.0),(0.0,0.0)]
        
        self.alpha = alpha
        self.beta = beta
        
        self.obstacles = obstacles
    
    # Play a trajectory and evaluate where you'd end up
    def roll_out(self,v,w,goal_x,goal_y,goal_th,x,y,th):
        
        for j in range(self.steps):
        
            x = x + self.dt*v*np.cos(th)
            y = y + self.dt*v*np.sin(th)
            th = (th + w*self.dt)
            
            if (self.check_collision(x,y)):
                return np.inf
        
        # Wrap angle error -pi,pi
        e_th = goal_th-th
        e_th = np.arctan2(np.sin(e_th),np.cos(e_th))
        
        cost = self.alpha*((goal_x-x)**2 + (goal_y-y)**2) + self.beta*(e_th**2)
        
        return cost
    
    def check_collision(self,x,y):
        
        min_dist = np.min(np.sqrt((x-self.obstacles[:,0])**2+(y-self.obstacles[:,1])**2))
        
        if (min_dist < 0.1):
            return True
        return False
        
    
    # Choose trajectory that will get you closest to the goal
    def plan(self,goal_x,goal_y,goal_th,x,y,th):
        
        costs =[]
        for v,w in self.tentacles:
            costs.append(self.roll_out(v,w,goal_x,goal_y,goal_th,x,y,th))
        
        best_idx = np.argmin(costs)
        
        return self.tentacles[best_idx]
    


#Function that sends PWM to robot
def move(duty1, duty2):
        pwm1.value = duty1
        pwm2.value = duty2    









#TINITIAL TEST: ROTATE MOTOR USING PWM SIGNALS----------------------
#The code below turns on a pwm control, toggles the direction of the pwm drive,
# and reads from a rotary encoder. Parameters aren't tuned or properly set-up at all.
#to do! - Assign theese to correct pins


# Step through duty cycle values, slowly increasing the speed and changing the direction of motion
pre_steps1 = 0
pre_steps2 = 0

for j in range(10):
    pwm1.value = j/10
    pwm2.value = j/10
    direction1.value = not direction1.value
    direction2.value = not direction2.value
    print('Duty cycle:',pwm1.value,'Direction:',direction1.value)
    print('Duty cycle:',pwm1.value,'Direction:',direction1.value)
    time.sleep(5.0)
    print('Counter:',encoder1.steps,'Speed:',(encoder1.steps-pre_steps1)/5.0,'steps per second\n')
    print('Counter:',encoder2.steps,'Speed:',(encoder2.steps-pre_steps2)/5.0,'steps per second\n')
    pre_steps1 = encoder1.steps
    pre_steps2 = encoder2.steps
    
    #NB, if steps keeps increasing, what about integer overflows?
    
    

#TEST 2: MOVE TOWARDS SPECIFIED GOAL----------------
#TO DO - PUT IN CORRECT ROBOT PARAMETERS
obstacles = 0 #get obstacle data from US
robot = DiffDriveRobot(inertia=5, dt=0.1, drag=1, wheel_radius=0.05, wheel_sep=0.15) #INITIALISE ROBOT AND PARAMETERS
controller = RobotController(Kp=1,Ki=0.25,wheel_radius=0.05,wheel_sep=0.15)
planner = TentaclePlanner(dt=0.1,steps=5,alpha=1,beta=1e-5)

#CODE TO CONTROL ROBOT MOTION BASED ON GOAL POSITION X, Y and THETA
goal_x = 2*np.random.rand()-1
goal_y = 2*np.random.rand()-1
goal_th = 2*np.pi*np.random.rand()-np.pi


#DO DO: Put this in a while loop to keep executing untill it reaches its goal
v,w = planner.plan(goal_x,goal_y,goal_th,robot.x,robot.y,robot.th) #Calculates required velocies to get to goal
duty_cycle_l,duty_cycle_r = controller.drive(v,w,robot.wl,robot.wr) #Calculates required duty cyle to get to get velocites
x,y,th = robot.pose_update(duty_cycle_l,duty_cycle_r) #simulate robot movement
move(duty_cycle_l, duty_cycle_r) #send duty cycles to robot
move(0,0) #stop robot



    
    
    
    
    
    
    
    

