import ultrasonicFor4 as US
import Computer_Vision as CV
import Motor_driver as MD
import numpy as np
import math

distance_front = []
distance_left = []
distance_right = []
distance_back = []
spd = 50  #speed
goal = [30,30]
initial = [0,0]






while True:
  
    dist_front = distance(GPIO_TRIGGER_FRONT,GPIO_ECHO_FRONT)
    dist_back = distance(GPIO_TRIGGER_BACK,GPIO_ECHO_BACK)
    dist_left = distance(GPIO_TRIGGER_LEFT,GPIO_ECHO_LEFT)
    dist_right = distance(GPIO_TRIGGER_RIGHT,GPIO_ECHO_RIGHT)
    distance_front.append(f)
    distance_left.append(l)
    distance_right.append(r)
    distance_back.append(b)
    
    # wall defense method
    if(distanceTrigger(dist_front)): # because our robot will move predominantly in the front direction
        stop()
        theta = math.atan(goal[1]/goal[0])
        # step 1 stop motor 
        robo_stop_flag = 1
        #code to turn robot in the desiered direction 
        if(!distanceTrigger(dist_left) && !distanceTrigger(dist_right)):# both is free to move
            # choose a predominat direction or 
            # my suggestion: choose the direction with greater distance to the wall 
            if (dist_left > dist_right):
                left(spd)
            else:
                right(spd)

        elif(distanceTrigger(dist_right) && !distanceTrigger(dist_left)): #if right side is blocked 
            left(spd)
        elif(distanceTrigger(dist_left) && !distanceTrigger(dist_right)): # if left side is blocked
            right(spd)
        elif(distanceTrigger(dist_left) && distanceTrigger(dist_right) and !distanceTrigger(dist_back)): #if both sides are blocked  
            # turn 180 deg 
            right(spd)
            right(spd)
        elif(distanceTrigger(dist_left) && distanceTrigger(dist_right) && distanceTrigger(dist_back)): # all directions blocked
            # wait for it to get clear maybe the other robot is close by
            time.sleep(5)
    
    else:
       forward(spd)
    print ("Measured Distance = %.1f cm", % dist)
    time.sleep(0.5)
    
    # Reset by pressing CTRL + C
    except KeyboardInterrupt:
        print("Measurement stopped by User")
        GPIO.cleanup()
        
        
