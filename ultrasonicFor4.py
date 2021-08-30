# Code for 4 Ultrasonic Sensors



#Libraries
import RPi.GPIO as GPIO
import time
 
#GPIO Mode (BOARD / BCM)
GPIO.setmode(GPIO.BCM)
 
#set GPIO Pins
  #for Front Ultrasound
GPIO_TRIGGER_FRONT = 18
GPIO_ECHO_FRONT = 24
  #for Left Ultrasound
GPIO_TRIGGER_LEFT = 18
GPIO_ECHO_LEFT = 25
  #for Right Ultrasound
GPIO_TRIGGER_RIGHT = 18
GPIO_ECHO_RIGHT = 22
  #for Back Ultrasound
GPIO_TRIGGER_BACK = 18
GPIO_ECHO_BACK = 27

#set GPIO direction (IN / OUT)
GPIO.setup(GPIO_TRIGGER_FRONT, GPIO.OUT)
GPIO.setup(GPIO_ECHO_FRONT, GPIO.IN)

GPIO.setup(GPIO_TRIGGER_LEFT, GPIO.OUT)
GPIO.setup(GPIO_ECHO_LEFT, GPIO.IN)

GPIO.setup(GPIO_TRIGGER_RIGHT, GPIO.OUT)
GPIO.setup(GPIO_ECHO_RIGHT, GPIO.IN)

GPIO.setup(GPIO_TRIGGER_BACK, GPIO.OUT)
GPIO.setup(GPIO_ECHO_BACK, GPIO.IN)


def distance(GPIO_TRIGGER,GPIO_ECHO):
    # set Trigger to HIGH
    GPIO.output(GPIO_TRIGGER, True)
 
    # set Trigger after 0.01ms to LOW
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER, False)
 
    StartTime = time.time()
    StopTime = time.time()
 
    # save StartTime
    while GPIO.input(GPIO_ECHO) == 0:
        StartTime = time.time()
 
    # save time of arrival
    while GPIO.input(GPIO_ECHO) == 1:
        StopTime = time.time()
 
    # time difference between start and arrival
    TimeElapsed = StopTime - StartTime
    # multiply with the sonic speed (34300 cm/s)
    # and divide by 2, because there and back
    distance = (TimeElapsed * 34300) / 2
 
    return distance

def distanceTrigger(distance)
    # returns true if the distnce is less than 5
    # false otherwise  
    return distance<=5



if __name__ == '__main__':
    try:
        while True:
            dist_front = distance(GPIO_TRIGGER_FRONT,GPIO_ECHO_FRONT)
            dist_back = distance(GPIO_TRIGGER_BACK,GPIO_ECHO_BACK)
            dist_left = distance(GPIO_TRIGGER_LEFT,GPIO_ECHO_LEFT)
            dist_right = distance(GPIO_TRIGGER_RIGHT,GPIO_ECHO_RIGHT)
            
            
            # wall defense method
            if(distanceTrigger(dist_front)): # because our robot will move predominantly in the front direction
                # step 1 stop motor 
                robo_stop_flag = 1
                #code to turn robot in the desiered direction 
                if(distanceTrigger(dist_right) and !distanceTrigger(dist_left)) # both sides are free to move
                    # choose a predominat direction or 
                    # my suggestion: choose the direction with greater distance to the wall 
                    
                
                elif(distanceTrigger(dist_right) and !distanceTrigger(dist_left)): #if right side is blocked 
                    #turn 90 deg to the left
                elif(distanceTrigger(dist_left) and !distanceTrigger(dist_right)): # if left side is blocked
                    #turn 90 deg to the right
                elif(distanceTrigger(dist_left) and distanceTrigger(dist_right) and !distanceTrigger(dist_back)): #if both sides are blocked  
                    # turn 180 deg 
                elif(distanceTrigger(dist_left) and distanceTrigger(dist_right) and distanceTrigger(dist_back)): # all directions blocked
                    # wait for it to get clear maybe the other robot is close by
                    time.sleep(5)
                    while(distanceTrigger(dist_left) and distanceTrigger(dist_right) and distanceTrigger(dist_back))
                        #rotate 10 deg 
                        dist_back = distance(GPIO_TRIGGER_BACK,GPIO_ECHO_BACK)
                        dist_left = distance(GPIO_TRIGGER_LEFT,GPIO_ECHO_LEFT)
                        dist_right = distance(GPIO_TRIGGER_RIGHT,GPIO_ECHO_RIGHT)
                    
            print ("Measured Distance = %.1f cm" % dist)
            time.sleep(1)
 
        # Reset by pressing CTRL + C
    except KeyboardInterrupt:
        print("Measurement stopped by User")
        GPIO.cleanup()


