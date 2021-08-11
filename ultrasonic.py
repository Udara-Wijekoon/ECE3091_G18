# This File include....

# this file at the time of coding assumes all electricals are done accordingly.
# therefore preplanned GPIO pins are used to check input and outputs 
# getting the signal from the 4 ultrasonic sensors
# processing the distance they measured and setting up the design expectations(6 cm?)
# the output is directed to main.py to control the wheels
# distace will be measured every second 

#Libraries
import RPI.GPIO as GPIO
import time

#GPIO Mode
GPIO.setmode(GPIO.BCM)


def distance
    #start sending a signal to trigger pin
    GPIO.output(GPIO_TRIGGER, True)
 
    # stop that signal in 0.00001 seconds
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER, False)
 
    StartTime = time.time()
    StopTime = time.time()
 
    #below code measures the time taken for the echo to get back
    # save StartTime
    while GPIO.input(GPIO_ECHO) == 0:
        StartTime = time.time()
 
    # save time on arrival
    while GPIO.input(GPIO_ECHO) == 1:
        StopTime = time.time()
 
    # time difference between start and arrival
    TimeElapsed = StopTime - StartTime
    # multiply with the sonic speed (34300 cm/s)
    # and divide by 2, because there and back
    distancetowall = (TimeElapsed * 34300) / 2
 
    return distancetowall

if __name__ == '__main__':
    while True:

        #****SENSOR 1
        #VCC to pin 2(VCC)
        #GND to pin 6(GND)
        #TRIP to pin 12 (GPIO 18)
        #ECHO to pin 6 (GND) and pin 18 (GPIO 24)

        #set GPIO pins
        GPIO_TRIG_1 = 18
        GPIO_ECHO_1 = 24

        #setting GPIO direction (IN/OUT)
        GPIO.setup(GPIO_TRIG_1, GPIO.OUT)
        GPIO.setup(GPIO_ECHO_1, GPIO.IN)
        dist = distance()
        print ("Measured Distance for sensor 1 = %.1f cm" % dist)
        

        #****SENSOR 2
        #VCC to pin 2(VCC)
        #GND to pin 6(GND)
        #TRIP to pin ?? (GPIO 18)
        #ECHO to pin 6 (GND) and pin ??? (GPIO 24)

        #set GPIO pins
        GPIO_TRIG_2 = ??
        GPIO_ECHO_2 = ??

        #setting GPIO direction (IN/OUT)
        GPIO.setup(GPIO_TRIG_2, GPIO.OUT)
        GPIO.setup(GPIO_ECHO_2, GPIO.IN)
        dist = distance()
        print ("Measured Distance for sensor 2 = %.1f cm" % dist)
       

        #****SENSOR 3
        #VCC to pin 2(VCC)
        #GND to pin 6(GND)
        #TRIP to pin ?? (GPIO 18)
        #ECHO to pin 6 (GND) and pin ??? (GPIO 24)

        #set GPIO pins
        GPIO_TRIG_3 = ??
        GPIO_ECHO_3 = ??

        #setting GPIO direction (IN/OUT)
        GPIO.setup(GPIO_TRIG_3, GPIO.OUT)
        GPIO.setup(GPIO_ECHO_3, GPIO.IN)
        dist = distance()
        print ("Measured Distance for sensor 3 = %.1f cm" % dist)
       

        #****SENSOR 3
        #VCC to pin 2(VCC)
        #GND to pin 6(GND)
        #TRIP to pin ?? (GPIO 18)
        #ECHO to pin 6 (GND) and pin ??? (GPIO 24)

        #set GPIO pins
        GPIO_TRIG_4 = ??
        GPIO_ECHO_4 = ??

        #setting GPIO direction (IN/OUT)
        GPIO.setup(GPIO_TRIG_4, GPIO.OUT)
        GPIO.setup(GPIO_ECHO_4, GPIO.IN)
        dist = distance()
        print ("Measured Distance for sensor 4 = %.1f cm" % dist)

        #right now it only outputs the distance in words. needs to connect that info to the main.py
        