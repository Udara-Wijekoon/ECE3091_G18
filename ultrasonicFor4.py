#!/usr/bin/python
import RPi.GPIO as GPIO
import time
from datetime import datetime
import os

# GPIO setup
GPIO.setmode (GPIO.BCM)
GPIO.setwarnings(False)

# setup gpio for echo & trig
echopin = [24,17,22,6]
trigpin = [23,4,27,5]
 
for j in range(4):
    GPIO.setup(trigpin[j], GPIO.OUT)
    GPIO.setup(echopin[j], GPIO.IN)
    print j, echopin[j], trigpin[j]
    print " "
    


# Get reading from HC-SR04   
def ping(echo, trig):
    
    GPIO.output(trig, False)
    # Allow module to settle
    time.sleep(0.5)
    # Send 10us pulse to trigger
    GPIO.output(trig, True)
    time.sleep(0.00001)
    GPIO.output(trig, False)
    pulse_start = time.time()

    # save StartTime
    while GPIO.input(echo) == 0:
        pulse_start = time.time()

    # save time of arrival
    while GPIO.input(echo) == 1:
        pulse_end = time.time()

    # time difference between start and arrival
    pulse_duration = pulse_end - pulse_start
    # mutiply with the sonic speed (34300 cm/s)
    # and divide by 2, because there and back
    distance = pulse_duration * 17150
    
    distance = round(distance, 2)
    
    return distance

print " press Ctrl+c to stop program "
try:
    # main loop
    while True:
        # get distances and assemble data line for writing 
        results = str(datetime.now()) + ","
        for j in range(2):

            distance = ping(echopin[j], trigpin[j])
            print ("sensor", j+1,": ",distance,"cm")
            results = results + str(distance) + ","
            
        results = results + "\n"
              
        # write results data to file        
                      
        with open("/home/pi/data_log.csv", "a") as file:         
            if os.stat("/home/pi/data_log.csv").st_size == 0:
                file.write("Time,Sensor1,Sensor2,Sensor3,Sensor4\n")      
            file.write(results)
        # if sensors write to file to often increase this time        
        print "wait"
        time.sleep (2)    
    
except KeyboardInterrupt:
    print("keyboard interrupt detected, File closed")       
    file.close()    
    
