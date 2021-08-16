# This file is the main file responsible for sending signals from the Pi to the DC Motor Drivers
# A lot files from sensors need to be connected to this one later

from time import sleep
import RPi.GPIO as GPIO  #importing the standard GPIO module

GPIO.setmode(GPIO.BOARD) #set the board mode so we can refer to the pin numbers
GPIO.setwarnings(false) 

#setting the PWM Frequency
PWM = 100 # might need to edit based on testing


#Motor controller pins
#standadising the port numbers would avoid possible errors
GPIO.setup(12, GPIO.OUT)    #pwn1
GPIO.setup(18, GPIO.OUT)    #AIN2
GPIO.setup(16, GPIO.OUT)    #AIN1
GPIO.setup(22, GPIO.OUT)    #STBY
GPIO.setup(15, GPIO.OUT)    #BTN1
GPIO.setup(13, GPIO.OUT)    #BIN2
GPIO.setup(11, GPIO.OUT)    #pwn2

#configuring the 2 PWM pins
pwm1 = GPIO.PWM(12, PWM)
pwm2 = GPIO.PWM(11, PWM)
pwm1.start(100)
pwm2.start(100)


#MOTOR FUNCTIONS*************************

def forward(speed):
    Motor(0, speed, 0)
    Motor(1, speed, 0)

def backward(speed):
    Motor(0, speed, 1)
    Motor(1, speed, 1)

#turn wheels in different direction
def left(speed):
    Motor(0, speed, 0)
    Motor(1, speed, 1)

def right(speed):
    Motor(0, speed, 1)
    Motor(1, speed, 0)

def motor(motor, speed, direction):
    GPIO.output(22, GPIO.HIGH);
    in1 = GPIO.HIGH
    in2 = GPIO.LOW

    if(direction = 1):
        in1 = GPIO.LOW
        in2 = GPIO.HIGH

    if(motor = 0):
        GPIO.output(16, in1)
        GPIO.output(18, in2)
        pwm1.ChangeDutyCycle(speed)
    elif(motor = 0):
        GPIO.output(15, in1)
        GPIO.output(13, in2)
        pwm2.ChangeDutyCycle(speed)



def stop():
    GPIO.output(22, GPIO.LOW)

# MAIN FUNCTIONS
# AT the moment this is only for testing the motor driver. we need to include sensor
# inputs from other stuff as we move on.

def main(args=None): #Thi is the test looop
    while True:# Move forwards for 1.5 seconds. stop and wait .5 seconds
        forward(50)
        sleep(2) 
        stop()
        sleep(.5)

        #turn left for 1 second and stop 
        #*****8we might have to figure out a way to measure the robot turning in degrees
        left(50)
        sleep(1) 
        stop()
        sleep(.5)

        #turn right for 2 seconds and stop
        right(50)
        sleep(2) 
        stop()
        sleep(.5)

        #turn left for 1 second and stop
        left(50)
        sleep(1) 
        stop()
        sleep(.5)

        #go backword for 2 seconds....the ROBOT SHOULD be in the initial position. facing the 
        #initial direction
        backward(50)
        sleep(2) 
        stop()
        sleep(.5)

if __name__ == "__main__":
    main 
