#THIS PROGRAM USES LEGO EDUCATION SPYKE LEGACY - 2.0.9
#Importing needed modules
from spike import PrimeHub, LightMatrix, Button, StatusLight, ForceSensor, MotionSensor, Speaker, ColorSensor, App, DistanceSensor, Motor, MotorPair
from spike.control import wait_for_seconds, wait_until, Timer
from math import *

#Creating Hub object
hub = PrimeHub()



#Gyro Turn Function
def gyroTurn(left, right, turn):#This is our Gyro Turn
#We first set
#the yaw angle to zero.
#Then, we wait Until the
#yaw angle is greater
#than the number of
#degrees we want the
    motor_pair = MotorPair('E', 'D')
    ya = get_yaw_angle()
    while math.abs(ya) > turn:
        def mv():
            motorE = Motor('E')
            motorE.start(left)
            motorD = Motor('D')
            motorD.start(right)
        mv()
    return "Finished GyroTurn"

#Gyro Straight
#This is our Gyro Straight
#to zero, so that the robot knows
#if the yaw angle changes, the
#robot adjust it's speed to get
def gyroStraight(speed, time):
    motor_pair = MotorPair('E', 'D')
    ya = get_yaw_angle()
    timer = Timer()
    timer.reset()
    while timer < time:
        correction = ya
        error = correction * -1
        def mv():
            motorE = Motor('E')
            motorE.start(speed + correction)
            motorD = Motor('D')
            motorD.start(speed + correction)
        mv()
    return "Finished GyroStraight"
#PID Line Follower 
def pidLineFollower(speed, time, sensor, side):
    #timer reset
    sign = 0
    timer = Timer()
    timer.reset()
    integral = 0
    last = 0
    if side == last: #enables us to line follow on both sides of the line
        sign = -1
    else:
        sign = 1
    while time < timer:
        rl = float(get_reflected_light())# calculates error 
        #between current position and that o the line
        e1 = float(sign) * (50-rl))
        pFix = e1 * 0.3
        integral = integral + e1 #Calculates the summation of all errors, which is the area of
        #under the curve the robot traced, otherwise known as the
        #integral of the curve.
        iFix = integral * 0.1
        derivative = e1 - last #Allows us to save all errors. and thus allowing us to
        #take the summation Of them to find the integral.
        last = e1
        dFix = derivative * 1
        c1 = pFix + iFix + dFix
        motor_pair.start(speed + c1, speed - c1)#The robot starts moving at a speed that
        #allows it to minimize the distance
        #between the sensors and the line.
    motor_pair.stop()


def lsf(speed):
    #this is our line square MyBlock the robot 
    #uses the two color sensors to line up
    # perpendicularly on the line
    motor_pair = MotorPair('B', 'A')
    motor_pair.set_motor_rotation(25.13, 'cm')
    set_default_speed(25)
    la = ColorSensor('A')
    co1 = la.get_reflected_light() < 50
    lf = ColorSensor('F') < 50
    co2 = lf.get_reflected_light() < 50
    while co1 or co2:
        motor_pair.start_at_power(15, 15)
    if co1:
        for i in range(0,1):
            while not co2:
                motor_pair.start_at_power(0, speed)
                motor_pair.stop()
            while not co1:
                motor_pair.start_at_power(1, speed)
            motor_pair.stop()
            while not co2:
                motor_pair.start_at_power(0, -1*speed)
            motor_pair.stop()
            while not co1:
                motor_pair.start_at_power(-1*speed, 0)
            motor_pair.stop()
            while not co1:
                motor_pair.start_at_power(speed, 0)
            motor_pair.stop()
            while not co2:
                motor_pair.start_at_power(-1*speed, 0)
            motor_pair.stop()
    else:
        for i in range(0,1):
            while not co1:
                motor_pair.start_at_power(speed, 0)
            motor_pair.stop()
            while not co2:
                motor_pair.start_at_power(0, -1*speed)
            motor_pair.stop()
            while not co1:
                motor_pair.start_at_power(-1*speed, 0)
            motor_pair.stop()
            while not co2:
                motor_pair.start_at_power(0, speed)
            motor_pair.stop()
        while not co1:
            motor_pair.start_at_power(0, -1*speed)
        motor_pair.stop()
