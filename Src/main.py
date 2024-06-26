#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                  UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from pybricks.iodevices import Ev3devSensor


# Create your objects here.
ev3 = EV3Brick()
steer_motor = Motor(Port.D)
left_wheel = Motor(Port.C, positive_direction=Direction.COUNTERCLOCKWISE)
right_wheel = Motor(Port.B, positive_direction=Direction.COUNTERCLOCKWISE)
gyro_sensor = Ev3devSensor(Port.S1)
color_sensor = Ev3devSensor(Port.S2)

kp = 2
ki = 0.005
kd = 0
last_error = 0
errors_sum = 0
speed_robot = 0
steer_angle = 0
orange = [345, 115, 65]
blue = [30, 42, 95]
white = [350, 340, 375]

#print(color_sensor.read('RGB-RAW'))

def move(speed):
    left_wheel.dc(speed)
    right_wheel.dc(speed)

def steer(angle):
    if angle > 100:
        angle = 100
    if angle < -100:
        angle = -100
    angle = angle * 4
    if angle < 2 and angle > -2:
        angle = 0
    steer_motor.track_target(angle)

def control(speed, angle):
    move(speed)
    steer(angle)

def pid_gyro(target):
    global steer_angle
    global last_error
    global errors_sum
    error = gyro_sensor.read('GYRO-ANG')[0] - target
    p_fix = kp * error
    i_fix = ki * errors_sum
    d_fix = kd * (error - last_error)
    pid_value = p_fix + i_fix + d_fix
    steer_angle = pid_value
    last_error = error
    errors_sum += error

def get_color():
    r, g, b = color_sensor.read('RGB-RAW')
    orange_i = abs(r - orange[0]) + abs(g - orange[1]) + abs(b - orange[2])
    blue_i = abs(r -blue[0]) + abs(g - blue[1]) + abs(b - blue[2])
    white_i = abs(r -white[0]) + abs(g - white[1]) + abs(b - white[2])
    if(orange_i < blue_i and orange_i < white_i):
        print("orange")
        return "orange"
    elif(blue_i < orange_i and blue_i < white_i):
        print("blue")
        return "blue"
    else:
        print("white")
        return "white"

# Write your program here.
ev3.speaker.beep()
steer_motor.reset_angle(0)

gyro_sensor.read('GYRO-CAL')
ev3.speaker.beep()
wait(1000)

i = 0
while True:
    dirr = 0
    color = get_color()
    while color == "white":
        speed_robot = 70
        pid_gyro(i * 90)
        control(speed_robot, steer_angle)
        if color == "blue":
            dirr = 1
            break
        elif color == "orange":
            dirr = -1
            break
        color = get_color()
    control(30, -60)
    wait(500)
    color = get_color()
    while color == "white":
        speed_robot = 30
        pid_gyro(dirr * 60 + i * 90)
        control(speed_robot, steer_angle)
        color = get_color()
    wait(500)
    i += 1
    
    

    