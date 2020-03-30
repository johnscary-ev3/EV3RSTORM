#!/usr/bin/env pybricks-micropython

from pybricks import ev3brick as brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import (Port, Stop, Direction, Button, Color,
                                 SoundFile, ImageFile, Align)
from pybricks.tools import print, wait, StopWatch
from pybricks.robotics import DriveBase
from threading import Thread


ColorSensorPort = Port.S3

color_detect_loop = True
color_detect_loop_fast_flag = False
color_sound_on = False
color_detected = Color.BLACK
color_detected_name = "Black"
color_rgb_int = [0,0,0]
color_rgb_int_ave = 0

#Set up color detect thread
def color_detect():
    global color_detect_loop
    global color_detect_loop_fast_flag
    global color_detected
    global color_detected_name
    global color_rgb_int
    global color_rgb_int_ave 
    color_detect_loop_wait_time_slow = 100
    color_detect_loop_wait_time_fast = 10
    cs = ColorSensor(ColorSensorPort)
    while color_detect_loop:
        if color_detect_loop_fast_flag:
            color_rgb_int_ave= cs.reflection()
        else:
            color_rgb_int= cs.rgb()       
            color_rgb_int_ave =sum(color_rgb_int)/3
            color_detected = cs.color()
            color_detected_name = color_name(color_detected)
            color_sound(color_detected) 

        if color_detect_loop_fast_flag:
            wait(color_detect_loop_wait_time_fast)
        else:
            wait(color_detect_loop_wait_time_slow)

#Start color detect thread
def start_color_detect():    
    t_color_detect = Thread(target=color_detect)
    t_color_detect.start()



def color_name(color):
    # This function changes color code into string
    if color == Color.BLACK:
        return "Black"
    elif color == Color.BLUE:
        return "Blue"
    elif color == Color.GREEN:
        return "Green"
    elif color == Color.YELLOW:
        return "Yellow"
    elif color == Color.RED:
        return "Red"
    elif color == Color.WHITE:
        return "White"
    elif color == Color.BROWN:
        return "Brown"
    elif color == Color.ORANGE:
        return "Orange"
    elif color == Color.PURPLE:
        return "Purple"
    else:
        return "None"

def color_sound(color):
    # This function  says color except based on color_sound_on
    global color_sound_on
    if color_sound_on:
        if color == Color.WHITE:
            brick.sound.file(SoundFile.WHITE)
        elif color == Color.BLACK:
            brick.sound.file(SoundFile.BLACK)
        elif color == Color.BROWN:
            brick.sound.file(SoundFile.BROWN)
        elif color == Color.BLUE:
            brick.sound.file(SoundFile.BLUE)
        elif color == Color.GREEN:
            brick.sound.file(SoundFile.GREEN)
        elif color == Color.YELLOW:
            brick.sound.file(SoundFile.YELLOW)
        elif color == Color.RED:
            brick.sound.file(SoundFile.RED)  
        elif color == Color.ORANGE:
            brick.sound.file(SoundFile.ORANGE)
        elif color == Color.PURPLE:
            brick.sound.file(SoundFile.PURPLE)