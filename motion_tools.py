#!/usr/bin/env pybricks-micropython

from pybricks import ev3brick as brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import (Port, Stop, Direction, Button, Color,
                                 SoundFile, ImageFile, Align)
from pybricks.tools import print, wait, StopWatch
from pybricks.robotics import DriveBase
from threading import Thread

#Variables
scan_head_loop = True
scan_head_move = False
scan_head_move_2 = False
GyroAvailable = False
GyroPort = Port.S1

#Rotaion Angles depends on robot wheel diameter and distance between wheels
Rotation_angle_90dg_tank_turn = 595
Rotation_angle_45dg_tank_turn = Rotation_angle_90dg_tank_turn/2
Rotation_angle_90dg_curve_turn = 1800

# Initialize Gyro
def Init_Gyro():
    global GyroAvailable
    GyroAvailable = True
    global gyro
    gyro = GyroSensor(GyroPort)
    gyro.reset_angle(0)

#init Head motors
def Init_MotorA():
    global motorA
    motorA = Motor(Port.A)

def Init_MotorD():
    global motorD
    motorD = Motor(Port.D)


# 90dg Left tank turn using gyro if available
def left_tank_turn(BC_turn_speed, motorB, motorC):
    global GyroAvailable
    global Rotation_angle_90dg_tank_turn
    if GyroAvailable:
        gyro.reset_angle(0)
        while gyro.angle() != 0:
            gyro.reset_angle(0)
            wait(50)

        motorB.run(BC_turn_speed/2)
        motorC.run(-BC_turn_speed/2)

        while gyro.angle() > -89:
            wait(10)
        
        motorB.stop(Stop.BRAKE)
        motorC.stop(Stop.BRAKE)
    else:
         motorB.run_angle( BC_turn_speed/2, Rotation_angle_90dg_tank_turn, Stop.BRAKE, False)
         motorC.run_angle( BC_turn_speed/2,-Rotation_angle_90dg_tank_turn, Stop.BRAKE, True)

# 90dg Right tank turn using gyro if available
def right_tank_turn(BC_turn_speed, motorB, motorC):
    global GyroAvailable
    global Rotation_angle_90dg_tank_turn
    if GyroAvailable:
        gyro.reset_angle(0)
        while gyro.angle() != 0:
            gyro.reset_angle(0)
            wait(50)

        motorB.run(-BC_turn_speed/2)
        motorC.run(BC_turn_speed/2)

        while gyro.angle() < 89:
            wait(10)
       
        motorB.stop(Stop.BRAKE)
        motorC.stop(Stop.BRAKE)
    else:
         motorC.run_angle( BC_turn_speed/2, Rotation_angle_90dg_tank_turn, Stop.BRAKE, False)
         motorB.run_angle( BC_turn_speed/2,-Rotation_angle_90dg_tank_turn, Stop.BRAKE, True)

# 45dg Left tank turn using gyro if available
def left_tank_turn_45(BC_turn_speed, motorB, motorC):
    global GyroAvailable
    global Rotation_angle_45dg_tank_turn
    if GyroAvailable:
        gyro.reset_angle(0)
        while gyro.angle() != 0:
            gyro.reset_angle(0)
            wait(50)

        motorB.run(BC_turn_speed/2)
        motorC.run(-BC_turn_speed/2)

        while gyro.angle() > -44:
            wait(10)
        
        motorB.stop(Stop.BRAKE)
        motorC.stop(Stop.BRAKE)
    else:
         motorB.run_angle( BC_turn_speed/2, Rotation_angle_45dg_tank_turn, Stop.BRAKE, False)
         motorC.run_angle( BC_turn_speed/2,-Rotation_angle_45dg_tank_turn, Stop.BRAKE, True)


# 45dg Right tank turn using gyro
def right_tank_turn_45(BC_turn_speed, motorB, motorC):
    global GyroAvailable
    global Rotation_angle_45dg_tank_turn
    if GyroAvailable:
        gyro.reset_angle(0)
        while gyro.angle() != 0:
            gyro.reset_angle(0)
            wait(50)

        motorB.run(-BC_turn_speed/2)
        motorC.run(BC_turn_speed/2)

        while gyro.angle() < 44:
            wait(10)
       
        motorB.stop(Stop.BRAKE)
        motorC.stop(Stop.BRAKE)
    else:
         motorC.run_angle( BC_turn_speed/2, Rotation_angle_45dg_tank_turn, Stop.BRAKE, False)
         motorB.run_angle( BC_turn_speed/2,-Rotation_angle_45dg_tank_turn, Stop.BRAKE, True)

# 90dg Left curve turn using gyro
def left_curve_turn(BC_turn_speed, motorB, motorC):
    global GyroAvailable
    global Rotation_angle_90dg_curve_turn
    if GyroAvailable:
        gyro.reset_angle(0)
        while gyro.angle() != 0:
            gyro.reset_angle(0)
            wait(50)
    
        motorB.run(BC_turn_speed)
        motorC.run(BC_turn_speed/3)

        while gyro.angle() > -90:
            wait(10)

        motorB.stop(Stop.BRAKE)
        motorC.stop(Stop.BRAKE)
    else:
        motorC.run_angle( BC_turn_speed/3,    Rotation_angle_90dg_curve_turn/3, Stop.BRAKE, False)
        motorB.run_angle( BC_turn_speed  ,    Rotation_angle_90dg_curve_turn,   Stop.BRAKE, True)


# 90dg Right curve turn using gyro
def right_curve_turn(BC_turn_speed, motorB, motorC):
    global GyroAvailable
    global Rotation_angle_90dg_curve_turn
    if GyroAvailable:
        gyro.reset_angle(0)
        while gyro.angle() != 0:
            gyro.reset_angle(0)
            wait(50)
    
        motorB.run(BC_turn_speed/3)
        motorC.run(BC_turn_speed)
    
        while gyro.angle() < 90:
            wait(10)
       
        motorB.stop(Stop.BRAKE)
        motorC.stop(Stop.BRAKE)
    else:
        motorB.run_angle( BC_turn_speed/3,    Rotation_angle_90dg_curve_turn/3, Stop.BRAKE, False)
        motorC.run_angle( BC_turn_speed  ,    Rotation_angle_90dg_curve_turn,   Stop.BRAKE, True)

#Init Scan Head
#Home at +90 or -90dg
def init_scan_head(head_speed,homedirection,homeoffset):
    if(homedirection):
        motorA.run_until_stalled(head_speed, Stop.BRAKE, 50)
        motorA.reset_angle(0)
        motorA.run_target(head_speed, homeoffset, Stop.BRAKE, True)
    else:
        motorA.run_until_stalled(-head_speed, Stop.BRAKE, 50)
        motorA.reset_angle(0)
        motorA.run_target(head_speed, homeoffset, Stop.BRAKE, True)

    motorA.reset_angle(0)

#Init Scan Head 2
#Home at +90 or -90dg
def init_scan_head_2(head_speed,homedirection,homeoffset):
    if(homedirection):
        motorD.run_until_stalled(head_speed, Stop.BRAKE, 50)
        motorD.reset_angle(0)
        motorD.run_target(head_speed, homeoffset, Stop.BRAKE, True)
    else:
        motorD.run_until_stalled(-head_speed, Stop.BRAKE, 50)
        motorD.reset_angle(0)
        motorD.run_target(head_speed, homeoffset, Stop.BRAKE, True)

    motorD.reset_angle(0)
    

# Move MotorA with Relative Angle
def move_motorA_angle(speed,rotation_angle):
    motorA.run_angle(speed, rotation_angle, Stop.BRAKE, True)

# Move MotorD with Relative Angle
def move_motorD_angle(speed,rotation_angle):
    motorD.run_angle(speed, rotation_angle, Stop.BRAKE, True)

# Move Scan Head with Relative Angle
def move_scan_head_angle(head_speed, rotation_angle):
    motorA.run_angle(head_speed, rotation_angle, Stop.BRAKE, True)

# Move Scan Head 2with Relative Angle
def move_scan_head_angle_2(head_speed, rotation_angle):
    motorD.run_angle(head_speed, rotation_angle, Stop.BRAKE, True)

# Move Scan Head with Target Angle
def move_scan_head_target(head_speed, target_angle):
    motorA.run_target(head_speed, target_angle, Stop.BRAKE, True)

#Move Scan Head with Target Angle
def move_scan_head_target_2(head_speed, target_angle):
    motorD.run_target(head_speed, target_angle, Stop.BRAKE, True)

# Home Scan Head to zero dg
def home_scan_head():
    global scan_head_speed
    move_scan_head_target(scan_head_speed, 0)

# Home Scan Head 2 to zero dg
def home_scan_head_2():
    global scan_head_speed
    move_scan_head_target_2(scan_head_speed, 0)


def direction_sound(direction,GoSound = True):
    # This function  says Direction Number
    if GoSound:
        brick.sound.file(SoundFile.GO)
    if direction == 0:
        brick.sound.file(SoundFile.ZERO)
    elif direction == 1:
        brick.sound.file(SoundFile.ONE)
    elif direction == 2:
        brick.sound.file(SoundFile.TWO)
    elif direction == 3:
        brick.sound.file(SoundFile.THREE)  
    elif direction == 4:
        brick.sound.file(SoundFile.FOUR)    
    elif direction == 5:
        brick.sound.file(SoundFile.FIVE)    
    elif direction == 6:
        brick.sound.file(SoundFile.SIX)    
    elif direction == 7:
        brick.sound.file(SoundFile.SEVEN)  
    elif direction == 8:
        brick.sound.file(SoundFile.EIGHT)
    elif direction == 9:
        brick.sound.file(SoundFile.NINE) 
    elif direction == 10:
        brick.sound.file(SoundFile.TEN)               


#Set up scan head thread
def scan_head_thread():
    global scan_head_loop
    global scan_head_move
    global scan_head_move_2
    global scan_head_speed
    while scan_head_loop:
        #Double check scan_head_loop since it could change while we are in the scan loop
        if scan_head_move and scan_head_loop:
            move_scan_head_target(scan_head_speed,-50)
        if scan_head_move_2 and scan_head_loop:
            move_scan_head_target_2(scan_head_speed,-50)
        if scan_head_loop:
            wait(200)

        if scan_head_move and scan_head_loop:
            move_scan_head_target(scan_head_speed,-25)
        if scan_head_move_2 and scan_head_loop:
            move_scan_head_target_2(scan_head_speed,-25)
        if scan_head_loop:
            wait(200)

        if scan_head_move and scan_head_loop:
            move_scan_head_target(scan_head_speed,  0)
        if scan_head_move_2 and scan_head_loop:
            move_scan_head_target_2(scan_head_speed,  0)
        if scan_head_loop:
            wait(200)

        if scan_head_move and scan_head_loop:
            move_scan_head_target(scan_head_speed, 50)
        if scan_head_move_2 and scan_head_loop:
            move_scan_head_target_2(scan_head_speed, 50)
        if scan_head_loop:
            wait(200)

        if scan_head_move and scan_head_loop:
            move_scan_head_target(scan_head_speed, 25)
        if scan_head_move_2 and scan_head_loop:
            move_scan_head_target_2(scan_head_speed, 25)
        if scan_head_loop:
            wait(200)

        if scan_head_move and scan_head_loop:
            move_scan_head_target(scan_head_speed,  0)  
        if scan_head_move_2 and scan_head_loop:
            move_scan_head_target_2(scan_head_speed,  0)
        if scan_head_loop:    
            wait(200)

#Start scan head thread
def start_scan_head_thread():
    t_scan_head_thread = Thread(target=scan_head_thread)
    t_scan_head_thread.start()
