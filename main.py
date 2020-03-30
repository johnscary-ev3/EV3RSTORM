#!/usr/bin/env pybricks-micropython

#Modified for EV3RSTORM ; started Feb 18 2020
#RC Mode, Detect objects and Ambient Light, Gyro turns, Check directions or Random reverse or Run blades on detect object modes

#KRAZ3 modified with Scan Head to get directions  Jan 9 2020
#KRAZ3_RC_SCAN_HEAD Project

from pybricks import ev3brick as brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import (Port, Stop, Direction, Button, Color,
                                 SoundFile, ImageFile, Align)
from pybricks.tools import print, wait, StopWatch
from pybricks.robotics import DriveBase
from threading import Thread

import random

#from color_tools import color_name, color_sound, start_color_detect, color_detected, color_detect_loop
import color_tools 

#from motion_tools import *
import motion_tools

import object_tools


# import os

#Hardware Configuration Flags 
InfraredSensorPortAvailable_2 = True
UltrasonicPresent =True
UltrasonicPresent_2 =True
GyroAvailable = False
PushButtonSensorAvailable = False
OpticalSensorAvailable = False

ScanHeadPresent = True
ScanHeadHomeRight = True

ScanHeadPresent_2 = True
ScanHeadHomeRight_2 = False

#Scanhead offset from home to forward position
ScanHeadHomeOffset = -55
ScanHeadHomeOffset_2 =  55

#Ports for Sensor Inputs
InfraredSensorPort = Port.S4
InfraredSensorPort_2 = Port.S1
PushButtonSensorPort = Port.S1
object_tools.UltrasonicSensorPort =Port.S2
object_tools.UltrasonicSensorPort_2 =Port.S3
motion_tools.GyroPort = Port.S3
color_tools.ColorSensorPort = Port.S3

#Drive Motor Config
MotorsClockwise = True
MotorsSidesReversed =True

#Config variables
UseScanHeadObjectDetect = True
UseScanHeadDirectionDetect =False
GeneralMotorAPresent = False
GeneralMotorDPresent = False

# Enable modes dependent on Optical Sensor if desired
if OpticalSensorAvailable:
    FollowLineAvailable = True
    CheckLineAvailable = True
# Dasble these modes if no Optical sensor
else:
    FollowLineAvailable = False
    CheckLineAvailable = False


#  Test head directions
#target_angle = [-90, -45, 0, 45, 90, 135, 180, 225]
#target_direc = [  6,   7, 0,  1,  2,   3,   4,   5]
#num_head_poistions =8

target_angle = [-45, 0, 45]
target_direc = [  7, 0,  1]
num_head_poistions =3


#variables used later
dist_scan=[]

# Play a sound.
brick.sound.beep()

 # Turn the light green
brick.light(Color.GREEN)

# Set Display
brick.display.image(ImageFile.EV3)

#Init the Ultrasonic Distance sensors
if UltrasonicPresent:
    object_tools.Init_UltrasonicSensor()
if UltrasonicPresent_2:
    object_tools.Init_UltrasonicSensor_2()

#Init Small Motors if present
if ScanHeadPresent or GeneralMotorAPresent:
    motion_tools.Init_MotorA()
if ScanHeadPresent_2 or GeneralMotorDPresent:
    motion_tools.Init_MotorD()

#Init Gyro if available
if GyroAvailable:
    motion_tools.Init_Gyro()

#Init PushButton if available
if PushButtonSensorAvailable:
    PushButtonSensor = TouchSensor(PushButtonSensorPort)

# Initialize a motor at ports  B C.
if MotorsSidesReversed:
    if MotorsClockwise:
        motorC = Motor(Port.B, Direction.CLOCKWISE)
        motorB = Motor(Port.C, Direction.CLOCKWISE)
    else:
        motorC = Motor(Port.B, Direction.COUNTERCLOCKWISE)
        motorB = Motor(Port.C, Direction.COUNTERCLOCKWISE)
else:
    if MotorsClockwise:
        motorB = Motor(Port.B, Direction.CLOCKWISE)
        motorC = Motor(Port.C, Direction.CLOCKWISE)
    else:
        motorB = Motor(Port.B, Direction.COUNTERCLOCKWISE)
        motorC = Motor(Port.C, Direction.COUNTERCLOCKWISE)

#set up some motor speeds
BC_move_speed = 300
BC_turn_speed = 300
head_speed = 300
General_motorA_speed =300
Follow_line_speed_max = 300


# Move GeneralMotorA and Turn on MotorA with object detect
if GeneralMotorAPresent:
    motion_tools.move_motorA_angle(General_motorA_speed, 180)
    motion_tools.move_motorA_angle(General_motorA_speed,-180)
    object_tools.object_detect_MoveMotorA_run = True

# Init the Scan Head
if ScanHeadPresent:
    motion_tools.init_scan_head(head_speed, ScanHeadHomeRight, ScanHeadHomeOffset)
    #  Test head directions
    for i in range(num_head_poistions):
        motion_tools.move_scan_head_target(head_speed, target_angle[i])
        motion_tools.direction_sound(target_direc[i], False)
        wait(250)
    #Reset head to 0 position
    motion_tools.move_scan_head_target(head_speed, 0)

# Init the Scan Head 2
if ScanHeadPresent_2:
    motion_tools.init_scan_head_2(head_speed, ScanHeadHomeRight_2, ScanHeadHomeOffset_2)
     #  Test head directions
    for i in range(num_head_poistions):
        motion_tools.move_scan_head_target_2(head_speed, target_angle[i])
        motion_tools.direction_sound(target_direc[i], False)
        wait(250)
    #Reset head to 0 position
    motion_tools.move_scan_head_target_2(head_speed, 0)

# Play another beep sound.
# This time with a higher pitch (1000 Hz) and longer duration (500 ms).
brick.sound.beep(1000, 500)

# Initialize IR sensors
ir =InfraredSensor(InfraredSensorPort)
chan = 1
if InfraredSensorPortAvailable_2:
    ir_2= InfraredSensor(InfraredSensorPort_2)

# Setup a Stop Watch
sw = StopWatch()
sw.reset()

#Start object detect threads
object_tools.start_object_detect()
object_tools.start_object_sound_thread()
if UseScanHeadObjectDetect:
    object_tools.object_detect_run = True
    if UltrasonicPresent_2:
        object_tools.object_detect_run_2 = True

if GeneralMotorAPresent:
    object_tools.start_object_MoveMotorA_thread()


#Start Up Color Detect Loop
if OpticalSensorAvailable:
    color_tools.start_color_detect()

#Start Up Scan heads move Loop
if ScanHeadPresent and  UseScanHeadObjectDetect:
    motion_tools.scan_head_move = True
    motion_tools.scan_head_speed = head_speed
    motion_tools.start_scan_head_thread()
if ScanHeadPresent_2 and  UseScanHeadObjectDetect:
    motion_tools.scan_head_move_2 = True

# Main loop 
# Will exit based on Exit Button

#Set some Control Flags
main_loop = True
Run_forward = False
Check_line = False
Follow_line = False
FollowBeacon = False
Follow_line_initialized = False
Follow_line_diagnostic = False
Follow_line_dark = 0
Follow_line_light = 100
butt_sum1 = 0
butt_sum2 = 0
butt_sum3 = 0
butt_sum4 = 0
FollowBeaconFoundCount = 0


while main_loop ==True:
    if not Follow_line:
        print(" ")
        print("GyroAvailable=", GyroAvailable, "OpticalSensorAvailable=", OpticalSensorAvailable, "PushButtonSensorAvailable=", PushButtonSensorAvailable )
        print("FollowBeacon=", FollowBeacon)
        print("dist=", object_tools.dist, "mm","    dist_2=", object_tools.dist_2, "mm")
        print("object_detected=", object_tools.object_detected, "object_detected_1=", object_tools.object_detected_1,"object_detected_2=", object_tools.object_detected_2)
        if OpticalSensorAvailable:
            print("color= ", color_tools.color_detected, "color_name= ", color_tools.color_detected_name)
            print("rgb_int= ",'R: {:.2f} G: {:.2f} B: {:.2f} '.format(color_tools.color_rgb_int[0],color_tools.color_rgb_int[1],color_tools.color_rgb_int[2]) )
            print("rgb_int_ave= ",'{:.2f} '.format(color_tools.color_rgb_int_ave) )
        print("butt_sum1=", butt_sum1, "butt_sum2=", butt_sum2, "butt_sum3=", butt_sum3, "butt_sum4=", butt_sum4)        
    elif Follow_line_diagnostic:
        print(" ")
        print("Follow_line_dark=",'{:.2f} '.format(Follow_line_dark),"Follow_line_light=",'{:.2f} '.format(Follow_line_light) )
        print("Follow_line_speed_right= ",'{:.2f}'.format(Follow_line_speed_right), "  Follow_line_speed_left= ",'{:.2f}'.format(Follow_line_speed_left) )
    
    # Push Button Sensor if available
    if PushButtonSensorAvailable:
        PushButtonSensorON=PushButtonSensor.pressed()
    else:
        PushButtonSensorON = False

    # Check the Push Button and switch modes on IR sensor
    if  PushButtonSensorON:
        if FollowBeacon == False:
            #Turn On Follow Beacon mode
            #Do Start mode sounds
            brick.sound.file(SoundFile.START)
            brick.sound.file(SoundFile.SEARCHING)
            brick.sound.file(SoundFile.FORWARD)
            #Set flags for this mode
            FollowBeacon= True
            color_tools.color_sound_on = False
            Run_forward =False
            Follow_line=False
        elif FollowBeacon == True:
            #Turn Off Follow Beacon mode
            FollowBeacon = False
            # Stop Motors in case runing
            motorB.stop(Stop.BRAKE)
            motorC.stop(Stop.BRAKE)
            motorB.stop(Stop.BRAKE)
            motorC.stop(Stop.BRAKE)
            #Do Stop sounds
            brick.sound.file(SoundFile.STOP)
            brick.sound.file(SoundFile.SEARCHING)
            brick.sound.file(SoundFile.FORWARD)
        # wait for button release
        wait(100)
    
   
    # IR buttons
    # Check for button press on Chan 1 and execute Move commands if Not FollowBeacon mode
    chan = 1
    buttons = 0
    butt_sum1 = 0
    
    # IR Button Command Mode
    if InfraredSensorPortAvailable_2 or (not InfraredSensorPortAvailable_2 and not FollowBeacon):
        buttons = ir.buttons(chan)
        butt_sum1 = sum(buttons)

    # If Follow Beacon Mode Check for Beacon and Valid Beacon data
    if FollowBeacon:
        Min_allowed_beacon_dist = 3
        Max_allowed_beacon_dist = 100
        Max_allowed_beacon_angle = 90
        beacon_chan = 4

        if InfraredSensorPortAvailable_2:
            beacon_info=ir_2.beacon(beacon_chan)
        else:
            beacon_info=ir.beacon(beacon_chan)

        print("beacon_info=", beacon_info)
        #Check for Beacon not found
        if beacon_info[0] == None or beacon_info[1] == None:
            brick.sound.file(SoundFile.ERROR)
            beacon_found = False 
        #Check for vaild beacon data
        elif (beacon_info[0] < Min_allowed_beacon_dist or beacon_info[0] >= Max_allowed_beacon_dist) or (beacon_info[1] > Max_allowed_beacon_angle or beacon_info[1] < -Max_allowed_beacon_angle) :
            brick.sound.file(SoundFile.ERROR)
            beacon_found = False 
        #Ok we got good beacon readings         
        else:
            beacon_found = True
           

    #Follow Beacon Mode Handling
    #Following beacon and have beacon info and no objects
    if FollowBeacon and beacon_found and not object_tools.object_detected:
        #Settings to Control Beacon Finding 
        # Dist target 15 if using Upper IR detector for Beacon mode
        # Dist target 10 if using Lower IR detector for Beacon mode 
        Dist_beacon_targ= 10 
        Head_beacon_targ=  0
        SpeedGainFactorDrive = 5.0
        SpeedGainFactorTurn  = 2.0
        Follow_beacon_speed_max_drive = 600
        Follow_beacon_speed_max_turn  = 400
        head_tol = 2
        dist_tol = 1

        #get dist and heading from beacon reeading info results
        Dist_beacon=beacon_info[0]
        Head_beacon=beacon_info[1]

        #Check to see if we are at target location. Use a tolerance so this does not need to be exact.
        if( abs(Head_beacon - Head_beacon_targ) <= head_tol and abs(Dist_beacon - Dist_beacon_targ) <= dist_tol ):
            #At location
            #stop motors
            motorB.stop(Stop.BRAKE)
            motorC.stop(Stop.BRAKE)
            motorB.stop(Stop.BRAKE)
            motorC.stop(Stop.BRAKE)
            #Celebrate
            FollowBeaconFoundCount = FollowBeaconFoundCount + 1
            if FollowBeaconFoundCount <= 2:
                brick.sound.file(SoundFile.CHEERING)

        #Not at target location yet
        else:
            #Keep going by setting motors
            #Normalize the Dist and Heading Signals and then apply Max Speed and Speed Gain Factors to get reasonable motor speeds
            #Calculate a Drive speed component based on Distance and Turn speed component based on angle
            Follow_beacon_speed_drive= SpeedGainFactorDrive * Follow_beacon_speed_max_drive * ( (Dist_beacon-Dist_beacon_targ)/100 )
            Follow_beacon_speed_turn = SpeedGainFactorTurn  * Follow_beacon_speed_max_turn  * ( (Head_beacon-Head_beacon_targ)/75  )
            
            #Additional Limit on Drive speed to + or - Max Beacon drive speed
            if (Follow_beacon_speed_drive   >  Follow_beacon_speed_max_drive):
                Follow_beacon_speed_drive   =  Follow_beacon_speed_max_drive
            if (Follow_beacon_speed_drive   < -Follow_beacon_speed_max_drive):
                Follow_beacon_speed_drive   = -Follow_beacon_speed_max_drive

            #Additional Limit on Turn Speeds  to + or - Max Beacon turn speed
            if (Follow_beacon_speed_turn    >  Follow_beacon_speed_max_turn):
                Follow_beacon_speed_turn    =  Follow_beacon_speed_max_turn
            if (Follow_beacon_speed_turn    < -Follow_beacon_speed_max_turn):
                Follow_beacon_speed_turn    = -Follow_beacon_speed_max_turn
       
            #Put the Left and Right Drive and Turn speeds together
            Follow_beacon_speed_right = Follow_beacon_speed_drive - Follow_beacon_speed_turn
            Follow_beacon_speed_left  = Follow_beacon_speed_drive + Follow_beacon_speed_turn

            print("Follow_beacon_speed_right=",'{:.2f}'.format(Follow_beacon_speed_right), "  Follow_beacon_speed_left=",'{:.2f}'.format(Follow_beacon_speed_left))
        
            #Set the motors
            motorB.run(Follow_beacon_speed_right)
            motorC.run(Follow_beacon_speed_left )

            #Reset FollowBeaconFoundCount
            FollowBeaconFoundCount = 0

    #Following beacon and have object detection
    elif FollowBeacon and beacon_found and object_tools.object_detected:    
        # stop and turn around
        motorB.stop(Stop.BRAKE)
        motorC.stop(Stop.BRAKE)
        motorB.stop(Stop.BRAKE)
        motorC.stop(Stop.BRAKE)
        motion_tools.right_tank_turn(BC_turn_speed, motorB, motorC)
        motion_tools.right_tank_turn(BC_turn_speed, motorB, motorC) 

    #Following beacon and Lost Beacon
    elif FollowBeacon and not beacon_found:   
        # Stop Motors in case runing
        motorB.stop(Stop.BRAKE)
        motorC.stop(Stop.BRAKE)
        motorB.stop(Stop.BRAKE)
        motorC.stop(Stop.BRAKE)


    # Normal RC Moves
    if Run_forward == False and Follow_line == False and FollowBeacon == False:
        #First test for Object Detected and do lights
        if object_tools.object_detected:
            brick.light(Color.ORANGE)
        else:
            brick.light(Color.GREEN)

        #Check for RC Buttons
        if butt_sum1 == Button.LEFT_UP or butt_sum1 == Button.LEFT_UP + Button.RIGHT_UP or butt_sum1 == Button.LEFT_UP + Button.RIGHT_DOWN:
            if object_tools.object_detected == False:
                motorB.run(BC_move_speed)
            else:
                motorB.stop(Stop.BRAKE)
        else:
            if butt_sum1 == Button.LEFT_DOWN or butt_sum1 == Button.LEFT_DOWN + Button.RIGHT_UP or butt_sum1 == Button.LEFT_DOWN + Button.RIGHT_DOWN:
                motorB.run(-BC_move_speed)
            else:
                motorB.stop(Stop.COAST)

        if butt_sum1 == Button.RIGHT_UP or butt_sum1 == Button.LEFT_UP + Button.RIGHT_UP or butt_sum1 == Button.LEFT_DOWN + Button.RIGHT_UP:
            if object_tools.object_detected == False:
                motorC.run(BC_move_speed)
            else:
                motorC.stop(Stop.BRAKE)
        else:
            if butt_sum1 == Button.RIGHT_DOWN or butt_sum1 == Button.LEFT_UP + Button.RIGHT_DOWN or butt_sum1 == Button.LEFT_DOWN + Button.RIGHT_DOWN:
                motorC.run(-BC_move_speed)
            else:
                motorC.stop(Stop.COAST)

    # 90dg Left tank turn using gyro
    if butt_sum1 == Button.LEFT_UP + Button.LEFT_DOWN:
        motion_tools.left_tank_turn(BC_turn_speed, motorB, motorC)

    # 90dg Right tank turn using gyro
    if butt_sum1 == Button.RIGHT_UP + Button.RIGHT_DOWN:
        motion_tools.right_tank_turn(BC_turn_speed, motorB, motorC)

    # Exit Button
    if butt_sum1 == Button.BEACON:
        main_loop =False
        motorB.stop(Stop.BRAKE)
        motorC.stop(Stop.BRAKE)


# Check for button press on Chan 2 and execute Move commands
    chan = 2
    buttons = 0
    butt_sum2 = 0
    if(FollowBeacon == False):
        buttons = ir.buttons(chan)
        butt_sum2 = sum(buttons)

    
     
    # 90dg Left Curve turn using gyro
    if butt_sum2 == Button.LEFT_UP + Button.LEFT_DOWN:
        motion_tools.left_curve_turn(BC_move_speed, motorB, motorC)

    # 90dg Right Curve turn using gyro
    if butt_sum2 == Button.RIGHT_UP + Button.RIGHT_DOWN:
        motion_tools.right_curve_turn(BC_move_speed, motorB, motorC)

    # Exit Button
    if butt_sum2 == Button.BEACON:
        main_loop =False
        motorB.stop(Stop.BRAKE)
        motorC.stop(Stop.BRAKE)

    # Check for button press on Chan 3 and execute Move commands
    chan = 3
    buttons = 0
    butt_sum3 = 0
    if(FollowBeacon == False):
        buttons = ir.buttons(chan)
        butt_sum3= sum(buttons)

    
     
    # 360g Left Curve turn using gyro
    if butt_sum3 == Button.LEFT_UP:
        for i in range(4):
            motion_tools.left_curve_turn(BC_move_speed, motorB, motorC)
        

    # 360g Left Tank turn using gyro
    if butt_sum3 == Button.LEFT_DOWN:
        for i in range(4):
            motion_tools.left_tank_turn(BC_turn_speed, motorB, motorC)
        

    # 360dg Right Curve turn using gyro
    if butt_sum3 == Button.RIGHT_UP:
        for i in range(4):
            motion_tools.right_curve_turn(BC_move_speed, motorB, motorC)
        

    # 360g Right Tank turn using gyro
    if butt_sum3 == Button.RIGHT_DOWN:
        for i in range(4):
            motion_tools.right_tank_turn(BC_turn_speed, motorB, motorC)
        

    # Exit Button
    if butt_sum3 == Button.BEACON:
        main_loop =False
        motorB.stop(Stop.BRAKE)
        motorC.stop(Stop.BRAKE)

    # Check for button press on Chan 4 and execute Move commands
    chan = 4
    buttons = 0
    butt_sum4 = 0
    if(not FollowBeacon or not PushButtonSensorAvailable):
        buttons = ir.buttons(chan)
        butt_sum4= sum(buttons)

    # check for Color Sound On/Off Button and Toggle control variable 
    # or Toggle Follow mode if No Push Button and no Color Sensor
    if butt_sum4 == Button.RIGHT_DOWN and OpticalSensorAvailable:
        if color_tools.color_sound_on == False:
            brick.sound.file(SoundFile.START)
            brick.sound.file(SoundFile.COLOR)
            color_tools.color_sound_on = True
        elif color_tools.color_sound_on == True:
            color_tools.color_sound_on = False
            brick.sound.file(SoundFile.STOP)
            brick.sound.file(SoundFile.COLOR)   
        # wait for button release
        wait(100)
    # Also Toggle Follow mode here if needed
    elif butt_sum4 == Button.RIGHT_DOWN and not OpticalSensorAvailable and not PushButtonSensorAvailable:
        if FollowBeacon == False:
            #Turn On Follow Beacon mode
            #Do Start mode sounds
            brick.sound.file(SoundFile.START)
            brick.sound.file(SoundFile.SEARCHING)
            brick.sound.file(SoundFile.FORWARD)
            #Set flags for this mode
            FollowBeacon= True
            color_tools.color_sound_on = False
            Run_forward =False
            Follow_line=False
        elif FollowBeacon == True:
            #Turn Off Follow Beacon mode
            FollowBeacon = False
            # Stop Motors in case runing
            motorB.stop(Stop.BRAKE)
            motorC.stop(Stop.BRAKE)
            motorB.stop(Stop.BRAKE)
            motorC.stop(Stop.BRAKE)
            #Do Stop sounds
            brick.sound.file(SoundFile.STOP)
            brick.sound.file(SoundFile.SEARCHING)
            brick.sound.file(SoundFile.FORWARD)
        # wait for button release
        wait(100)
    
    elif butt_sum4 == Button.RIGHT_DOWN and not OpticalSensorAvailable:
        brick.sound.file(SoundFile.SORRY)
        brick.sound.file(SoundFile.NO)
        brick.sound.file(SoundFile.COLOR)
        # wait for button release
        wait(100)
     
    # Toggle Run_forward control variable
    if butt_sum4 == Button.LEFT_UP and not FollowBeacon:
        if Run_forward == False:
            #Stop Motors first
            motorB.stop(Stop.BRAKE)
            motorC.stop(Stop.BRAKE)
            motorB.stop(Stop.BRAKE)
            motorC.stop(Stop.BRAKE)          
            brick.sound.file(SoundFile.START)
            brick.sound.file(SoundFile.FORWARD)
            Run_forward= True
            Follow_line =False
        elif Run_forward == True:
            #Stop Motors first
            motorB.stop(Stop.BRAKE)
            motorC.stop(Stop.BRAKE)
            motorB.stop(Stop.BRAKE)
            motorC.stop(Stop.BRAKE)
            brick.sound.file(SoundFile.STOP)
            brick.sound.file(SoundFile.FORWARD)
            Run_forward = False
        # wait for button release
        wait(100)

        # Toggle Follow_line control variable
    if butt_sum4 == Button.LEFT_DOWN and FollowLineAvailable and not FollowBeacon:
        if Follow_line == False: 
            #Stop Motors first
            motorB.stop(Stop.BRAKE)
            motorC.stop(Stop.BRAKE)
            motorB.stop(Stop.BRAKE)
            motorC.stop(Stop.BRAKE)          
            brick.sound.file(SoundFile.START)
            brick.sound.file(SoundFile.BLACK)
            brick.sound.file(SoundFile.FORWARD)
            Run_forward= False
            Follow_line = True
            print("\r\nFollow Line Mode Activated")
            color_tools.color_detect_loop_fast_flag = True 
            wait(100) 

            #Init Follow_line Dark and Light Levels
            if Follow_line_initialized == False:
                #speed up the color Intensity loop first
                

                # Dark Area
                brick.sound.file(SoundFile.ACTIVATE)
                brick.sound.file(SoundFile.BLACK)
                # Wait until any of the buttons are pressed
                while not any(brick.buttons()):
                    brick.sound.file(SoundFile.CONFIRM)
                    wait(100)
                Follow_line_dark = color_tools.color_rgb_int_ave

                # Light Area
                brick.sound.file(SoundFile.ACTIVATE)
                brick.sound.file(SoundFile.WHITE)
                # Wait until any of the buttons are pressed
                while not any(brick.buttons()):
                    brick.sound.file(SoundFile.CONFIRM)
                    wait(100)
                Follow_line_light = color_tools.color_rgb_int_ave
              
                Follow_line_initialized = True
        elif Follow_line == True:   
            #Stop Motors first
            motorB.stop(Stop.BRAKE)
            motorC.stop(Stop.BRAKE)
            motorB.stop(Stop.BRAKE)
            motorC.stop(Stop.BRAKE)
            brick.sound.file(SoundFile.STOP)
            brick.sound.file(SoundFile.BLACK)
            brick.sound.file(SoundFile.FORWARD)
            Follow_line = False
            color_tools.color_detect_loop_fast_flag = False
            print("\r\nFollow Line Mode Deactivated")
        # wait for button release
        wait(100)
    elif butt_sum4 == Button.LEFT_DOWN and not FollowLineAvailable and not FollowBeacon:
        brick.sound.file(SoundFile.SORRY)
        brick.sound.file(SoundFile.NO)
        brick.sound.file(SoundFile.BLACK)
        brick.sound.file(SoundFile.FORWARD)

    # Toggle Check_line and Follow_line control variable
    if butt_sum4 == Button.RIGHT_UP and CheckLineAvailable and not FollowBeacon:
        if Check_line == False:
            brick.sound.file(SoundFile.START)
            brick.sound.file(SoundFile.BLACK)
            brick.sound.file(SoundFile.BACKWARDS)
            Check_line= True
            color_tools.color_sound_on = False
        elif Check_line == True:
            Check_line = False
            brick.sound.file(SoundFile.STOP)
            brick.sound.file(SoundFile.BLACK)
            brick.sound.file(SoundFile.BACKWARDS)
        # wait for button release
        wait(100)
    elif butt_sum4 == Button.RIGHT_UP and not CheckLineAvailable and not FollowBeacon:
        brick.sound.file(SoundFile.SORRY)
        brick.sound.file(SoundFile.NO)
        brick.sound.file(SoundFile.BLACK)
        brick.sound.file(SoundFile.BACKWARDS)


    if butt_sum1 != 0 or butt_sum2 != 0 or butt_sum3 != 0: 
        Run_forward = False
        Check_line =False
        Follow_line = False

    #Turn Off Scan Head and Home Head when not doing Run_foward mode
    #if Run_forward == False and ScanHeadPresent:
    #    motion_tools.scan_head_move = False
     #   motion_tools.home_scan_head()

    # Do Run Forward mode
    if Run_forward == True:
        # if doing Check_line and Black line detected then take action
        if Check_line == True and color_tools.color_detected == Color.BLACK:
            # First Stop
            motorB.stop(Stop.BRAKE)
            motorC.stop(Stop.BRAKE)
            motorB.stop(Stop.BRAKE)
            motorC.stop(Stop.BRAKE)
            # Make the light red
            brick.light(Color.RED) 
            # Back up some
            motorB.run_angle(BC_move_speed, -270, Stop.COAST, False)
            motorC.run_angle(BC_move_speed, -270, Stop.COAST, True ) 
            # Announce black detected       
            brick.sound.file(SoundFile.BLACK)
            brick.sound.file(SoundFile.DETECTED)
            # Pick a random way to go
            random_direction =random.randint(0,3)        
            if random_direction == 0:
                motion_tools.right_tank_turn(BC_turn_speed, motorB, motorC)
            elif random_direction == 1:
                motion_tools.right_tank_turn_45(BC_turn_speed, motorB, motorC)
            elif random_direction == 2:
                motion_tools.left_tank_turn(BC_turn_speed, motorB, motorC)
            elif random_direction == 3:
                motion_tools.left_tank_turn_45(BC_turn_speed, motorB, motorC)
            # Turn the light green
            brick.light(Color.GREEN)

        # Do Run_forward mode unless Object Detected  
        elif object_tools.object_detected == False:
            motorB.run(BC_move_speed)
            motorC.run(BC_move_speed)
            motion_tools.scan_head_move = True
        else:
            #Object detected or #4 Button.LEFT_DOWN pressed
            motorB.stop(Stop.BRAKE)
            motorC.stop(Stop.BRAKE)
            motorB.stop(Stop.BRAKE)
            motorC.stop(Stop.BRAKE)

            # Make the light orange
            brick.light(Color.ORANGE) 
            
            # Turn off Head scan loop mode to do Full Scan here
            if ScanHeadPresent and UseScanHeadObjectDetect:
                motion_tools.scan_head_move = False
                wait(100)
                motion_tools.move_scan_head_target(head_speed, 0)


            #pause object detect loop
            object_tools.object_detect_run = False
            wait(200)

            #Scan for Distances 
            dist_scan.clear()
            if ScanHeadPresent and UseScanHeadDirectionDetect:
                #Use scan head 
                #Scan for Distances at -90,-45,0,45,90,135,180,225 dg
                #Start at -90 since that is position of mechanical Stop on Head
                motion_tools.move_scan_head_target(head_speed, -90)
                wait(100)
                motion_tools.direction_sound(6, False)
                object_dist=object_tools.get_object_dist()
                dist_scan.insert(6,object_dist)
            
                motion_tools.move_scan_head_target(head_speed, -45)
                wait(100)
                motion_tools.direction_sound(7, False)
                object_dist=object_tools.get_object_dist()
                dist_scan.insert(7,object_dist)
           
                motion_tools.move_scan_head_target(head_speed,   0)
                wait(100)
                motion_tools.direction_sound(0, False)
                object_dist=object_tools.get_object_dist()
                dist_scan.insert(0,object_dist)          

                motion_tools.move_scan_head_target(head_speed,  45)
                wait(100)
                motion_tools.direction_sound(1, False)
                object_dist=object_tools.get_object_dist()
                dist_scan.insert(1,object_dist)
            
                motion_tools.move_scan_head_target(head_speed,  90)
                wait(100)
                motion_tools.direction_sound(2, False)
                object_dist=object_tools.get_object_dist()
                dist_scan.insert(2,object_dist)         

                motion_tools.move_scan_head_target(head_speed, 135)
                wait(100)
                motion_tools.direction_sound(3, False)
                object_dist=object_tools.get_object_dist()
                dist_scan.insert(3,object_dist)
            
                motion_tools.move_scan_head_target(head_speed, 180)
                wait(100)
                motion_tools.direction_sound(4, False)
                object_dist=object_tools.get_object_dist()
                dist_scan.insert(4,object_dist)
            
                motion_tools.move_scan_head_target(head_speed, 225)
                wait(100)
                motion_tools.direction_sound(5, False)
                object_dist=object_tools.get_object_dist()
                dist_scan.insert(5,object_dist)

                #Reset Head to home position
                motion_tools.move_scan_head_target(head_speed,   0)  
            else:
                #No Scan Head; move body to scan
                #Scan for Distances at 0,45,90,135,180,225,270,315 dg
                motion_tools.direction_sound(0, False)
                object_dist=object_tools.get_object_dist()
                dist_scan.insert(0,object_dist)
            
                motion_tools.right_tank_turn_45(BC_turn_speed, motorB, motorC)
                wait(100)
                motion_tools.direction_sound(1, False)
                object_dist=object_tools.get_object_dist()
                dist_scan.insert(1,object_dist)        

                motion_tools.right_tank_turn_45(BC_turn_speed, motorB, motorC)
                wait(100)
                motion_tools.direction_sound(2, False)
                object_dist=object_tools.get_object_dist()
                dist_scan.insert(2,object_dist)
            
                motion_tools.right_tank_turn_45(BC_turn_speed, motorB, motorC)
                wait(100)
                motion_tools.direction_sound(3, False)
                object_dist=object_tools.get_object_dist()
                dist_scan.insert(3,object_dist)
            
                motion_tools.right_tank_turn_45(BC_turn_speed, motorB, motorC)
                wait(100)
                motion_tools.direction_sound(4, False)
                object_dist=object_tools.get_object_dist()
                dist_scan.insert(4,object_dist)
            
                motion_tools.right_tank_turn_45(BC_turn_speed, motorB, motorC)
                wait(100)
                motion_tools.direction_sound(5, False)
                object_dist=object_tools.get_object_dist()
                dist_scan.insert(5,object_dist)
            
                motion_tools.right_tank_turn_45(BC_turn_speed, motorB, motorC)
                wait(100)
                motion_tools.direction_sound(6, False)
                object_dist=object_tools.get_object_dist()
                dist_scan.insert(6,object_dist)
            
                motion_tools.right_tank_turn_45(BC_turn_speed, motorB, motorC)
                wait(100)
                motion_tools.direction_sound(7, False)
                object_dist=object_tools.get_object_dist()
                dist_scan.insert(7,object_dist)

                #Reset Body to home position
                motion_tools.right_tank_turn_45(BC_turn_speed, motorB, motorC)
                
            print("dist_scan= ",dist_scan)

            #pick direction with  distance > path_min_distance with Forward direction Priority
            path_min_distance =600
            path_dist_min_limit =100
            path_dist_step = 50
            direction=-1

            #Try to find a path by reducing the path min dist until we find a path or hit the min distance limit
            while path_min_distance >= path_dist_min_limit and direction < 0:
                #First Check direct forward and +/-1 direction 0 to give preference
                #Then check other directions and +/- directions with priority dir 1,7,2,6,3,5,4
                if dist_scan[0] > path_min_distance and dist_scan[7] > path_min_distance  and dist_scan[1] > path_min_distance:
                    direction=0
                elif dist_scan[1] > path_min_distance and dist_scan[0] > path_min_distance and dist_scan[2] > path_min_distance:
                    direction=1
                elif dist_scan[7] > path_min_distance and dist_scan[6] > path_min_distance and dist_scan[0] > path_min_distance:
                    direction=7
                elif dist_scan[2] > path_min_distance and dist_scan[1] > path_min_distance and dist_scan[3] > path_min_distance:
                    direction=2
                elif dist_scan[6] > path_min_distance and dist_scan[5] > path_min_distance and dist_scan[7] > path_min_distance:
                    direction=6
                elif dist_scan[3] > path_min_distance and dist_scan[2] > path_min_distance and dist_scan[4] > path_min_distance:
                    direction=3
                elif dist_scan[5] > path_min_distance and dist_scan[4] > path_min_distance and dist_scan[6] > path_min_distance:
                    direction=5
                elif dist_scan[4] > path_min_distance and dist_scan[3] > path_min_distance and dist_scan[5] > path_min_distance:
                    direction=4
                #Reduce the path min distance and try again to find a path
                path_min_distance = path_min_distance - path_dist_step
           
            print("direction= ", direction, "direction_dist =", dist_scan[direction])

            #random_direction = random.randint(1,4)

            # if no new direction found (=-1) give Error message
            # else give direction
            if direction == -1:
                brick.sound.file(SoundFile.ERROR)
            else:
                motion_tools.direction_sound(direction)

            # Go to selected direction starting from dir 0 dg            
            if direction == 1:
                motion_tools.right_tank_turn_45(BC_turn_speed, motorB, motorC)
            elif direction == 2:
                motion_tools.right_tank_turn(BC_turn_speed, motorB, motorC)
            elif direction == 3:
                motion_tools.right_tank_turn(BC_turn_speed, motorB, motorC)
                motion_tools.right_tank_turn_45(BC_turn_speed, motorB, motorC)
            elif direction == 4:
                motion_tools.right_tank_turn(BC_turn_speed, motorB, motorC)
                motion_tools.right_tank_turn(BC_turn_speed, motorB, motorC)
            elif direction == 5:
                motion_tools.left_tank_turn(BC_turn_speed, motorB, motorC)
                motion_tools.left_tank_turn_45(BC_turn_speed, motorB, motorC)
            elif direction == 6:
                motion_tools.left_tank_turn(BC_turn_speed, motorB, motorC)
            elif direction == 7:
                motion_tools.left_tank_turn_45(BC_turn_speed, motorB, motorC)

            #Restart object detect loop
            object_tools.object_detect_run = True

            # Turn the light green
            brick.light(Color.GREEN)

    #Do Follow Line Mode
    if Follow_line:
        # first check if Object detected
        if object_tools.object_detected:

            # Turn OFF Head scan loop mode while we turn around; center head 
            if ScanHeadPresent and UseScanHeadObjectDetect:
                motion_tools.scan_head_move = False
                wait(100)
                motion_tools.move_scan_head_target(head_speed, 0)

            #turn around until we detect dark line
            #Set the motors
            motorB.run(-Follow_line_speed_max/2 )
            motorC.run(Follow_line_speed_max/2)
            wait(200)
            mid_point = (Follow_line_light + Follow_line_dark)/2
            #wait for dark line 
            while color_tools.color_rgb_int_ave > mid_point:
                wait(50)
            #Stop motors
            motorB.stop(Stop.BRAKE)
            motorC.stop(Stop.BRAKE)
            motorB.stop(Stop.BRAKE)
            motorC.stop(Stop.BRAKE)

             # Turn ON Head scan loop mode
            if ScanHeadPresent and UseScanHeadObjectDetect:
                motion_tools.scan_head_move = True

        # No Object so follow line
        else:
            #Calculate the Follow Line Speeds
            # Use Speed Ratio (SpeedR) for fixed to variable speed part ratio
            SpeedR=.3
            Follow_line_speed_right=Follow_line_speed_max*(SpeedR+(1-SpeedR)*(Follow_line_light - color_tools.color_rgb_int_ave)/(Follow_line_light-Follow_line_dark))
            Follow_line_speed_left =Follow_line_speed_max*(SpeedR+(1-SpeedR)*(color_tools.color_rgb_int_ave - Follow_line_dark )/(Follow_line_light-Follow_line_dark))
        
            #Set the motors
            motorB.run(Follow_line_speed_left )
            motorC.run(Follow_line_speed_right)


    # Exit Button
    if butt_sum4 == Button.BEACON and not FollowBeacon:
        main_loop =False
        motorB.stop(Stop.BRAKE)
        motorC.stop(Stop.BRAKE)

    

    wait(10)
    

# Terminate progam  after saying stop
object_tools.object_detect_loop = False
color_tools.color_detect_loop = False
motion_tools.scan_head_loop =False
brick.sound.file(SoundFile.STOP)
