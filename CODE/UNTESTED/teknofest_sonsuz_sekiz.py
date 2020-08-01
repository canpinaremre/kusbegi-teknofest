from dronekit import connect, Command, LocationGlobal,VehicleMode
from pymavlink import mavutil
import time, sys, argparse, math
import threading
import os
import cv2
import numpy as np
#import picamera
"""
Not needed for now

from statistics import mean
import cv2
import numpy as np
import picamera
"""

#################################### Connection String and vehicle connection

# For Jmavsim
connection_string       = '127.0.0.1:14540'

print ("Connecting")
vehicle = connect(connection_string, wait_ready=True)



"""
connection_string       = "/dev/ttyAMA0"

print ("Connecting")
vehicle = connect(connection_string,baud=57600, wait_ready=True)
"""

##################################### Objects


##################################### SETUP


x,y,z =0,0,0

center_of_object = 320,240

number_of_detection = 0

pixel_square_of_image = 0

pixel_square_needed = 60000 # 3/4 x 3/4

landing_area_counter = 3

number_of_being_sure = 3 #how many detections in a row to be sure

counter_no_flag = 3 #cant see flag for this many time and velocity is zero

vision_altitude = 5.5 #meter

distance_tolerance = 0.15 # meter

threshold_for_tf = 0.3 # %60

time_to_takeoff_again = 3 #second

drive_with_meter = 0b101111111000
drive_with_speed = 0b101111000111
yaw_global = 0
drive_type = drive_with_meter #initial

###################################### FUNCTIONS

def setpoint_buffer():
    global x,y,z,drive_type,yaw_global
    while True:
        msg=vehicle.message_factory.set_position_target_local_ned_encode(
            0,
            0,0,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            drive_type,
            x,y,z,
            x,y,z,
            0,0,0,
            yaw_global,0)
        vehicle.send_mavlink(msg)
        vehicle.flush()
        time.sleep(0.3)


def land():
    print("Land !")
    while vehicle.mode != "LAND":
        vehicle._master.set_mode_px4('LAND',None,None)
        print ("Trying land")
        time.sleep(0.3)
    time.sleep(10)
    print ("Landed!")

def tryArming():
    while True:
        vehicle._master.mav.command_long_send(
        1, # autopilot system id
        1, # autopilot component id
        400, # command id, ARM/DISARM
        0, # confirmation
        1, # arm!
        0,0,0,0,0,0 # unused parameters for this command
        )
        print("Trying arming")
        time.sleep(0.3)
        if (vehicle.armed == True):
            break
    print ("Armed :",vehicle.armed)


def startThread():
    t = threading.Thread(target=setpoint_buffer)

    t.daemon = True

    t.start()


def startOffboardMode():
    while vehicle.mode != "OFFBOARD":
        vehicle._master.set_mode_px4('OFFBOARD',None,None)
        print ("Trying offboard")
        time.sleep(0.3)

def print_status():
    # Display basic vehicle state
    print (" Type: %s" % vehicle._vehicle_type)
    print (" Armed: %s" % vehicle.armed)
    print (" System status: %s" % vehicle.system_status.state)
    print (" GPS: %s" % vehicle.gps_0)
    print (" Alt: %s" % vehicle.location.global_relative_frame.alt)

def readyToTakeoff():
    global drive_with_meter,drive_type

    drive_type = drive_with_meter

    print_status()
    tryArming()
    startOffboardMode()
    print_status()

    return True

def tryDisArming():
    #disarm
    while True:
        vehicle._master.mav.command_long_send(
        1, # autopilot system id
        1, # autopilot component id
        400, # command id, ARM/DISARM
        0, # confirmation
        0, # arm!
        0,0,0,0,0,0 # unused parameters for this command
        )
        print("Trying Disarming")
        time.sleep(0.3)
        if (vehicle.armed == False):
            break
    print ("Armed :",vehicle.armed)

    return True

def shutDownTheMotors():
    #stop motors afer landing
    print("motor shutting down\n")
    print_status()
    tryDisArming()
    print_status()
    return True

def goToLocation(xTarget,yTarget,altTarget):
    global startYaw,x,y,z,distance_tolerance
    #goto desired locaiton

    x,y = bodyToNedFrame(xTarget,yTarget,startYaw)
    z = -altTarget

    while not atTheTargetYet(xTarget,yTarget,altTarget):
        print("Target: ",xTarget,yTarget,altTarget)
        time.sleep(0.2)

    return True


def landWithVision(flagName):
    #land with vision to the flag
    global center_of_object,detected_flag_name,number_of_detection,x,y,z,drive_type,drive_with_meter,drive_with_speed,pixel_square_of_image,pixel_square_needed,startYaw,counter_no_flag,landing_area_counter,yaw_global


    temp_number = number_of_detection
    landing_area_counter_temp = 0
    drive_type = drive_with_speed
    x,y,z = 0,0,0
    no_flag = 0
    while True:
        if number_of_detection > temp_number:
            if detected_flag_name == flagName:
                no_flag = 0
                temp_number = number_of_detection
                yCenter,xCenter = center_of_object # y,x = center_of_object

                xPix = (240 - xCenter) * 0.004 #Max speed is 1 m/s
                yPix = (yCenter - 320) * 0.003 #Max speed is 1 m/s
                print("Go forward :",xPix," m/s Go right :",yPix," m/s Go Down : 0.1 m/s")
                x,y = bodyToNedFrame(xPix,yPix,vehicle.attitude.yaw)
                z = 0.18 # m/s down speed
                if (pixel_square_of_image >= pixel_square_needed):
                    landing_area_counter_temp += 1
                    if  (landing_area_counter_temp >= landing_area_counter):
                        x,y,z = 0,0,0
                        break
                    #go to land mode

            else:
                temp_number = number_of_detection
                no_flag += 1
                print("Wrong flag")
                yaw_global += 0.01
                #ignore wrong detections and wait with zero speed.
                if no_flag >= counter_no_flag:
                    print("Velocity is zero")
                    x,y,z = 0,0,0
        else:
            print("There is no flag")
            no_flag +=1
            yaw_global += 0.01
            #if there is no detection set all the speed to zero
            if no_flag >= counter_no_flag:
                print("Velocity is zero")
                x,y,z = 0,0,0
        time.sleep(0.1)


    land()
    drive_type = drive_with_meter
    #after landing
    shutDownTheMotors()
    return True

def rtl():
    goToLocation(0,0,-vision_altitude)
    landWithVision("turkishflag")
    #return to launch
    return True

def bodyToNedFrame(xBody,yBody,yawBody):
    xNed =  (xBody * math.cos(yawBody) ) - ( yBody * math.sin(yawBody) )
    yNed =  (xBody * math.sin(yawBody) ) + ( yBody * math.cos(yawBody) )
    return xNed,yNed

def atTheTargetYet(xTarget,yTarget,zTarget):
    global startYaw,distance_tolerance
    #goto desired locaiton

    xTarget,yTarget = bodyToNedFrame(xTarget,yTarget,startYaw)
    zTarget = -zTarget

    north = vehicle.location.local_frame.north
    east = vehicle.location.local_frame.east
    down = vehicle.location.local_frame.down
    if (abs(xTarget-north) < distance_tolerance):
        if(abs(yTarget-east) < distance_tolerance):
            if (abs(zTarget-down) < distance_tolerance):
                print("Target point reached")#add x y z
                return True
    print("Not at target yet")
    return False

##################################### START
home_position_set = False
#Create a message listener for home position fix
@vehicle.on_message('HOME_POSITION')
def listener(self, name, home_position):
    global home_position_set
    home_position_set = True


while not home_position_set:
    print ("Waiting for home position...")
    time.sleep(1)

#starting setpoint buffer thread
startThread()
#arming, mode switching,
readyToTakeoff()

startYaw = vehicle.attitude.yaw

i = 1
print("Takeoff!!")
time.sleep(2)
z = -3


###################################### LOOP

print("starting figure 8\n")
time.sleep(5)
radius = 5
yaw_global =(math.pi / 2)
alfa = 0
time.sleep(5)
while (True):
    time.sleep(0.13)
    
    alfa = alfa + 0.015
    yaw_global = (math.pi/2) - alfa
    x = radius - radius * math.cos(alfa)
    
    y = radius * math.sin(alfa)
    
    if (alfa >= (2*math.pi)):
        break
alfa = 0
yaw_global =(math.pi / 2)
print("stage 2")
while (True):
    time.sleep(0.13)
    
    alfa = alfa + 0.015
    yaw_global = (math.pi/2) + alfa
    x = -radius + radius * math.cos(alfa)
    
    y = radius * math.sin(alfa)
    
    if (alfa >= (2*math.pi)):
        break
    

land()
tryDisArming()

    


    
