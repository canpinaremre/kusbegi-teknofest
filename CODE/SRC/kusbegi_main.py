from tk_dronekit_kusbegi import Kusbegi

from dronekit import connect, LocationLocal, VehicleMode, Battery, SystemStatus
from pymavlink import mavutil
import threading
from time import sleep
import math


#Create drone
#
drone = Kusbegi()
drone.connection_string = '127.0.0.1:14540'
drone.connect_vehicle()
#
#End of Create drone



#Listeners
#
@drone.vehicle.on_message('EXTENDED_SYS_STATE')
def listener(self, name, msg):               
    if(msg.landed_state == 1):
        if(drone.state_on_ground == False):
            print("On ground state received")
        drone.state_on_ground = True
    if(msg.landed_state != 1):
        if(drone.state_on_ground == True):
            print("Not on ground state received")
        drone.state_on_ground = False

@drone.vehicle.on_message('HOME_POSITION')
def listener(self, name, home_position):
    drone.home_position_set = True
#
#End of Listeners


#Mission parameters
#
MISSION_ALTITUDE = -10 #Down (meters)
MISSION_CIRCLE_RADIUS = 8 #Meters
MISSION_CIRCLE_YAW_BOTTOM_TO_TOP = 1 #Radians
MISSION_COORDINATE_HOME = 'tk_g1_home.txt' #txt file for coordiantes
MISSION_COORDINATE_FINISH = 'tk_g1_finish.txt' #txt file for coordiantes
MISSION_COORDINATE_RALLY1 = 'tk_g1_rally1.txt' #txt file for coordiantes
MISSION_COORDINATE_RALLY2 = 'tk_g1_rally2.txt' #txt file for coordiantes
MISSION_COORDINATE_BOTTOM_OF_CIRCLE = 'tk_g1_bottom_of_circle.txt' #txt file for coordiantes
MISSION_COORDINATE_TOP_OF_CIRCLE  = 'tk_g1_top_of_circle.txt' #txt file for coordiantes
MISSION_COORDINATE_RALLY3 = 'tk_g1_rally3.txt' #txt file for coordiantes
MISSION_COORDINATE_RALLY4 = 'tk_g1_rally4.txt' #txt file for coordiantes
#
#End of Mission parameters



#Start of Mission
#

drone.ready_to_takeoff()

drone.print_status()

drone.mode_takeoff(MISSION_ALTITUDE) #Takeoff to MISSION_ALTITUDE

#Loop
#
for i in range(2):
    #Mission needs 2 tour

    drone.print_status()

    #First go straight and pass start/finish line
    drone.go_to_location(MISSION_COORDINATE_FINISH)

    drone.go_to_location(MISSION_COORDINATE_RALLY1)
    #These can be done more otonom by only 1 rally point and radius value
    drone.go_to_location(MISSION_COORDINATE_RALLY2)
    
    #First go to circle then do circle
    drone.go_to_location(MISSION_COORDINATE_BOTTOM_OF_CIRCLE)

    drone.do_circle(MISSION_COORDINATE_BOTTOM_OF_CIRCLE,
        MISSION_COORDINATE_TOP_OF_CIRCLE,#If MISSION_CIRCLE_RADIUS != 0 then use MISSION_CIRCLE_RADIUS
        MISSION_CIRCLE_RADIUS,#If MISSION_CIRCLE_RADIUS = 0 then use MISSION_COORDINATE_TOP_OF_CIRCLE
        MISSION_CIRCLE_YAW_BOTTOM_TO_TOP
        )


    drone.go_to_location(MISSION_COORDINATE_RALLY3)
    #These can be done more otonom by only 1 rally point and radius value
    drone.go_to_location(MISSION_COORDINATE_RALLY4)

    drone.go_to_location(MISSION_COORDINATE_HOME)

    
#
#End of Loop

#Go to start/finish line
drone.go_to_location(MISSION_COORDINATE_FINISH)

#Start Land
drone.mode_land()

#Wait for Landed
while not drone.state_on_ground:
    print ("Landing...")
    sleep(1)
    
print("Landed!")
drone.print_status()

#Disarm
drone.disarm()

#
#End of Mission
