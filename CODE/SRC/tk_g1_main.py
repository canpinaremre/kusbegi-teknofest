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
if not (drone.connect_vehicle()):
    exit()
#
#End of Create drone


#Listeners
#
@drone.vehicle.on_message('EXTENDED_SYS_STATE')
def listener(self, name, msg):               
    if(msg.landed_state == 1):
        if(drone.state_on_ground == False):
            print("On ground state received")
            drone.log.logger("On ground state received")
        drone.state_on_ground = True
    if(msg.landed_state != 1):
        if(drone.state_on_ground == True):
            print("Not on ground state received")
            drone.log.logger("Not on ground state received")
        drone.state_on_ground = False

@drone.vehicle.on_message('HOME_POSITION')
def listener(self, name, home_position):
    drone.home_position_set = True
#
#End of Listeners


#Mission parameters
#
MISSION_ALTITUDE = -6 #Down (meters)
MISSION_CIRCLE_DIAMETER = 10 #Meters ÇAP
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
drone.default_alt = MISSION_ALTITUDE

drone.ready_to_takeoff()

drone.mode_takeoff(MISSION_ALTITUDE) #Takeoff to MISSION_ALTITUDE

#Loop
#
for i in range(2):
    #Mission needs 2 tour
    drone.log.logger("Tour "+ str(i+1) + " started")

    #First go straight and pass start/finish line
    drone.go_to_coordinate(MISSION_COORDINATE_FINISH)

    """
    drone.go_to_coordinate(MISSION_COORDINATE_RALLY1)
    #These can be done more otonom by only 1 rally point and radius value
    drone.go_to_coordinate(MISSION_COORDINATE_RALLY2)
    """

    drone.right_half_circle(MISSION_COORDINATE_RALLY1,10,MISSION_COORDINATE_HOME)



    #First go to circle then do circle
    drone.go_to_coordinate(MISSION_COORDINATE_BOTTOM_OF_CIRCLE)

    drone.do_circle(MISSION_COORDINATE_BOTTOM_OF_CIRCLE,
        MISSION_COORDINATE_TOP_OF_CIRCLE,#If MISSION_CIRCLE_DIAMETER != 0 then use MISSION_CIRCLE_DIAMETER
        MISSION_CIRCLE_DIAMETER,#If MISSION_CIRCLE_DIAMETER = 0 then use MISSION_COORDINATE_TOP_OF_CIRCLE
        MISSION_COORDINATE_HOME #Send home position for yaw calculation
        )

    """
    drone.go_to_coordinate(MISSION_COORDINATE_RALLY3)
    #These can be done more otonom by only 1 rally point and radius value
    drone.go_to_coordinate(MISSION_COORDINATE_RALLY4)
    """

    drone.left_half_circle(MISSION_COORDINATE_RALLY3,10,MISSION_COORDINATE_HOME)


    drone.go_to_coordinate(MISSION_COORDINATE_HOME)

    
#
#End of Loop

#Go to start/finish line
drone.go_to_coordinate(MISSION_COORDINATE_FINISH)

#Start Land
drone.mode_land()

#Wait for Landed
drone.wait_for_land()

#Disarm
drone.disarm()
drone.stop()
exit()
#
#End of Mission
