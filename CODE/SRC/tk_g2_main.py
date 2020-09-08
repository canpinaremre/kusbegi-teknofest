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
MISSION_COORDINATE_HOME = 'tk_g2_home.txt' #txt file for coordiantes
MISSION_COORDINATE_FINISH = 'tk_g2_finish.txt' #txt file for coordiantes
MISSION_COORDINATE_RALLY1 = 'tk_g2_rally1.txt' #txt file for coordiantes
MISSION_COORDINATE_POOL = 'tk_g2_pool.txt' #txt file for coordiantes
MISSION_COORDINATE_RED_AREA = 'tk_g2_red_area.txt' #txt file for coordiantes
MISSION_COORDINATE_RALLY2  = 'tk_g2_rally2.txt' #txt file for coordiantes
MISSION_COORDINATE_RALLY3  = 'tk_g2_rally3.txt' #txt file for coordiantes
MISSION_COORDINATE_RALLY4  = 'tk_g2_rally4.txt' #txt file for coordiantes
#
#End of Mission parameters


#Start of Mission
#
drone.default_alt = MISSION_ALTITUDE

drone.ready_to_takeoff()

drone.mode_takeoff(MISSION_ALTITUDE) #Takeoff to MISSION_ALTITUDE

drone.go_to_coordinate(MISSION_COORDINATE_FINISH)

drone.go_to_coordinate(MISSION_COORDINATE_RALLY1)

drone.go_to_coordinate(MISSION_COORDINATE_POOL)

drone.go_to_coordinate(MISSION_COORDINATE_RALLY2)
#Start opencv

#Start opencv thread and detect red area coordinate

drone.go_to_coordinate(MISSION_COORDINATE_RALLY3)

drone.go_to_coordinate(MISSION_COORDINATE_RALLY4)

drone.go_to_coordinate(MISSION_COORDINATE_HOME)



#START TOUR 2
drone.go_to_coordinate(MISSION_COORDINATE_FINISH)

drone.go_to_coordinate(MISSION_COORDINATE_RALLY1)

drone.go_to_coordinate(MISSION_COORDINATE_POOL)
# Land and pump water in

drone.go_to_coordinate(MISSION_COORDINATE_RED_AREA)
#Descend and pump water out

drone.go_to_coordinate(MISSION_COORDINATE_RALLY4)

drone.go_to_coordinate(MISSION_COORDINATE_HOME)

#GO TO FINISH
drone.go_to_coordinate(MISSION_COORDINATE_FINISH)

#LAND!
drone.mode_land()

drone.wait_for_land()

drone.disarm

#
#End of Mission


#Exit
exit()

#NOTLAR
"""
Disarm olma süresini kapat oto disarm olmayacak
kod üzerinden ground speed ayarı yapılacak
su motoru ve sensörü class denenecek
görüntü işleme class denenecek


"""