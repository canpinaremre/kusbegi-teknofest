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
MISSION_CIRCLE_RADIUS = 8 #Meters
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

#Def
#
def wait_key():
    input("Press Enter to continue...")
    print("Taking coordinate in:")
    print("3")
    sleep(1)
    print("2")
    sleep(1)
    print("1")
    sleep(1)
    print("SAVING!")

#
#End of Def

file_names = list()

file_names.append( MISSION_COORDINATE_HOME )
file_names.append( MISSION_COORDINATE_FINISH )
file_names.append( MISSION_COORDINATE_RALLY1 )
file_names.append( MISSION_COORDINATE_RALLY2 )
file_names.append( MISSION_COORDINATE_BOTTOM_OF_CIRCLE )
#file_names.append( MISSION_COORDINATE_TOP_OF_CIRCLE )
file_names.append( MISSION_COORDINATE_RALLY3 )
file_names.append( MISSION_COORDINATE_RALLY4 )

for name in file_names:
    wait_key()
    #drone.coordinate.file_dir = name
    #drone.coordinate.write_coordinates(drone.vehicle.location.global_relative_frame.lat, drone.vehicle.location.global_relative_frame.lon, drone.vehicle.attitude.yaw)
    drone.save_coordinate(name)
    print("Coordiante saved for file : ",name)
print("Done")
