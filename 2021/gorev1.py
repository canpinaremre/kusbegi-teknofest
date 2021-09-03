from tk_dronekit_kusbegi import Kusbegi

from dronekit import connect, LocationLocal, VehicleMode, Battery, SystemStatus
from pymavlink import mavutil
import threading
from time import sleep, time
import math


MISSION_COORDINATE_HOME    = 'tk_home.txt' #txt file for coordiantes
MISSION_COORDINATE_FINISH  = 'tk_finish.txt' #txt file for coordiantes
MISSION_COORDINATE_RALLY1  = 'tk_rally1.txt' #txt file for coordiantes
MISSION_COORDINATE_RALLY2  = 'tk_rally2.txt' #txt file for coordiantes
MISSION_COORDINATE_RALLY3  = 'tk_rally3.txt' #txt file for coordiantes
MISSION_COORDINATE_RALLY4  = 'tk_rally4.txt' #txt file for coordiantes
MISSION_MIDDLE             = 'tk_middle.txt'


drone = Kusbegi()
drone.connection_string = '127.0.0.1:14540'
if not (drone.connect_vehicle()):
    exit()


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

drone.position_frame = drone.frame_local_ned
drone.ready_to_takeoff()

drone.mode_takeoff(-10) #Takeoff to MISSION_ALTITUDE at MISSION_COORDINATE_HOME
drone.default_alt_global = drone.vehicle.location.global_frame.alt

drone.drive_type = drone.drive_w_speed
drone.position_frame = drone.frame_local_ned
drone.req_pos_x = 0
drone.req_pos_y = 0
drone.req_pos_z = 0

print("sleep 1 sec")
sleep(1)


drone.set_mode("LOITER")
drone.pause_offboard()
drone.distance_tolerance = 3


# first round
# drone.go_to_coordinate(MISSION_COORDINATE_RALLY1)
# drone.go_to_coordinate(MISSION_COORDINATE_RALLY2)
drone.go_to_coordinate(MISSION_COORDINATE_RALLY3)

#circle
drone.wait_for_circle(MISSION_COORDINATE_RALLY3,10)

drone.go_to_coordinate(MISSION_COORDINATE_RALLY3)
drone.go_to_coordinate(MISSION_COORDINATE_RALLY4)

# second round
drone.go_to_coordinate(MISSION_COORDINATE_RALLY1)
drone.go_to_coordinate(MISSION_COORDINATE_RALLY2)
drone.go_to_coordinate(MISSION_MIDDLE)

#circle
drone.wait_for_circle(MISSION_MIDDLE,10)

drone.go_to_coordinate(MISSION_COORDINATE_RALLY3)
drone.go_to_coordinate(MISSION_COORDINATE_RALLY4)

drone.go_to_coordinate(MISSION_COORDINATE_HOME)
drone.go_to_coordinate(MISSION_COORDINATE_FINISH)


#LAND!
drone.mode_land()
drone.wait_for_land()
drone.stop()
drone.disarm()

#Exit
exit()


