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
MISSION_RED_START          = 'tk_red_start.txt'
MISSION_RED_STOP           = 'tk_red_stop.txt'
MISSION_POOL               = 'tk_pool.txt'
MISSION_RED_AREA           = 'tk_red_area.txt'

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
drone.go_to_coordinate(MISSION_COORDINATE_RALLY1)
drone.go_to_coordinate(MISSION_COORDINATE_RALLY2)
drone.go_to_coordinate(MISSION_RED_START)

#wait for yaw
sleep(2)

#Go with speed and yaw
drone.continue_offboard()

drone.coordinate.file_dir = MISSION_RED_START
lat,lon,yaw = drone.coordinate.read_coordinates()


setSpeed = 5
spd = 0

while(spd < setSpeed):
    spd = spd + 0.08
    drone.drive_speed_yaw(spd,yaw)
    sleep(0.05)

drone.coordinate.file_dir = MISSION_RED_STOP
lat,lon,yaw = drone.coordinate.read_coordinates()

#start opencv

drone.distance_tolerance = 10
while not(drone.at_the_target_yet_global(lat,lon)):
    #OPENCV
    #save red area
    print(drone.target_distance_global(lat,lon))
    sleep(0.5)
    
drone.distance_tolerance = 3
drone.set_mode("LOITER")
drone.pause_offboard()
drone.go_to_coordinate(MISSION_RED_STOP)
drone.go_to_coordinate(MISSION_COORDINATE_RALLY3)
drone.go_to_coordinate(MISSION_COORDINATE_RALLY4)
drone.distance_tolerance = 1
drone.go_to_coordinate(MISSION_POOL)

#take water
sleep(10)

drone.distance_tolerance = 3
drone.go_to_coordinate(MISSION_COORDINATE_RALLY1)
drone.go_to_coordinate(MISSION_COORDINATE_RALLY2)
drone.distance_tolerance = 1
drone.go_to_coordinate(MISSION_RED_AREA)

#dump water
sleep(10)

drone.distance_tolerance = 3
drone.go_to_coordinate(MISSION_COORDINATE_RALLY3)
drone.go_to_coordinate(MISSION_COORDINATE_RALLY4)
drone.distance_tolerance = 4
drone.go_to_coordinate(MISSION_COORDINATE_HOME)
drone.distance_tolerance = 1
drone.go_to_coordinate(MISSION_COORDINATE_FINISH)


#LAND!
drone.mode_land()
drone.wait_for_land()
drone.stop()
drone.disarm()

#Exit
exit()


