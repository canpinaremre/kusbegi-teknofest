from tk_dronekit_kusbegi import Kusbegi

from dronekit import connect, LocationLocal, VehicleMode, Battery, SystemStatus
from pymavlink import mavutil
import threading
from time import sleep, time
import math




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

drone.mode_takeoff(-5) #Takeoff to MISSION_ALTITUDE at MISSION_COORDINATE_HOME
drone.default_alt_global = drone.vehicle.location.global_frame.alt

drone.drive_type = drone.drive_w_speed
drone.position_frame = drone.frame_local_ned
drone.req_pos_x = 0
drone.req_pos_y = 0
drone.req_pos_z = 0

print("sleep 2 sec")
sleep(2)
setSpeed = 5

while(1):
    while(drone.req_pos_x < setSpeed):
        drone.req_pos_x = drone.req_pos_x + 0.08
        sleep(0.05)

    sleep(2)

    while(drone.req_pos_x > 0):
        drone.req_pos_x = drone.req_pos_x - 0.08
        sleep(0.05)

    sleep(2)
