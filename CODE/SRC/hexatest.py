from tk_dronekit_kusbegi import Kusbegi
from utility import WaterLevelSensor, Pump

from dronekit import connect, LocationLocal, VehicleMode, Battery, SystemStatus
from pymavlink import mavutil
import threading
from time import sleep, time
import math
from Test_Target_Red_Thread import CSI_Camera,

#Create drone
#
drone = Kusbegi()
#drone.connection_string = '127.0.0.1:14540'
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
MISSION_PUMP_TIMEOUT = 10 #Seconds
MISSION_MAX_WATER_SENSOR = 300 # ADC max value to stop motors
MISSION_ALTITUDE = -6 #Down (meters) #Min 4 - max 30 mt
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

drone.mode_takeoff(MISSION_ALTITUDE) #Takeoff to MISSION_ALTITUDE at MISSION_COORDINATE_HOME

drone.go_to_coordinate(MISSION_COORDINATE_FINISH)
"""
drone.go_to_coordinate(MISSION_COORDINATE_RALLY1)

drone.go_to_coordinate(MISSION_COORDINATE_POOL)

cam = CSI_Camera()
cam.open()
cam.start()

drone.go_to_coordinate(MISSION_COORDINATE_RALLY2)
#Detect red

while cam.running is False:
    sleep(0.5)

drone.go_to_coordinate_dont_wait(MISSION_COORDINATE_RALLY3)
sleep(0.31)

max_area = 0

while True:
    if not (drone.at_the_target_yet_global(drone.req_pos_x,drone.req_pos_y)):
        self.log.logger("Not at the target yet with global frame")
            
            if (cam.con_area > max_area):
                max_area = cam.con_area
                drone.save_coordinate(MISSION_COORDINATE_RED_AREA)
    else:
        break


drone.go_to_coordinate(MISSION_COORDINATE_RALLY4)

drone.go_to_coordinate(MISSION_COORDINATE_HOME)



#START TOUR 2
drone.go_to_coordinate(MISSION_COORDINATE_FINISH)

drone.go_to_coordinate(MISSION_COORDINATE_RALLY1)

drone.go_to_coordinate(MISSION_COORDINATE_POOL)
# Land and pump water in
"""
drone.mode_land()

drone.wait_for_land()

"""
pump = Pump()

water_level_sensor = WaterLevelSensor()

print("Start pumping")
drone.log.logger("Start pumping")

ts = time()

while ( (time()- ts) < MISSION_PUMP_TIMEOUT ):
    pump.pump_water_in()

    if (water_level_sensor.readadc > MISSION_MAX_WATER_SENSOR):
        print("Water level reached!")
        drone.log.logger("Water level reached!")
        break

print("Stop pumping")
drone.log.logger("Stop pumping")

pump.close_and_clean()
water_level_sensor.close_and_clean()
"""
###
### buraya olduğu yerde takeoff 
### --- go_to deyince çapraz gidiyor, 
### takılmamız yan yatmamız olası.......
### hatta su üzerinde takeoff hızını elektronikleri
### korumak için azaltabiliriz de
###
sleep(10)

drone.req_pos_x = drone.vehicle.location.local_frame.north
drone.req_pos_y = drone.vehicle.location.local_frame.east
drone.req_pos_z = drone.vehicle.location.local_frame.down

drone.drive_type = drone.drive_w_setpnt
drone.position_frame = drone.frame_local_ned

drone.ready_to_takeoff()
drone.mode_offboard()

drone.req_pos_z = MISSION_ALTITUDE

while (abs(drone.pos_z - drone.req_pos_z ) > 0.2):
    print("     Altitude: ", self.pos_z, " meter")
    self.log.logger("     Altitude: "+ str(self.pos_z) + " meter")
    sleep(1)
self.log.logger("Target altitude reached : " + str(req_height))

"""
drone.go_to_coordinate(MISSION_COORDINATE_RED_AREA)
#Descend and pump water out

###
### --- buraya kendini hizalaması gerekiyor
### tam orataladı dediği zaman inerek 1 kere daha ortalatabiliriz
### sonrasında hedefin yüksekliği+1metre olacak şekilde 
### inip hold yapıp boşaltabiliriz.
### dezavantajımız pervanelerin ya da olası rüzgarın 
### suyu dağıtabilecek olması 
###


#Descend and pump water out

drone.go_to_coordinate(MISSION_COORDINATE_RALLY4)

drone.go_to_coordinate(MISSION_COORDINATE_HOME)

#GO TO FINISH
drone.go_to_coordinate(MISSION_COORDINATE_FINISH)

#LAND!
"""
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
