from tk_dronekit_kusbegi import Kusbegi
from utility import WaterLevelSensor, Pump

from dronekit import connect, LocationLocal, VehicleMode, Battery, SystemStatus
from pymavlink import mavutil
import threading
from time import sleep, time
import math



import cv2
import numpy as np

# gstreamer_pipeline returns a GStreamer pipeline for capturing from the CSI camera
# Defaults to 1280x720 @ 60fps
# Flip the image by setting the flip_method (most common values: 0 and 2)
# display_width and display_height determine the size of the window on the screen

cx,cy,area = 0,0,0
def gstreamer_pipeline(
    capture_width=1280,
    capture_height=720,
    display_width=640,
    display_height=360,
    framerate=60,
    flip_method=0,
):
    return (
        "nvarguscamerasrc ! "
        "video/x-raw(memory:NVMM), "
        "width=(int)%d, height=(int)%d, "
        "format=(string)NV12, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )

color_red = 5
color_blue = 3
blue_or_red = color_red
def show_camera():
    global cx,cy,area,blue_or_red,color_blue,color_red
    # To flip the image, modify the flip_method parameter (0 and 2 are the most common)
    print(gstreamer_pipeline(flip_method=0))
    cap = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)
    if cap.isOpened():
        window_handle = cv2.namedWindow("CSI Camera", cv2.WINDOW_AUTOSIZE)
        # Window
        while cv2.getWindowProperty("CSI Camera", 0) >= 0:
            ret_val, img = cap.read()
            if(blue_or_red == color_red):
                lower_red = np.array([0, 150, 50])
                upper_red = np.array([5, 255, 155])
                lower_red2 = np.array([161, 155, 84])
                upper_red2 = ur2 = np.array([180, 255, 255])
                hsv_frame = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
                mask1 = cv2.inRange(hsv_frame, lower_red2, upper_red2)
                mask2 = cv2.inRange(hsv_frame, lower_red, upper_red)
                mask = cv2.bitwise_or(mask1, mask2)
            elif(blue_or_red == color_blue):
                lb1 = np.array([90, 0, 50])
                ub1 = np.array([131, 255, 255])
                hsv_frame = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
                mask = cv2.inRange(hsv_frame, lb1, ub1)
            #cv2.imshow("CSI Camera", mask)
            M = cv2.moments(mask) #moment is centroid
            try:
                cx,cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
            except:
                cx,cy = 0,0
            
            

            #cv2.circle(img,(cx,cy),5,(0,0,255),-1)
            #cv2.imshow("Real", img)
            area = cv2.countNonZero(mask)
            #print(area)
            # This also acts as
            keyCode = cv2.waitKey(30) & 0xFF
            # Stop the program on the ESC key
            if keyCode == 27:
                break
        cap.release()
        cv2.destroyAllWindows()
    else:
        print("Unable to open camera")


#Create drone
#
drone = Kusbegi()
#drone.connection_string = '127.0.0.1:14540'
if not (drone.connect_vehicle()):
    exit()
#
#End of Create drone
cam_th = threading.Thread(target=show_camera)
cam_th.start()

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
MIN_AREA = 500
RED_PUMP_OUT_HEIGHT = -3
MISSION_PUMP_TIMEOUT = 10 #Seconds
MISSION_PUMP_TIMEOUT_OUT = 15 #Seconds , disari basma suresi
MISSION_MAX_WATER_SENSOR = 450 # ADC max value to stop motors
MISSION_ALTITUDE = -10 #Down (meters) #Min 4 - max 30 mt
MISSION_ALTITUDE_RED = -6 #After pool
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
drone.position_frame = drone.frame_local_ned
drone.ready_to_takeoff()

drone.mode_takeoff(MISSION_ALTITUDE) #Takeoff to MISSION_ALTITUDE at MISSION_COORDINATE_HOME
drone.default_alt_global = drone.vehicle.location.global_frame.alt
#drone.go_to_coordinate(MISSION_COORDINATE_FINISH)

drone.go_to_coordinate(MISSION_COORDINATE_RALLY1)

#drone.go_to_coordinate(MISSION_COORDINATE_POOL)
"""
drone.drive_type = drone.drive_w_setpnt
drone.position_frame = drone.frame_local_ned
drone.req_pos_x = drone.pos_x
drone.req_pos_y = drone.pos_y
drone.req_pos_z = drone.pos_z

sleep(2)
drone.req_pos_z = MISSION_ALTITUDE_RED


while (abs(drone.pos_z - drone.req_pos_z ) > 0.2):    
    sleep(0.2)

drone.default_alt_global =drone.vehicle.location.global_frame.alt
"""
drone.go_to_coordinate(MISSION_COORDINATE_RALLY2)
#yaw icin biraz bekle


#drone.go_to_coordinate_dont_wait(MISSION_COORDINATE_RALLY3)
drone.drive_type = drone.drive_w_speed
drone.position_frame = drone.frame_local_ned
drone.req_pos_x = 0
drone.req_pos_y = 0
drone.req_pos_z = 0

forward_speed = 1
sleep(1)

ned_n,ned_e = drone.body_to_ned_frame(forward_speed,0,drone.yaw_ang)

drone.req_pos_x = ned_n
drone.req_pos_y = ned_e

drone.distance_tolerance = 9
max_area = 0
blue_or_red = color_red


while True:
    if not (drone.at_the_target_yet_global(37.0749168,37.276977)):
        drone.log.logger("Not at the target yet with global frame")
            
        if(area > max_area):
            max_area = area
            drone.save_coordinate(MISSION_COORDINATE_RED_AREA)
    else:
        break

drone.distance_tolerance = 1

drone.go_to_coordinate(MISSION_COORDINATE_RALLY4)

#drone.go_to_coordinate(MISSION_COORDINATE_HOME)


#START TOUR 2
#drone.go_to_coordinate(MISSION_COORDINATE_FINISH)

drone.go_to_coordinate(MISSION_COORDINATE_RALLY1)

drone.go_to_coordinate(MISSION_COORDINATE_POOL)


blue_or_red = color_blue

drone.position_frame = drone.frame_local_ned
drone.drive_type = drone.drive_w_speed

drone.req_pos_x = 0
drone.req_pos_y = 0
drone.req_pos_z = 0

# 640,360
sleep(1)
while True:
    
    if(area < MIN_AREA):
        drone.req_pos_x = 0
        drone.req_pos_y = 0
        drone.req_pos_z = 0
        continue
    
    forward = 180 - cy
    right = cx - 320

     

    dist_V_pixs = 0.0025 #* 2 * (-1) * drone.pos_z * math.tan(math.radians(48.8 / 2)) / 360
    dist_H_pixs = 0.0025 #* 2 * (-1) * drone.pos_z * math.tan(math.radians(62.2 / 2)) / 640

    northgo, eastgo = drone.body_to_ned_frame(dist_V_pixs*forward, dist_H_pixs*right, drone.vehicle.attitude.yaw)
    print("north: ",northgo, "-  east: ", eastgo)
    if abs(northgo)*0.7 <= (0.08):
        northgo = 0.0
    if abs(eastgo)*0.7 <= (0.08):
        eastgo = 0.0
    drone.req_pos_x = 0.7*northgo
    drone.req_pos_y = 0.7*eastgo
    drone.req_pos_z = 0.15

    if drone.pos_z >= -4:
        drone.req_pos_x = 0
        drone.req_pos_y = 0
        drone.req_pos_z = 0
        break


drone.mode_land()

drone.wait_for_land()

pump = Pump()

#sens = WaterLevelSensor()

print("Start pumping")
drone.log.logger("Start pumping")

ts = time()

while ( (time()- ts) < MISSION_PUMP_TIMEOUT ):
    pump.pump_water_in()
    sleep(0.1)
    drone.log.logger("pumping in")
    #sens_value = sens.readadc(sens.photo_ch, sens.SPICLK, sens.SPIMOSI, sens.SPIMISO, sens.SPICS)
    #print(sens_value)
    """
    if (sens_value > MISSION_MAX_WATER_SENSOR):
        print("Water level reached!")
        drone.log.logger("Water level reached!")
        break
    """

print("Stop pumping")
drone.log.logger("Stop pumping")

pump.close_and_clean()
#sens.close_and_clean()

###
### buraya olduğu yerde takeoff 
### --- go_to deyince çapraz gidiyor, 
### takılmamız yan yatmamız olası.......
### hatta su üzerinde takeoff hızını elektronikleri
### korumak için azaltabiliriz de
###
drone.req_pos_x = drone.vehicle.location.local_frame.north
drone.req_pos_y = drone.vehicle.location.local_frame.east
drone.req_pos_z = drone.vehicle.location.local_frame.down

drone.drive_type = drone.drive_w_setpnt
drone.position_frame = drone.frame_local_ned

#drone.ready_to_takeoff()
drone.mode_offboard()

drone.req_pos_z = MISSION_ALTITUDE_RED

while (abs(drone.pos_z - drone.req_pos_z ) > 0.2):
    print("     Altitude: ", drone.pos_z, " meter")
    drone.log.logger("     Altitude: "+ str(drone.pos_z) + " meter")
    sleep(1)
drone.log.logger("Target altitude reached : " + str(drone.req_pos_z))


drone.go_to_coordinate(MISSION_COORDINATE_RED_AREA)

blue_or_red = color_red

drone.position_frame = drone.frame_local_ned
drone.drive_type = drone.drive_w_speed

drone.req_pos_x = 0
drone.req_pos_y = 0
drone.req_pos_z = 0

# 640,360
sleep(1)

while True:
    
    if(area < MIN_AREA):
        drone.req_pos_x = 0
        drone.req_pos_y = 0
        drone.req_pos_z = 0
        continue
    
    forward = 180 - cy
    right = cx - 320

     

    dist_V_pixs = 0.0025 #* 2 * (-1) * drone.pos_z * math.tan(math.radians(48.8 / 2)) / 360
    dist_H_pixs = 0.0025 #* 2 * (-1) * drone.pos_z * math.tan(math.radians(62.2 / 2)) / 640

    northgo, eastgo = drone.body_to_ned_frame(dist_V_pixs*forward, dist_H_pixs*right, drone.vehicle.attitude.yaw)
    print("north: ",northgo, "-  east: ", eastgo)
    if abs(northgo)*0.7 <= (0.08):
        northgo = 0.0
    if abs(eastgo)*0.7 <= (0.08):
        eastgo = 0.0
    drone.req_pos_x = 0.7*northgo
    drone.req_pos_y = 0.7*eastgo
    drone.req_pos_z = 0.15

    if drone.pos_z >= RED_PUMP_OUT_HEIGHT:
        drone.position_frame = drone.frame_local_ned
        drone.drive_type = drone.drive_w_setpnt
        drone.req_pos_x = drone.pos_x
        drone.req_pos_y = drone.pos_y
        drone.req_pos_z = RED_PUMP_OUT_HEIGHT
        break



#Descend and pump water out
pump2 = Pump()

ts = time()

while( (time()- ts) < MISSION_PUMP_TIMEOUT_OUT ):
    pump2.pump_water_out()
    sleep(0.5)
    print("pumping in")
    """
    sens_value = sens.readadc(sens.photo_ch, sens.SPICLK, sens.SPIMOSI, sens.SPIMISO, sens.SPICS)
    print(sens_value)
    """

print("Stop pumping")
pump2.close_and_clean()

drone.req_pos_z = MISSION_ALTITUDE

while (abs(drone.pos_z - drone.req_pos_z ) > 0.2):
    print("     Altitude: ", drone.pos_z, " meter")
    drone.log.logger("     Altitude: "+ str(drone.pos_z) + " meter")
    sleep(1)
drone.log.logger("Target altitude reached : " + str(drone.req_pos_z))

drone.default_alt_global= drone.vehicle.location.global_frame.alt


drone.go_to_coordinate(MISSION_COORDINATE_RALLY4)

#drone.go_to_coordinate(MISSION_COORDINATE_HOME)

#GO TO FINISH
drone.go_to_coordinate(MISSION_COORDINATE_FINISH)

#LAND!
drone.mode_land()

drone.wait_for_land()

drone.disarm()

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
