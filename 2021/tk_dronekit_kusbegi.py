from dronekit import connect, LocationLocal, VehicleMode, Battery, SystemStatus, LocationGlobalRelative
from pymavlink import mavutil
import threading
from time import sleep
import math
from utility import *



class Kusbegi:
    def __init__(self, connection_string=None):
        self.connection_string = "/dev/ttyTHS1"
        self.vehicle = None
        self.loc_home = None
        self.home_yaw = None
        self.default_alt = None
        self.default_alt_global = None

        self.pos_x = 0.0
        self.pos_y = 0.0
        self.pos_z = 0.0

        self.yaw_ang = 0.0 # (rad)

        self.state_on_ground = True
        self.home_position_set = False
        
        self.batt_lvl = None
        self.batt_volt = None
        # self.batt_lvl_critical =  # ??????
        # self.batt_volt_critical =  # ?????

        self.distance_tolerance = 1 #meter
        self.distance_tolerance_global = self.distance_tolerance / 1.113195e5
        self.circle_step_magnitude = 0.10

        self.spd_x = 0.0
        self.spd_y = 0.0
        self.airspd = 0.0

        # do not forget to restore default 0 (m) after use
        self.req_pos_x = 0.0
        self.req_pos_y = 0.0
        self.req_pos_z = 0.0

        # do not forget to restore default 0 (rad) after use
        self.req_yaw_ang = 0.0
        
        self.drive_w_speed = 0b101111000111
        self.drive_w_setpnt = 0b101111111000
        self.frame_local_ned = mavutil.mavlink.MAV_FRAME_LOCAL_NED
        self.frame_global_relative_alt = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
        #self.frame_global = mavutil.mavlink.MAV_FRAME_GLOBAL
        
        self.drive_type = self.drive_w_setpnt
        self.position_frame = self.frame_local_ned



        self.should_stop_thread = False
        self.pauseOffboard = False

        self.msg_thread = None

        self.log = Log()
        self.log.logger("Logger initialized")

        self.coordinate = CoordinateFile()




    def connect_vehicle(self):
        #Try connect vehicle

        try:
            self.vehicle = connect(
                self.connection_string, baud=57600)

            while self.vehicle.location.local_frame is None:
                sleep(1)
            print("Home = ", self.vehicle.location.local_frame)
            print("Connected on: ", self.connection_string)
            self.log.logger("Home = " + str(self.vehicle.location.local_frame))
            self.log.logger("Connected on: " + self.connection_string)
            return True

        except RuntimeError:
            self.vehicle = None
            print("Unable to connect vehicle!")
            print("Connection_string: ", self.connection_string)
            self.log.logger("Unable to connect vehicle!")
            self.log.logger("Connection_string: " + self.connection_string)
            self.log.write_logs()
            return False

    def updateParams(self):
        #Update vehicle position and paramters

        self.pos_x = self.vehicle.location.local_frame.north
        self.pos_y = self.vehicle.location.local_frame.east
        self.pos_z = self.vehicle.location.local_frame.down

        self.yaw_ang = self.vehicle.attitude.yaw

        #self.airspd = round(self.vehicle.groundspeed, 2)
        #self.spd_x = self.vehicle._vx
        #self.spd_y = self.vehicle._vy
        
        #self.batt_lvl = self.vehicle._level
        #self.batt_volt = self.vehicle._voltage

    def reposition(self,position,yaw):
        self.vehicle._master.mav.command_long_send(
                1,  # autopilot system id
                1,  # autopilot component id
                192,  # command id, reposition
                0,  # confirmation
                -1,  # ground speed -1 for default
                1, # MAV_DO_REPOSITION_FLAGS
                0, #reserved
                yaw, 
                position.lat, 
                position.lon, 
                position.alt
            )

    def arm(self):
        #Try arm

        self.log.logger("Try Arm")

        while not self.home_position_set:
                print("Arm error: Home position not set!")
                self.log.logger("Home position not set!")
                sleep(1)

        while self.vehicle.armed == False:
            self.vehicle._master.mav.command_long_send(
                1,  # autopilot system id
                1,  # autopilot component id
                400,  # command id, ARM/DISARM
                0,  # confirmation
                1,  # arm!
                0, 0, 0, 0, 0, 0  # unused parameters for this command
            )
            print("Trying arming")
            self.log.logger("Trying arming")
            sleep(0.3)

        print("VEHICLE ARMED!")
        self.log.logger("VEHICLE ARMED!")

        if self.vehicle.armed:
            while self.vehicle.location.local_frame is None:
                sleep(1)
            self.loc_home = self.vehicle.location.local_frame.north, self.vehicle.location.local_frame.east, self.vehicle.location.local_frame.down
            self.req_yaw_ang = self.yaw_ang
            self.home_yaw = self.yaw_ang
            print("Home Location" + str(self.loc_home))
            self.log.logger("Home Location" + str(self.loc_home))
   
    def disarm(self):
        while self.vehicle.armed == True:
            self.vehicle._master.mav.command_long_send(
                1,  # autopilot system id
                1,  # autopilot component id
                400,  # command id, ARM/DISARM
                0,  # confirmation
                0,  # Disarm!
                0, 0, 0, 0, 0, 0  # unused parameters for this command
            )
            print("Trying Disarming")
            self.log.logger("Trying Disarming")
            sleep(0.3)

        print("VEHICLE DISARMED!")
        self.log.logger("VEHICLE DISARMED!")

        
    def send_message(self):
        

        # time_boot_ms (not used)
        # target_system, target_component
        # frame
        # type_mask (only speeds enabled)
        # x, y, z NED positions (m)
        # x, y, z NED velocity (m/s)
        # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        # yaw(rad), yaw_rate(rad/s)
        while True:
            if(self.pauseOffboard):
                sleep(1) #wait 1 sec water resume offboard
                continue
            msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
                0,
                0, 0,
                self.position_frame,
                self.drive_type,
                self.req_pos_x, self.req_pos_y, self.req_pos_z, # NED!
                self.req_pos_x, self.req_pos_y, self.req_pos_z, # NED!
                0, 0, 0,
                self.req_yaw_ang, 0)

            # send command to vehicle
            self.vehicle.send_mavlink(msg)
            self.vehicle.flush()
            self.updateParams()
            sleep(0.05)
            if(self.should_stop_thread == True):
                print("Thread killed")
                self.log.logger("Thread killed")
                break
     
    def mode_takeoff(self, req_height):
        #Take off to desired altitude

        if self.vehicle.armed == False:
            self.arm()

        print("Initialising: TAKEOFF")
        self.log.logger("Initialising: TAKEOFF")
        self.req_pos_z = req_height
        while (abs(self.pos_z - req_height ) > 0.2):
            print("     Altitude: ", self.pos_z, " meter")
            self.log.logger("     Altitude: "+ str(self.pos_z) + " meter")
            sleep(1)
        self.log.logger("Target altitude reached : " + str(req_height))

    def mode_offboard(self):
        self.set_mode("OFFBOARD")

    def set_mode(self,TargetMode):
        self.log.logger("-> " + TargetMode)
        print(self.vehicle.mode, "-> " + TargetMode)
        while self.vehicle.mode != TargetMode:
            self.vehicle._master.set_mode_px4(TargetMode, None, None)
            self.log.logger("Trying " + TargetMode)
            sleep(0.3)

        print(self.vehicle.mode)
        print("Mode " +TargetMode +"!")
        self.log.logger("Mode " +TargetMode +"!")

    def mode_land(self):
        self.set_mode("LAND")

    def wait_for_land(self):
        #Wait for land detected

        self.log.logger("Landing...")
        print ("Landing...")
        while True:
            self.log.logger("Waiting for land detected")
            if (self.state_on_ground == True):
                self.log.logger("Land detected!")
                break
            sleep(1)
        print("Landed!")

        self.print_status()

        return True

    def pause_offboard(self):
        self.pause_offboard = True

    def continue_offboard(self):
        self.pause_offboard = False

    # düzelt
    def start_drive(self):
        # a thread to send messages

        #Benim thread koduma bakılabilir
        self.msg_thread = threading.Thread(target=self.send_message)
        self.msg_thread.start()
        self.log.logger("Burkut setpoint buffer thread activated")
        print("Burkut setpoint buffer thread activated")

    def stop(self):
        #Dangerous method to stop ofboard thread!

        self.log.logger("Setpoint buffer STOP!")
        self.should_stop_thread = True

    def print_status(self):
        #   Display basic vehicle state

        self.log.logger("Print Status and write logs")
        self.log.logger(" Armed: " + str(self.vehicle.armed))
        self.log.logger(" System status: " + str(self.vehicle.system_status.state))
        self.log.logger(" GPS: " + str(self.vehicle.gps_0))
        self.log.logger(" Alt: " + str(self.vehicle.location.global_relative_frame.alt))
        print (" Armed: %s" % self.vehicle.armed)
        print (" System status: %s" % self.vehicle.system_status.state)
        print (" GPS: %s" % self.vehicle.gps_0)
        print (" Alt: %s" % self.vehicle.location.global_relative_frame.alt)
        self.log.write_logs()

    def ready_to_takeoff(self):
        #Get ready to take off!

        self.log.logger("Ready to takeoff!")

        self.drive_type = self.drive_w_setpnt
        self.print_status()
        self.start_drive()
        self.arm()
        self.mode_offboard()
        self.print_status()
        return True

    def go_to_coordinate(self,filename):
        #Go to coordinate location with file name and wait for it

        self.coordinate.file_dir = filename
        lat,lon,yaw = self.coordinate.read_coordinates()

        self.log.logger("Go to coordinate with file name = " + filename)
        target = self.vehicle.location.global_frame
        target.lat = lat
        target.lon = lon
        self.reposition(target,yaw)

        self.log.logger("Target : " + str(lat) + " , " + str(lon))

        while not (self.at_the_target_yet_global(lat,lon)):
            self.log.logger("Not at the target yet with global frame")
            sleep(1)
        
        self.log.logger("Target reached : " + str(self.vehicle.location.global_frame.lat) + " , " + str(self.vehicle.location.global_frame.lon))
        self.print_status()

        return True
    
    def get_distance_metres(self,aLocation1, aLocation2):
        #Get distance between 2 global location coordinate in meters

        self.log.logger("Get distance between 2 coordinates:")

        self.coordinate.file_dir = aLocation1
        lat_aLocation1, lon_aLocation1 = self.coordinate.read_coordinates(aLocation1)
        self.log.logger("First coordinate: " + str(lat_aLocation1) + ", " + str(lon_aLocation1))

        self.coordinate.file_dir = aLocation2
        lat_aLocation2, lon_aLocation2 = self.coordinate.read_coordinates(aLocation2)
        self.log.logger("Second coordinate: " + str(lat_aLocation2) + ", " + str(lon_aLocation2))

        dlat = lat_aLocation2 - lat_aLocation1
        dlong = lon_aLocation2 - lon_aLocation1
        result = math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5
        self.log.logger("Distance is " + str(result) + " meters" )
        return result

    def body_to_ned_frame(self,xBody,yBody,yawBody):
        #Convert body frame to ned frame

        xNed =  (xBody * math.cos(yawBody) ) - ( yBody * math.sin(yawBody) )
        yNed =  (xBody * math.sin(yawBody) ) + ( yBody * math.cos(yawBody) )

        return xNed,yNed

    def at_the_target_yet_global(self,xTarget,yTarget):
        #Wait for reaching target with global frame

        if (abs(xTarget-self.vehicle.location.global_relative_frame.lat) < self.distance_tolerance_global):
            if(abs(yTarget-self.vehicle.location.global_relative_frame.lon) < self.distance_tolerance_global):
                print("Target point reached")
                self.log.logger("Target point reached")
                return True
        return False


    def save_coordinate(self,file_name):
        #For script to save coorinates and yaw
        lat = self.vehicle.location.global_relative_frame.lat
        lon = self.vehicle.location.global_relative_frame.lon
        yaw = self.vehicle.attitude.yaw

        self.log.logger("Save lat,lon,yaw to " + file_name + " as:")
        self.log.logger("Lat-" + str(lat) + " lon-" + str(lon) + " yaw-" + str(yaw))
        
        self.coordinate.file_dir = file_name
        self.coordinate.write_coordinates(lat, lon, yaw)

        return True

   

    def go_to_coordinate_dont_wait(self,filename):
        self.coordinate.file_dir = filename
        lat,lon,yaw = self.coordinate.read_coordinates()

        self.log.logger("Go to coordinate with file name = " + filename)
        target = self.vehicle.location.global_frame
        target.lat = lat
        target.lon = lon
        self.reposition(target,yaw)

        self.log.logger("Target : " + str(lat) + " , " + str(lon))

        return True
