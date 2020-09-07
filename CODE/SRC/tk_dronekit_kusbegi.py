from dronekit import connect, LocationLocal, VehicleMode, Battery, SystemStatus
from pymavlink import mavutil
import threading
from time import sleep
import math

# add to main connection_string = "/dev/ttyTHS1"


class Kusbegi:
    def __init__(self, connection_string=None):
        self.connection_string = "/dev/ttyTHS1"
        self.vehicle = None
        self.loc_home = None
        self.home_yaw = None

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

        self.distance_tolerance = 0.5 #meter

        self.spd_x = 0.0
        self.spd_y = 0.0
        self.airspd = 0.0

        # do not forget to restore default 0 (m) after use
        self.req_pos_x = 0.0
        self.req_pos_y = 0.0
        self.req_pos_z = 0.0

        # do not forget to restore default 0 (rad) after use
        self.req_yaw_ang = 0.0
        
        #self.drive_w_speed = 0b101111000111
        self.drive_w_setpnt = 0b101111111000
        self.frame_local_ned = mavutil.mavlink.MAV_FRAME_LOCAL_NED
        self.frame_global_relative_alt = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
        self.frame_global = mavutil.mavlink.MAV_FRAME_GLOBAL
        
        self.drive_type = self.drive_w_setpnt
        self.position_frame = self.frame_local_ned



        self.should_stop_thread = False

        self.msg_thread = None




    def connect_vehicle(self):
        try:
            self.vehicle = connect(
                self.connection_string, baud=57600)
            while self.vehicle.location.local_frame is None:
                sleep(1)
            print("Home = ", self.vehicle.location.local_frame)
            print("Connected on: ", self.connection_string)

        except RuntimeError:
            self.vehicle = None
            print("Unable to connect vehicle!")
            print("Connection_string: ", self.connection_string)
            return

    def updateParams(self):
        self.pos_x = self.vehicle.location.local_frame.north
        self.pos_y = self.vehicle.location.local_frame.east
        self.pos_z = self.vehicle.location.local_frame.down

        self.yaw_ang = self.vehicle.attitude.yaw

        self.airspd = round(self.vehicle.groundspeed, 2)
        self.spd_x = self.vehicle._vx
        self.spd_y = self.vehicle._vy
        
        self.batt_lvl = self.vehicle._level
        self.batt_volt = self.vehicle._voltage

    def arm(self):
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
            sleep(0.3)

        print("VEHICLE ARMED!")

        if self.vehicle.armed:
            while self.vehicle.location.local_frame is None:
                sleep(1)
            self.loc_home = self.vehicle.location.local_frame.north, self.vehicle.location.local_frame.east, self.vehicle.location.local_frame.down
            self.req_yaw_ang = self.yaw_ang
            self.home_yaw = self.yaw_ang
            print("Home Location", self.loc_home)
   
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
            sleep(0.3)

        print("VEHICLE DISARMED!")

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
            msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
                0,
                0, 0,
                self.position_frame,
                self.drive_type,
                self.req_pos_x, self.req_pos_y, self.req_pos_z, # NED!
                0, 0, 0, # NED!
                0, 0, 0,
                self.req_yaw_ang, 0)
            # send command to vehicle
            self.vehicle.send_mavlink(msg)
            self.vehicle.flush()
            self.updateParams()
            sleep(0.3)
            if(self.should_stop_thread == True):
                break

    #düzelt      
    def mode_takeoff(self, req_height):
        if self.vehicle.armed == False:
            self.arm()

        print("Initialising: TAKEOFF")
        self.req_pos_z = req_height
        while self.pos_z > req_height * 0.95:
            print("     Altitude: ", self.pos_z, " meter")
            sleep(1)

    def mode_offboard(self):

        print(self.vehicle.mode, "-> OFFBOARD")
        while self.vehicle.mode != "OFFBOARD":
            self.vehicle._master.set_mode_px4("OFFBOARD", None, None)
            print("Trying offboard")
            sleep(0.3)
        print(self.vehicle.mode)

    def mode_land(self):

        print(self.vehicle.mode, "-> LAND")
        while self.vehicle.mode != "LAND":
            self.vehicle._master.set_mode_px4("LAND", None, None)
            sleep(0.3)

        print(self.vehicle.mode)
        print("LANDING")
        
    def wait_for_land(self):
        while True:
            print ("Landing...")
            if (self.state_on_ground == True):
                break
            sleep(1)       
        return True

    # düzelt
    def start_drive(self):
        # a thread to send messages
        #Benim thread koduma bakılabilir
        self.msg_thread = threading.Thread(target=self.send_message)
        self.msg_thread.start()
        print("Burkut setpoint buffer thread activated")

    def stop(self):
        #Dangerous method to stop ofboard thread!
        self.should_stop_thread = True

    #Düzelt -- Log basma ve buffer temizleme bu fonksiyona eklenecek
    def print_status(self):
        #Elif utility.py bekleniyor
        #   Display basic vehicle state
        print (" Type: %s" % self.vehicle._vehicle_type)
        print (" Armed: %s" % self.vehicle.armed)
        print (" System status: %s" % self.vehicle.system_status.state)
        print (" GPS: %s" % self.vehicle.gps_0)
        print (" Alt: %s" % self.vehicle.location.global_relative_frame.alt)

    def ready_to_takeoff(self):
        #Get ready to take off!
        self.drive_type = self.drive_w_setpnt
        self.print_status()
        self.start_drive()
        self.arm()
        self.mode_offboard()
        self.print_status()
        return True

    #düzelt
    def go_to_location(xTarget,yTarget,altTarget):

        return True

    
    def body_to_ned_frame(self,xBody,yBody,yawBody):
        xNed =  (xBody * math.cos(yawBody) ) - ( yBody * math.sin(yawBody) )
        yNed =  (xBody * math.sin(yawBody) ) + ( yBody * math.cos(yawBody) )
        return xNed,yNed

    #düzelt
    def at_the_target_yet(self,xTarget,yTarget,zTarget):
        xTarget,yTarget = self.bodyToNedFrame(xTarget,yTarget,self.home_yaw)
        zTarget = -zTarget

        north = self.vehicle.location.local_frame.north
        east = self.vehicle.location.local_frame.east
        down = self.vehicle.location.local_frame.down
        if (abs(xTarget-north) < self.distance_tolerance):
            if(abs(yTarget-east) < self.distance_tolerance):
                if (abs(zTarget-down) < self.distance_tolerance):
                    print("Target point reached")#add x y z
                    return True
        print("Not at target yet")
        return False
    
    #Düzelt
    def calculate_distance(self,globalCoordinates1,globalCoordinates2):
        #This will return the distance of 2 global coordiante in meters
        #Math!
        return True

    #Düzelt
    def wait_for_circle(self,bottomCoordinates,radius,yaw_bottom_to_top):
        #Math!
        self.go_to_location(bottomCoordinates)
        
        while (True):
            self.req_pos_x = self.pos_x

            if(False):
                break
        return True

    
    def do_circle(self,bottomCoordinates,topCoordinates,radius,yaw_bottom_to_top):
        if(radius != 0):
            self.wait_for_circle(bottomCoordinates,radius,yaw_bottom_to_top)
        else:
            radius = self.calculate_distance(bottomCoordinates,topCoordinates,yaw_bottom_to_top)
            self.wait_for_circle(bottomCoordinates,radius)

        return True

    def save_coordinate(self,file_name):
        #For script to save coorinates and yaw
        #Elifin utility.py bekleniyor

        return True

