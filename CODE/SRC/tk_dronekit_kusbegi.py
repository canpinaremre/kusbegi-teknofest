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
        self.default_alt = None

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
        self.circle_step_magnitude = 0.001

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
        #self.frame_global = mavutil.mavlink.MAV_FRAME_GLOBAL
        
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

    def go_to_location(self,xTarget,yTarget,altTarget,yaw,frame):
        #Go to location with frame type and wait for it
        self.position_frame = frame
        self.req_pos_x = xTarget
        self.req_pos_y = yTarget
        self.req_pos_z = altTarget
        self.req_yaw_ang = yaw

        if (frame == self.frame_global_relative_alt):
            while not (self.at_the_target_yet_global(xTarget,yTarget)):
                sleep(1)
        
        if (frame == self.frame_local_ned):
            while not (self.at_the_target_yet_ned(xTarget,yTarget)):
                sleep(1)
        
        return True
    
    def get_distance_metres(aLocation1, aLocation2):
        #Global locaiton meters
        dlat = aLocation2.lat - aLocation1.lat
        dlong = aLocation2.lon - aLocation1.lon
        return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

    def body_to_ned_frame(self,xBody,yBody,yawBody):
        xNed =  (xBody * math.cos(yawBody) ) - ( yBody * math.sin(yawBody) )
        yNed =  (xBody * math.sin(yawBody) ) + ( yBody * math.cos(yawBody) )
        return xNed,yNed

    def at_the_target_yet_global(self,xTarget,yTarget):

        if (abs(xTarget-self.vehicle.location.global_relative_frame) < self.distance_tolerance_global):
            if(abs(yTarget-self.vehicle.location.global_relative_frame) < self.distance_tolerance_global):
                print("Target point reached")
                return True
        print("Not at target yet")
        return False

    def at_the_target_yet_ned(self,xTarget,yTarget):

        if (abs(xTarget-self.pos_x) < self.distance_tolerance):
            if(abs(yTarget-self.pos_y) < self.distance_tolerance):
                print("Target point reached")#add x y z
                return True
        print("Not at target yet")
        return False

    #Düzelt
    def wait_for_circle(self,bottomCoordinates,radius,yaw_bottom_to_top):
        #Math!
        
        x,y = 0,0 #Elif utilt.py


        ts_yaw = yaw_bottom_to_top + math.pi/2
        self.go_to_location(x,y,self.default_alt,ts_yaw,self.frame_global_relative_alt)

        self.drive_type = self.drive_w_setpnt
        self.position_frame = self.frame_local_ned
        self.req_pos_x = self.pos_x
        self.req_pos_y = self.pos_y
        self.req_pos_z = self.default_alt

        ts_x = self.pos_x
        ts_y = self.pos_y
        ts_z = self.default_alt
        


        alfa = 0

        while (alfa < 360):
            sleep(0.01)
            alfa = alfa + self.circle_step_magnitude
            x_ned, y_ned = body_to_ned_frame( (ts_x - (radius*math.sin(alfa)/2)),(ts_y + radius/2 - radius*math.cos(alfa)/2) ,yaw_bottom_to_top )
            self.req_pos_x = x_ned
            self.req_pos_y = y_ned
            self.req_pos_z = self.default_alt
            self.req_yaw_ang = ts_yaw + math.pi/2 - alfa

            if(alfa >= 360):
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

