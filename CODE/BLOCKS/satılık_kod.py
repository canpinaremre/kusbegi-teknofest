        self.drive_w_setpnt = 0b101111111000
        self.frame_local_ned = mavutil.mavlink.MAV_FRAME_LOCAL_NED


        
        self.distance_tolerance = 1 #meter
        self.distance_tolerance_global = self.distance_tolerance / 1.113195e5

                self.drive_type = self.drive_w_setpnt
        self.position_frame = self.frame_local_ned

         self.should_stop_thread = False


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
            msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
                0,
                0, 0,
                self.position_frame,
                self.drive_type,
                self.req_pos_x, self.req_pos_y, self.req_pos_z, # NED!
                0, 0, 0, # NED!
                0, 0, 0,
                self.req_yaw_ang, 0)

            if (self.position_frame == self.frame_global_relative_alt):
                msg = self.vehicle.message_factory.set_position_target_global_int_encode(
                    0,
                    0, 0,
                    mavutil.mavlink.MAV_FRAME_GLOBAL,
                    0b101111111000,
                    int(self.req_pos_x*1e7), int(self.req_pos_y*1e7), self.default_alt_global, # NED!
                    0, 0, 0, # NED!
                    0, 0, 0,
                    self.req_yaw_ang, 0)

            # send command to vehicle
            self.vehicle.send_mavlink(msg)
            self.vehicle.flush()
            self.updateParams()
            sleep(0.3)
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
        #Try offboard mode

        self.log.logger("Try mode OFFBOARD")
        print(self.vehicle.mode, "-> OFFBOARD")
        while self.vehicle.mode != "OFFBOARD":
            self.vehicle._master.set_mode_px4("OFFBOARD", None, None)
            self.log.logger("Waiting offboard mode")
            sleep(0.3)
        print(self.vehicle.mode)
        self.log.logger("Mode : OFFBOARD!")

    def mode_land(self):
        #Try land mode

        self.log.logger("-> LAND")
        print(self.vehicle.mode, "-> LAND")
        while self.vehicle.mode != "LAND":
            self.vehicle._master.set_mode_px4("LAND", None, None)
            self.log.logger("Trying land mode")
            sleep(0.3)

        print(self.vehicle.mode)
        print("Mode Land!")
        self.log.logger("Mode Land!")

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
        x,y,yaw = self.coordinate.read_coordinates()

        self.log.logger("Go to coordinate with file name = " + filename)

        self.position_frame = self.frame_global_relative_alt
        self.drive_type = self.drive_w_setpnt
        self.default_alt_global = self.vehicle.location.global_frame.alt
        #self.vehicle.groundspeed = 0.5
        #self.vehicle.airspeed = 0.5

        self.req_pos_x = x
        self.req_pos_y = y
        self.req_pos_z = self.default_alt_global
        self.req_yaw_ang = yaw

        self.log.logger("Target : " + str(x) + " , " + str(y))

        while not (self.at_the_target_yet_global(x,y)):
            self.log.logger("Not at the target yet with global frame")
            sleep(1)
        
        self.log.logger("Target reached : " + str(self.vehicle.location.global_frame.lat) + " , " + str(self.vehicle.location.global_frame.lon))
        self.print_status()

        return True

    def go_to_location(self,xTarget,yTarget,altTarget,yaw,frame):
        #Go to location with frame type, x, y, z, yaw and wait for it

        self.log.logger("Go to location with frame type, x, y, z, yaw and wait for it")
        print("Go to location with frame type, x, y, z, yaw and wait for it")#DEBUG
        self.position_frame = frame

        self.req_pos_x = xTarget
        self.req_pos_y = yTarget
        self.req_pos_z = altTarget
        self.req_yaw_ang = yaw
        self.log.write_logs() #DEBUG
        if (frame == self.frame_global_relative_alt):
            while not (self.at_the_target_yet_global(xTarget,yTarget)):
                self.log.logger("Waiting to reach target with global frame")
                print("Waiting to reach target with global frame") #DEBUG
                sleep(1)
        
        if (frame == self.frame_local_ned):
            while not (self.at_the_target_yet_ned(xTarget,yTarget)):
                self.log.logger("Waiting to reach target with ned frame")
                self.log.write_logs() #DEBUG
                sleep(1)

        self.log.logger("Target reached")

        return True




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

    def at_the_target_yet_ned(self,xTarget,yTarget):
        #Wait for reaching target with NED frame

        if (abs(xTarget-self.pos_x) < self.distance_tolerance):
            if(abs(yTarget-self.pos_y) < self.distance_tolerance):
                print("Target point reached")
                self.log.logger("Target point reached")
                return True
        return False



