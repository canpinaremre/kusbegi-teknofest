def FRD_NED(right_err, yaw_ned, altitude, fwd_err):

        # calculation of distance between 2 pixels in real world
        # iaw camera fov features and altitude
        dist_V_pixs = 2 * (-1) * altitude * math.tan(math.radians(cam_V_fov / 2)) / DISPLAY_HEIGHT
        dist_H_pixs = 2 * (-1) * altitude * math.tan(math.radians(cam_H_fov / 2)) / DISPLAY_WIDTH

        if yaw_ned == 0:
            yaw_ned = 0.001
        north_body = ((-1)*(right_err * dist_H_pixs) * math.sin(yaw_ned)) + ((fwd_err) * dist_V_pixs) * math.cos(yaw_ned)
        east_body = ((right_err * dist_H_pixs) * math.cos(yaw_ned)) + (((fwd_err) * dist_V_pixs) * math.sin(yaw_ned))

        north = north_body
        east = east_body

        return north, east
