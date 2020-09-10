
import threading
import cv2
import numpy as np
from csi_camera import CSI_Camera
from time import perf_counter, sleep
import math


show_fps = True


def new_red_area_detected(pixel):
    print("New red area detected : ",pixel)#WRITE COORDİNATES


show_fps = True

# Simple draw label on an image; in our case, the video frame
def draw_label(cv_image, label_text, label_position):
    font_face = cv2.FONT_HERSHEY_SIMPLEX
    scale = 0.5
    color = (255, 255, 255)
    # You can get the size of the string with cv2.getTextSize here
    cv2.putText(cv_image, label_text, label_position,
                font_face, scale, color, 1, cv2.LINE_AA)

# Read a frame from the camera, and draw the FPS on the image if desired
# Return an image
def read_camera(csi_camera, display_fps):
    _, camera_image = csi_camera.read()

    if display_fps:
        draw_label(camera_image, "Frames Displayed (PS): " + str(csi_camera.last_frames_displayed), (10, 20))
        draw_label(camera_image, "Frames Read (PS): " + str(csi_camera.last_frames_read), (10, 40))

    return camera_image

cam_H_fov = 62.2 # (deg)
cam_V_fov = 48.8 # (deg)

# Good for 1280x720
DISPLAY_WIDTH = 640
DISPLAY_HEIGHT = 360
# For 1920x1080
# DISPLAY_WIDTH=960
# DISPLAY_HEIGHT=540

# 1920x1080, 30 fps
SENSOR_MODE_1080 = 2
# 1280x720, 60 fps
SENSOR_MODE_720 = 3



def start_camera():

    cam = CSI_Camera()
    cam.create_gstreamer_pipeline(
        sensor_id=0,
        sensor_mode=SENSOR_MODE_720,
        framerate=30,
        flip_method=2,
        display_height=DISPLAY_HEIGHT,
        display_width=DISPLAY_WIDTH,
    )

    cam.open(cam.gstreamer_pipeline)
    cam.start()
    

    cv2.namedWindow("CSI Cameras", cv2.WINDOW_AUTOSIZE)

    if not cam.video_capture.isOpened():
        # Cameras did not open, or no camera attached
        print("Unable to open camera")
        # TODO: Proper Cleanup
        SystemExit(0)

    try:
        # Start counting the number of frames read and displayed
        cam.start_counting_fps()
        while cv2.getWindowProperty("CSI Cameras", 0) >= 0:
            t1 = perf_counter()
            frame = read_camera(cam, show_fps)

            
            cam.frames_displayed += 1

            lower_red2 = np.array([80, 100, 70])
            upper_red2 = np.array([180, 255, 255])
            hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask2 = cv2.inRange(hsv_frame, lower_red2, upper_red2)
            pxiels = cv2.countNonZero(mask2)
            cv2.imshow('mask2', mask2)
            # This also acts as a frame limiter
            print(pxiels)
            try:
                M = cv2.moments(pxiels) #moment is centroid
                cx,cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
                cv2.circle(frame,(cx,cy),5,(0,0,255),-1)

            except:
                print("No center")
            t2 = perf_counter()
            #print("time: ", str((t2-t1)*1000))
            #print("----------------------------------------------")
            cv2.imshow("CSI Camera", frame)
            # Stop the program on the ESC key
            if (cv2.waitKey(5) & 0xFF) == 27:
                break

    finally:
        cam.stop()
        cam.release()
    cv2.destroyAllWindows()


def startThread():
    t = threading.Thread(target=start_camera)

    t.start()




startThread()
