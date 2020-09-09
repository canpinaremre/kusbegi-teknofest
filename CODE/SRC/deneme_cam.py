from csi_camera import CSI_Camera
from time import sleep
import threading

show_fps = True
stop_thread_csi = False

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

def new_red_area_detected(pixel):
    print("New red area detected : ",pixel)#WRITE COORDİNATES


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

RED_AREA_VOLUME = 0
OLD_RED_AREA_VOLUME = 0

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

            OLD_RED_AREA_VOLUME = RED_AREA_VOLUME

            frame = read_camera(cam, show_fps)

            
            cam.frames_displayed += 1
            
            hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # Red Color
            low_red = np.array([161, 155, 84])
            high_red = np.array([180, 255, 255])
            red_mask = cv2.inRange(hsv_frame, low_red, high_red)
            #red = cv2.bitwise_and(frame, frame, mask=red_mask)

            cv2.imshow("Red", red_mask)

            #Calculate red area
            RED_AREA_VOLUME = cv2.countNonZero(red_mask)


            if(RED_AREA_VOLUME > OLD_RED_AREA_VOLUME):
                new_red_area_detected(RED_AREA_VOLUME)

            # Stop the program on the ESC key
            if stop_thread_csi == True:
                break

    finally:
        cam.stop()
        cam.release()
        cv2.destroyAllWindows()



def startThread():
    t = threading.Thread(target=start_camera)

    t.daemon = True

    t.start()

def stop():
    stop_thread_csi = True


print("Start mission")

i = 0
while True:
    print("second :",i)
    sleep(1)
    i = i+1
    if(i == 3):
        print("start cam")
        startThread()
    