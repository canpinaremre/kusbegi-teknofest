import cv2
import threading
import math
import numpy as np
import time

class RepeatTimer(threading.Timer):
    def run(self):
        while not self.finished.wait(self.interval):
            self.function(*self.args, **self.kwargs)

class CSI_Camera:

    def __init__ (self) :
        # Initialize instance variables
        # OpenCV video capture element
        self.video_capture = None
        # The last captured image from the camera
        self.frame = None
        self.grabbed = False
        # The thread where the video capture runs
        self.read_thread = None
        self.read_lock = threading.Lock()
        self.running = False
        self.fps_timer=None
        self.frames_read=0
        self.frames_displayed=0
        self.last_frames_read=0
        self.last_frames_displayed=0
        self.show_fps = False

        self.contourCenterX = 0
        self.contourCenterY = 0
        self.MainContour = None
        self.errX = 0
        self.errY = 0
        self.con_area = 0
        self.lower_red = np.array([0, 150, 100])
        self.upper_red = np.array([25, 255, 255])

        self.gstreamer_pipeline_string = (
            "nvarguscamerasrc sensor-id=%d sensor-mode=%d ! "
            "video/x-raw(memory:NVMM), "
            "format=(string)NV12, framerate=(fraction)%d/1 ! "
            "nvvidconv flip-method=%d ! "
            "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
            "videoconvert ! "
            "video/x-raw, format=(string)BGR ! appsink"
            % (
                0, #sensor_id,
                3, #sensor_mode, 1280x720
                30, #framerate, 30fps
                2, #flip_method,
                640, #display_width,
                360, #display_height,
            )
        )


    def open(self):
        try:
            self.video_capture = cv2.VideoCapture(
                self.gstreamer_pipeline_string, cv2.CAP_GSTREAMER
            )
            
        except RuntimeError:
            self.video_capture = None
            print("Unable to open camera")
            print("Pipeline: " + self.gstreamer_pipeline_string)
            return
        # Grab the first frame to start the video capturing

        self.grabbed, self.frame = self.video_capture.read()

    def start(self):
        if self.running:
            print('Video capturing is already running')
            return None
        # create a thread to read the camera image
        if self.video_capture != None:
            self.start_counting_fps() # if no need for fps make commentout
            self.running=True
            self.read_thread = threading.Thread(target=self.updateCamera)
            self.read_thread.start()
        
        return self
    
    def stop(self):
        self.running=False
        self.read_thread.join()
    
    def updateCamera(self):
        # This is the thread to read images from the camera
        try:
            while self.running:
            
                grabbed, frame = self.video_capture.read()
                with self.read_lock:
                    self.grabbed=grabbed
                    self.frame=frame
                    self.frames_read += 1


                hsv_frame = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
                imgray = cv2.inRange(hsv_frame, self.lower_red, self.upper_red)
                #cv2.imshow("imgray", imgray)
                
                kernel = np.ones((3,3), np.uint8)
                imgray = cv2.erode(imgray, kernel, iterations=5)
                imgray = cv2.dilate(imgray, kernel, iterations=9)

                contours, _ = cv2.findContours(imgray,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE) #Get contour
                height, width  = self.frame.shape[:2]
                contours_len = len(contours)
                if contours_len > 0:
                    if contours == 1:
                        self.MainContour = contours
                        self.con_area = cv2.contourArea(contours)
                    else:
                        canditates = []

                        for con_num in range(contours_len):
                            self.MainContour = cv2.minAreaRect(contours[con_num])
                            self.con_area = cv2.contourArea(contours[con_num])

                            canditates.append((self.con_area, con_num))
                        canditates = sorted(canditates, reverse=True)
                        #print(canditates)

                        self.con_area, con_num = canditates[0]
                        self.MainContour = contours[con_num]

                    middleX = int(width/2) #Get X coordenate of the middle point
                    middleY = int(height/2) #Get Y coordenate of the middle point
                    
                    prev_cX = self.contourCenterX
                    prev_cY = self.contourCenterY

                    if self.getContourCenter(self.MainContour) != 0:
                        self.contourCenterX = self.getContourCenter(self.MainContour)[0]
                        self.contourCenterY = self.getContourCenter(self.MainContour)[1]
                        if abs(prev_cX-self.contourCenterX) > 5 or abs(prev_cY-self.contourCenterY) > 5:
                            self.correctMainContour(prev_cX, contours)
                            self.correctMainContour(prev_cY, contours)
                    else:
                        self.contourCenterX = 0
                        self.contourCenterY = 0
                
                    self.errX = self.contourCenterX - middleX
                    self.errY = middleY - self.contourCenterY

                    font = cv2.FONT_HERSHEY_SIMPLEX
                    cv2.drawContours(self.frame,self.MainContour,-1,(0,255,0),3) #Draw Contour GREEN
                    cv2.circle(self.frame, (self.contourCenterX, self.contourCenterY), 7, (255,255,255), -1) #Draw dX circle WHITE
                    cv2.circle(self.frame, (middleX, middleY), 3, (0,0,255), -1) #Draw middle circle RED
                    cv2.putText(self.frame,(str(self.errX)+ ","+ str(self.errY)),(self.contourCenterX+20, middleY), font, 1,(0,0,200),2,font)

                self.frames_displayed += 1


                if self.show_fps:                    

                    self.draw_label(self.frame, ("Frames Displayed (PS): " + str(self.last_frames_displayed)), (10, 20))
                    self.draw_label(self.frame, ("Frames Read (PS): " + str(self.last_frames_read)), (10, 40))

                
                
                ########### at competition commentout 

                print("errX= ", self.errX)
                print("errY= ", self.errY)
                print("con_area= ", self.con_area)
                cv2.imshow("image", self.frame)
                if (cv2.waitKey(5) & 0xFF) == 27:
                    break

        except RuntimeError:
            print("Could not read image from camera")
        # FIX ME - stop and cleanup thread
        # Something bad happened
        
        """ 
        finally:
            self.stop()
            self.release()
        """    
        
        #cv2.destroyAllWindows()

    def read(self):
        with self.read_lock:
            frame = self.frame.copy()
            grabbed=self.grabbed
        return grabbed, frame

    def release(self):
        if self.video_capture != None:
            self.video_capture.release()
            self.video_capture = None
        # Kill the timer
        self.fps_timer.cancel()
        self.fps_timer.join()
        # Now kill the thread
        if self.read_thread != None:
            self.read_thread.join()

    def update_fps_stats(self):
        self.last_frames_read=self.frames_read
        self.last_frames_displayed=self.frames_displayed
        # Start the next measurement cycle
        self.frames_read=0
        self.frames_displayed=0

    def start_counting_fps(self):
        self.fps_timer=RepeatTimer(1.0,self.update_fps_stats)
        self.fps_timer.start()

    def draw_label(self, cv_image, label_text, label_position):
        font_face = cv2.FONT_HERSHEY_SIMPLEX
        scale = 0.5
        color = (255, 255, 255)
        # You can get the size of the string with cv2.getTextSize here
        cv2.putText(cv_image, label_text, label_position, font_face, scale, color, 1, cv2.LINE_AA)

    def getContourCenter(self,  contour):
        M = cv2.moments(contour)
        
        if M["m00"] == 0:
            return 0
        
        x = int(M["m10"]/M["m00"])
        y = int(M["m01"]/M["m00"])
        
        return [x,y]
            
    def Aprox(self, a, b, error):
        if abs(a - b) < error:
            return True
        else:
            return False
            
    def correctMainContour(self, prev_cx, contours):
        if abs(prev_cx-self.contourCenterX) > 5:
            for i in range(len(contours)):
                if self.getContourCenter(contours[i]) != 0:
                    tmp_cx = self.getContourCenter(contours[i])[0]
                    if self.Aprox(tmp_cx, prev_cx, 5) == True:
                        self.MainContour = contours[i]
                        if self.getContourCenter(self.MainContour) != 0:
                            self.contourCenterX = self.getContourCenter(self.MainContour)[0]
