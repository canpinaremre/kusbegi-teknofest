import math

import cv2
import numpy as np
import time

#color settings
hue_lower = 160
hue_upper = 180
saturation_lower = 100
saturation_upper = 255
value_lower = 20
value_upper = 255

hue_lower2 = 0
hue_upper2 = 10
saturation_lower2 = 100
saturation_upper2 = 255
value_lower2 = 20
value_upper2 = 255




min_contour_area = 200 # the smallest number of pixels in a contour before it will register this as a target

#camera
horizontal_fov = 118.2 * math.pi/180
vertical_fov = 69.5 * math.pi/180
horizontal_resolution = 1280
vertical_resolution = 720

camera = cv2.VideoCapture(0)


time.sleep(3)

while(1):
    _,capture = camera.read()
    hsvcapture = cv2.cvtColor(capture,cv2.COLOR_BGR2HSV)   
    inrangepixels1 = cv2.inRange(hsvcapture,np.array((hue_lower,saturation_lower,value_lower)),np.array((hue_upper,saturation_upper,value_upper)))#in opencv, HSV is 0-180,0-255,0-255
    inrangepixels2 = cv2.inRange(hsvcapture,np.array((hue_lower2,saturation_lower2,value_lower2)),np.array((hue_upper2,saturation_upper2,value_upper2)))#in opencv, HSV is 0-180,0-255,0-255
    inrangepixels = inrangepixels1 + inrangepixels2
    tobecontourdetected = inrangepixels.copy()
    #TODO filter better. binary morphology would be a good start.
    contours,hierarchy = cv2.findContours(tobecontourdetected,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
    
    contour_sizes=[]
    contour_centroids = []
    for contour in contours:  
        real_area = cv2.contourArea(contour)
        if real_area > min_contour_area:
            M = cv2.moments(contour) #moment is centroid
            cx,cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
            cv2.circle(capture,(cx,cy),5,(0,0,255),-1)
            contour_sizes.append(real_area)
            contour_centroids.append((cx,cy))
    
    #find biggest contour (by area)    
    biggest_contour_index = 0
    for i in range(1,len(contour_sizes)):
        if contour_sizes[i] > contour_sizes[biggest_contour_index]:
            biggest_contour_index = i
    biggest_contour_centroid=None
    if len(contour_sizes)>0:
        biggest_contour_centroid=contour_centroids[biggest_contour_index]
    
    #if the biggest contour was found, color it blue and send the message
    if biggest_contour_centroid is not None:
        cv2.circle(capture,biggest_contour_centroid,5,(255,0,0),-1)
        x,y = biggest_contour_centroid
        #SEND
    
    cv2.imshow('capture',capture) 
    cv2.imshow('inrangepixels',inrangepixels)
        
    if cv2.waitKey(1) == 27:
        break
    
cv2.destroyAllWindows()
camera.release()