import cv2
import numpy as np

cap = cv2.VideoCapture(0)

while True:
    _, frame = cap.read()
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Red Color
    low_red = np.array([161, 155, 84])
    high_red = np.array([180, 255, 255])
    red_mask = cv2.inRange(hsv_frame, low_red, high_red)
    #red = cv2.bitwise_and(frame, frame, mask=red_mask)
    M = cv2.moments(red_mask)
    try:
        cx,cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
        cv2.circle(red_mask,(cx,cy),5,(0,0,255),-1)
    except:
        print("No")
    cv2.imshow("Red", red_mask)
    #cv2.imshow("frame",frame)

    print(cv2.countNonZero(red_mask))
    
    key = cv2.waitKey(1)
    if key == 27:
        break



