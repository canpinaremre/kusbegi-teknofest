#Kırmızı renk için bulduğun kodları videoda gösterecek şekilde dönüştürdüm.

import cv2
import numpy as np

cap = cv2.VideoCapture(0)

while True:
    _, frame = cap.read()
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Red Color
    # lower red
    lower_red = np.array([0, 50, 50])
    upper_red = np.array([10, 255, 255])

    # upper red
    lower_red2 = np.array([161, 155, 84])
    upper_red2 = np.array([180, 255, 255])

   # red_mask = cv2.inRange(hsv_frame, lower_red, upper_red)
    # red = cv2.bitwise_and(frame, frame, mask=red_mask)

    mask = cv2.inRange(hsv_frame, lower_red, upper_red)
    res = cv2.bitwise_and(frame, frame, mask=mask)

    mask2 = cv2.inRange(hsv_frame, lower_red2, upper_red2) #mask2 kırmızı rengi beyaz olarak göstererek ayırt ediyor. Kullanabiliriz
    res2 = cv2.bitwise_and(frame, frame, mask=mask2)   #güzel ayırt ediyor sadece tam kırmızı rengi ayırt ediyor.

    img3 = res + res2
    img4 = cv2.add(res, res2)
    img5 = cv2.addWeighted(res, 0.5, res2, 0.5, 0)

    kernel = np.ones((15, 15), np.float32) / 225
    smoothed = cv2.filter2D(res, -1, kernel)
    smoothed2 = cv2.filter2D(img3, -1, kernel)

    cv2.imshow("Frame", frame)
    cv2.imshow("Red", res)

    cv2.imshow('Averaging', smoothed)
    cv2.imshow('mask', mask)
    cv2.imshow('res', res)
    cv2.imshow('mask2', mask2)
    cv2.imshow('res2', res2)
    cv2.imshow('res3', img3)
    cv2.imshow('res4', img4)
    cv2.imshow('res5', img5)
    cv2.imshow('smooth2', smoothed2)


    key = cv2.waitKey(1)
    if key == 27:
        break


