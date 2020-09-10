# to get X and Y errors and to get biggest detected red contour's area

import math
import cv2
import numpy as np

class TargetRed:
    
    def __init__(self):
        self.image = None
        self.contourCenterX = 0
        self.contourCenterY = 0
        self.MainContour = None
        self.errX = 0
        self.errY = 0
        self.con_area = 0
        self.lower_red = np.array([0, 150, 100])
        self.upper_red = np.array([25, 255, 255])

    def Process(self):
        hsv_frame = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
        imgray = cv2.inRange(hsv_frame, self.lower_red, self.upper_red)
        #cv2.imshow("imgray", imgray)
        
        kernel = np.ones((3,3), np.uint8)
        imgray = cv2.erode(imgray, kernel, iterations=5)
        imgray = cv2.dilate(imgray, kernel, iterations=9)

        contours, _ = cv2.findContours(imgray,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE) #Get contour
        height, width  = self.image.shape[:2]
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
            cv2.drawContours(self.image,self.MainContour,-1,(0,255,0),3) #Draw Contour GREEN
            cv2.circle(self.image, (self.contourCenterX, self.contourCenterY), 7, (255,255,255), -1) #Draw dX circle WHITE
            cv2.circle(self.image, (middleX, middleY), 3, (0,0,255), -1) #Draw middle circle RED
            cv2.putText(self.image,(str(self.errX)+ ","+ str(self.errY)),(self.contourCenterX+20, middleY), font, 1,(0,0,200),2,font)

        #cv2.imshow("image", self.image)
            
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
