from Test_Target_Red_Thread import CSI_Camera
import threading
import cv2

cam = CSI_Camera()
cam.open()
cam.start()
#cam.start_counting_fps()
while True:

	print("main", cam.con_area)
	print("errX", cam.errX)
	print("errY", cam.errY)
	"""
	if cam.frame is not None:
		
		cv2.imshow("frame", cam.frame)
	
	if (cv2.waitKey(5) & 0xFF) == 27:
    	break
	"""
"""
finally:
    cam.stop()
    cam.release()
"""
