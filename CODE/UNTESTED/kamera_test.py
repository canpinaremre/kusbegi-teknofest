from Test_Target_Red_Thread import CSI_Camera
from time import sleep



cam = CSI_Camera()
cam.open()
cam.start()



while cam.running is False:
    print("wait cam")
    sleep(0.5)

sleep(0.31)

max_area = 0

while True:
    if not False:            
            if (cam.con_area > max_area):
                max_area = cam.con_area
                print("daha büyük alan algılandı")
                print(max_area)
    else:
        break

