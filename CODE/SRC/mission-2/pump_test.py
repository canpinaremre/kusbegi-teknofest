from utility import WaterLevelSensor, Pump
from time import sleep, time

pump = Pump()

sens = WaterLevelSensor()

print("Start pumping in")
#drone.log.logger("Start pumping")


MISSION_PUMP_TIMEOUT = 11
 #20
MISSION_MAX_WATER_SENSOR = 450

ts = time()

while ( (time()- ts) < MISSION_PUMP_TIMEOUT ):
    pump.pump_water_in()
    sleep(0.1)
    print("pumping in")
    
    


print("Stop pumping")
#drone.log.logger("Stop pumping")

pump.close_and_clean()
sens.close_and_clean()
input()
pump = Pump()
sens = WaterLevelSensor()

print("Start pumping out")

ts = time()

while ( (time()- ts) < MISSION_PUMP_TIMEOUT*1.5 ):
    pump.pump_water_out()
    sleep(0.1)
    print("pumping in")
    
    

print("Stop pumping")

pump.close_and_clean()
sens.close_and_clean()