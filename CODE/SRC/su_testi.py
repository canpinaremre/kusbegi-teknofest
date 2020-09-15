
from utility import Pump, WaterLevelSensor
from time import time, sleep

MISSION_PUMP_TIMEOUT = 8

MISSION_MAX_WATER_SENSOR = 300

pump = Pump()

water_sens = WaterLevelSensor()


print("Start pumping")
drone.log.logger("Start pumping")

ts = time()

while ( (time()- ts) < MISSION_PUMP_TIMEOUT ):
    pump.pump_water_in()

    print(water_level_sensor.readadc)

    if (water_level_sensor.readadc > MISSION_MAX_WATER_SENSOR):
        print("Water level reached!")
        #drone.log.logger("Water level reached!")
        break

    

print("Stop pumping")
drone.log.logger("Stop pumping")

pump.close_and_clean()
water_level_sensor.close_and_clean()

