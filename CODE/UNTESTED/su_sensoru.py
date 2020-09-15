from utility import WaterLevelSensor, Pump
from time import time, sleep

pump = Pump()

water_level_sensor = WaterLevelSensor()

print("Start pumping")


ts = time()
"""
while ( (time()- ts) < MISSION_PUMP_TIMEOUT ):
    pump.pump_water_in()

    if (water_level_sensor.readadc > MISSION_MAX_WATER_SENSOR):
        print("Water level reached!")
        
        break

print("Stop pumping")

"""

while True:

    print(water_level_sensor.readadc)
    


pump.close_and_clean()
water_level_sensor.close_and_clean()
