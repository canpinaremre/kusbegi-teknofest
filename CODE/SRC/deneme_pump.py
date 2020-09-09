from utility import WaterLevelSensor, Pump
from time import sleep, time



MISSION_PUMP_TIMEOUT = 20
MISSION_MAX_WATER_SENSOR = 350

pump = Pump()
water_sens = WaterLevelSensor()

ts = time()

print("Start pumping")


while ( (time()- ts) < MISSION_PUMP_TIMEOUT ):
    pump.pump_water_in()

    if (water_level_sensor.readadc > MISSION_MAX_WATER_SENSOR):
        print("Water level reached!")
        drone.log.logger("Water level reached!")
        break

print("Stop pumping")


