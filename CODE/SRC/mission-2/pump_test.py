from utility import WaterLevelSensor, Pump


pump = Pump()

sens = WaterLevelSensor()

print("Start pumping")
drone.log.logger("Start pumping")


MISSION_PUMP_TIMEOUT = 10 #20
MISSION_MAX_WATER_SENSOR = 450

ts = time()

while ( (time()- ts) < MISSION_PUMP_TIMEOUT ):
    pump.pump_water_in()

    if (sens.readadc(sens.photo_ch, sens.SPICLK, sens.SPIMOSI, sens.SPIMISO, sens.SPICS) > MISSION_MAX_WATER_SENSOR):
        print("Water level reached!")
        drone.log.logger("Water level reached!")
        break

print("Stop pumping")
drone.log.logger("Stop pumping")

pump.close_and_clean()
sens.close_and_clean()
