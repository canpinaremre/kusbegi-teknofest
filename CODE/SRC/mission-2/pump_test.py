from utility import WaterLevelSensor, Pump
from time import sleep, time

pump = Pump()

sens = WaterLevelSensor()

print("Start pumping in")
#drone.log.logger("Start pumping")


MISSION_PUMP_TIMEOUT = 10 #20
MISSION_MAX_WATER_SENSOR = 450

ts = time()

while ( (time()- ts) < MISSION_PUMP_TIMEOUT ):
    pump.pump_water_in()
    sleep(0.5)
    print("pumping in")
    sens_value = sens.readadc(sens.photo_ch, sens.SPICLK, sens.SPIMOSI, sens.SPIMISO, sens.SPICS)
    print(sens_value)
    if (sens_value > MISSION_MAX_WATER_SENSOR):
        print("Water level reached!")
        #drone.log.logger("Water level reached!")
        break

print("Stop pumping")
#drone.log.logger("Stop pumping")

pump.close_and_clean()
sens.close_and_clean()

pump = Pump()
sens = WaterLevelSensor()

print("Start pumping out")

ts = time()

while ( (time()- ts) < MISSION_PUMP_TIMEOUT ):
    pump.pump_water_out()
    sleep(0.5)
    print("pumping in")
    sens_value = sens.readadc(sens.photo_ch, sens.SPICLK, sens.SPIMOSI, sens.SPIMISO, sens.SPICS)
    print(sens_value)
    

print("Stop pumping")