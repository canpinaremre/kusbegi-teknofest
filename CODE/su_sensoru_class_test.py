from utility import WaterLevelSensor
from time import sleep


sens = WaterLevelSensor()



while True:
    sleep(0.2)
    adc_value = readadc(sens.photo_ch, sens.SPICLK, sens.SPIMOSI, sens.SPIMISO, sens.SPICS)
    print(str(adc_value))
