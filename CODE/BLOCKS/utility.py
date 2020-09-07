from datetime import datetime
import queue
from collections import deque

class Coordinate_File:
    def write_coordinates(file_path, lat, lon, yaw):
        file = open(file_path, 'w')
        file.write(str(lat) + "\n")
        file.write(str(lon) + "\n")
        file.write(str(yaw))
        file.close()

    def read_coordinates(file_path):
        file = open(file_path, 'r')
        lat = float(file.readline())
        lon = float(file.readline())
        yaw = float(file.readline())
        file.close()
        return lat, lon, yaw

class Log:
    
    def __init__(self):
        self.log1 = deque()
        self.file_name = self.create_file_w_date()
    
    def logger(self,log):
        self.log1.append(log)

    def write_logs(self):
        file_date = open(self.file_name, "a")
        file_date.write('\n'.join(self.log1) + '\n')
        file_date.close()
        self.log1.clear()

    def create_file_name_date(self):
        date_today = datetime.now()
        date_today_st = str(date_today.day) + "-" + str(date_today.month) + "-" + str(date_today.year) + "/" + str(date_today.hour)+ ":" + str(date_today.min)
        return date_today_st

    def create_file_w_date(self):
        file_name_date = create_file_name_date()
        file_name1 = file_name_date + ".txt"
        file_date = open(file_name1, "w+")
        file_date.close()
        return file_name1

class Water_Motor:
    def init():
        gpio.setmode(gpio.BCM)
        gpio.setup(7, gpio.OUT)
        gpio.setup(8, gpio.OUT)
        gpio.setup(19, gpio.OUT)
        gpio.setup(25, gpio.OUT)

    def forward(sec):
        init()
        gpio.output(7, True)
        gpio.output(8, False)
        gpio.output(19, True)
        gpio.output(25, False)
        time.sleep(sec)
        gpio.cleanup()

    def reverse(sec):
        init()
        gpio.output(7, False)
        gpio.output(8, True)
        gpio.output(19, False)
        gpio.output(25, True)
        time.sleep(sec)
        gpio.cleanup()

class Water_Level_Sensor:
    SPICLK = 13
    SPIMISO = 21
    SPIMOSI = 19
    SPICS = 16

    photo_ch = 0

    # port init
    def init():
        GPIO.setwarnings(False)
        GPIO.cleanup()  # clean up at the end of your script
        GPIO.setmode(GPIO.BOARD)  # to specify whilch pin numbering system
        # set up the SPI interface pins
        GPIO.setup(SPIMOSI, GPIO.OUT)
        GPIO.setup(SPIMISO, GPIO.IN)
        GPIO.setup(SPICLK, GPIO.OUT)
        GPIO.setup(SPICS, GPIO.OUT)

    # read SPI data from MCP3008(or MCP3204) chip,8 possible adc's (0 thru 7)
    def readadc(adcnum, clockpin, mosipin, misopin, cspin):
        if ((adcnum > 7) or (adcnum < 0)):
            return -1
        GPIO.output(cspin, True)

        GPIO.output(clockpin, False)  # start clock low
        GPIO.output(cspin, False)  # bring CS low

        commandout = adcnum
        commandout |= 0x18  # start bit + single-ended bit
        commandout <<= 3  # we only need to send 5 bits here
        for i in range(5):
            if (commandout & 0x80):
                GPIO.output(mosipin, True)
            else:
                GPIO.output(mosipin, False)
            commandout <<= 1
            GPIO.output(clockpin, True)
            GPIO.output(clockpin, False)

        adcout = 0
        # read in one empty bit, one null bit and 10 ADC bits
        for i in range(12):
            GPIO.output(clockpin, True)
            GPIO.output(clockpin, False)
            adcout <<= 1
            if (GPIO.input(misopin)):
                adcout |= 0x1

        GPIO.output(cspin, True)

        adcout >>= 1  # first bit is 'null' so drop it
        return adcout

    def main():
        init()
        time.sleep(2)
        print
        "will start detec water level\n"
        while True:
            adc_value = readadc(photo_ch, SPICLK, SPIMOSI, SPIMISO, SPICS)
            print
            "water level:" + str("%.1f" % (adc_value / 200. * 100)) + "%\n"
            print
            "adc_value= " + str(adc_value) + "\n"
            time.sleep(1)