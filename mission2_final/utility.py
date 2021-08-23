from datetime import datetime
import queue
from collections import deque
#import RPi.GPIO as gpio

class CoordinateFile:
    def __init__(self):
        self.file_dir = None

    def write_coordinates(self, lat, lon, yaw):
        file = open(self.file_dir, 'w')
        file.write(str(lat) + "\n")
        file.write(str(lon) + "\n")
        file.write(str(yaw))
        file.close()

    def read_coordinates(self):
        file = open(self.file_dir, 'r')
        lat = float(file.readline())
        lon = float(file.readline())
        yaw = float(file.readline())
        file.close()
        return lat, lon, yaw


class Log:
    def __init__(self):
        self.log1 = deque()
        self.file_name = self.create_file_w_date()

    def logger(self, log):
        self.log1.append(str(datetime.now().hour) + ":" + str(datetime.now().minute) + ":"+ str(datetime.now().second) + "- " + log)

    def write_logs(self):
        file_date = open(self.file_name, "a")
        file_date.write('\n'.join(self.log1) + '\n')
        file_date.close()
        self.log1.clear()

    def create_file_name_date(self):
        date_today = datetime.now()
        date_today_st = str(date_today.day) + "-" + str(date_today.month) + "-" + str(
            date_today.year) + "-" + str(date_today.hour) + "-" + str(date_today.minute)
        return date_today_st

    def create_file_w_date(self):
        file_name_date = self.create_file_name_date()
        file_name1 = file_name_date + ".txt"
        file_date = open(file_name1, "w+")
        file_date.close()
        return file_name1


class Pump:
    def __init__(self):
        self.pin26 = 26
        self.pin24 = 24
        self.pin35 = 35
        self.pin22 = 22
        self.set_motor_GPIOs()

    def set_motor_GPIOs(self):
        gpio.setmode(gpio.BOARD)
        gpio.setup(self.pin26, gpio.OUT)
        gpio.setup(self.pin24, gpio.OUT)
        gpio.setup(self.pin35, gpio.OUT)
        gpio.setup(self.pin22, gpio.OUT)

    def pump_water_in(self):
        gpio.output(self.pin26, True)
        gpio.output(self.pin24, False)
        gpio.output(self.pin35, False)
        gpio.output(self.pin22, False)

    def pump_water_out(self):
        gpio.output(self.pin26, False)
        gpio.output(self.pin24, False)
        gpio.output(self.pin35, True)
        gpio.output(self.pin22, False)

    def close_and_clean(self):
        gpio.output(self.pin26, False)
        gpio.output(self.pin24, False)
        gpio.output(self.pin35, False)
        gpio.output(self.pin22, False)
        gpio.cleanup()

class WaterLevelSensor:
    def __init__(self):
        self.SPICLK = 13
        self.SPIMISO = 21
        self.SPIMOSI = 19
        self.SPICS = 16

        self.photo_ch = 0
        self.set_level_sensor_GPIOs()

    def set_level_sensor_GPIOs(self):
        gpio.setwarnings(False)
        gpio.setmode(gpio.BOARD)  # to specify whilch pin numbering system
        # set up the SPI interface pins
        gpio.setup(self.SPIMOSI, gpio.OUT)
        gpio.setup(self.SPIMISO, gpio.IN)
        gpio.setup(self.SPICLK, gpio.OUT)
        gpio.setup(self.SPICS, gpio.OUT)
        

    def close_and_clean(self):
        gpio.cleanup()  # clean up at the end of your script

    # read SPI data from MCP3008(or MCP3204) chip,8 possible adc's (0 thru 7)
    def readadc(self,adcnum, clockpin, mosipin, misopin, cspin):
        
        if ((adcnum > 7) or (adcnum < 0)):
            return -1
        gpio.output(cspin, True)

        gpio.output(clockpin, False)  # start clock low
        gpio.output(cspin, False)  # bring CS low

        commandout = adcnum
        commandout |= 0x18  # start bit + single-ended bit
        commandout <<= 3  # we only need to send 5 bits here
        for i in range(5):
            if (commandout & 0x80):
                gpio.output(mosipin, True)
            else:
                gpio.output(mosipin, False)
            commandout <<= 1
            gpio.output(clockpin, True)
            gpio.output(clockpin, False)

        adcout = 0
        # read in one empty bit, one null bit and 10 ADC bits
        for i in range(12):
            gpio.output(clockpin, True)
            gpio.output(clockpin, False)
            adcout <<= 1
            if (gpio.input(misopin)):
                adcout |= 0x1

        gpio.output(cspin, True)

        adcout >>= 1  # first bit is 'null' so drop it
        return adcout


