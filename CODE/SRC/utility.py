from datetime import datetime
import queue
from collections import deque

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


class WaterMotor:
    def __init__(self):
        self.pin7 = 7
        self.pin8 = 8
        self.pin19 = 19
        self.pin25 = 25

    def set_motor_GPIOs(self):
        gpio.setmode(gpio.BCM)
        gpio.setup(self.pin7, gpio.OUT)
        gpio.setup(self.pin8, gpio.OUT)
        gpio.setup(self.pin19, gpio.OUT)
        gpio.setup(self.pin25, gpio.OUT)

    def forward(self, sec):
        self.set_motor_GPIOs()
        gpio.output(self.pin7, True)
        gpio.output(self.pin8, False)
        gpio.output(self.pin19, True)
        gpio.output(self.pin25, False)
        time.sleep(sec)
        gpio.cleanup()

    def reverse(self, sec):
        self.set_motor_GPIOs()
        gpio.output(self.pin7, False)
        gpio.output(self.pin8, True)
        gpio.output(self.pin19, False)
        gpio.output(self.pin25, True)
        time.sleep(sec)
        gpio.cleanup()


class WaterLevelSensor:
    def __init__(self):
        self.SPICLK = 13
        self.SPIMISO = 21
        self.SPIMOSI = 19
        self.SPICS = 16

        self.photo_ch = 0

    def set_level_sensor_GPIOs(self):
        GPIO.setwarnings(False)
        GPIO.cleanup()  # clean up at the end of your script
        GPIO.setmode(GPIO.BOARD)  # to specify whilch pin numbering system
        # set up the SPI interface pins
        GPIO.setup(self.SPIMOSI, GPIO.OUT)
        GPIO.setup(self.SPIMISO, GPIO.IN)
        GPIO.setup(self.SPICLK, GPIO.OUT)
        GPIO.setup(self.SPICS, GPIO.OUT)

    # read SPI data from MCP3008(or MCP3204) chip,8 possible adc's (0 thru 7)
    def readadc(self):

        self.set_level_sensor_GPIOs()

        if ((self.photo_ch > 7) or (self.photo_ch < 0)):
            return -1

        GPIO.output(self.SPICS, True)
        GPIO.output(self.SPICLK, False)  # start clock low
        GPIO.output(self.SPICS, False)  # bring CS low

        commandout = self.photo_ch
        commandout |= 0x18  # start bit + single-ended bit
        commandout <<= 3  # we only need to send 5 bits here

        for i in range(5):

            if (commandout & 0x80):
                GPIO.output(self.SPIMOSI, True)
            
            else:
                GPIO.output(self.SPIMOSI, False)
            
            commandout <<= 1
            GPIO.output(self.SPICLK, True)
            GPIO.output(self.SPICLK, False)

        adcout = 0
        # read in one empty bit, one null bit and 10 ADC bits
        for i in range(12):
            GPIO.output(self.SPICLK, True)
            GPIO.output(self.SPICLK, False)
            adcout <<= 1
            if (GPIO.input(self.SPIMISO)):
                adcout |= 0x1

        GPIO.output(self.SPICS, True)
        adcout >>= 1  # first bit is 'null' so drop it

        water_level = round((adc_value / 200. * 100), 1)

        return water_level

