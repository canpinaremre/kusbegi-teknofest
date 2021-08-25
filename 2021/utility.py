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
        file_name1 = "2021/logs/" + file_name_date + ".txt"
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

