import RPi.GPIO as gpio
import time

def init():
 gpio.setmode(gpio.BOARD)
 gpio.setup(26, gpio.OUT)
 gpio.setup(24, gpio.OUT)
 gpio.setup(35, gpio.OUT)
 gpio.setup(22, gpio.OUT)

def forward(sec):
 init()
 gpio.setup(26, gpio.OUT)
 gpio.setup(24, gpio.OUT)
 gpio.setup(35, gpio.OUT)
 gpio.setup(22, gpio.OUT)

 time.sleep(sec)
 gpio.cleanup()

def reverse(sec):
 init()
 gpio.setup(26, gpio.OUT)
 gpio.setup(24, gpio.OUT)
 gpio.setup(35, gpio.OUT)
 gpio.setup(22, gpio.OUT)

 time.sleep(sec)
 gpio.cleanup()

print "forward"
forward(4)
print "reverse"
reverse(2)