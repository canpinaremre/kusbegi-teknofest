import RPi.GPIO as gpio
import time

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

print "forward"
forward(4)
print "reverse"
reverse(2)