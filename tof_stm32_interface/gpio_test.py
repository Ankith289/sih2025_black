import Jetson.GPIO as gpio
import time

led_pin=7

gpio.setmode(gpio.BOARD)
gpio.setup(led_pin,gpio.OUT,initial=gpio.LOW)

try:
  while True:
    gpio.output(led_pin,gpio.HIGH)
    time.sleep(0.5)
    gpio.output(led_pin,gpio.LOW)
    time.sleep(0.5)
except KeyboardInterrupt:
  print("Exiting")
  gpio.cleanup()
