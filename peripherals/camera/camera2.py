from picamera import PiCamera
from time import sleep
import sys
camera = PiCamera()

camera.start_preview()
sleep(5)
camera.capture(sys.argv[1])
camera.stop_preview()
