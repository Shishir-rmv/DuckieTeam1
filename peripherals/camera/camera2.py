from picamera import PiCamera
from time import sleep
import sys
camera = PiCamera()
camera.resolution = (640, 480)
camera.start_preview()
sleep(1)
camera.capture(sys.argv[1])
camera.stop_preview()
