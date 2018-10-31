from picamera import PiCamera
from time import sleep

camera = PiCamera()

camera.start_preview()
for x in range(10):
  sleep(5)
  camera.capture('road' + str(x) + '.jpg')

camera.stop_preview()

