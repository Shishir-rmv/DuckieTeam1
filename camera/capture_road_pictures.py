from picamera import PiCamera
from time import sleep

camera = PiCamera()

camera.start_preview()
for x in range(10):
  print("waiting for 5 seconds")
  sleep(5)
  print("Capturing")
  camera.capture('road' + str(x) + '.jpg')

camera.stop_preview()

