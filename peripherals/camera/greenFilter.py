import io
import time

import cv2
import numpy as np
from picamera import PiCamera


def convert_hls(image):
    return cv2.cvtColor(image, cv2.COLOR_RGB2HLS)


def select_green(image):
    converted = convert_hls(image)
    # yellow color mask
    lower = np.uint8([50, 0, 50])
    upper = np.uint8([80, 200, 200])
    green_mask = cv2.inRange(converted, lower, upper)
    return cv2.bitwise_and(image, image, mask=green_mask)


# Create the in-memory stream
stream = io.BytesIO()
with PiCamera() as camera:
    camera.resolution = (640, 480)
    camera.start_preview()
    time.sleep(1)
    camera.capture(stream, format='jpeg')
# Construct a numpy array from the stream
data = np.fromstring(stream.getvalue(), dtype=np.uint8)
# "Decode" the image from the array, preserving colour
image = cv2.imdecode(data, 1)
# OpenCV returns an array with data in BGR order. If you want RGB instead
# use the following...
# image = image[:, :, ::-1]
cropped_for_green = image[200:480, 0:640].copy()
green_img = select_green(cropped_for_green)
cv2.imwrite("greenNew.jpg", green_img)
cv2.imwrite("OriginalGreen.jpg", image)
