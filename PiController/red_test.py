import traceback

import datetime
import picamera, io, cv2, time
import numpy as np
import warnings
import glob
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import warnings
from multiprocessing import Process, Value

WIDTH = 640
HEIGHT = 480
expected_center = 293
COUNT = 0

def region_of_interest(img, vertices):
    mask = np.zeros_like(img)
    match_mask_color = 255
    cv2.fillPoly(mask, vertices, match_mask_color)
    masked_image = cv2.bitwise_and(img, mask)
    return masked_image

def select_red(image):
    converted = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower = np.uint8([160, 100, 100])
    upper = np.uint8([179, 255, 255])
    red_mask = cv2.inRange(converted, lower, upper)
    return cv2.bitwise_and(image, image, mask=red_mask)

def process(stream, vIntersection):
    global COUNT
    if (True):
        stream.seek(0)  # seek to location 0 of stream_img
        # Truncate the stream to the current position (in case
        # prior iterations output a longer image))
        # Read the image and do some processing on it
        data = np.fromstring(stream.getvalue(), dtype=np.uint8)
        # "Decode" the image from the array, preserving colour
        image = cv2.imdecode(data, 1)
        height, width, temp = image.shape
        region_of_interest_red = [(0, height), (0, 400), (width, 400), (width, height)]
        red_img = select_red(image)
        cv2.imwrite("original%d.jpeg" % COUNT, image)
        cv2.imwrite("red%d.jpeg" % COUNT, red_img)
        COUNT += 1
        cropped_red_img = region_of_interest(red_img, np.array([region_of_interest_red], np.int32))
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", category=RuntimeWarning)
            red_px = np.mean(np.where(np.any(cropped_red_img != [0, 0, 0], axis=-1)), axis=1)
            red_exist = not np.all(np.isnan(red_px))
            if not red_exist:
                red_px = np.array([-1, -1])
                print("No red pixels found")
            else:
                print(red_px)

def gen_seq(vIntersection):
    stream = io.BytesIO()
    while True:
        yield stream
        process(stream, vIntersection)


if __name__ == '__main__':
    global WIDTH, HEIGHT
    vIntersection = Value('i', 0)
    with picamera.PiCamera() as camera:
        camera.resolution = (WIDTH, HEIGHT)
        camera.framerate = 30
        camera.start_preview()
        time.sleep(1)
        camera.capture_sequence(gen_seq(vIntersection), format='jpeg', use_video_port=True)
