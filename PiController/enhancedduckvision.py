import datetime
import traceback
import warnings

import cv2
import io
import numpy as np
import picamera
import time
import logging, sys

logging.basicConfig(stream=sys.stderr, level=logging.ERROR)
WIDTH = 640
HEIGHT = 480
expected_center = 293


def region_of_interest(img, vertices):
    mask = np.zeros_like(img)
    match_mask_color = 255
    cv2.fillPoly(mask, vertices, match_mask_color)
    masked_image = cv2.bitwise_and(img, mask)
    return masked_image


def convert_hls(image):
    return cv2.cvtColor(image, cv2.COLOR_RGB2HLS)


def select_green(image):
    converted = convert_hls(image)
    # Green color mask
    lower = np.uint8([50, 0, 50])
    upper = np.uint8([80, 200, 200])
    green_mask = cv2.inRange(converted, lower, upper)
    return cv2.bitwise_and(image, image, mask=green_mask)


def select_red(image):
    lower = np.array([0, 0, 200], dtype="uint8")
    upper = np.array([150, 150, 255], dtype="uint8")
    mask = cv2.inRange(image, lower, upper)
    output = cv2.bitwise_and(image, image, mask=mask)
    return output


def select_white(image, converted):
    # white color mask
    lower = np.uint8([0, 215, 0])
    upper = np.uint8([255, 255, 255])
    white_mask = cv2.inRange(converted, lower, upper)
    return cv2.bitwise_and(image, image, mask=white_mask)


def select_yellow(image, converted):
    # yellow color mask
    lower = np.uint8([50, 100, 130])
    upper = np.uint8([100, 200, 255])
    yellow_mask = cv2.inRange(converted, lower, upper)
    return cv2.bitwise_and(image, image, mask=yellow_mask)


def is_green_light_on(image):
    height, width, temp = image.shape
    cropped_for_green = image[200:height, 0:width].copy()
    green_img = select_green(cropped_for_green)
    num_of_green_px = np.where(np.any(green_img != [0, 0, 0], axis=-1))[1].size
    if num_of_green_px > 1000:
        return True
    else:
        return False


def is_at_red_line(image):
    height, width, temp = image.shape
    cropped_for_red = image[280:height, 280:360].copy()
    red_image = select_red(cropped_for_red)
    num_of_red_px = np.where(np.any(red_image != [0, 0, 0], axis=-1))[1].size
    if num_of_red_px > 1000:
        return True
    else:
        return False


def process(stream, vOffset, vOffsetOld, stopLine, greenLight):
    global expected_center

    if (True):
        stream.seek(0)  # seek to location 0 of stream_img
        # Truncate the stream to the current position (in case
        # prior iterations output a longer image))
        # Read the image and do some processing on it
        data = np.fromstring(stream.getvalue(), dtype=np.uint8)
        # "Decode" the image from the array, preserving colour
        image = cv2.imdecode(data, 1)
        height, width, temp = image.shape

        try:
            with warnings.catch_warnings():
                warnings.simplefilter("ignore", category=RuntimeWarning)
                # Check if green light is ON
                if stopLine.value and not greenLight.value:
                    if is_green_light_on(image=image):
                        greenLight.value = True
                        logging.info("%s\tFOUND GREEN>>>: Starting Now" % (datetime.datetime.now()))
                else: # Check if it is at a Red line
                    if is_at_red_line(image=image):
                        # Check if the Green light is already ON
                        if is_green_light_on(image=image):
                            stopLine.value = True
                            greenLight.value = True
                            logging.info("%s\tFOUND GREEN>>>: Starting Now" % (datetime.datetime.now()))
                        else:
                            stopLine.value = True
                            logging.info("%s\tFOUND RED!!!: Stop Now" % (datetime.datetime.now()))

                cropped_for_white_yellow = image[360:480, 0:640].copy()
                hls_image = convert_hls(cropped_for_white_yellow)

                # Filters White and Yellow colors in the image
                white_image = select_white(cropped_for_white_yellow, hls_image)
                yellow_image = select_yellow(cropped_for_white_yellow, hls_image)

                white_px = np.mean(np.where(np.any(white_image != [0, 0, 0], axis=-1)), axis=1)
                white_exist = not np.all(np.isnan(white_px))
                # Check if white pixels are found
                if not white_exist:
                    white_px = np.array([-1, -1])

                yellow_px = np.mean(np.where(np.any(yellow_image != [0, 0, 0], axis=-1)), axis=1)
                yellow_exist = not np.all(np.isnan(yellow_px))
                # Check if yellow pixels are found
                if not yellow_exist:
                    yellow_px = np.array([-1, -1])

                if white_exist and yellow_exist:
                    if white_px[1] < yellow_px[1]:
                        # Probably seeing the white on the other side
                        current_center = 260 - int(yellow_px[1])
                        diff = current_center - expected_center
                        vOffset.value = int(diff)
                        logging.debug("%s\tNo white pixel found!\tYellow Pixel: x = %d, y = %d\t diff: %d" % (
                            datetime.datetime.now(), int(yellow_px[1]), int(yellow_px[0]), diff))
                    else:
                        current_center = (white_px[1] + yellow_px[1]) / 2
                        diff = expected_center - current_center
                        logging.debug(
                            "%s\tWhite Pixel: x = %d, y = %d\t Yellow Pixel: x = %d, y = %d\t center: %d\t, diff: %d" % (
                                datetime.datetime.now(), int(white_px[1]), int(white_px[0]), int(yellow_px[1]),
                                int(yellow_px[0]), current_center, diff))
                        vOffset.value = int(diff)
                elif white_exist and not yellow_exist:
                    current_center = int(white_px[1]) - 320
                    diff = expected_center - current_center
                    vOffset.value = int(diff)
                    logging.debug("%s\tNo yellow pixel found!\tWhite Pixel: x = %d, y = %d\t diff: %d" % (
                        datetime.datetime.now(), int(white_px[1]), int(white_px[0]), diff))
                elif yellow_exist and not white_exist:
                    current_center = 260 - int(yellow_px[1])
                    diff = current_center - expected_center
                    vOffset.value = int(diff)
                    logging.debug("%s\tNo white pixel found!\tYellow Pixel: x = %d, y = %d\t diff: %d" % (
                        datetime.datetime.now(), int(yellow_px[1]), int(yellow_px[0]), diff))

                if vOffset.value != vOffsetOld.value:
                    vOffsetOld.value = vOffset.value
        except Exception as e:
            traceback.print_exc()

        stream.seek(0)
        stream.truncate()


def gen_seq(vOffset, vOffsetOld, go, stopLine, greenLight):
    stream = io.BytesIO()
    while go.value:
        # print("VISION going")
        yield stream
        process(stream, vOffset, vOffsetOld, stopLine, greenLight)


# this will be the process that we split off for Dmitry to do computer vision work in
# we use shared memory to make passing information back and fourth
def vision(vOffset, vOffsetOld, go, stopLine, greenLight):
    global WIDTH, HEIGHT
    print("Starting Vision")
    with picamera.PiCamera() as camera:
        camera.resolution = (WIDTH, HEIGHT)
        # Set the framerate appropriately; too fast and the image processors
        # will stall the image pipeline and crash the script
        camera.framerate = 40
        camera.start_preview()
        time.sleep(1)
        camera.capture_sequence(gen_seq(vOffset, vOffsetOld, go, stopLine, greenLight), format='jpeg', use_video_port=True)
    print("Vision Finished")
