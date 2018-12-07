import traceback

import datetime
import picamera, io, cv2, time
import numpy as np
import warnings

WIDTH = 1280
HEIGHT = 720
expected_center = 575


def region_of_interest(img, vertices):
    mask = np.zeros_like(img)
    match_mask_color = 255
    cv2.fillPoly(mask, vertices, match_mask_color)
    masked_image = cv2.bitwise_and(img, mask)
    return masked_image


def convert_hls(image):
    return cv2.cvtColor(image, cv2.COLOR_RGB2HLS)


def select_white(image):
    converted = convert_hls(image)
    # white color mask
    lower = np.uint8([0, 215, 0])
    upper = np.uint8([255, 255, 255])
    white_mask = cv2.inRange(converted, lower, upper)
    return cv2.bitwise_and(image, image, mask=white_mask)


def select_yellow(image):
    converted = convert_hls(image)
    # yellow color mask
    lower = np.uint8([50, 100, 130])
    upper = np.uint8([100, 200, 255])
    yellow_mask = cv2.inRange(converted, lower, upper)
    return cv2.bitwise_and(image, image, mask=yellow_mask)


def process(stream, vOffset):
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

        # Defines Region of Interest
        # Bottom right quadrant of the image for white line
        region_of_interest_white = [(width / 2, height), (width / 2, height / 2), (width, height / 2), (width, height)]
        # Bottom left quadrant of the image for yellow line
        region_of_interest_yellow = [(0, height), (0, height / 2), (width / 2, height / 2), (width, height)]

        try:
            with warnings.catch_warnings():
                warnings.simplefilter("ignore", category=RuntimeWarning)

                # Filters White and Yellow colors in the image
                white_image = select_white(image)
                yellow_image = select_yellow(image)

                #
                cropped_white_img = region_of_interest(white_image, np.array([region_of_interest_white], np.int32))
                cropped_yellow_img = region_of_interest(yellow_image, np.array([region_of_interest_yellow], np.int32))

                white_px = np.mean(np.where(np.any(cropped_white_img != [0, 0, 0], axis=-1)), axis=1)
                white_exist = not np.all(np.isnan(white_px))
                # Check if white pixels are found
                if not white_exist:
                    white_px = np.array([-1, -1])
                    print("No white pixels found")

                yellow_px = np.mean(np.where(np.any(cropped_yellow_img != [0, 0, 0], axis=-1)), axis=1)
                yellow_exist = not np.all(np.isnan(yellow_px))
                # Check if yellow pixels are found
                if not yellow_exist:
                    yellow_px = np.array([-1, -1])
                    print("No yellow pixels found")

                # if white_exist:
                #     diff = int(white_px[1]) - 1100
                #     vOffset.value = int(diff)
                #     print("White Pixel: x = %d, y = %d\t diff: %d" % (int(white_px[1]), int(white_px[0]), diff))
                # elif yellow_exist:
                #     diff = int(yellow_px[1]) - 65
                #     vOffset.value = int(diff)
                #     print("Yellow Pixel: x = %d, y = %d\t diff: %d" % (int(yellow_px[1]), int(yellow_px[0]), diff))

                if white_exist and yellow_exist:
                    current_center = (white_px[1] + yellow_px[1]) / 2
                    diff = expected_center - current_center
                    print("%s\tWhite Pixel: x = %d, y = %d\t Yellow Pixel: x = %d, y = %d\t center: %d\t, diff: %d" % (
                        datetime.datetime.now(), int(white_px[1]), int(white_px[0]), int(yellow_px[1]),
                        int(yellow_px[0]), current_center, diff))
                    vOffset.value = int(diff)
                elif white_exist and not yellow_exist:
                    diff = (1100 - int(white_px[1]))/2
                    vOffset.value = int(diff)
                    print("%s\tWhite Pixel: x = %d, y = %d\t diff: %d" % (datetime.datetime.now(), int(white_px[1]), int(white_px[0]), diff))
                elif yellow_exist and not white_exist:
                    diff = (67 - int(yellow_px[1]))/2
                    vOffset.value = int(diff)
                    print("%s\tYellow Pixel: x = %d, y = %d\t diff: %d" % (datetime.datetime.now(), int(yellow_px[1]), int(yellow_px[0]), diff))

        except Exception as e:
            traceback.print_exc()

        stream.seek(0)
        stream.truncate()


def gen_seq(vOffset, go):
    stream = io.BytesIO()
    while go.value:
        # print("VISION going")
        yield stream
        process(stream, vOffset)


# this will be the process that we split off for Dmitry to do computer vision work in
# we use shared memory to make passing information back and fourth
def vision(vOffset, go):
    global WIDTH, HEIGHT
    print("Starting Vision")
    with picamera.PiCamera() as camera:
        camera.resolution = (WIDTH, HEIGHT)
        # Set the framerate appropriately; too fast and the image processors
        # will stall the image pipeline and crash the script
        camera.framerate = 30
        camera.start_preview()
        time.sleep(1)
        camera.capture_sequence(gen_seq(vOffset, go), format='jpeg', use_video_port=True)
    print("Vision Finished")
