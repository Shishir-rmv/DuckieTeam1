import picamera
import numpy as np
import io
import cv2
import time


def region_of_interest(img, vertices):
    mask = np.zeros_like(img)
    match_mask_color = 255
    cv2.fillPoly(mask, vertices, match_mask_color)
    masked_image = cv2.bitwise_and(img, mask)
    return masked_image


def make_line_points(y1, y2, line):
    """
    Convert a line represented in slope and intercept into pixel points
    """
    if line is None:
        return None

    slope, intercept = line

    # make sure everything is integer as cv2.line requires it
    x1 = int((y1 - intercept) / slope)
    x2 = int((y2 - intercept) / slope)
    y1 = int(y1)
    y2 = int(y2)

    return ((x1, y1), (x2, y2))


def average_slope_intercept(lines):
    left_lines = []  # (slope, intercept)
    left_weights = []  # (length,)
    right_lines = []  # (slope, intercept)
    right_weights = []  # (length,)

    for line in lines:
        for x1, y1, x2, y2 in line:
            if x2 == x1:
                continue  # ignore a vertical line
            slope = (y2 - y1) / (x2 - x1)
            intercept = y1 - slope * x1
            length = np.sqrt((y2 - y1) ** 2 + (x2 - x1) ** 2)
            if slope < 0:  # y is reversed in image
                left_lines.append((slope, intercept))
                left_weights.append((length))
            else:
                right_lines.append((slope, intercept))
                right_weights.append((length))

    # add more weight to longer lines
    left_lane = np.dot(left_weights, left_lines) / np.sum(left_weights) if len(left_weights) > 0 else None
    right_lane = np.dot(right_weights, right_lines) / np.sum(right_weights) if len(right_weights) > 0 else None

    return left_lane, right_lane  # (slope, intercept), (slope, intercept)


def lane_lines(image, lines):
    left_lane, right_lane = average_slope_intercept(lines)

    y1 = image.shape[0]  # bottom of the image
    y2 = y1 * 0.6  # slightly lower than the middle

    left_line = make_line_points(y1, y2, left_lane)
    right_line = make_line_points(y1, y2, right_lane)

    return left_line, right_line


def draw_lane_lines(image, lines, color=[255, 0, 0], thickness=20):
    # make a separate image to draw lines and combine with the orignal later
    line_image = np.zeros_like(image)
    for line in lines:
        if line is not None:
            cv2.line(line_image, *line, color, thickness)
    return cv2.addWeighted(image, 1.0, line_image, 0.95, 0.0)


def convert_hls(image):
    return cv2.cvtColor(image, cv2.COLOR_RGB2HLS)


def select_white_yellow(image):
    converted = convert_hls(image)
    # white color mask
    lower = np.uint8([0, 200, 0])
    upper = np.uint8([255, 255, 255])
    white_mask = cv2.inRange(converted, lower, upper)
    # yellow color mask
    lower = np.uint8([10, 0, 100])
    upper = np.uint8([40, 255, 255])
    yellow_mask = cv2.inRange(converted, lower, upper)
    # combine the mask
    mask = cv2.bitwise_or(white_mask, yellow_mask)
    return cv2.bitwise_and(image, image, mask=mask)


def process(stream):
    start = time.time()
    stream.seek(0)  # seek to location 0 of stream_img
    # Truncate the stream to the current position (in case
    # prior iterations output a longer image))
    # Read the image and do some processing on it
    image = np.fromstring(stream.getvalue(), dtype=np.uint8)
    print(image.shape)
    height, width = image.shape
    region_of_interest_vert = [(0, height), (0, 300), (width, 300), (width, height)]
    white_yellow_image = select_white_yellow(image)
    gray_img = cv2.cvtColor(white_yellow_image, cv2.COLOR_RGB2GRAY)
    cannyed_img = cv2.Canny(gray_img, 200, 300)
    cropped_img = region_of_interest(cannyed_img, np.array([region_of_interest_vert], np.int32))

    lines = cv2.HoughLinesP(cropped_img, rho=1, theta=np.pi / 60, threshold=10, minLineLength=40, maxLineGap=25)
    left_line, right_line = lane_lines(image, lines)

    left_slope = 0
    right_slope = 0
    if left_line:
        (x1, y1), (x2, y2) = left_line
        left_slope = (y2 - y1) / (x2 - x1)

    if right_line:
        (x1, y1), (x2, y2) = right_line
        right_slope = (y2 - y1) / (x2 - x1)

    slope_ratio = np.absolute(left_slope / right_slope)

    print("Left Slope: %f, Right Slope: %f, Slope Ratio: %f" % (left_slope, right_slope, slope_ratio))

    if (-0.7 <= left_slope <= -0.6) and (0.6 <= right_slope <= 0.7):
        print("Going Straight")
    elif (left_slope < -0.66 or right_slope < 0.66) or right_slope is None:
        print("Turn Right")
    elif (left_slope > -0.66 or right_slope > 0.66) or left_slope is None:
        print("Turn Left")

    duration = time.time() - start
    print("Took: %x" % duration)
    stream.seek(0)
    stream.truncate()


def gen_seq():
    stream = io.BytesIO()
    while True:
        yield stream
        process(stream)


# this will be the process that we split off for Dmitry to do computer vision work in
# we use shared memory to make passing information back and fourth
def vision(see, x1, y1, x2, y2, outSlope):
    print("Starting Vision")
    with picamera.PiCamera() as camera:
        print("Got the camera")
        camera.capture_sequence(gen_seq(), use_video_port=False)
