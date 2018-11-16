import picamera
import numpy as np
import io
import cv2
import time
from multiprocessing import Value

COUNT = 1

# Jake needed to initialize these since they were drawing errors
white_line_x1, white_line_y1, white_line_x2, white_line_y2 = 0,0,0,0
yellow_line_x1, yellow_line_y1, yellow_line_x2, yellow_line_y2 = 0,0,0,0

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


def select_white_line(image, lines):
    if lines is None:
        return None
    leftmost_x1 = image.shape[1]
    leftmost_x2 = image.shape[1]

    leftmost_y1 = int(image.shape[0] * 0.6)
    leftmost_y2 = image.shape[0]
    slope = 0
    intercept = 0
    longest_line = 0
    for line in lines:
        for x1, y1, x2, y2 in line:
            if x1 < leftmost_x1 and x2 < leftmost_x2:
                leftmost_x1 = x1
                leftmost_x2 = x2
                vlength = np.abs(y2 - y1)
                if vlength > longest_line:
                    longest_line = vlength
                    slope = (y2 - y1) / (x2 - x1)
                    intercept = y1 - slope * x1

    white_line = make_line_points(leftmost_y1, leftmost_y2, (slope, intercept))

    return white_line


def select_yellow_line(image, lines):
    if lines is None:
        return None
    rightmost_x1 = 0
    rightmost_y1 = int(image.shape[0] * 0.6)
    rightmost_y2 = image.shape[0]
    slope = 0
    intercept = 0
    longest_line = 0
    for line in lines:
        for x1, y1, x2, y2 in line:
            vlength = np.abs(y2 - y1)
            if vlength > longest_line:
                longest_line = vlength
                if x1 > rightmost_x1:
                    rightmost_x1 = x1
                    slope = (y2 - y1) / (x2 - x1)
                    intercept = y1 - slope * x1

    yellow_line = make_line_points(rightmost_y1, rightmost_y2, (slope, intercept))
    return yellow_line


def draw_lane_lines(image, lines, center, color=[0, 0, 255], thickness=20):
    # make a separate image to draw lines and combine with the orignal later
    line_image = np.zeros_like(image)
    for line in lines:
        if line is not None:
            cv2.line(line_image, *line,  color, thickness)
    # image1 * alpha + image2 * beta + gamma
    # image1 and image2 must be the same shape.
    (center_of_lane_x, center_of_lane_y) = center
    cv2.circle(line_image, (center_of_lane_x, center_of_lane_y), 10, (255,0,0), -1)
    return cv2.addWeighted(image, 1.0, line_image, 0.95, 0.0)


def convert_hls(image):
    return cv2.cvtColor(image, cv2.COLOR_RGB2HLS)


def select_white(image):
    converted = convert_hls(image)
    # white color mask
    lower = np.uint8([  0, 240, 0])
    upper = np.uint8([255, 255, 255])
    white_mask = cv2.inRange(converted, lower, upper)
    return cv2.bitwise_and(image, image, mask = white_mask)


def select_yellow(image):
    converted = convert_hls(image)
    # yellow color mask
    lower = np.uint8([ 10,   0, 100])
    upper = np.uint8([ 40, 220, 220])
    yellow_mask = cv2.inRange(converted, lower, upper)
    return cv2.bitwise_and(image, image, mask = yellow_mask)


def process(stream, vOffset):

    global white_line_x1, white_line_y1, white_line_x2, white_line_y2
    global yellow_line_x1, yellow_line_y1, yellow_line_x2, yellow_line_y2

    try:
    #if(True):
        stream.seek(0)  # seek to location 0 of stream_img
        # Truncate the stream to the current position (in case
        # prior iterations output a longer image))
        # Read the image and do some processing on it
        data = np.fromstring(stream.getvalue(), dtype=np.uint8)
        # "Decode" the image from the array, preserving colour
        image = cv2.imdecode(data, 1)

        # Debug stuff:
        # cv2.imwrite('original_image%d.jpeg' % COUNT, image)

        height, width, temp = image.shape

        # Defines Region of Interest
        region_of_interest_vert = [(0, height), (0, 0), (width, 0), (width, height)]

        # Filters White and Yellow colors in the image
        white_image = select_white(image)
        yellow_image = select_yellow(image)

        # Debug stuff:
        # cv2.imwrite('white_image%d.jpeg' % COUNT, white_image)
        # cv2.imwrite('yellow_image%d.jpeg' % COUNT, yellow_image)

        # Convert to Grayscale
        gray_of_white_img = cv2.cvtColor(white_image, cv2.COLOR_RGB2GRAY)
        gray_of_yellow_img = cv2.cvtColor(yellow_image, cv2.COLOR_RGB2GRAY)

        cannyed_of_white_img = cv2.Canny(gray_of_white_img, 50, 200)
        cannyed_of_yellow_img = cv2.Canny(gray_of_yellow_img, 50, 200)

        cropped_white_img = region_of_interest(cannyed_of_white_img, np.array([region_of_interest_vert], np.int32))
        cropped_yellow_img = region_of_interest(cannyed_of_yellow_img, np.array([region_of_interest_vert], np.int32))

        white_lines = cv2.HoughLinesP(cropped_white_img, rho=1, theta=np.pi / 180, threshold=30, minLineLength=1,
                                      maxLineGap=100)
        yellow_lines = cv2.HoughLinesP(cropped_yellow_img, rho=1, theta=np.pi / 180, threshold=30, minLineLength=1,
                                       maxLineGap=100)
        white_line = select_white_line(image, white_lines)
        yellow_line = select_yellow_line(image, yellow_lines)

        if white_line:
            (white_line_x1, white_line_y1), (white_line_x2, white_line_y2) = white_line
            # print("wx1: %d\twy1: %d\twx2: %d\twy2: %d" %(white_line_x1, white_line_y1, white_line_x2, white_line_y2))
        else:
            #print("No white line detected")
            pass

        if yellow_line:
            (yellow_line_x1, yellow_line_y1), (yellow_line_x2, yellow_line_y2) = yellow_line
            # print("yx1: %d\tyy1: %d\tyx2: %d\tyy2: %d" %(yellow_line_x1, yellow_line_y1, yellow_line_x2, yellow_line_y2))
        else:
            #print("No yellow line detected")
            pass

        # print("wx1: %d\twy1: %d\twx2: %d\twy2: %d" %(white_line_x1, white_line_y1, white_line_x2, white_line_y2))
        # print("yx1: %d\tyy1: %d\tyx2: %d\tyy2: %d" %(yellow_line_x1, yellow_line_y1, yellow_line_x2, yellow_line_y2))
        
        white_midpoint_x = (white_line_x1 + white_line_x2) / 2
        yellow_midpoint_x = (yellow_line_x1 + yellow_line_x2) / 2
        center_of_lane_x = int((white_midpoint_x + yellow_midpoint_x) / 2)

        # Jake added error suppression, was getting values in the range of -4000 to 4000 (possibly larger in magnitude)
        if (center_of_lane_x <= 640 and center_of_lane_x >= 0):
            vOffset.value = 262 - center_of_lane_x

        # Debug stuff:
        # print("Center of the lane: (%d, %d)" % (center_of_lane_x, center_of_lane_y))
        # line_image = draw_lane_lines(image, (yellow_line, white_line), (center_of_lane_x, center_of_lane_y))
        # cv2.imwrite('lined_image%d.jpeg' % COUNT, line_image)
        # COUNT = COUNT + 1
        # duration = time.time() - start
        # print("Took: %f" % duration)

        stream.seek(0)
        stream.truncate()
    except Exception as e:
        print(str(e))
        pass


def gen_seq(vOffset, go):
    stream = io.BytesIO()
    while go.value:
        #print("VISION going")
        yield stream
        process(stream, vOffset)


# this will be the process that we split off for Dmitry to do computer vision work in
# we use shared memory to make passing information back and fourth
def vision(vOffset, go):
    print("Starting Vision")
    with picamera.PiCamera() as camera:
        camera.resolution = (640, 480)
        # Set the framerate appropriately; too fast and the image processors
        # will stall the image pipeline and crash the script
        camera.framerate = 30
        camera.start_preview()
        time.sleep(1)
        camera.capture_sequence(gen_seq(vOffset, go), format='jpeg', use_video_port=True)
    print("Vision Finished")
