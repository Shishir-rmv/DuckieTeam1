import picamera, io, cv2, time, pdb
import numpy as np
from multiprocessing import Value

COUNT = 1
WIDTH = 1280
HEIGHT = 720
exp_dist_frm_white = 1205
exp_dist_frm_yellow = -49

# Jake needed to initialize these since they were drawing errors
white_line_x1, white_line_y1, white_line_x2, white_line_y2 = 0, 0, 0, 0
yellow_line_x1, yellow_line_y1, yellow_line_x2, yellow_line_y2 = 0, 0, 0, 0

expected_slope_yellow = -1.5
expected_slope_white = 2.5

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
    if (line == (0, 0)):
        return None

    slope, intercept = line

    # make sure everything is integer as cv2.line requires it
    x1 = int((y1 - intercept) / slope)
    x2 = int((y2 - intercept) / slope)
    y1 = int(y1)
    y2 = int(y2)

    return (x1, y1), (x2, y2)


def select_white_line(image, lines):
    img_width = image.shape[1]
    if lines is None:
        return None

    least_intercept = None
    slope = None
    for line in lines:
        for x1, y1, x2, y2 in line:
            # any line which is tilted towards right and lies entirely
            # in the right half of the image is discarded
            if x1 > img_width/2 and x2 > img_width/2:
                current_slope = (y2 - y1) / (x2 - x1)
                if current_slope > 0:
                    intercept = y1 - current_slope * x1
                    if least_intercept is None:
                        least_intercept = intercept
                    elif 0 < intercept < least_intercept:
                        least_intercept = intercept
                    slope = current_slope

    y1 = image.shape[0]  # bottom of the image
    y2 = y1 * 0.6  # slightly lower than the middle
    white_line = make_line_points(y1, y2, (slope, least_intercept))
    return white_line


def select_yellow_line(image, lines):
    img_width = image.shape[1]
    if lines is None:
        return None

    least_intercept = None
    slope = None
    for line in lines:
        for x1, y1, x2, y2 in line:
            # any line which is tilted towards left and lies entirely
            # in the left half of the image is discarded
            if x1 <= img_width / 2 and x2 <= img_width / 2:
                current_slope = (y2 - y1) / (x2 - x1)
                if current_slope < -1.4:
                    intercept = y1 - current_slope * x1
                    if least_intercept is None:
                        least_intercept = intercept
                    elif 0 < intercept < least_intercept:
                        least_intercept = intercept
                    slope = current_slope

    y1 = image.shape[0]  # bottom of the image
    y2 = y1 * 0.6  # slightly lower than the middle
    yellow_line = make_line_points(y1, y2, (slope, least_intercept))
    return yellow_line


def draw_lane_lines(image, lines, center, color=[0, 0, 255], thickness=20):
    # make a separate image to draw lines and combine with the orignal later
    line_image = np.zeros_like(image)
    for line in lines:
        if line is not None:
            cv2.line(line_image, *line, color, thickness)
    # image1 * alpha + image2 * beta + gamma
    # image1 and image2 must be the same shape.
    (center_of_lane_x, center_of_lane_y) = center
    cv2.circle(line_image, (center_of_lane_x, center_of_lane_y), 10, (255, 0, 0), -1)
    return cv2.addWeighted(image, 0.95, line_image, 1.0, 0.0)


def convert_hls(image):
    return cv2.cvtColor(image, cv2.COLOR_RGB2HLS)


def select_white(image):
    converted = convert_hls(image)
    # white color mask
    lower = np.uint8([0, 220, 0])
    upper = np.uint8([255, 255, 255])
    white_mask = cv2.inRange(converted, lower, upper)
    return cv2.bitwise_and(image, image, mask=white_mask)


def select_yellow(image):
    converted = convert_hls(image)
    # yellow color mask
    lower = np.uint8([50, 120, 130])
    upper = np.uint8([100, 200, 255])
    yellow_mask = cv2.inRange(converted, lower, upper)
    return cv2.bitwise_and(image, image, mask=yellow_mask)


def process(stream, vOffset):
    global exp_dist_frm_white, exp_dist_frm_yellow
    global white_line_x1, white_line_y1, white_line_x2, white_line_y2
    global yellow_line_x1, yellow_line_y1, yellow_line_x2, yellow_line_y2
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

        # Defines Region of Interest
        region_of_interest_vert = [(0, height), (0, height / 2), (width - 200, height / 2), (width - 100, height)]

        try:
            # Filters White and Yellow colors in the image
            white_image = select_white(image)
            yellow_image = select_yellow(image)

            # Detecting edges of white and yellow blocks:
            cannyed_of_white_img = cv2.Canny(white_image, 50, 200)
            cannyed_of_yellow_img = cv2.Canny(yellow_image, 50, 200)

            # Crop the images to region of interest
            cropped_white_img = region_of_interest(cannyed_of_white_img, np.array([region_of_interest_vert], np.int32))
            cropped_yellow_img = region_of_interest(cannyed_of_yellow_img, np.array([region_of_interest_vert], np.int32))

            # Detecting all the lines in both the images
            white_lines = cv2.HoughLinesP(cropped_white_img, rho=1, theta=np.pi / 180, threshold=30, minLineLength=1,
                                          maxLineGap=100)
            yellow_lines = cv2.HoughLinesP(cropped_yellow_img, rho=1, theta=np.pi / 180, threshold=30, minLineLength=1,
                                           maxLineGap=100)

            # Filter the lines
            white_line = select_white_line(image, white_lines)
            yellow_line = select_yellow_line(image, yellow_lines)

            slope_white = None
            slope_yellow = None
            center_of_lane_x = None
            center_of_lane_y = None

            if white_line and yellow_line:
                # When both the lines are visible
                (white_line_x1, white_line_y1), (white_line_x2, white_line_y2) = white_line
                (yellow_line_x1, yellow_line_y1), (yellow_line_x2, yellow_line_y2) = yellow_line

                slope_white = (white_line_y2 - white_line_y1) / (white_line_x2 - white_line_x1)
                slope_yellow = (yellow_line_y2 - yellow_line_y1) / (yellow_line_x2 - yellow_line_x1)

                white_midpoint_x = (white_line_x1 + white_line_x2) / 2
                white_midpoint_y = (white_line_y1 + white_line_y2) / 2

                yellow_midpoint_x = (yellow_line_x1 + yellow_line_x2) / 2
                yellow_midpoint_y = (yellow_line_y1 + yellow_line_y2) / 2

                yellow_angle = np.arctan(expected_slope_yellow - slope_yellow) / (
                            1 + (expected_slope_yellow * slope_yellow))
                white_angle = np.arctan(expected_slope_white - slope_white) / (1 + (expected_slope_white * slope_white))

                center_of_lane_x = int((white_midpoint_x + yellow_midpoint_x) / 2)
                center_of_lane_y = int((white_midpoint_y + yellow_midpoint_y) / 2)

                # Jake added error suppression, was getting values in the range of -4000 to 4000 (possibly larger in magnitude)
                if (center_of_lane_x <= WIDTH and center_of_lane_x >= 0):
                    vOffset.value = 555 - center_of_lane_x
                print("White: Yes\t Yellow: Yes\t Slope_White: %f\t Slope_Yellow: %f\t VOffset: %d" % (slope_white, slope_yellow, vOffset.value))
            elif white_line and not yellow_line:
                slope_white = (white_line_y2 - white_line_y1) / (white_line_x2 - white_line_x1)
                diff = exp_dist_frm_white - white_line_x1
                vOffset.value = diff
                print("White: Yes\t Yellow: No\t Slope_White: %f\t VOffset: %d" % (slope_white, vOffset.value))
            elif yellow_line and not white_line:
                slope_yellow = (yellow_line_y2 - yellow_line_y1) / (yellow_line_x2 - yellow_line_x1)
                diff = expected_slope_yellow - yellow_line_x1
                vOffset.value = diff
                print("White: No\t Yellow: Yes\t Slope_Yellow: %f\t VOffset: %d" % (slope_yellow, vOffset.value))
            else:
                print("No lines found!")
                vOffset.value = 0


            # Debug stuff - To save images uncomment this:
            COUNT = COUNT + 1
            if COUNT <= 50:
                cv2.imwrite('original_image%d.jpeg' % COUNT, image)
                cv2.imwrite('white_image%d.jpeg' % COUNT, white_image)
                cv2.imwrite('yellow_image%d.jpeg' % COUNT, yellow_image)
                line_image = draw_lane_lines(image, (yellow_line, white_line), (center_of_lane_x, center_of_lane_y))
                cv2.imwrite('lined_image%d.jpeg' % COUNT, line_image)
        except Exception as e:
            print(str(e))

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
