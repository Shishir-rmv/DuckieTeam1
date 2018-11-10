import picamera
from PIL import Image, ImageFilter
import numpy as np
import io


# this will be the process that we split off for Dmitry to do computer vision work in
# we use shared memory to make passing information back and fourth
def vision(see, x1, y1, x2, y2, outSlope):
    with picamera.PiCamera() as camera:
        stream = io.BytesIO()

    for foo in camera.capture_continuous(stream, format='jpeg'):
        # Truncate the stream to the current position (in case
        # prior iterations output a longer image)
        stream.truncate()
        stream.seek(0)
        # Read the image and do some processing on it
        data = np.fromstring(stream.getvalue(), dtype=np.uint8)
        height, width, _ = data.shape
        cropped_img = data[336:384, :, :]
        cropped_blurred_image = np.array(Image.fromarray(cropped_img).filter(ImageFilter.SMOOTH_MORE))

        outer_x1, outer_y1, inner_x1, inner_y1 = 0, 0, 0, 0
        outer_x2, outer_y2, inner_x2, inner_y2 = 0, 0, 0, 0
        cropped_img_height, cropped_img_width, _ = cropped_blurred_image.shape
        start_y = 0
        while outer_x1 == 0:
            for px in range(cropped_img_width, int(cropped_img_width /4), -1):
                if (cropped_blurred_image[start_y, px - 1] > np.array([190 ,200 ,200])).all():
                    outer_x1, outer_y1 = px, start_y
                    break
            start_y += 1

        start_y = 0
        while inner_x1 == 0:
            for px in range(outer_x1, int(cropped_img_width /4), -1):
                if (cropped_blurred_image[start_y, px - 1] < np.array([70 ,80 ,80])).all():
                    inner_x1, inner_y1 = px, start_y
                    break
            start_y += 1

        end_y = cropped_img_height - 1
        while outer_x2 == 0:
            for px in range(cropped_img_width, int(cropped_img_width /4), -1):
                if (cropped_blurred_image[end_y, px - 1] > np.array([190 ,200 ,200])).all():
                    outer_x2, outer_y2 = px, end_y
                    break
            end_y -= 1

        end_y = cropped_img_height - 1
        while inner_x2 == 0:
            for px in range(outer_x2, int(cropped_img_width /4), -1):
                if (cropped_blurred_image[end_y, px - 1] < np.array([70 ,80 ,80])).all():
                    inner_x2, inner_y2 = px, end_y
                    break
            end_y -= 1

        slope = (inner_y2 - inner_y1 ) /(inner_x2 - inner_y1)

        # set variables (in shared memory)
        x1.value = inner_x1
        y1.value = inner_y1
        x2.value = inner_x2
        y2.value = inner_y2
        outSlope.value = slope

        # print(inner_x1, inner_y1, inner_x2, inner_y2, slope)
        # break out if we sent the killswitch elsewhere
        if (see.value):
            break