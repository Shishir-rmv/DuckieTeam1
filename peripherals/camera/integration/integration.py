import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from picamera import PiCamera
import time
import numpy as np
import cv2, math
import io

def region_of_interest(img, vertices):
	mask = np.zeros_like(img)
	match_mask_color = 255
	cv2.fillPoly(mask, vertices, match_mask_color)
	masked_image = cv2.bitwise_and(img, mask)
	return masked_image


def draw_lines(img, lines, color=[255, 0, 0], thickness=3):
	if lines is None:
		return
	img = np.copy(img)
	line_img = np.zeros((img.shape[0], img.shape[1], 3), dtype=np.uint8)
	for line in lines:
		for x1, y1, x2, y2 in line:
			cv2.line(line_img, (x1, y1), (x2, y2), color, thickness)
	img = cv2.addWeighted(img, 0.8, line_img, 1.0, 0.0)
	return img

def computeLines(image):
	#image = mpimg.imread('test.jpg')
	height, width, temp = image.shape
	region_of_interest_vert = [(0, height), (width/2,height/2), (width, height)]

	gray_img = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
	cannyed_img = cv2.Canny(gray_img, 200, 300)
	cropped_img = region_of_interest(cannyed_img, np.array([region_of_interest_vert], np.int32))

	lines = cv2.HoughLinesP(cropped_img, rho=6, theta=np.pi/60, threshold=160,
		lines=np.array([]), minLineLength=40, maxLineGap=25)

	line_image = draw_lines(image, lines)

# main method
if __name__ == '__main__':
	sumTime = 0.0
	sumCameraTime = 0.0
	sumPreprocessingTime = 0.0
	sumOpenCVTime = 0.0

	num = 10
	camera = PiCamera()
	camera.resolution = (640, 480)
	camera.framerate = 30
	camera.start_preview()
	outputs = [io.BytesIO() for i in range(num)]
	for x in range(num):
		start = time.time()
		stream = outputs[x]
		camera.capture(stream, format='jpeg')
		cameraCaptureDuration = time.time() - start
		
		preprocessingStartTime = time.time()
		# camera.capture('test.jpg')
		# Construct a numpy array from the stream
		data = np.fromstring(stream.getvalue(), dtype=np.uint8)
		# "Decode" the image from the array, preserving colour
		image = cv2.imdecode(data, 1)
		# OpenCV returns an array with data in BGR order. If you want RGB instead
		# use the following...
		image = image[:, :, ::-1]

		preprocessingDuration = time.time() - preprocessingStartTime
		
		openCVStartTime = time.time()
		computeLines(image)
		openCVDuration = time.time() - openCVStartTime
		
		duration = time.time() - start

		sumCameraTime += cameraCaptureDuration
		sumPreprocessingTime += preprocessingDuration
		sumOpenCVTime += openCVDuration
		sumTime += duration
		print("Run: %d, Camera: %f seconds, Preprocessing: %f seconds, OpenCV: %f seconds, Total: %f seconds" % ((x+1), cameraCaptureDuration, preprocessingDuration, openCVDuration, duration))
	print("Avg Camera Time: %f, Avg Preprocessing Time: %f, Avg OpenCV Time: %f, Avg Total Time: %f" % (sumCameraTime/num, sumPreprocessingTime/num, sumOpenCVTime/num, sumTime/num))

	# plt.figure()
	# plt.imshow(line_image)
	# plt.show()
