import io
import time
import threading
import picamera
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
import cv2, math


# Create a pool of image processors
done = False
lock = threading.Lock()
pool = []



class ImageProcessor(threading.Thread):
	def __init__(self):
		super(ImageProcessor, self).__init__()
		self.stream = io.BytesIO()
		self.event = threading.Event()
		self.terminated = False
		self.counter = 0
		self.start()

	def region_of_interest(self, img, vertices):
		mask = np.zeros_like(img)
		match_mask_color = 255
		cv2.fillPoly(mask, vertices, match_mask_color)
		masked_image = cv2.bitwise_and(img, mask)
		return masked_image
	
	def draw_lines(self, img, lines, color=[255, 0, 0], thickness=3):
		if lines is None:
			return img
		img = np.copy(img)
		line_img = np.zeros((img.shape[0], img.shape[1], 3), dtype=np.uint8)
		for line in lines:
			for x1, y1, x2, y2 in line:
				cv2.line(line_img, (x1, y1), (x2, y2), color, thickness)
			img = cv2.addWeighted(img, 1.0, line_img, 1.0, 0.0)
		return img
	
	def computeLines(self, image):
		#image = mpimg.imread('test.jpg')
		height, width, temp = image.shape
		region_of_interest_vert = [(0, height), (width/2,height/2), (width, height)]
	
		gray_img = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
		cannyed_img = cv2.Canny(gray_img, 200, 300)
		cropped_img = self.region_of_interest(cannyed_img, np.array([region_of_interest_vert], np.int32))
	
		lines = cv2.HoughLinesP(cropped_img, rho=6, theta=np.pi/60, threshold=160, lines=np.array([]), minLineLength=40, maxLineGap=25)
		line_image = self.draw_lines(image, lines)
		
		return line_image
	

	def run(self):
		# This method runs in a separate thread
		global done
		while not self.terminated:
			if self.event.wait(1):
				try:
					processStart = time.time()
					self.stream.seek(0)
					# Read the image and do some processing on it
					data = np.fromstring(self.stream.getvalue(), dtype=np.uint8)
					# "Decode" the image from the array, preserving colour
					image = cv2.imdecode(data, 1)
					# OpenCV returns an array with data in BGR order. If you want RGB instead
					# use the following...
					#image = image[:, :, ::-1]
					computeStart = time.time()
					image_with_line = self.computeLines(image)
					
					computeDuration = time.time() - computeStart

					cv2.imwrite(threading.current_thread().getName() + str(self.counter) +  ".jpg", image_with_line)
					# Set done to True if you want the script to terminate
					# at some point
					self.counter += 1
					processDuration = time.time() - processStart
					print("Thread %s, ComputeTime: %f, SaveTime: %f" % (threading.current_thread().getName(), computeDuration, processDuration))
					if self.counter >= 20:
						done=True
				finally:
					# Reset the stream and event
					self.stream.seek(0)
					self.stream.truncate()
					self.event.clear()
					# Return ourselves to the pool
					with lock:
						pool.append(self)

def streams():
	while not done:
		with lock:
			try:
				processor = pool.pop()
			except IndexError as e:
				pass
		yield processor.stream
		processor.event.set()



if __name__ == '__main__':
    with picamera.PiCamera() as camera:
        pool = [ImageProcessor() for i in range (4)]
        camera.resolution = (640, 480)
        # Set the framerate appropriately; too fast and the image processors
        # will stall the image pipeline and crash the script
        camera.framerate = 30
        camera.start_preview()
        time.sleep(1)
        camera.capture_sequence(streams(), use_video_port=False)

    # Shut down the processors in an orderly fashion
    while pool:
        with lock:
            processor = pool.pop()
        processor.terminated = True
        processor.join()
