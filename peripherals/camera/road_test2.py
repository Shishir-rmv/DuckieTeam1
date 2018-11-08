import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
import cv2	
import math

image = mpimg.imread('RoadTest4.jpg')
height, width, temp = image.shape
region_of_interest_vert = [(0, height), (width/2,height/2), (width, height)]

def region_of_interest(img, vertices):
	mask = np.zeros_like(img)
	match_mask_color = 255
	cv2.fillPoly(mask, vertices, match_mask_color)
	masked_image = cv2.bitwise_and(img, mask)
	return masked_image

gray_img = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
cannyed_img = cv2.Canny(gray_img, 200, 300)
cropped_img = region_of_interest(cannyed_img, np.array([region_of_interest_vert], np.int32))

lines = cv2.HoughLinesP(cropped_img, rho=6, theta=np.pi/60, threshold=160,
	lines=np.array([]), minLineLength=40, maxLineGap=25)

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

line_image = draw_lines(image, lines) 

plt.figure()
plt.imshow(line_image)
plt.show()