import cv2
import numpy as np

def runPipeline(input, llrobot):
	ycrcb = cv2.cvtColor(input, cv2.COLOR_RGB2YCrCb)
	ycrcb_thresh = cv2.inRange(ycrcb, (0.0, 0.0, 129.0, 0.0), (255.0, 255.0, 255.0, 0.0))

	ycrcb_thresh_eroded_dilated = ycrcb_thresh.copy()

	contours, hierarchy = cv2.findContours(ycrcb_thresh_eroded_dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

	crosshair = []
	crosshair_image = input.copy()

	height, width, channels = crosshair_image.shape

	crosshair_point_x = width / 2
	crosshair_point_y = height / 2
	scale_factor = (height + width) / 2

	adjusted_crosshair_size = (5 * scale_factor) / 100

	cv2.line(crosshair_image, (int(crosshair_point_x - adjusted_crosshair_size), int(crosshair_point_y)), (int(crosshair_point_x + adjusted_crosshair_size), int(crosshair_point_y)), (0.0, 255.0, 0.0, 0.0), 3)
	cv2.line(crosshair_image, (int(crosshair_point_x), int(crosshair_point_y - adjusted_crosshair_size)), (int(crosshair_point_x), int(crosshair_point_y + adjusted_crosshair_size)), (0.0, 255.0, 0.0, 0.0), 3)

	for contour in contours:
		x, y, w, h = cv2.bounding_rect(contour)

		if (((crosshair_point_x >= x) and (crosshair_point_x <= x + w)) and (crosshair_point_y >= y)) and (crosshair_point_y <= y + h):
			crosshair.append(contour)

	crosshair_rot_rects = []
	for points in crosshair:
		crosshair_rot_rects.append(cv2.minAreaRect(points))

	crosshair_image_rot_rects = crosshair_image.copy()
	for rect in crosshair_rot_rects:
		if rect != None:
			box = cv2.boxPoints(rect)
			box = np.int0(box)
			cv2.drawContours(crosshair_image_rot_rects, box, 0, (0.0, 255.0, 0.0, 0.0), 3)

	biggest_contour = max(crosshair, key=cv2.contourArea)

	llpython = []

	return (biggest_contour, crosshair_image_rot_rects, llpython)