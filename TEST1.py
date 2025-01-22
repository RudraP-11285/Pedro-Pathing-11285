'''
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
'''
import cv2
import numpy as np
import math


def find_largest_contour(mask, min_area=500):
    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Filter out contours smaller than the minimum area
    contours = [cnt for cnt in contours if cv2.contourArea(cnt) >= min_area]

    # If no contours found after filtering, return None
    if not contours:
        return None

    # Return the largest contour
    return max(contours, key=cv2.contourArea)



def process_contour(contour, image):
    # Get the minimum area rectangle
    x, y, axisAlignedWidth, axisAlignedHeight = cv2.boundingRect(contour)
    rect = cv2.minAreaRect(contour)
    box = cv2.boxPoints(rect)
    box = np.int0(box)

    # Get center, dimensions, and angle
    center, (width, height), angle = rect

    # Determine if vertical
    is_vertical = np.abs(axisAlignedWidth) < np.abs(axisAlignedHeight)

    # Calculate offset from image center
    image_center = (image.shape[1] / 2, image.shape[0] / 2)
    offset_x = center[0] - image_center[0]
    offset_y = center[1] - image_center[1]

    # Draw the rotated rectangle
    cv2.drawContours(image, [box], 0, (0, 255, 0), 2)

    # Draw center point
    cv2.circle(image, (int(center[0]), int(center[1])), 5, (0, 0, 255), -1)

    return center, offset_x, offset_y, width, height, angle, is_vertical


def runPipeline(image, llrobot):
    # Convert to HSV color space
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Define color ranges for blue and yellow
    lower_blue = np.array([100, 50, 50])
    upper_blue = np.array([130, 255, 255])
    lower_yellow = np.array([20, 100, 100])
    upper_yellow = np.array([30, 255, 255])

    # Create masks for blue and yellow
    mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
    mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)

    # Apply a slight blur to reduce noise
    mask_blue = cv2.GaussianBlur(mask_blue, (5, 5), 0)
    mask_yellow = cv2.GaussianBlur(mask_yellow, (5, 5), 0)

    # Find the largest contours for each color
    contour_blue = find_largest_contour(mask_blue, min_area=500)
    contour_yellow = find_largest_contour(mask_yellow, min_area=500)

    # Initialize results
    results = {}
    llpython = [0] * 8

    # Process blue contour if found
    if contour_blue is not None:
        results['blue'] = process_contour(contour_blue, image)

    # Process yellow contour if found
    if contour_yellow is not None:
        results['yellow'] = process_contour(contour_yellow, image)

    # Prepare data to send back to the robot
    if 'blue' in results:
        center, offset_x, offset_y, width, height, angle, is_vertical = results['blue']
        llpython[0] = 1  # Blue detected
        llpython[1] = offset_x
        llpython[2] = offset_y
        llpython[3] = angle
        llpython[4] = 1 if is_vertical else 0

    if 'yellow' in results:
        center, offset_x, offset_y, width, height, angle, is_vertical = results['yellow']
        llpython[5] = 1  # Yellow detected
        llpython[6] = offset_x
        llpython[7] = offset_y

    # Draw crosshair
    cv2.line(image, (image.shape[1]//2, 0), (image.shape[1]//2, image.shape[0]), (255, 0, 0), 1)
    cv2.line(image, (0, image.shape[0]//2), (image.shape[1], image.shape[0]//2), (255, 0, 0), 1)

    # Add text annotations
    cv2.putText(image, "Blue and Yellow Sample Detector", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

    if 'blue' in results:
        #cv2.putText(image, f"Blue: {'Vertical' if results['blue'][6] else 'Horizontal'}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        cv2.putText(image, f"Blue: {'Vertical' if results['blue'][6] else 'Horizontal'}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
    if 'yellow' in results:
        cv2.putText(image, f"Yellow: {'Vertical' if results['yellow'][6] else 'Horizontal'}", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

    # Return the largest contour (blue if exists, otherwise yellow), the processed image, and the llpython data
    return contour_blue if contour_blue is not None else (contour_yellow if contour_yellow is not None else np.array([[]])), image, llpython
