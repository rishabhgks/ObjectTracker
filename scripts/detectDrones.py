# Import all the necessary packages
from __future__ import print_function
import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
from imutils import paths
import numpy as np
import argparse
import imutils
import cv2
import csv
from matplotlib import pyplot as plt

image_path = "../drone2_cam"
out_image_path = "../drone2_detected_cam"
csv_path = "../drone2_center.csv"

# Load all the image frames of the simulation for every timestep based on camera frequency
image_list = sorted(list(paths.list_images(image_path)))
# Opening the CSV file to store data
with open(csv_path, mode='w') as drone_center:
# Declare the CSV writer object
    drone_writer = csv.writer(drone_center, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
# Loop over the image frames
    for i in np.arange(0, len(image_list)):
        drone_x = 0
        drone_y = 0
# Read the image
        image = cv2.imread(image_list[i])
# Convert the image to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
# Perform Adaptive Threhsolding over the image
        thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)[1]
# Perform a set number of erosion operation on the thresholded image
        thresh = cv2.erode(thresh, None, iterations=12)
        # thresh = cv2.dilate(thresh, None, iterations=2)
# Detect the contours in the image
        cnts = cv2.findContours(thresh.copy(), cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        regions = []
# Loop over the contours
        for c in cnts:
            (w, h) = cv2.boundingRect(c)[2:]
# Store the aspect ratio of the bounding box over the current contour
            aspectRatio = w / float(h)
# Get the bounding box rectangle positions
            rect = cv2.minAreaRect(c)
            box = np.int0(cv2.cv.BoxPoints(rect)) if imutils.is_cv2() else cv2.boxPoints(rect)
# If the aspect ratio matches a criteria and the size of the bounding box is within a defined limit then the detected contour is a UAV
            if (aspectRatio > 1.4 and aspectRatio < 3.2 and w < 640 and w > 80):
                # print(aspectRatio, w, h)
# Append the bounding box region in a list and store the x and y location of the center of the bounding box
                regions.append(box)
                drone_x = int((box[0][0]+box[2][0])/2)
                drone_y = int((box[0][1]+box[2][1])/2)
 # Draw all such detected contours
        for lpBox in regions:
            lpBox = np.array(lpBox).reshape((-1,1,2)).astype(np.int32)
            cv2.drawContours(image, [lpBox], -1, (255, 0, 0), 2)
        line_thickness = 1
        cv2.line(image, (int(image.shape[1]/2), 0), (int(image.shape[1]/2), image.shape[0]), (128, 0, 0),
                 thickness=line_thickness)
        cv2.line(image, (0, int(image.shape[0]/2)), (image.shape[1], int(image.shape[0]/2)), (128, 0, 0),
                 thickness=line_thickness)
# Store all the images with the UAV bounding box detection
        cv2.imwrite(out_image_path + "/" + image_list[i].split("/")[-1], image)
# Write the drone location in the CSV file
        drone_writer.writerow([drone_x, drone_y, image_list[i].split("/")[-1].split(".")[0]])