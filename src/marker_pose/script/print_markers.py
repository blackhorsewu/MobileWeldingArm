#!/usr/bin/env python3
# -*- coding: utf-8 -*-

'''
  Chinese National Engineering Research Centre for Steel Structure
                        (Hong Kong Branch)

  Hong Kong Polytechnic University

  Author: Victor W H Wu
  Date: 14 August 2023.

  File name: print_markers.py

  Description:
    This python script prints aruco markers.

  It requires:
  1. OpenCV

  2. It will use the HP LaserJet Pro M426fdw Laser Printer, which has a resolution of 
     600 DPI (Dots per inch) and work outs to be 2.963 pixel/mm
  3. It uses the 'DICT_6x6_100' aruco dictionary

'''
import cv2
import numpy as np

# Define ArUco dictionary
aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_100)

# Create the first marker
marker_size = 178 # 60mm square; 60 x 2.963 = 177.78
marker_image1 = np.zeros((marker_size, marker_size), dtype=np.uint8)
cv2.aruco.drawMarker(aruco_dict, 2, marker_size, marker_image1, 1)

# Create the second marker
marker_image2 = np.zeros((marker_size, marker_size), dtype=np.uint8)
cv2.aruco.drawMarker(aruco_dict, 3, marker_size, marker_image2, 1)

# Define the vertical spacing between the markers; say 10mm
spacing = 30

# Create a blank image to hold both markers
column_image = 255*np.ones((marker_size * 2 + spacing, marker_size), dtype=np.uint8)

# Place the first marker at the top of the blank image
column_image[0:marker_size, :] = marker_image1

# Place the second marker below the first, with the defined spacing
column_image[marker_size + spacing:, :] = marker_image2

# Display the markers in a column
cv2.imshow('Two ArUco Markers in a Column', column_image)
cv2.waitKey(0)
cv2.destroyAllWindows()

# Save the image if you want to print it
cv2.imwrite('markers_in_column.png', column_image)
