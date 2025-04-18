import numpy as np
import cv2 as cv
import glob

"""
This file is used to obtain the intrinsic and extrinsic properties of the drone camera.
Given the image captured by the drone's camera with some distortion, this code aims to undistort the images so that the
drone can easily identify the location of gates.
This in turn will help to convert a 2D point to 3D coordinates for effective navigation.
Based on the pattern size, object points are generated to overlay the checkerboard and calibrate the camera.
The calibration returns the camera matrix, distortion coefficients, rotation and translation vectors
"""

# Set the flags
flags = cv.CALIB_CB_ADAPTIVE_THRESH + cv.CALIB_CB_NORMALIZE_IMAGE + cv.CALIB_CB_FAST_CHECK

# Termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

CHECKERBOARD = [10, 9]

# Prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((CHECKERBOARD[0]*CHECKERBOARD[1],3), np.float32)
objp[:,:2] = np.mgrid[0:CHECKERBOARD[0],0:CHECKERBOARD[1]].T.reshape(-1,2)

# Arrays to store object points and image points from all the images.
objpoints = []  # 3d point in real world space
imgpoints = []  # 2d points in image plane.

# Get the location of the captured checkerboard images
images = glob.glob('D:\Purdue\Spring 2025\VIP 37920\Gate_Images\*.png')

for i, fname in enumerate(images):
    img = cv.imread(fname)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    # Find the chess board corners
    ret, corners = cv.findChessboardCorners(gray, (CHECKERBOARD[0], CHECKERBOARD[1]), flags=flags)
    print(ret)

    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)

        corners2 = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners2)

        # Draw and display the corners
        cv.drawChessboardCorners(img, (CHECKERBOARD[0], CHECKERBOARD[1]), corners2, ret)
        # Save the file
        print(f"Image {i+1} being saved")
        output_filename = f"D:\Purdue\Spring 2025\VIP 37920\Calibrated_Images\corners_{i+1}.png"
        cv.imwrite(output_filename, img=img)
        # Show the image
        cv.imshow('img', img)
        cv.waitKey(5000)

# Get the rotation & translation vectors, and distortion coefficients
ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

cv.destroyAllWindows()


# Image 1 = 7,6 (or 10,7 or 7,10 or 6,7)
# Image 2 = 9,7 (or 4,4)
# Image 3 = 8,7 (or 7,8)
# Image 4 =
# Image 5 =
# Image 6 =
# Image 7 = 10,6 (or 6,10)


# Don't use (4,3), (4,9), (4,8), (4,7), (4,6), (4,5), (8,6), (8,5), (8,4) (8,3), (8,8) (8,9), (9,8), (9,6), (6,9), (9,10)
#


# Need to reduce window size for more accurate calibration of corners