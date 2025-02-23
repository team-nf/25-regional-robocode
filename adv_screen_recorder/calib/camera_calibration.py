#!/usr/bin/env python
  
'''
Welcome to the Camera Calibration Program!
  
This program:
  - Performs camera calibration using a chessboard.
'''
 
from __future__ import print_function
import ctypes # Python 2/3 compatibility 
import cv2 # Import the OpenCV library to enable computer vision
import mss
import numpy as np # Import the NumPy scientific computing library
import glob # Used to get retrieve files that have a specified pattern
import pygetwindow as gw

# Project: Camera Calibration Using Python and OpenCV
# Date created: 12/19/2021
# Python version: 3.8
  
# Chessboard dimensions
number_of_squares_X = 10 # Number of chessboard squares along the x-axis
number_of_squares_Y = 7  # Number of chessboard squares along the y-axis
nX = number_of_squares_X - 1 # Number of interior corners along x-axis
nY = number_of_squares_Y - 1 # Number of interior corners along y-axis
square_size = 0.025 # Size, in meters, of a square side 
  
# Set termination criteria. We stop either when an accuracy is reached or when
# we have finished a certain number of iterations.
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001) 
 
# Define real world coordinates for points in the 3D coordinate frame
# Object points are (0,0,0), (1,0,0), (2,0,0) ...., (5,8,0)
object_points_3D = np.zeros((nX * nY, 3), np.float32)  
  
# These are the x and y coordinates                                              
object_points_3D[:,:2] = np.mgrid[0:nY, 0:nX].T.reshape(-1, 2) 
 
object_points_3D = object_points_3D * square_size
 
# Store vectors of 3D points for all chessboard frames (world coordinate frame)
object_points = []
  
# Store vectors of 2D points for all chessboard frames (camera coordinate frame)
frame_points = []

user32 = ctypes.windll.user32
user32.SetProcessDPIAware()  
width = user32.GetSystemMetrics(0)
height = user32.GetSystemMetrics(1)

window_title = "AdvantageScope"
windows = gw.getWindowsWithTitle(window_title)
if not windows:
    print(f"No window found with title: {window_title}")
    exit()
window = windows[0]
print(f"Recording window: {window_title}")

# Define screen capture region based on the window's coordinates
# (example region: left half of the screen)
monitor = {
    "top": window.top + int(height / 6),
    "left": window.left,
    "width": int(width / 2),
    "height": int(height*3.2/5)
}

def main():
      
  # Get the file path for frames in the current directory
  cap = cv2.VideoCapture('C:/Users/Tuna/Desktop/Lab/FRC/TunaninEfsoKodu/adv_screen_recorder/calib/calib.mp4')
  if not cap.isOpened():
    print("Error: Could not open video file or capture device.")
    exit(1)
  # Go through each chessboard frame, one by one
  frame_count = 0
  with mss.mss() as sct:
    while True:
    
      # Load the image
      img = sct.grab(monitor) 
      # Convert the captured image to a format OpenCV understands (BGR)
      frame = np.array(img)
      # Convert the frame to grayscale
      gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  
    
      # Find the corners on the chessboard
      success, corners = cv2.findChessboardCorners(gray, (nY, nX), None)
      # If the corners are found by the algorithm, draw them
      if success == True:
    
        # Append object points
    
        # Find more exact corner pixels       
        corners_2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)       
          
        # Append frame points
        frame_count += 1
        if frame_count % 40 == 0:
          frame_points.append(corners_2)
          object_points.append(object_points_3D)


        print(frame_count)
        # Draw the corners
        cv2.drawChessboardCorners(frame, (nY, nX), corners_2, success)
        
        # Display the frame. Used for testing.
        
      if len(frame_points) == 30:
        break

        # Display the window for a short period. Used for testing.
      cv2.imshow('Chessboard Calibration', frame)
      cv2.waitKey(1)                                                                                                                     
  # Perform camera calibration to return the camera matrix, distortion coefficients, rotation and translation vectors etc 
  ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(object_points, frame_points, gray.shape[::-1], None, None)
 
  # Save parameters to a file
  cv_file = cv2.FileStorage('C:/Users/Tuna/Desktop/Lab/FRC/TunaninEfsoKodu/adv_screen_recorder/calibration_chessboard.yaml', cv2.FILE_STORAGE_WRITE)
  cv_file.write('K', mtx)
  cv_file.write('D', dist)
  cv_file.release()
  
  # Load the parameters from the saved file
  cv_file = cv2.FileStorage('C:/Users/Tuna/Desktop/Lab/FRC/TunaninEfsoKodu/adv_screen_recorder/calibration_chessboard.yaml', cv2.FILE_STORAGE_READ) 
  mtx = cv_file.getNode('K').mat()
  dst = cv_file.getNode('D').mat()
  cv_file.release()
   
  # Display key parameter outputs of the camera calibration process
  print("Camera matrix:") 
  print(mtx) 
  
  print("\n Distortion coefficient:") 
  print(dist) 
    
  # Close all windows
  cv2.destroyAllWindows() 
      
if __name__ == '__main__':
  print(__doc__)
  main()