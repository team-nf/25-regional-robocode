import math
import cv2
import numpy as np
import pygetwindow as gw
import mss
import time
import ctypes
from scipy.spatial.transform import Rotation as R
from networktables import NetworkTables

NetworkTables.initialize(server='127.0.0.1')
vtable = NetworkTables.getTable('Vision/Raw0')

# Set DPI awareness for accurate screen metrics on Windows
user32 = ctypes.windll.user32
user32.SetProcessDPIAware()  
width = user32.GetSystemMetrics(0)
height = user32.GetSystemMetrics(1)

# Specify the window title (replace with your target window's title)
window_title = "AdvantageScope"

camera_calibration_parameters_filename = 'C:/Users/Tuna/Desktop/Lab/FRC/TunaninEfsoKodu/adv_screen_recorder/calibration_chessboard.yaml'

def euler_from_quaternion(x, y, z, w):
  """
  Convert a quaternion into euler angles (roll, pitch, yaw)
  roll is rotation around x in radians (counterclockwise)
  pitch is rotation around y in radians (counterclockwise)
  yaw is rotation around z in radians (counterclockwise)
  """
  t0 = +2.0 * (w * x + y * z)
  t1 = +1.0 - 2.0 * (x * x + y * y)
  roll_x = math.atan2(t0, t1)
      
  t2 = +2.0 * (w * y - z * x)
  t2 = +1.0 if t2 > +1.0 else t2
  t2 = -1.0 if t2 < -1.0 else t2
  pitch_y = math.asin(t2)
      
  t3 = +2.0 * (w * z + x * y)
  t4 = +1.0 - 2.0 * (y * y + z * z)
  yaw_z = math.atan2(t3, t4)
      
  return roll_x, pitch_y, yaw_z # in radians

# Find the window
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

cv_file = cv2.FileStorage(camera_calibration_parameters_filename, cv2.FILE_STORAGE_READ) 
mtx = cv_file.getNode('K').mat()
dst = cv_file.getNode('D').mat()
cv_file.release()

# Set up the ArUco dictionary and parameters
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36H11)
parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
aruco_marker_side_length = 0.1651
with mss.mss() as sct:
    while True:
        start_time = time.time()
        m_id = -1
        # Capture the screen
        img = sct.grab(monitor)
        
        # Convert the captured image to a format OpenCV understands (BGR)
        frame = np.array(img)
        frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)

        # Convert to grayscale for ArUco detection
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Detect ArUco markers in the grayscale image
        (corners, ids, rejected) = detector.detectMarkers(frame)        
        # If markers are detected, draw the bounding box
        if len(corners) > 0:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
         
            # Print the pose for the ArUco marker
             # The pose of the marker is with respect to the camera lens frame.
            # Imagine you are looking through the camera viewfinder, 
            # the camera lens frame's:
            # x-axis points to the right
            # y-axis points straight down towards your toes
            # z-axis points straight ahead away from your eye, out of the camera
            
                  
            rvecs, tvecs, obj_points = cv2.aruco.estimatePoseSingleMarkers(corners,aruco_marker_side_length,mtx,dst)
            

            for i, marker_id in enumerate(ids):
    
                # Store the translation (i.e. position) information
                transform_translation_x = tvecs[i][0][0]
                transform_translation_y = tvecs[i][0][1]
                transform_translation_z = tvecs[i][0][2]
        
                # Store the rotation information
                rotation_matrix = np.eye(4)
                rotation_matrix[0:3, 0:3] = cv2.Rodrigues(np.array(rvecs[i][0]))[0]
                r = R.from_matrix(rotation_matrix[0:3, 0:3])
                quat = r.as_quat()   
                
                # Quaternion format     
                transform_rotation_x = quat[0] 
                transform_rotation_y = quat[1] 
                transform_rotation_z = quat[2] 
                transform_rotation_w = quat[3] 
                
                # Euler angle format in radians
                roll_x, pitch_y, yaw_z = euler_from_quaternion(transform_rotation_x, 
                                                            transform_rotation_y, 
                                                            transform_rotation_z, 
                                                            transform_rotation_w)
                
                roll_x = math.degrees(roll_x)
                pitch_y = math.degrees(pitch_y)
                yaw_z = math.degrees(yaw_z)
                print("transform_translation_x: {}".format(transform_translation_x))
                print("transform_translation_y: {}".format(transform_translation_y))
                print("transform_translation_z: {}".format(transform_translation_z))
                print("roll_x: {}".format(roll_x))
                print("pitch_y: {}".format(pitch_y))
                print("yaw_z: {}".format(yaw_z))
                print()
                if(i==0):
                    vtable.putNumber("x", -transform_translation_x )
                    vtable.putNumber("z", -transform_translation_y)
                    vtable.putNumber("y", -transform_translation_z )
                    vtable.putNumber("roll", roll_x)
                    vtable.putNumber("pitch", pitch_y)
                    vtable.putNumber("yaw", yaw_z)
                    m_id = marker_id
                # Draw the axes on the marker
                cv2.drawFrameAxes(frame, mtx, dst, rvecs[i], tvecs[i], 0.1, 3)
        # Show the processed frame
        vtable.putNumber("marker_id", m_id) 
        cv2.imshow("Window Recording with ArUco Detection", frame)

        # Press 'q' to exit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
