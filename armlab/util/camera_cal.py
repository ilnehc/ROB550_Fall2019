"""
FILE: camera_cal.py

OVERVIEW

  This is a simple example for using python OpenCV bindings to calibrate
  a point grey camera using a checkerboard.
  
  The code must be modified to match your camera and checkerboard specifics

USAGE

  Run, and move the checkerboard in the field of view and hit <space> to capure
  images.  When enough images have been captured, press ESC to begin calibration.


  Calibration will be saved to calibration.cfg and Video will now display corrected immage.

  Press ESC again to exit. 
  
  Peter Gaskell, U Michigan 2015
"""

import numpy as np
import cv2
import freenect

font = cv2.FONT_HERSHEY_SIMPLEX


h=480
w=640
# size of each checker square [mm]
square_size = 25.6
# pattern of corners on checker board
pattern_size = (8, 6)

# builds array of reference corner locations 
pattern_points = np.zeros((pattern_size[0]*pattern_size[1],3), np.float32) 
pattern_points[:,:2] = np.mgrid[0:pattern_size[0],0:pattern_size[1]].T.reshape(-1,2)
pattern_points *= square_size

# stores the locations of corners on the checkerboard
obj_points = []

# stores the pixel locations of corners for all frames
img_points = []

# termination criteria for finding fit
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

if __name__ == '__main__':
    import sys
    import getopt
    import time


    lineThickness = 2


    # create main window
    cv2.namedWindow("camera",1)
    
    frame_count = 0
    while True:
        ch = 0xFF & cv2.waitKey(10)
        if True:
            # convert Bayer GB to RGB for display
            rgb_frame = cv2.cvtColor(freenect.sync_get_video()[0],cv2.COLOR_RGB2BGR)
            # convert Bayer BG to Grayscale for corner detections
            grey_frame = cv2.cvtColor(rgb_frame,cv2.COLOR_BGR2GRAY)
            frames_str = "Frames for Calibration:" + str(frame_count)
            cv2.putText(rgb_frame, frames_str, (50,50), font, 1, (255,255,255),2, cv2.LINE_AA)
            if ch == 32:
                # find location of corners in image, this is really slow if no corners are seen.
                found, corners = cv2.findChessboardCorners(grey_frame, pattern_size, None)
                if found:
                    # find sub pixel estimate for corner location
                    corners2= cv2.cornerSubPix(grey_frame, corners, (5, 5), (-1, -1), criteria)
                    # add detected corners to RGB image
                    frame_count += 1    
                    #img_points.append(corners.reshape(-1, 2))
                    img_points.append(corners2)
                    obj_points.append(pattern_points)
                    cv2.drawChessboardCorners(rgb_frame, pattern_size, corners, found)
                    cv2.imshow('camera', rgb_frame)
            else:
                cv2.imshow('camera', rgb_frame)
        # continue until ESC
        if ch == 27:
            break
    # Perform actual calibration
    rms, camera_matrix, dist_coefs, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, (w,h), None, None)
    print "Performing calibration with", frame_count, "frames"
    print "RMS Error:", rms
    print "camera matrix:\r\n", camera_matrix
    print "distortion coefficients:\r\n", dist_coefs.ravel()

    f = open('calibration.cfg', 'w')
    f.write("RMS error:\r\n'")
    f.write(str(rms))
    f.write("\r\nrvecs\r\n")
    f.write(str(rvecs))
    f.write("\r\ntvecs\r\n")
    f.write(str(tvecs))
    f.write("\r\nintrinsic matrix:\r\n")
    f.write(str(camera_matrix))
    f.write("\r\ndistortion coefficients:\r\n")
    f.write(str( dist_coefs.ravel()))
    f.close()
    
    
    # Use new calibration to undistort camera feed, exit on ESC
    new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(camera_matrix,dist_coefs,(w,h),1,(w,h))
    while True:
        rgb_frame = cv2.cvtColor(freenect.sync_get_video()[0],cv2.COLOR_RGB2BGR)
        undistorted = cv2.undistort(rgb_frame, camera_matrix, dist_coefs, None, new_camera_matrix)
        cv2.imshow('camera', undistorted)
        ch = 0xFF & cv2.waitKey(10)
        if ch == 27:
            break

    cv2.destroyAllWindows()
