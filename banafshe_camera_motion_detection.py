#
# banafshe_camera_motion_detection.py
#
# Banafshe Bamdad
# Di Jun 13 08:38:32
# A Python code snippet generated by CHGPT that uses OpenCV to estimate the motion of a camera with respect to an ArUco marker
#
import cv2
import cv2.aruco as aruco
import numpy as np

# Camera parameters (adjust according to your setup)
camera_matrix = np.array([[1.0, 0, 0],
                          [0, 1.0, 0],
                          [0, 0, 1.0]])
dist_coeffs = np.zeros((4, 1))

# ArUco dictionary
aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
parameters = aruco.DetectorParameters_create()

# Capture video from the camera
cap = cv2.VideoCapture(0)

while True:
    # Read frame from the camera
    ret, frame = cap.read()
    
    # Detect ArUco markers
    corners, ids, _ = aruco.detectMarkers(frame, aruco_dict, parameters=parameters)
    
    if ids is not None:
        # Estimate pose for each detected marker
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, 0.05, camera_matrix, dist_coeffs)
        
        for rvec, tvec in zip(rvecs, tvecs):
            # Draw coordinate axes on the marker
            aruco.drawAxis(frame, camera_matrix, dist_coeffs, rvec, tvec, 0.1)
            
            # Convert rotation vector to rotation matrix
            rot_mat, _ = cv2.Rodrigues(rvec)
            
            # Estimate camera pose with respect to the marker
            marker_to_camera = np.hstack((rot_mat, tvec.T))
            marker_to_camera = np.vstack((marker_to_camera, [0, 0, 0, 1]))
            
            camera_to_marker = np.linalg.inv(marker_to_camera)
            
            # Extract translation and rotation from the camera-to-marker matrix
            camera_translation = camera_to_marker[:3, 3]
            camera_rotation = cv2.Rodrigues(camera_to_marker[:3, :3])[0]
            
            print("Camera Translation (marker frame):", camera_translation)
            print("Camera Rotation (marker frame):\n", camera_rotation)
    
    # Display the frame
    cv2.imshow('ArUco Marker Tracking', frame)
    
    # Exit if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the capture and close the window
cap.release()
cv2.destroyAllWindows()
