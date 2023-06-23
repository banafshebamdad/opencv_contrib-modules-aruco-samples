/**
 * banafshe_camera_motion_detection.cpp
 * Banafshe Bamdad
 * Di Jun 13 08:42:37
 * A CPP code snippet that uses OpenCV to estimate the motion of a camera with respect to an ArUco marker
 * Usage: g++ -o banafshe_camera_motion_detection banafshe_camera_motion_detection.cpp `pkg-config --cflags --libs opencv4`
 * 
 * Output: The camera translation and rotation in the marker frame are printed for each detected marker
*/

/**
 * cv::Mat: the class in OpenCV used to represent matrices and multi-dimensional arrays.
 * cv::Mat::eye(3, 3, CV_64F): static member function of the cv::Mat class that creates a 3x3 identity matrix of type CV_64F (64-bit floating-point).
*/
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include <string>

int main()
{
    cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat distCoeffs = cv::Mat::zeros(4, 1, CV_64F);

    cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    cv::aruco::DetectorParameters parameters = cv::aruco::DetectorParameters();

    cv::aruco::ArucoDetector detector(dictionary, parameters);

    std::string path = "/home/banafshe/Documents/kimera/colmap_Winti_HB/frames_from_rosbag/";  // Set the path to the frames directory

    cv::String extension = "*.jpg";  // Set the file extension of the frames

    cv::String pattern = path + extension;

    std::vector<cv::String> framePaths;
    // to obtain the paths of all the frames matching the specified extension in the directory
    // false: only search for files in the specified directory nor recursively
    cv::glob(pattern, framePaths, false); 

    for (const auto& framePath : framePaths)
    {
        cv::Mat frame = cv::imread(framePath);
        cv::Mat outputImage = frame.clone();
        // frame.copyTo(outputImage);

        if (frame.empty())
        {
            std::cerr << "Failed to read frame: " << framePath << std::endl;
            continue;
        }

        // cv::Point2f class: reperesents a 2D point. Each element in the vector will store the (x, y) coordinates of a detected corner.
        std::vector<std::vector<cv::Point2f>> corners, rejectedCandidates;
        std::vector<int> ids;
        
        //CHGPT  cv::aruco::detectMarkers(frame, dictionary, corners, ids, parameters);
        detector.detectMarkers(frame, corners, ids, rejectedCandidates);

        if (!ids.empty())
        {
            std::vector<cv::Vec3d> rvecs, tvecs;
            /**
             * to estimate the pose (translation and rotation) of ArUco markers based on their detected corners.
             * 0.24: the size of the markers in meters. 
             * the rvecs and tvecs vectors will be populated with the rotation and translation vectors, respectively, 
             * for each detected marker. These vectors provide information about the spatial relationship between the markers and the camera.
             * By estimating the pose of ArUco markers, you can determine the position and orientation of the markers in the camera's coordinate system
            */
            cv::aruco::estimatePoseSingleMarkers(corners, 0.24, cameraMatrix, distCoeffs, rvecs, tvecs);

            for (size_t i = 0; i < ids.size(); ++i)
            {
                // 0.1: It is the length of the axis lines in meters.
                cv::drawFrameAxes(outputImage, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1);

                cv::Mat rotMat;
                // to convert a rotation vector (rvecs[i]) into a rotation matrix (rotMat) using the Rodrigues' rotation formula.
                cv::Rodrigues(rvecs[i], rotMat);

                /**
                 * markerToCamera: a 4x4 transformation matrix that represents the transformation from the marker's coordinate system to the camera's coordinate system.
                */
                cv::Mat markerToCamera = cv::Mat::eye(4, 4, CV_64F);

                // transpose of the rotMat
                cv::Mat rMat = rotMat.t();
                /**
                 * update the rotation component of a transformation matrix
                 * cv::Rect(0, 0, 3, 3): defines a rectangle region within the markerToCamera matrix. 
                */
                rMat.copyTo(markerToCamera(cv::Rect(0, 0, 3, 3)));

                /**
                 * cv::Rect(3, 0, 1, 3): defines a rectangle region within the markerToCamera matrix. 
                 * The rectangle specifies the top-left corner coordinates (3, 0) and the width and height (1, 3) of the region.
                */
                std::cout << "Hello, world!" << tvecs[i] << std::endl;

                // B.B convert std::vector<cv::Vec3d> to a cv::Mat
                cv::Mat mat(3, 1, CV_64FC1);
                mat.at<double>(0, 0) = tvecs[i][0];
                mat.at<double>(1, 0) = tvecs[i][1];
                mat.at<double>(2, 0) = tvecs[i][2];
                mat.copyTo(markerToCamera(cv::Rect(3, 0, 1, 3)));
                // B.B

                cv::Mat cameraToMarker = markerToCamera.inv();

                // translation components extracted from the transformation matrix.
                cv::Vec3d cameraTranslation(cameraToMarker.at<double>(0, 3),
                                            cameraToMarker.at<double>(1, 3),
                                            cameraToMarker.at<double>(2, 3));
                
                cv::Mat cameraRotation;

                // extracts the rotation matrix from a 4x4 camera-to-marker transformation matrix (cameraToMarker) 
                // and converts it into a rotation vector using the Rodrigues' rotation formula.
                cv::Rodrigues(cameraToMarker(cv::Rect(0, 0, 3, 3)), cameraRotation);

                std::cout << "Camera Translation (marker frame): " << cameraTranslation << std::endl;
                std::cout << "Camera Rotation (marker frame):" << std::endl << cameraRotation << std::endl;
            }
        }

        cv::imshow("ArUco Marker Tracking", frame);

        if (cv::waitKey(0) == 'q')
            break;
    }

    cv::destroyAllWindows();

    return 0;
}
