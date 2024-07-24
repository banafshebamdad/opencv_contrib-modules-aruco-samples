/*
 * Author: Banafshe Bamdad
 * Created on Thu May 30 2024 09:41:19 CET
 *
 * This script computes the camera pose with respect to 43 markers, starting from ID 1 to 43. 
 * Marker with ID=0 is not in the field of view of the camera mounted on Wheeled table
 * The center of Marker ID 1 is the origin of the world frame. 
 * The distance between the centers of each two markers is 2 tiles (2 * 59.5 cm). 
 * The camera moves with an almost fixed X=0 and fixed Z=86 cm.
 * 
 * Usage: 
 *  g++ -std=c++17 banafshe_generate_ground_truth_using_44_markers.cpp -o banafshe_generate_ground_truth_using_44_markers `pkg-config --cflags --libs opencv4`
 */

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <vector>

namespace fs = std::filesystem;

// see https://gist.github.com/shubh-agrawal/76754b9bfb0f4143819dbd146d15d4c8
cv::Vec4d rotationMatrixToQuaternion(const cv::Mat& R) {
    cv::Vec4d q;
    double trace = R.at<double>(0, 0) + R.at<double>(1, 1) + R.at<double>(2, 2);
 
    if (trace > 0.0) {
        double s = sqrt(trace + 1.0);
        q[3] = s * 0.5;
        s = 0.5 / s;
        q[0] = (R.at<double>(2, 1) - R.at<double>(1, 2)) * s;
        q[1] = (R.at<double>(0, 2) - R.at<double>(2, 0)) * s;
        q[2] = (R.at<double>(1, 0) - R.at<double>(0, 1)) * s;
    } else {
        int i = R.at<double>(0,0) < R.at<double>(1,1) ? (R.at<double>(1,1) < R.at<double>(2,2) ? 2 : 1) : (R.at<double>(0,0) < R.at<double>(2,2) ? 2 : 0); 
        int j = (i + 1) % 3;  
        int k = (i + 2) % 3;

        double s = sqrt(R.at<double>(i, i) - R.at<double>(j,j) - R.at<double>(k,k) + 1.0);
        q[i] = s * 0.5;
        s = 0.5 / s;

        q[3] = (R.at<double>(k,j) - R.at<double>(j,k)) * s;
        q[j] = (R.at<double>(j,i) + R.at<double>(i,j)) * s;
        q[k] = (R.at<double>(k,i) + R.at<double>(i,k)) * s;
    }
    
    return q;
}


cv::Mat createTransformationMatrix(const cv::Vec3d& rvec, const cv::Vec3d& tvec) {
    cv::Mat R;
    cv::Rodrigues(rvec, R);

    cv::Mat T = cv::Mat::eye(4, 4, R.type());
    R.copyTo(T(cv::Rect(0, 0, 3, 3)));
    cv::Mat(tvec).copyTo(T(cv::Rect(3, 0, 1, 3)));
    return T;
}

void drawCameraOrigin(cv::Mat& image, const cv::Mat& cameraMatrix) {
    double cx = cameraMatrix.at<double>(0, 2);
    double cy = cameraMatrix.at<double>(1, 2);

    cv::circle(image, cv::Point(static_cast<int>(cx), static_cast<int>(cy)), 10, cv::Scalar(0, 0, 255), -1); // Red color

    std::string text = "Principal Point";
    int fontFace = cv::FONT_HERSHEY_SIMPLEX;
    double fontScale = 0.5;
    int thickness = 1;
    int baseline = 0;
    cv::Size textSize = cv::getTextSize(text, fontFace, fontScale, thickness, &baseline);

    cv::Point textOrg(static_cast<int>(cx) + 15, static_cast<int>(cy) - 15);

    cv::putText(image, text, textOrg, fontFace, fontScale, cv::Scalar(0, 0, 255), thickness);
}

// void drawAxes(cv::Mat& image, const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs, const cv::Vec3d& rvec, const cv::Vec3d& tvec, float length) {
//     // Define the points in 3D space for the axes
//     std::vector<cv::Point3f> axisPoints;
//     axisPoints.push_back(cv::Point3f(-length, 0, 0)); // Negative X axis
//     axisPoints.push_back(cv::Point3f(length, 0, 0));  // Positive X axis
//     axisPoints.push_back(cv::Point3f(0, -length, 0)); // Negative Y axis
//     axisPoints.push_back(cv::Point3f(0, length, 0));  // Positive Y axis
//     axisPoints.push_back(cv::Point3f(0, 0, -length)); // Negative Z axis
//     axisPoints.push_back(cv::Point3f(0, 0, length));  // Positive Z axis
//     axisPoints.push_back(cv::Point3f(0, 0, 0));       // Origin

//     // Project 3D points to the image plane
//     std::vector<cv::Point2f> imagePoints;
//     cv::projectPoints(axisPoints, rvec, tvec, cameraMatrix, distCoeffs, imagePoints);

//     // Draw the axes
//     cv::line(image, imagePoints[6], imagePoints[0], cv::Scalar(0, 0, 255), 2); // X negative
//     cv::line(image, imagePoints[6], imagePoints[1], cv::Scalar(0, 0, 255), 2); // X positive
//     cv::line(image, imagePoints[6], imagePoints[2], cv::Scalar(0, 255, 0), 2); // Y negative
//     cv::line(image, imagePoints[6], imagePoints[3], cv::Scalar(0, 255, 0), 2); // Y positive
//     cv::line(image, imagePoints[6], imagePoints[4], cv::Scalar(255, 0, 0), 2); // Z negative
//     cv::line(image, imagePoints[6], imagePoints[5], cv::Scalar(255, 0, 0), 2); // Z positive
// }

void drawCameraAxes(cv::Mat& image, const cv::Mat& cameraMatrix, double length = 0.2) {
    // Extract the principal point (cx, cy) from the camera matrix
    double cx = cameraMatrix.at<double>(0, 2);
    double cy = cameraMatrix.at<double>(1, 2);

    // Define the camera coordinate axes in 3D
    std::vector<cv::Point3d> axesPoints;
    axesPoints.push_back(cv::Point3d(0, 0, 0));       // Origin
    axesPoints.push_back(cv::Point3d(length, 0, 0));  // X-axis
    axesPoints.push_back(cv::Point3d(0, length, 0));  // Y-axis
    axesPoints.push_back(cv::Point3d(0, 0, length));  // Z-axis

    // Project 3D points to 2D image points
    std::vector<cv::Point2d> imagePoints;
    cv::projectPoints(axesPoints, cv::Vec3d(0, 0, 0), cv::Vec3d(0, 0, 0), cameraMatrix, cv::Mat::zeros(4, 1, CV_64F), imagePoints);

    // Draw the camera's principal point
    cv::circle(image, cv::Point(static_cast<int>(cx), static_cast<int>(cy)), 5, cv::Scalar(0, 0, 255), -1); // Red color

    // Draw the camera axes
    cv::line(image, imagePoints[0], imagePoints[1], cv::Scalar(0, 0, 255), 2); // X-axis (red)
    cv::line(image, imagePoints[0], imagePoints[2], cv::Scalar(0, 255, 0), 2); // Y-axis (green)
    cv::line(image, imagePoints[0], imagePoints[3], cv::Scalar(255, 0, 0), 2); // Z-axis (blue)

    std::string text = "The projection of the camera principal point";
    int fontFace = cv::FONT_HERSHEY_SIMPLEX;
    double fontScale = 0.5;
    int thickness = 1;
    cv::putText(image, text, cv::Point(static_cast<int>(cx) + 15, static_cast<int>(cy) - 15), fontFace, fontScale, cv::Scalar(0, 0, 255), thickness);
}

void saveAnnotatedImage(const cv::Mat& image, const std::vector<int>& ids, const std::vector<std::vector<cv::Point2f>>& corners, const std::vector<cv::Vec3d>& rvecs, const std::vector<cv::Vec3d>& tvecs, const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs, const std::string& outputPath) {
    cv::Mat annotatedImage = image.clone();

    // for (size_t i = 0; i < ids.size(); ++i) {
    //     cv::aruco::drawDetectedMarkers(annotatedImage, corners, ids);
    //     drawAxes(annotatedImage, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 1.0); // Draw axes in both directions
    // }
    for (size_t i = 0; i < ids.size(); ++i) {
        cv::aruco::drawDetectedMarkers(annotatedImage, corners, ids);

        /*
         The Z-axis appears to go to infinity (is excessively long), it typically indicates an issue 
         with the scale of the length parameter relative to the units of the translation vectors (tvecs) 
         obtained from pose estimation.*/
        cv::drawFrameAxes(annotatedImage, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 1.0);
    }

    drawCameraOrigin(annotatedImage, cameraMatrix);
    drawCameraAxes(annotatedImage, cameraMatrix);

    cv::imwrite(outputPath, annotatedImage);
}

std::map<int, cv::Vec3d> getMarkerPositions(float tile_size) {
    float double_tile_size = tile_size * 2;
    std::map<int, cv::Vec3d> markerPositions;
    markerPositions[1] = cv::Vec3d(0, 0, 0);
    markerPositions[2] = cv::Vec3d(0, 1 * double_tile_size, 0);
    markerPositions[3] = cv::Vec3d(0, 2 * double_tile_size, 0);
    markerPositions[4] = cv::Vec3d(0, 3 * double_tile_size, 0);
    markerPositions[5] = cv::Vec3d(0, 4 * double_tile_size, 0);
    markerPositions[6] = cv::Vec3d(0, 5 * double_tile_size, 0);
    markerPositions[7] = cv::Vec3d(0, 6 * double_tile_size, 0);
    markerPositions[8] = cv::Vec3d(0, 7 * double_tile_size, 0);
    markerPositions[9] = cv::Vec3d(0, 8 * double_tile_size, 0);
    markerPositions[10] = cv::Vec3d(0, 9 * double_tile_size, 0);
    markerPositions[11] = cv::Vec3d(0, 10 * double_tile_size, 0);
    markerPositions[12] = cv::Vec3d(0, 11 * double_tile_size, 0);
    markerPositions[13] = cv::Vec3d(0, 12 * double_tile_size, 0);
    markerPositions[14] = cv::Vec3d(0, 13 * double_tile_size, 0);
    markerPositions[15] = cv::Vec3d(0, 14 * double_tile_size, 0);
    markerPositions[16] = cv::Vec3d(0, 15 * double_tile_size, 0);
    markerPositions[17] = cv::Vec3d(0, 16 * double_tile_size, 0);
    markerPositions[18] = cv::Vec3d(0, 17 * double_tile_size, 0);
    markerPositions[19] = cv::Vec3d(0, 18 * double_tile_size, 0);
    markerPositions[20] = cv::Vec3d(0, 19 * double_tile_size, 0);
    markerPositions[21] = cv::Vec3d(0, 20 * double_tile_size, 0);
    markerPositions[22] = cv::Vec3d(0, 21 * double_tile_size, 0);
    markerPositions[23] = cv::Vec3d(0, 22 * double_tile_size, 0);
    markerPositions[24] = cv::Vec3d(0, 23 * double_tile_size, 0);
    markerPositions[25] = cv::Vec3d(0, 24 * double_tile_size, 0);
    markerPositions[26] = cv::Vec3d(0, 25 * double_tile_size, 0);
    markerPositions[27] = cv::Vec3d(0, 26 * double_tile_size, 0);
    markerPositions[28] = cv::Vec3d(0, 27 * double_tile_size, 0);
    markerPositions[29] = cv::Vec3d(0, 28 * double_tile_size, 0);
    markerPositions[30] = cv::Vec3d(0, 29 * double_tile_size, 0);
    markerPositions[31] = cv::Vec3d(0, 30 * double_tile_size, 0);
    markerPositions[32] = cv::Vec3d(0, 31 * double_tile_size, 0);
    markerPositions[33] = cv::Vec3d(0, 32 * double_tile_size, 0);
    markerPositions[34] = cv::Vec3d(0, 33 * double_tile_size, 0);
    markerPositions[35] = cv::Vec3d(0, 34 * double_tile_size, 0);
    markerPositions[36] = cv::Vec3d(0, 35 * double_tile_size, 0);
    markerPositions[37] = cv::Vec3d(0, 36 * double_tile_size, 0);
    markerPositions[38] = cv::Vec3d(0, 37 * double_tile_size, 0);
    markerPositions[39] = cv::Vec3d(0, 38 * double_tile_size, 0);
    markerPositions[40] = cv::Vec3d(0, 39 * double_tile_size, 0);
    markerPositions[41] = cv::Vec3d(0, 40 * double_tile_size, 0);
    markerPositions[42] = cv::Vec3d(0, 41 * double_tile_size, 0);
    markerPositions[43] = cv::Vec3d(0, 42 * double_tile_size, 0);

    return markerPositions;
}


int main() {

    // Initialization
    // std::string imageDirectory = "/home/banafshe/Desktop/my_folders/junk/TH_AruCo/44_markers";
    std::string root_path = "/media/banafshe/c1710f43-f3d3-4655-aa3e-24baac02544e/home/banafshe/Documents/projectaria_sandbox/Banafshe_recordings/11_TH_44markers_wageli_leftside_20240531/RGB_images";
    std::string imageDirectory = root_path + "/undistorted_microsec";
    
    std::string outputDirectory = root_path + "/annotated_images";
    std::string posesFile = root_path + "/ground_truth.txt";
    std::string posesIDFile = root_path + "/ground_truth_ID.txt";
    float markerLength = 0.17; // 17cm = 0.17m
    float tile_size = 0.595; // size of the tiles in meter
    float z = 0.86; // The center of the RGB camera was 86 cm from the ground while the glasses were parallel to the ground. The height in the perpendicular position was 70 cm.

    cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 610.638, 0, 717.441, 
                                                     0, 610.638, 710.694, 
                                                     0, 0, 1);
    cv::Mat distCoeffs = cv::Mat::zeros(4, 1, CV_64F);


    std::vector<fs::path> imagePaths;
    for (const auto& entry : fs::directory_iterator(imageDirectory)) {
        imagePaths.push_back(entry.path());
    }
    std::sort(imagePaths.begin(), imagePaths.end());

    if (!fs::exists(outputDirectory)) {
        fs::create_directory(outputDirectory);
    }

    std::ofstream posesOutput(posesFile);
    if (!posesOutput.is_open()) {
        std::cerr << "Unable to open poses file for writing: " << posesFile << std::endl;
        return -1;
    }
    std::ofstream posesIDOutput(posesIDFile);
    if (!posesIDOutput.is_open()) {
        std::cerr << "Unable to open poses IDs file for writing: " << posesIDFile << std::endl;
        return -1;
    }

    posesOutput << "# timestamp tx ty tz qx qy qz qw\n";
    posesIDOutput << "# ID timestamp m_tx m_ty m_tz                     c_tx c_ty c_tz c_qx c_qy c_qz c_qw\n";

    std::map<int, cv::Vec3d> markerPositions = getMarkerPositions(tile_size);

    for (const auto& entry : markerPositions) {
        int markerId = entry.first;
        cv::Vec3d position = entry.second;
        std::cout << "Marker ID: " << markerId
                  << " Position: [" << position[0] << ", " << position[1] << ", " << position[2] << "]"
                  << std::endl;
    }
    std::cin.get();

    cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_7X7_50);
    cv::aruco::DetectorParameters parameters = cv::aruco::DetectorParameters();
    cv::aruco::ArucoDetector detector(dictionary, parameters);

    for (const auto& imagePath : imagePaths) {
        std::string imagePathStr = imagePath.string();
        cv::Mat image = cv::imread(imagePathStr);
        cv::Mat outputImage = image.clone();

        std::string fileName = imagePath.filename().string(); // Extracts the file name
        std::cout << "File name: " << fileName << std::endl;

        if (image.empty()) {
            std::cerr << "Image not found or unable to load: " << imagePath << std::endl;
            continue;
        }

        std::vector<std::vector<cv::Point2f>> corners, rejectedCandidates;
        std::vector<int> ids;
        detector.detectMarkers(image, corners, ids, rejectedCandidates);

        if (ids.empty()) {
            std::cerr << "No markers detected in image: " << imagePath << std::endl;
            continue;
        }

        std::vector<cv::Vec3d> rvecs, tvecs; 
        cv::aruco::estimatePoseSingleMarkers(corners, markerLength, cameraMatrix, distCoeffs, rvecs, tvecs);

        for (size_t i = 0; i < ids.size(); ++i) {

            int detectedMarkerId = ids[i];
            cv::Vec3d rvec = rvecs[i];
            cv::Vec3d tvec = tvecs[i];

            cv::Mat T_marker_center_to_camera = createTransformationMatrix(rvec, tvec);

            if (markerPositions.find(detectedMarkerId) != markerPositions.end()) {
                cv::Vec3d markerPosition = markerPositions[detectedMarkerId];

                std::cout << "MarkerId: " << detectedMarkerId << ", markerPosition: " << markerPosition << std::endl;

                cv::Mat T_center_to_global = cv::Mat::eye(4, 4, CV_64F);
                T_center_to_global.at<double>(0, 3) = markerPosition[0];
                T_center_to_global.at<double>(1, 3) = markerPosition[1];
                T_center_to_global.at<double>(2, 3) = markerPosition[2];

                cv::Mat T_camera_to_global = T_center_to_global * T_marker_center_to_camera.inv();

                cv::Vec3d translation(T_camera_to_global.at<double>(0, 3), T_camera_to_global.at<double>(1, 3), T_camera_to_global.at<double>(2, 3));
                cv::Mat rotationMatrix = T_camera_to_global(cv::Rect(0, 0, 3, 3));

                cv::Vec4d quaternion = rotationMatrixToQuaternion(rotationMatrix);

                posesOutput << fs::path(imagePath).stem().string() << " "
                            << translation[0] << " "
                            << translation[1] << " "
                            << translation[2] << " "
                            << quaternion[0] << " "
                            << quaternion[1] << " "
                            << quaternion[2] << " "
                            << quaternion[3] << "\n";

                posesIDOutput << detectedMarkerId << " " << fs::path(imagePath).stem().string() << " "
                            << tvec[0] << " "
                            << tvec[1] << " "
                            << tvec[2] << " "
                            << "          "
                            << translation[0] << " "
                            << translation[1] << " "
                            << translation[2] << " "
                            << quaternion[0] << " "
                            << quaternion[1] << " "
                            << quaternion[2] << " "
                            << quaternion[3] << "\n";
            } else {
                std::cerr << "Marker ID not found in marker positions." << std::endl;
            }
        }

        // Saves the annotated image
        std::string outputImagePath = outputDirectory + "/" + imagePath.filename().string();
        saveAnnotatedImage(image, ids, corners, rvecs, tvecs, cameraMatrix, distCoeffs, outputImagePath);
    }

    posesOutput.close();
    posesIDOutput.close();

    return 0;
}


