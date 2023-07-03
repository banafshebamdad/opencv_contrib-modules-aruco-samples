/**
* Banafshe Bamdad 
* Mo Jul 3, 2023 11:20:25 CET
* This script get the camera intrinsic parameters estimated by Kalibr, 
* which is a 1x4 array, and convert it into a 3x3 matrix
* fx, fy, cx, and cy values com from the following file created by Kalibr
* /media/banafshe/Banafshe_2TB/calibration/Kalibr/my_calibration_RPG_D435i/distortion_model_equidistant
*/

#include <iostream>
#include <opencv2/core.hpp>

cv::Mat convertToCameraMatrix(double fx, double fy, double cx, double cy) {
    cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    cameraMatrix.at<double>(0, 0) = fx;
    cameraMatrix.at<double>(1, 1) = fy;
    cameraMatrix.at<double>(0, 2) = cx;
    cameraMatrix.at<double>(1, 2) = cy;
    return cameraMatrix;
}

int main() {
    // intrinsics computed by Kalibr
    double fx = 350.10253197133255;
    double fy = 350.32543928475854;
    double cx = 425.48700423685244;
    double cy = 242.78893971191576;
    
    cv::Mat cameraMatrix = convertToCameraMatrix(fx, fy, cx, cy);
    
    std::cout << "Camera matrix:\n" << cameraMatrix << std::endl;
    
    return 0;
}
