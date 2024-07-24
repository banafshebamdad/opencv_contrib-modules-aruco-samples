
/*
 * Author: Banafshe Bamdad
 * Created on Mon May 20 2024 19:16:31 CET
 *
 * Usage: 
 *  g++ -std=c++17 banafshe_generate_ground_truth.cpp -o banafshe_generate_ground_truth `pkg-config --cflags --libs opencv4`
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

// cv::Vec4d rotationMatrixToQuaternion(const cv::Mat& R) {
//     cv::Vec4d q;
//     double trace = R.at<double>(0, 0) + R.at<double>(1, 1) + R.at<double>(2, 2);
//     if (trace > 0) {
//         double s = 0.5 / sqrt(trace + 1.0);
//         q[3] = 0.25 / s;
//         q[0] = (R.at<double>(2, 1) - R.at<double>(1, 2)) * s;
//         q[1] = (R.at<double>(0, 2) - R.at<double>(2, 0)) * s;
//         q[2] = (R.at<double>(1, 0) - R.at<double>(0, 1)) * s;
//     } else {
//         if (R.at<double>(0, 0) > R.at<double>(1, 1) && R.at<double>(0, 0) > R.at<double>(2, 2)) {
//             double s = 2.0 * sqrt(1.0 + R.at<double>(0, 0) - R.at<double>(1, 1) - R.at<double>(2, 2));
//             q[3] = (R.at<double>(2, 1) - R.at<double>(1, 2)) / s;
//             q[0] = 0.25 * s;
//             q[1] = (R.at<double>(0, 1) + R.at<double>(1, 0)) / s;
//             q[2] = (R.at<double>(0, 2) + R.at<double>(2, 0)) / s;
//         } else if (R.at<double>(1, 1) > R.at<double>(2, 2)) {
//             double s = 2.0 * sqrt(1.0 + R.at<double>(1, 1) - R.at<double>(0, 0) - R.at<double>(2, 2));
//             q[3] = (R.at<double>(0, 2) - R.at<double>(2, 0)) / s;
//             q[0] = (R.at<double>(0, 1) + R.at<double>(1, 0)) / s;
//             q[1] = 0.25 * s;
//             q[2] = (R.at<double>(1, 2) + R.at<double>(2, 1)) / s;
//         } else {
//             double s = 2.0 * sqrt(1.0 + R.at<double>(2, 2) - R.at<double>(0, 0) - R.at<double>(1, 1));
//             q[3] = (R.at<double>(1, 0) - R.at<double>(0, 1)) / s;
//             q[0] = (R.at<double>(0, 2) + R.at<double>(2, 0)) / s;
//             q[1] = (R.at<double>(1, 2) + R.at<double>(2, 1)) / s;
//             q[2] = 0.25 * s;
//         }
//     }
//     return q;
// }

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


// Converts rotation and translation vectors to a 4x4 transformation matrix
/*
 * T(cv::Rect(3, 0, 1, 3)): Creates a submatrix of T using the cv::Rect function. 
 * 3: The x-coordinate of the top-left corner of the submatrix (the fourth column).
 * 0: The y-coordinate of the top-left corner of the submatrix (the first row).
 * 1: The width of the submatrix (one column).
 * 3: The height of the submatrix (three rows).
*/
cv::Mat createTransformationMatrix(const cv::Vec3d& rvec, const cv::Vec3d& tvec) {
    cv::Mat R;
    cv::Rodrigues(rvec, R);

    cv::Mat T = cv::Mat::eye(4, 4, R.type());
    R.copyTo(T(cv::Rect(0, 0, 3, 3)));
    cv::Mat(tvec).copyTo(T(cv::Rect(3, 0, 1, 3)));
    return T;
}

// Draw a small circle at the principal point to represent the camera's origin
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

    // Position the text below the circle
    cv::Point textOrg(static_cast<int>(cx) + 15, static_cast<int>(cy) - 15);

    cv::putText(image, text, textOrg, fontFace, fontScale, cv::Scalar(0, 0, 255), thickness);
}

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

    for (size_t i = 0; i < ids.size(); ++i) {
        cv::aruco::drawDetectedMarkers(annotatedImage, corners, ids);

        /*
         The Z-axis appears to go to infinity (is excessively long), it typically indicates an issue 
         with the scale of the length parameter relative to the units of the translation vectors (tvecs) 
         obtained from pose estimation.*/
        cv::drawFrameAxes(annotatedImage, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 1.0);
    }

    // drawCameraOrigin(annotatedImage, cameraMatrix);
    drawCameraAxes(annotatedImage, cameraMatrix);

    cv::imwrite(outputPath, annotatedImage);
}

std::map<int, cv::Vec3d> getMarkerPositions22_40(double tile_size) {
    std::map<int, cv::Vec3d> markerPositions;
    markerPositions[40] = cv::Vec3d(0, 0, 0);
    markerPositions[39] = cv::Vec3d(0, 4 * tile_size, 0);
    markerPositions[38] = cv::Vec3d(0, 9 * tile_size, 0);
    markerPositions[37] = cv::Vec3d(0, 14 * tile_size, 0);
    markerPositions[36] = cv::Vec3d(0, 19 * tile_size, 0);
    markerPositions[35] = cv::Vec3d(0, 24 * tile_size, 0);
    markerPositions[34] = cv::Vec3d(0, 29 * tile_size, 0);
    markerPositions[33] = cv::Vec3d(0, 34 * tile_size, 0);
    markerPositions[32] = cv::Vec3d(0, 39 * tile_size, 0);
    markerPositions[31] = cv::Vec3d(0, 44 * tile_size, 0);
    markerPositions[30] = cv::Vec3d(0, 49 * tile_size, 0);
    markerPositions[29] = cv::Vec3d(0, 54 * tile_size, 0);
    markerPositions[28] = cv::Vec3d(0, 59 * tile_size, 0);
    markerPositions[27] = cv::Vec3d(0, 64 * tile_size, 0);
    markerPositions[26] = cv::Vec3d(0, 69 * tile_size, 0);
    markerPositions[25] = cv::Vec3d(0, 74 * tile_size, 0);
    markerPositions[24] = cv::Vec3d(0, 79 * tile_size, 0);
    markerPositions[23] = cv::Vec3d(0, 84 * tile_size, 0);
    markerPositions[22] = cv::Vec3d(0, 87 * tile_size, 0);

    return markerPositions;
}

std::map<int, cv::Vec3d> getMarkerPositions1_40(double tile_size) {
    std::map<int, cv::Vec3d> markerPositions;
    // one direction
    markerPositions[40] = cv::Vec3d(0, 0, 0);
    markerPositions[39] = cv::Vec3d(0, 4 * tile_size, 0);
    markerPositions[38] = cv::Vec3d(0, 9 * tile_size, 0);
    markerPositions[37] = cv::Vec3d(0, 14 * tile_size, 0);
    markerPositions[36] = cv::Vec3d(0, 19 * tile_size, 0);
    markerPositions[35] = cv::Vec3d(0, 24 * tile_size, 0);
    markerPositions[34] = cv::Vec3d(0, 29 * tile_size, 0);
    markerPositions[33] = cv::Vec3d(0, 34 * tile_size, 0);
    markerPositions[32] = cv::Vec3d(0, 39 * tile_size, 0);
    markerPositions[31] = cv::Vec3d(0, 44 * tile_size, 0);
    markerPositions[30] = cv::Vec3d(0, 49 * tile_size, 0);
    markerPositions[29] = cv::Vec3d(0, 54 * tile_size, 0);
    markerPositions[28] = cv::Vec3d(0, 59 * tile_size, 0);
    markerPositions[27] = cv::Vec3d(0, 64 * tile_size, 0);
    markerPositions[26] = cv::Vec3d(0, 69 * tile_size, 0);
    markerPositions[25] = cv::Vec3d(0, 74 * tile_size, 0);
    markerPositions[24] = cv::Vec3d(0, 79 * tile_size, 0);
    markerPositions[23] = cv::Vec3d(0, 84 * tile_size, 0);
    markerPositions[22] = cv::Vec3d(0, 87 * tile_size, 0);

    // return path
    double x = 2 * 0.4305; //meter
    double top_margin = 2 * 0.1145; //meter
    double id1_y = 0.4255; //meter

    markerPositions[21] = cv::Vec3d(x, (86 * tile_size) + top_margin, 0); 
    markerPositions[20] = cv::Vec3d(x, (83 * tile_size) + top_margin, 0);
    markerPositions[19] = cv::Vec3d(x, (78 * tile_size) + top_margin, 0);
    markerPositions[18] = cv::Vec3d(x, (73 * tile_size) + top_margin, 0);
    markerPositions[17] = cv::Vec3d(x, (68 * tile_size) + top_margin, 0);
    markerPositions[16] = cv::Vec3d(x, (63 * tile_size) + top_margin, 0);
    markerPositions[15] = cv::Vec3d(x, (58 * tile_size) + top_margin, 0);
    markerPositions[14] = cv::Vec3d(x, (53 * tile_size) + top_margin, 0);
    markerPositions[13] = cv::Vec3d(x, (48 * tile_size) + top_margin, 0);
    markerPositions[12] = cv::Vec3d(x, (43 * tile_size) + top_margin, 0);
    markerPositions[11] = cv::Vec3d(x, (38 * tile_size) + top_margin, 0);
    markerPositions[10] = cv::Vec3d(x, (33 * tile_size) + top_margin, 0);
    markerPositions[9] = cv::Vec3d(x, (28 * tile_size) + top_margin, 0);
    markerPositions[8] = cv::Vec3d(x, (23 * tile_size) + top_margin, 0);
    markerPositions[7] = cv::Vec3d(x, (19 * tile_size) + top_margin, 0);
    markerPositions[6] = cv::Vec3d(x, (13 * tile_size) + top_margin, 0);
    markerPositions[5] = cv::Vec3d(x, (10 * tile_size) + top_margin, 0);
    markerPositions[4] = cv::Vec3d(x, (7 * tile_size) + top_margin, 0);
    markerPositions[3] = cv::Vec3d(x, (4 * tile_size) + top_margin, 0);
    markerPositions[2] = cv::Vec3d(x, tile_size + top_margin, 0);
    markerPositions[1] = cv::Vec3d(x, - (2 * id1_y), 0);

    return markerPositions;
}

std::map<int, cv::Vec3d> getMarkerPositions1_21(double tile_size) {
    std::map<int, cv::Vec3d> markerPositions;
    // markerPositions[21] = cv::Vec3d(0, 0, 0); // distance to 20, 3 tiles
    markerPositions[20] = cv::Vec3d(0, 0, 0);
    markerPositions[19] = cv::Vec3d(0, 5 * tile_size, 0);
    markerPositions[18] = cv::Vec3d(0, 10 * tile_size, 0);
    markerPositions[17] = cv::Vec3d(0, 15 * tile_size, 0);
    markerPositions[16] = cv::Vec3d(0, 20 * tile_size, 0);
    markerPositions[15] = cv::Vec3d(0, 25 * tile_size, 0);
    markerPositions[14] = cv::Vec3d(0, 30 * tile_size, 0);
    markerPositions[13] = cv::Vec3d(0, 35 * tile_size, 0);
    markerPositions[12] = cv::Vec3d(0, 40 * tile_size, 0);
    markerPositions[11] = cv::Vec3d(0, 45 * tile_size, 0);
    markerPositions[10] = cv::Vec3d(0, 50 * tile_size, 0);
    markerPositions[9] = cv::Vec3d(0, 55 * tile_size, 0);
    markerPositions[8] = cv::Vec3d(0, 60 * tile_size, 0);
    markerPositions[7] = cv::Vec3d(0, 65 * tile_size, 0);
    markerPositions[6] = cv::Vec3d(0, 70 * tile_size, 0);
    markerPositions[5] = cv::Vec3d(0, 73 * tile_size, 0);
    markerPositions[4] = cv::Vec3d(0, 76 * tile_size, 0);
    markerPositions[3] = cv::Vec3d(0, 79 * tile_size, 0);
    markerPositions[2] = cv::Vec3d(0, 82 * tile_size, 0);
    markerPositions[1] = cv::Vec3d(0, 84 * tile_size, 0);

    return markerPositions;
}

int main() {

    // Initialization
    // std::string imageDirectory = "/media/banafshe/c1710f43-f3d3-4655-aa3e-24baac02544e/home/banafshe/Desktop/junk/images/undistorted_microsec";
    std::string imageDirectory = "/home/banafshe/Desktop/44_markers";
    // std::string imageDirectory = "/media/banafshe/c1710f43-f3d3-4655-aa3e-24baac02544e/home/banafshe/Desktop/junk/images/undistorted_one_way";
    // std::string imageDirectory = "/media/banafshe/c1710f43-f3d3-4655-aa3e-24baac02544e/home/banafshe/Desktop/junk/images/undistorted_return";
    
    std::string outputDirectory = imageDirectory + "/annotated_images";
    std::string posesFile = outputDirectory + "/ground_truth.txt";
    float markerLength = 0.175; // 17.5cm = 0.175m
    double tile_size = 0.54; // size of the tiles in meter
    int lastMarkerId2Detect = 21; //from 21 to 40: Marjers in the lesft sides
    
    // My images are already undistorted and I do not need to apply any additional distortion correction.
    // So, there is no distortion to account for during the pose estimation process.
    // cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
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

    posesOutput << "# timestamp tx ty tz qx qy qz qw\n";

    // Define marker positions in global coordinate system
    // The markers are placed in a straight line with a known distance between them.
    /*
     I placed the markers (17.5 cm, printed on A4 paper) on the top-left side of the tiles, 
     with a length of 54 cm, 2, 3, 4, or 5 tiles apart. 
     Marker IDs start from 40 down to 22 in one direction, and from 21 to 1 on the return. 
     I consider the center of Marker ID=40 as the origin of the global frame.
     */
    
    std::map<int, cv::Vec3d> markerPositions = getMarkerPositions1_40(tile_size);
    // std::map<int, cv::Vec3d> markerPositions = getMarkerPositions22_40(tile_size);

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
    // for (const auto& entry : fs::directory_iterator(imageDirectory)) {
        // std::string imagePath = entry.path().string();
        std::string imagePathStr = imagePath.string();
        cv::Mat image = cv::imread(imagePathStr);
        // cv::Mat image = cv::imread(imagePath);
        cv::Mat outputImage = image.clone();

        std::string fileName = imagePath.filename().string(); // Extracts the file name
        std::cout << "File name: " << fileName << std::endl;

        if (image.empty()) {
            std::cerr << "Image not found or unable to load: " << imagePath << std::endl;
            continue;
        }

        // Detect ArUco markers in the image
        std::vector<std::vector<cv::Point2f>> corners, rejectedCandidates;
        std::vector<int> ids;
        detector.detectMarkers(image, corners, ids, rejectedCandidates);

        // std::cout << "Detected marker corners:\n";
        // for (size_t i = 0; i < corners.size(); ++i) {
        //     std::cout << "Marker ID: " << ids[i] << "\n";
        //     for (size_t j = 0; j < corners[i].size(); ++j) {
        //         std::cout << "Corner " << j << ": (" << corners[i][j].x << ", " << corners[i][j].y << ")\n";
        //     }
        // }

        if (ids.empty()) {
            std::cerr << "No markers detected in image: " << imagePath << std::endl;
            continue;
        }

        std::vector<cv::Vec3d> rvecs, tvecs; // ??? the marker's orientation and position relative to the camera.
        // tvecs: the position of the marker's origin (center) in the camera coordinate system
        // markerLength=17.5cm=0.175m
        // Estimate the pose of each detected marker relative to the camera
        cv::aruco::estimatePoseSingleMarkers(corners, markerLength, cameraMatrix, distCoeffs, rvecs, tvecs);

        for (size_t i = 0; i < ids.size(); ++i) {

            int detectedMarkerId = ids[i];
            // if (detectedMarkerId > 20) {
            // if (detectedMarkerId > lastMarkerId2Detect) { // ID 40-22
            // if (detectedMarkerId < 21) { // return path ID 20-1
            if (detectedMarkerId < 50) { // both directions
                cv::Vec3d rvec = rvecs[i];
                cv::Vec3d tvec = tvecs[i];

                // std::cout << "Detected marker ID: " << detectedMarkerId << std::endl;
                // std::cout << "Rotation vector: " << rvec << std::endl;
                // std::cout << "Translation vector: " << tvec << std::endl;

                // Create the transformation matrix for the camera pose relative to the marker center
                cv::Mat T_marker_center_to_camera = createTransformationMatrix(rvec, tvec);

                // Transformation from the center of the marker to the top-left corner
                // cv::Mat T_center_to_topleft = cv::Mat::eye(4, 4, CV_64F);
                // T_center_to_topleft.at<double>(0, 3) = -0.1; // half of marker length (0.2m / 2)
                // T_center_to_topleft.at<double>(1, 3) = 0.1;

                // cv::Mat T_marker_topleft_to_camera = T_marker_center_to_camera * T_center_to_topleft.inv();

                if (markerPositions.find(detectedMarkerId) != markerPositions.end()) {
                    cv::Vec3d markerPosition = markerPositions[detectedMarkerId];

                    std::cout << "MarkerId: " << detectedMarkerId << ", markerPosition: " << markerPosition << std::endl;

                    cv::Mat T_center_to_global = cv::Mat::eye(4, 4, CV_64F);
                    T_center_to_global.at<double>(0, 3) = markerPosition[0];
                    T_center_to_global.at<double>(1, 3) = markerPosition[1];
                    T_center_to_global.at<double>(2, 3) = markerPosition[2];

                    // Using the known positions of the markers, transform the camera pose to the global coordinate system.
                    // cv::Mat T_camera_to_global = T_topleft_to_global * T_marker_topleft_to_camera.inv();
                    cv::Mat T_camera_to_global = T_center_to_global * T_marker_center_to_camera.inv();

                    // std::cout << "Camera pose in global frame:\n" << T_camera_to_global << std::endl;

                    // Extracts the translation and rotation from the transformation matrix
                    cv::Vec3d translation(T_camera_to_global.at<double>(0, 3), T_camera_to_global.at<double>(1, 3), T_camera_to_global.at<double>(2, 3));
                    cv::Mat rotationMatrix = T_camera_to_global(cv::Rect(0, 0, 3, 3));

                    // converts the rotation matrix to a quaternion
                    cv::Vec4d quaternion = rotationMatrixToQuaternion(rotationMatrix);

                    // writes the pose information to the output file
                    posesOutput << fs::path(imagePath).stem().string() << " "
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
        }

        // Saves the annotated image
        std::string outputImagePath = outputDirectory + "/" + imagePath.filename().string();
        saveAnnotatedImage(image, ids, corners, rvecs, tvecs, cameraMatrix, distCoeffs, outputImagePath);
    }

    posesOutput.close();

    return 0;
}


