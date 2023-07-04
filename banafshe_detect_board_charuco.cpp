/*
By downloading, copying, installing or using the software you agree to this
license. If you do not agree to this license, do not download, install,
copy or use the software.

                          License Agreement
               For Open Source Computer Vision Library
                       (3-clause BSD License)

Copyright (C) 2013, OpenCV Foundation, all rights reserved.
Third party copyrights are property of their respective owners.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

  * Redistributions of source code must retain the above copyright notice,
    this list of conditions and the following disclaimer.

  * Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.

  * Neither the names of the copyright holders nor the names of the contributors
    may be used to endorse or promote products derived from this software
    without specific prior written permission.

This software is provided by the copyright holders and contributors "as is" and
any express or implied warranties, including, but not limited to, the implied
warranties of merchantability and fitness for a particular purpose are
disclaimed. In no event shall copyright holders or contributors be liable for
any direct, indirect, incidental, special, exemplary, or consequential damages
(including, but not limited to, procurement of substitute goods or services;
loss of use, data, or profits; or business interruption) however caused
and on any theory of liability, whether in contract, strict liability,
or tort (including negligence or otherwise) arising in any way out of
the use of this software, even if advised of the possibility of such damage.
*/

#include <opencv2/highgui.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <vector>
#include <iostream>
#include "aruco_samples_utility.hpp"

using namespace std;
using namespace cv;


namespace {
const char* about = "Pose estimation using a ChArUco board";
const char* keys  =
        "{w        |       | Number of squares in X direction }"
        "{h        |       | Number of squares in Y direction }"
        "{sl       |       | Square side length (in meters) }"
        "{ml       |       | Marker side length (in meters) }"
        "{d        |       | dictionary: DICT_4X4_50=0, DICT_4X4_100=1, DICT_4X4_250=2,"
        "DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, DICT_5X5_250=6, DICT_5X5_1000=7, "
        "DICT_6X6_50=8, DICT_6X6_100=9, DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12,"
        "DICT_7X7_100=13, DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16}"
        "{cd       |       | Input file with custom dictionary }"
        "{c        |       | Output file with calibrated camera parameters }"
        "{v        |       | Input from video or image file, if ommited, input comes from camera }"
        "{ci       | 0     | Camera id if input doesnt come from video (-v) }"
        "{dp       |       | File of marker detector parameters }"
        "{rs       |       | Apply refind strategy }"
        "{r        |       | show rejected candidates too }";
}


/*
 * Banafshe Bamdad
 * Di Jul 4, 2023 12:0:06 CET
*/
void createVideoFromFramess(const std::vector<std::string>& framePaths, const std::string& outputVideoPath, double frameRate) {
    cv::Size frameSize;
    cv::Mat frame;
    std::vector<cv::Mat> frames;

    // Read each image and store them in the frames vector
    for (const std::string& framePath : framePaths) {
        frame = cv::imread(framePath);
        if (!frame.empty()) {
            frames.push_back(frame);
            frameSize = frame.size();
        }
    }

    // Create a video writer
    cv::VideoWriter video(outputVideoPath, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), frameRate, frameSize);

    // Write each frame to the video
    for (const cv::Mat& frame : frames) {
        video.write(frame);
    }

    // Release the video writer
    video.release();

    std::cout << "Video created successfully: " << outputVideoPath << std::endl;
}

/* 
 * Banafshe Bamdad
 * Di Jul 4, 2023 08:49:21 CET
 * I add processFile method to this script. This method get the first frame /full/path/and/name and an integer number that shows the number of frames starting from 
 * given frame. This method creates the full path of num_of_frames of consequtive frames and return them as a vector
*/

#include <string>
#include <sstream>
#include <iomanip>

// & the parameter baseFilename is passed by reference. So, the function directly accesses and multipulates the original object, without making a copy of it. 
// This can be more efficient than passing by value, esp. when dealing with larg objects, as it avoids the overhead of copying the object.
// baseFileName is a reference to a std::string object.
// using const ensures that the baseFileName object is not modified within the processFile function
std::vector<std::string> processFiles(const std::string& baseFileName, int num_of_frames) {
    std::string folderPath, frameName, fileExtension;
    size_t pos = baseFileName.find_last_of("/\\");
    
    //std::string::npos is a static member constant of the std::string class in C++. 
    // It represents a special value that is typically used to indicate the absence or invalidity of a position or index within a string.
    if (pos != std::string::npos) {
        folderPath = baseFileName.substr(0, pos + 1);
        
        frameName = baseFileName.substr(pos + 1);
    }
    
    pos = baseFileName.find_last_of(".");
    if (pos != std::string::npos) {
        fileExtension = baseFileName.substr(pos);
    }
    
    // Extract the frame number
    
    size_t start_pos = frameName.find_first_of("0123456789");
    size_t end_pos = frameName.find_last_of(".");
    size_t length = end_pos - start_pos;
    string frame_num_str = frameName.substr(start_pos, length);
    
    int frameNumber = std::stoi(frame_num_str);
    
    std::vector<std::string> fileNames;
    for (int i = frameNumber; i < frameNumber + num_of_frames; i++) {
        std::ostringstream fileName;
        fileName << folderPath << "frame" << std::setfill('0') << std::setw(4) << i << fileExtension;
        str:string file_to_process = fileName.str();
        fileNames.push_back(file_to_process);
    }
    
    return fileNames;
}

int main(int argc, char *argv[]) {

    CommandLineParser parser(argc, argv, keys);
    parser.about(about);

    if(argc < 6) {
        parser.printMessage();
        return 0;
    }

    int squaresX = parser.get<int>("w");
    int squaresY = parser.get<int>("h");
    float squareLength = parser.get<float>("sl");
    float markerLength = parser.get<float>("ml");
    bool showRejected = parser.has("r");
    bool refindStrategy = parser.has("rs");
    int camId = parser.get<int>("ci");

    String video;
    if(parser.has("v")) {
        video = parser.get<String>("v");
    }

    Mat camMatrix, distCoeffs;
    if(parser.has("c")) {
        bool readOk = readCameraParameters(parser.get<string>("c"), camMatrix, distCoeffs);
        if(!readOk) {
            cerr << "Invalid camera file" << endl;
            return 0;
        }
    }

    Ptr<aruco::DetectorParameters> detectorParams = makePtr<aruco::DetectorParameters>();
    if(parser.has("dp")) {
        FileStorage fs(parser.get<string>("dp"), FileStorage::READ);
        bool readOk = detectorParams->readDetectorParameters(fs.root());
        if(!readOk) {
            cerr << "Invalid detector parameters file" << endl;
            return 0;
        }
    }

    if(!parser.check()) {
        parser.printErrors();
        return 0;
    }

    aruco::Dictionary dictionary = aruco::getPredefinedDictionary(0);
    if (parser.has("d")) {
        int dictionaryId = parser.get<int>("d");
        dictionary = aruco::getPredefinedDictionary(aruco::PredefinedDictionaryType(dictionaryId));
    }
    else if (parser.has("cd")) {
        FileStorage fs(parser.get<std::string>("cd"), FileStorage::READ);
        bool readOk = dictionary.aruco::Dictionary::readDictionary(fs.root());
        if(!readOk) {
            cerr << "Invalid dictionary file" << endl;
            return 0;
        }
    }
    else {
        cerr << "Dictionary not specified" << endl;
        return 0;
    }


    float axisLength = 0.5f * ((float)min(squaresX, squaresY) * (squareLength));

    // create charuco board object
    Ptr<aruco::CharucoBoard> charucoboard = new aruco::CharucoBoard(Size(squaresX, squaresY), squareLength, markerLength, dictionary);
    Ptr<aruco::Board> board = charucoboard.staticCast<aruco::Board>();

     /* Banafshe Bamdad */
    
    int num_of_frames;
    
    std::cout << "Enter the number of consecutive frames to process: ";
    std::cin >> num_of_frames;

    std::vector<std::string> fileNames = processFiles(video, num_of_frames);
    for (std::string element : fileNames) {
        std::cout << element << std::endl;
    }
    
    std::string outputVideoPath = "/home/banafshe/Desktop/output.avi";
    double frameRate = 30.0;

    createVideoFromFramess(fileNames, outputVideoPath, frameRate);
    
    video = outputVideoPath;
    /* B.B */
    
    VideoCapture inputVideo;
    int waitTime;
    if(!video.empty()) {
        inputVideo.open(video);
        waitTime = 0;
    } else {
        inputVideo.open(camId);
        waitTime = 10;
    }
    
    double totalTime = 0;
    int totalIterations = 0;

    while(inputVideo.grab()) {
        Mat image, imageCopy;
        inputVideo.retrieve(image);

        double tick = (double)getTickCount();

        vector< int > markerIds, charucoIds;
        vector< vector< Point2f > > markerCorners, rejectedMarkers;
        vector< Point2f > charucoCorners;
        Vec3d rvec, tvec;

        // detect markers
        aruco::detectMarkers(image, makePtr<aruco::Dictionary>(dictionary), markerCorners, markerIds, detectorParams,
                             rejectedMarkers);

        // refind strategy to detect more markers
        if(refindStrategy)
            aruco::refineDetectedMarkers(image, board, markerCorners, markerIds, rejectedMarkers,
                                         camMatrix, distCoeffs);

        // interpolate charuco corners
        int interpolatedCorners = 0;
        if(markerIds.size() > 0)
            interpolatedCorners =
                aruco::interpolateCornersCharuco(markerCorners, markerIds, image, charucoboard,
                                                 charucoCorners, charucoIds, camMatrix, distCoeffs);

        // estimate charuco board pose
        bool validPose = false;
        if(camMatrix.total() != 0)
            validPose = estimatePoseCharucoBoard(charucoCorners, charucoIds, charucoboard, camMatrix, distCoeffs, rvec, tvec);



        double currentTime = ((double)getTickCount() - tick) / getTickFrequency();
        totalTime += currentTime;
        totalIterations++;
        if(totalIterations % 30 == 0) {
            cout << "Detection Time = " << currentTime * 1000 << " ms "
                 << "(Mean = " << 1000 * totalTime / double(totalIterations) << " ms)" << endl;
        }



        // draw results
        image.copyTo(imageCopy);
        if(markerIds.size() > 0) {
            aruco::drawDetectedMarkers(imageCopy, markerCorners);
        }

        if(showRejected && rejectedMarkers.size() > 0)
            aruco::drawDetectedMarkers(imageCopy, rejectedMarkers, noArray(), Scalar(100, 0, 255));

        if(interpolatedCorners > 0) {
            Scalar color;
            color = Scalar(255, 0, 0);
            aruco::drawDetectedCornersCharuco(imageCopy, charucoCorners, charucoIds, color);
        }

        if(validPose)
            cv::drawFrameAxes(imageCopy, camMatrix, distCoeffs, rvec, tvec, axisLength);
            cout << "banafshe says rvec =" << rvec << endl;
            cout << "banafshe says tvec =" << tvec << endl;

        imshow("out", imageCopy);
        char key = (char)waitKey(waitTime);
        if(key == 27) break;
    }

    return 0;
}
