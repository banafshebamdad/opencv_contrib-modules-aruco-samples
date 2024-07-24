/* Banafshe Bamdad
* Usage: 
* $ g++ -std=c++17 -o banafshe_detect_board_charuco_with_timestamp banafshe_detect_board_charuco_with_timestamp.cpp `pkg-config --cflags --libs opencv4`
* $ ./banafshe_detect_board_charuco_with_timestamp -w=8 -h=8 -sl=0.075 -ml=0.055 -d=5 -dp=detector_params.yml -v=/home/banafshe/Desktop/demonstration/colmap_ws/frames_by_cv/1687071888178114423.jpg -c=banafshe_tutorial_camera_charuco.yml
*/

#include <opencv2/highgui.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <vector>
#include <iostream>
#include "aruco_samples_utility.hpp"

// B.B
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <opencv2/imgproc.hpp>
#include <sstream>
#include <string>

// B.B
namespace fs = std::filesystem;

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

// B.B
std::string wrkdir = "/home/banafshe/Desktop/";
bool show_frames = false;

/*
 * Banafshe Bamdad
 * Do Jul 13, 2023 09:00:33 CET
 * Compute the center of ChArUco board
*/
Point2f centerOfCarucoBoard(vector< Point2f > charucoCorners) {
    Point2f center(0, 0);
    int numCorners = charucoCorners.size();

    for (const Point2f& corner : charucoCorners) {
        center.x += corner.x;
        center.y += corner.y;
    }

    center.x /= numCorners;
    center.y /= numCorners;

    cout << "Center of ChArUco board: (" << center.x << ", " << center.y << ")" << endl;

    return center;
}

/*
 * Banafshe Bamdad
 * Fr. Jul 14, 2023 10:31:51 CET
 * get the rotation vector and compute the scalar component of rotation quaternion
 * !!!
 * !!! Achtubng !!!
 *  This is not mathematically correct. I can change it later.
 * !!!
*/
double computeQw(Vec3d r) {
    
    double qx = r(0);
    double qy = r(1);
    double qz = r(2);

    double abs_qw = std::abs(1 - (qx * qx + qy * qy + qz * qz));
    return (qx >= 0 && qy >= 0 && qz >= 0) ? abs_qw : -abs_qw;
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
 * Do Jul 13, 2023 09:29:48 CET
 * to display vectors on the image
*/
void displayVectorsOnTheImage (Mat imageCopy, Vec3d rvec, Vec3d tvec) {
    string rvecText = "Rotation Vector: " + to_string(rvec[0]) + ", " + to_string(rvec[1]) + ", " + to_string(rvec[2]);
    string tvecText = "Translation Vector: " + to_string(tvec[0]) + ", " + to_string(tvec[1]) + ", " + to_string(tvec[2]);

    Point textPosition(10, 30); // Position to display the text on the image
    Scalar textColor(0, 255, 0);
    int textThickness = 2;
    int textFont = FONT_HERSHEY_SIMPLEX;
    double textScale = 0.8;

    putText(imageCopy, rvecText, textPosition, textFont, textScale, textColor, textThickness);
    putText(imageCopy, tvecText, Point(textPosition.x, textPosition.y + 30), textFont, textScale, textColor, textThickness);

}

/*
 * Banafshe Bamdad
 * Mit Jul 19, 2023 12:47:07 CET
 * Considering that the file name is equal to the timestamp, 
 * this method returns the file name without extension and path.
 */

std::string get_timestamp(std::string full_path) {

    fs::path file_path(full_path);
    // std::string filename = file_path.filename().string();
    std::string filename_without_extension = file_path.stem().string();

    return filename_without_extension;
}

/*
 * Banafshe Bamdad
 * Mi Jul 19, 2023 11:31:00 CET
 * This method takes the first frame /full/path/and/name and an integer number that shows the number of frames starting from 
 * given frame. This method creates the full path of num_of_frames of consequtive frames and return them as a vector
 */

std::vector<std::string> get_N_images(const std::string& start_image_path, int N, std::vector<std::string>& timestamps ) {
    std::vector<std::string> image_paths;

    // Extract the directory and filename from the start_image_path
    fs::path dir_path = fs::path(start_image_path).parent_path();
    std::string filename = fs::path(start_image_path).filename().string();

    // Read the directory and store all the image file paths in image_paths
    for (const auto& entry : fs::directory_iterator(dir_path)) {
        if (entry.is_regular_file() && entry.path().extension() == ".jpg") {
            image_paths.push_back(entry.path().string());
        }
    }

    // Sort the image paths in ascending order
    std::sort(image_paths.begin(), image_paths.end());

    // Find the iterator pointing to the start_image_path in the sorted list
    auto it = std::find(image_paths.begin(), image_paths.end(), start_image_path);
    if (it == image_paths.end()) {
        std::cerr << "Start image not found in the directory.\n";
        return std::vector<std::string>();
    }

    // Calculate the start index of N images after the start point
    size_t start_idx = std::distance(image_paths.begin(), it);
    size_t end_idx = std::min(start_idx + static_cast<size_t>(N), image_paths.size());

    // Extract the image paths of N images after the start point, including the start_image_path
    std::vector<std::string> result(image_paths.begin() + start_idx, image_paths.begin() + end_idx);
    for (size_t i = 0; i < result.size(); i++) {
        std::cout << result[i] << endl;
        timestamps.push_back(get_timestamp(result[i]));
    }

    return result;
}

/*
 * Banafshe Bamdad
 * Do Jul 13, 2023 15:08:59 CET
 * to save the estimated pose in a file
*/
void saveEstimatedPoses (vector<Vec3d> cameraTranslations, vector<Vec3d> cameraRotations, std::vector<std::string> timestamps) {
    std::string filepath = wrkdir + "/stamped_traj_estimate.txt"; // @TODO Banafshe Bamdad Sa Jul 15, 2023 09:16:10 CET: Add path separator based on operating system, instead of hard-coding
    std::ofstream outputFile(filepath);
    if (outputFile.is_open()) {
        for (size_t i = 0; i < cameraTranslations.size(); i++) {
            const auto& translation = cameraTranslations[i];
            const auto& rotation = cameraRotations[i];

            outputFile << timestamps[i] << " ";

            for (int j = 0; j < translation.rows; j++ ) {
                outputFile << translation(j) << " ";
            }
            for (int j = 0; j < rotation.rows; j++ ) {
                outputFile << rotation(j) << " ";
            }
            outputFile << computeQw(rotation);
            outputFile << "\n";
        }
        outputFile.close();
        std::cout << "poses written successfully in " << filepath << endl;
    } else {
        std::cerr << "Error opening " << filepath << endl;
    }
}

int main(int argc, char *argv[]) {

    // B.B to store previos translation vectors
    Vec3d prevTvec;

    // B.B to compte camera pose w.r.t ChArUco board
    Vec3d cameraTvec, cameraRvec;

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
    std::string user_input_show_frames = "N";
    
    std::cout << "Enter working directory to save the output files or press Enter for default. Default value is /home/banafshe/Desktop:";
    std::getline(std::cin, wrkdir);
    if (wrkdir.empty()) {
        wrkdir = "/home/banafshe/Desktop";
    }
    std::cout << "\n working directory: " << wrkdir << endl;

    std::cout << "Show frames (N/y)? Default is No.";
    std::getline(std::cin, user_input_show_frames);
    if (user_input_show_frames == "Y" || user_input_show_frames == "y") {
        show_frames = true;
    } else {
        show_frames = false;
    }
    std::cout << "Enter the number of consecutive frames to process: ";
    std::cin >> num_of_frames;

    std::vector<std::string> timestamps;

    std::vector<std::string> fileNames = get_N_images(video, num_of_frames, timestamps);
    
    std::string outputVideoPath = wrkdir + "/video_from_selected_frames.avi";
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

    // Banafshe Bamdad: to save the trajectory of a camera
    vector<Vec3d> cameraTranslations;
    vector<Vec3d> cameraRotations;

    int imgIndex = 0;
    while(inputVideo.grab()) {
        imgIndex++;
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

        // Banafshe Bamdad
        // cout << "\nMarkerIds: " << endl;
        // for (int mi = 0; mi < markerIds.size(); mi++) {
        //     cout << markerIds.at(mi) << ' ';
        // }

        // cout << "\nmarkerCorners: " << endl;
        // for (int mi = 0; mi < markerCorners.size(); mi++) {
        //     for (int mj = 0; mj < markerCorners[mi].size(); mj++) {
        //         cout << markerCorners[mi][mj] << " ";
        //     }    
        //     cout << endl;
        // }
        // cout << "\n" << endl;

        // B.B

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

        // B.B Pose estimation: determining the position and orientation of ChArUco board in 3D space relative to the camera's coordinate system. 
        if(validPose)
            cv::drawFrameAxes(imageCopy, camMatrix, distCoeffs, rvec, tvec, axisLength);
            cout << "Banafshe: rotation vector (rvec) =" << rvec << endl;
            cout << "Banafshe: translation vector (tvec) =" << tvec << endl;
            cout << "\nThese vectors represent the position and orientation of the board in 3D space (w.r.t the camera coordinate system).\n" << endl;

            // store the camera pose information to store trajectory
            cameraTranslations.push_back(tvec);
            cameraRotations.push_back(rvec);

            // Banafshe bamdad
            if (prevTvec != Vec3d()) {
                // displacement between current and previous translation vector
                Vec3d displacement = tvec - prevTvec;

                // visualize camera motion
                Point2f center = centerOfCarucoBoard(charucoCorners);
                Point startPoint(center.x, center.y);
                Point endPoint(center.x + displacement[0], center.y + displacement[1]);
                Scalar color(0, 255, 0); // Green color
                int thickness = 2;
                int lineType = LINE_MAX;
                int shift = 0;
                double tipLength = 0.1;

            }
            // Update previous translation vector
            prevTvec = tvec;

            displayVectorsOnTheImage (imageCopy, rvec, tvec);

            // Compute the pose of camera w.r.t the board
            Mat cameraRmat;
            Rodrigues(rvec, cameraRmat); // convert rotation vector to rotation matrix

            // compute the inverse transformation to obtain the pose of the camera with respect to the ChArUco board. 
            Mat invCameraRmat = cameraRmat.t(); // Transpose of the rotation matrix
            Vec3d invCameraRvec;
            Rodrigues(invCameraRmat, invCameraRvec); // Convert inverse rotation matrix to rotation vector
            Mat invCameraTvecMat = -invCameraRmat * Mat(tvec); // Convert tvec to Mat and perform matrix multiplication
            Vec3d invCameraTvec(invCameraTvecMat.ptr<double>());

            // B.B

        if (show_frames) { // B.B
            // imshow(std::to_string(imgIndex), imageCopy);
            imshow("Frame", imageCopy);
            char key = (char)waitKey(waitTime);
            if(key == 27) break;
        } else {
            std::cout << "The valuse of show frames is: " << show_frames << endl;
        }
    }

    saveEstimatedPoses (cameraTranslations, cameraRotations, timestamps);
    // B.B

    return 0;
}
