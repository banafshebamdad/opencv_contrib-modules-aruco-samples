/*
 * Author: Banafshe Bamdad
 * Created on Thu May 30 2024 08:58:37 CET
 *
 * This script creates 44 separate A4 pages, each containing a single Aruco marker
 * 
 * Compile:
 *  $ g++ banafshe_create_44_markers.cpp -o banafshe_create_44_markers `pkg-config --cflags --libs opencv4`
 * Usage:
 *  $ ./banafshe_create_44_markers <output directory>
 */

#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <sstream>
#include <iomanip>

using namespace cv;

int main(int argc, char *argv[]) {
    if(argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <output directory>" << std::endl;
        return 1;
    }

    // Output directory for the images
    std::string outputDir = argv[1];

    // A4 paper size in pixels for 300 DPI
    const int A4_WIDTH_PX = 2480;  // 210mm * 300 DPI / 25.4
    const int A4_HEIGHT_PX = 3508; // 297mm * 300 DPI / 25.4

    // Define the dictionary
    aruco::Dictionary dictionary = aruco::getPredefinedDictionary(aruco::DICT_7X7_100);

    // Calculate the maximum marker size that fits on A4 paper with margins
    int marker_size = std::min(A4_WIDTH_PX, A4_HEIGHT_PX) - 200; // 100px margin on each side
    int borderBits = 1;

    // Font settings for the text
    int fontFace = FONT_HERSHEY_SIMPLEX;
    double fontScale = 1;
    int thickness = 2;
    int baseline = 0;

    // Generate and save the markers
    for (int marker_id = 0; marker_id < 44; ++marker_id) {
        // Create a new canvas for each marker
        Mat canvas = Mat::zeros(A4_HEIGHT_PX, A4_WIDTH_PX, CV_8UC1);
        canvas.setTo(Scalar(255));

        // Generate marker image
        Mat markerImg;
        aruco::generateImageMarker(dictionary, marker_id, marker_size, markerImg, borderBits);

        // Calculate offsets to center the marker on the A4 page
        int x_offset = (A4_WIDTH_PX - marker_size) / 2;
        int y_offset = (A4_HEIGHT_PX - marker_size) / 2;

        // Place the marker in the center of the canvas
        Rect roi(x_offset, y_offset, marker_size, marker_size);
        markerImg.copyTo(canvas(roi));

        // Create the marker information text
        std::ostringstream text;
        text << "DICT_7X7_100 - ID: " << marker_id << "/43 - 17cm";

        // Get the text size
        Size textSize = getTextSize(text.str(), fontFace, fontScale, thickness, &baseline);

        // Position the text at the bottom of the page, centered horizontally
        Point textOrg((A4_WIDTH_PX - textSize.width) / 2, A4_HEIGHT_PX - 100);

        // Put the text on the canvas
        putText(canvas, text.str(), textOrg, fontFace, fontScale, Scalar(0), thickness);

        // Create the output filename
        std::ostringstream filename;
        filename << outputDir << "/aruco_marker_" << std::setw(2) << std::setfill('0') << marker_id << ".png";

        // Save the image
        imwrite(filename.str(), canvas);
    }

    return 0;
}
