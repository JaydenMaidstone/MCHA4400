
#include <filesystem>
#include "calibrate.h"
#include <iostream>
#include "Camera.h"
#include <cstdlib>
#include <iostream>
#include <filesystem>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

void calibrateCamera(const std::filesystem::path & configPath)
{
    // TODO
    // - Read XML at configPath
    // Get calibration configuration
    std::cout << "Configuration file: " << configPath.string() << std::endl;

    ChessboardData chessboardData(configPath);

    // - Parse XML and extract relevant frames from source video containing the chessboard
    // - Perform camera calibration

    Camera cam;
    cam.calibrate(chessboardData);

    // - Write the camera matrix and lens distortion parameters to camera.xml file in same directory as configPath
    
    std::filesystem::path cameraPath = configPath.parent_path() / "camera.xml";
    cv::FileStorage fs(cameraPath.string(), cv::FileStorage::WRITE);
    fs << "camera" << cam;
    fs.release();

    // - Visualise the camera calibration results


    //chessboardData.drawCorners();
    chessboardData.drawBoxes(cam);


    for (const auto & chessboardImage : chessboardData.chessboardImages)
    {
        cv::imshow("Calibration images (press ESC, q or Q to quit)", chessboardImage.image);
        char c = static_cast<char>(cv::waitKey(0));
        if (c == 27 || c == 'q' || c == 'Q') // ESC, q or Q to quit, any other key to continue
            break;
    }
    
}