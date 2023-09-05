#include <cstdlib>
#include <iostream>
#include <filesystem>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include "Camera.h"

int main(int argc, char* argv[])
{
    const cv::String keys =
        // Argument names | defaults | help message
        "{help h usage ?  |        | print this message }"
        "{verbose v       |        | display calibration images with detected chessboard}"
        "{box b           |        | draw box on top of chessboard (implies verbose)}"
        ;

    cv::CommandLineParser parser(argc, argv, keys);
    parser.about("MCHA4400 Lab 3");
    
    if (parser.has("help"))
    {
        parser.printMessage();
        return EXIT_SUCCESS;
    }

    // Parse input arguments
    bool hasVerbose = parser.has("verbose");
    bool hasBox = parser.has("box");
    hasVerbose = hasVerbose || hasBox;  // hasBox implies hasVerbose

    // Check for syntax errors
    if (!parser.check())
    {
        parser.printMessage();
        parser.printErrors();
        return EXIT_FAILURE;
    }

    // Get calibration configuration
    std::filesystem::path appPath = parser.getPathToApplication();
    std::filesystem::path configPath = appPath / ".." / "data" / "config.xml";

    if (!std::filesystem::exists(configPath))
    {
        std::cout << "File: " << configPath << " does not exist" << std::endl;
        return EXIT_FAILURE;
    }

    // Read chessboard data using configuration file
    ChessboardData chessboardData(configPath);

    // Calibrate camera from chessboard data
    Camera cam;
    cam.calibrate(chessboardData);

    // Write camera calibration to file
    std::filesystem::path cameraPath = configPath.parent_path() / "camera.xml";
    cv::FileStorage fs(cameraPath.string(), cv::FileStorage::WRITE);
    fs << "camera" << cam;
    fs.release();

    if (hasVerbose)
    {
        if (hasBox)
        {
            chessboardData.drawBoxes(cam);
        }
        else
        {
            chessboardData.drawCorners();
        }

        for (const auto & chessboardImage : chessboardData.chessboardImages)
        {
            cv::imshow("Calibration images (press ESC, q or Q to quit)", chessboardImage.image);
            char c = static_cast<char>(cv::waitKey(0));
            if (c == 27 || c == 'q' || c == 'Q') // ESC, q or Q to quit, any other key to continue
                break;
        }
    }

    return EXIT_SUCCESS;
}

