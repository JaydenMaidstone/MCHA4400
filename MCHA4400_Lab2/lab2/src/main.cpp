#include <cstdlib>
#include <iostream>
#include <string>  
#include <filesystem>
#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "imagefeatures.h"

int main(int argc, char *argv[])
{
    cv::Mat img, img_gray, imgout;
    cv::Mat dst_norm, dst_norm_scaled;
    const char* source_window = "Source image";
    const char* corners_window = "Corners detected";

    cv::String keys = 
        // Argument names | defaults | help message
        "{help h usage ?  |          | print this message}"
        "{@input          | <none>   | input can be a path to an image or video (e.g., ../data/lab.jpg)}"
        "{export e        |          | export output file to the ./out/ directory}"
        "{N               | 10       | maximum number of features to find}"
        "{detector d      | orb      | feature detector to use (e.g., harris, shi, aruco, orb)}"
        ;
    cv::CommandLineParser parser(argc, argv, keys);
    parser.about("MCHA4400 Lab 2");

    if (parser.has("help"))
    {
        parser.printMessage();
        return EXIT_SUCCESS;
    }

    // Parse input arguments
    bool doExport = parser.has("export");
    int maxNumFeatures = parser.get<int>("N");
    cv::String detector = parser.get<std::string>("detector");
    std::filesystem::path inputPath = parser.get<std::string>("@input");

    // Check for syntax errors
    if (!parser.check())
    {
        parser.printMessage();
        parser.printErrors();
        return EXIT_FAILURE;
    }

    if (!std::filesystem::exists(inputPath))
    {
        std::cout << "File: " << inputPath.string() << " does not exist" << std::endl;
        return EXIT_FAILURE;
    }

    // Prepare output directory
    std::filesystem::path outputDirectory;
    if (doExport)
    {
        std::filesystem::path appPath = parser.getPathToApplication();
        outputDirectory = appPath / ".." / "out";

        // Create output directory if we need to
        if (!std::filesystem::exists(outputDirectory))
        {
            std::cout << "Creating directory " << outputDirectory.string() << std::endl;
            std::filesystem::create_directory(outputDirectory);
        }
        std::cout << "Output directory set to " << outputDirectory.string() << std::endl;
    }

    // Prepare output file path
    std::filesystem::path outputPath;
    if (doExport)
    {
        std::string outputFilename = inputPath.stem().string()
                                   + "_"
                                   + detector
                                   + inputPath.extension().string();
        outputPath = outputDirectory / outputFilename;
        std::cout << "Output name: " << outputPath.string() << std::endl;
    }

    // Check if input is an image or video (or neither)
    cv::VideoCapture cap(inputPath.string());
    bool isVideo = (cap.get(cv::CAP_PROP_FRAME_COUNT)>1); // TODO
    bool isImage = !isVideo; // TODO

    if (!isImage && !isVideo)
    {
        std::cout << "Could not read file: " << inputPath.string() << std::endl;
        return EXIT_FAILURE;
    }

    if (isImage)
    { 
        img = cv::imread( cv::samples::findFile( parser.get<cv::String>( "@input" ) ) );

        if (detector == "harris") {
        // Code for harris detection
        cv::namedWindow( source_window );
        cv::imshow( source_window, img );
        imgout = detectAndDrawHarris(img, maxNumFeatures);
    } else if (detector == "shi") {
        // Code for edge detection
        cv::namedWindow( source_window );
        cv::imshow( source_window, img );
        imgout = detectAndDrawShiAndTomasi(img, maxNumFeatures);
    } else if (detector == "orb") {
        // Code for blob detection
        cv::namedWindow( source_window );
        cv::imshow( source_window, img );
        imgout = detectAndDrawORB(img, maxNumFeatures);
    } else if (detector == "aruco") {
        // Code for aruco detection
        cv::namedWindow( source_window );
        cv::imshow( source_window, img );
        imgout = detectAndDrawArUco(img, maxNumFeatures);
    } else {
        // Code for the default case (invalid detector value)
        std::cout << "Invalid detector value." << std::endl;
        return 1;
    }

        if (doExport)
        {
            // TODO: Write image returned from detectAndDraw to outputPath
            cv::imwrite(outputPath.string(), imgout);
        }
        else
        {
            // TODO: Display image returned from detectAndDraw on screen and wait for keypress
            cv::namedWindow( corners_window );
            cv::imshow( corners_window, imgout );
            cv::waitKey();
        }
    }

    if (isVideo)
    {
        cv::VideoWriter writer;

            // Get the input video's frame width, height, and FPS
            int frameWidth = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH));
            int frameHeight = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT));
            double fps = cap.get(cv::CAP_PROP_FPS);
            int fourcc = cv::VideoWriter::fourcc('m', 'p', '4', 'v');

        if (doExport)
        {
            // TODO: Open output video for writing using the same fps as the input video
            //       and the codec set to cv::VideoWriter::fourcc('m', 'p', '4', 'v')
            printf("Made it here ey");
            

            if (!cap.isOpened())
            {
                std::cout << "Error opening the video file." << std::endl;
                return -1;
            }


            // Create the output video writer
            writer.open(outputPath.string(), fourcc, fps, cv::Size(cap.get(cv::CAP_PROP_FRAME_WIDTH), cap.get(cv::CAP_PROP_FRAME_HEIGHT)));
            printf("Made it here 3");

            // Check if the output video writer is opened successfully
            if (!writer.isOpened())
            {
                 std::cout << "Error opening the output video file." << std::endl;
                 return -1;
            }
        }


        while (true)
        {
            // TODO: Get next frame from input video
            cv::Mat frame;
            cap >> frame;


            // TODO: If frame is empty, break out of the while loop
            if (frame.empty()) {
                break;
            }

        imshow("Frame", frame);
            
        // TODO: Call one of the detectAndDraw functions from imagefeatures.cpp according to the detector option specified at the command line
        // Using Harris detector
        if(detector == "harris")
        {
            printf("Using Harris feature detector\n");
            printf("Features requested: %d", maxNumFeatures);
            imgout = detectAndDrawHarris(frame, maxNumFeatures);
        }


        // Using Shi and Tomasi Detector
        if(detector == "shi")
        {
            printf("Using Shi & Tomasi feature detector\n");
            printf("Features requested: %d", maxNumFeatures);
            imgout = detectAndDrawShiAndTomasi(frame, maxNumFeatures);
        }


        // ORB feature detector
        if(detector == "orb")
        {
            printf("Sus");
           printf("Using ORB feature detector\n");
            printf("Features requested: %d", maxNumFeatures);
            imgout = detectAndDrawORB(frame, maxNumFeatures);
        }



        // ARUCO feature detector
        if(detector == "aruco")
        {
           printf("Using aruco feature detector\n");
            printf("Features requested: %d", maxNumFeatures);
            imgout = detectAndDrawArUco(frame, maxNumFeatures);
        }


            if (doExport)
            {
                // TODO: Write image returned from detectAndDraw to frame of output video
                writer.write(imgout);


            }
            else
            {
                // TODO: Display image returned from detectAndDraw on screen and wait for 1000/fps milliseconds
                cv::imshow("The THING", imgout);
                cv::waitKey(1000 / fps);
            }
        }


        // TODO: release the input video object
        cap.release();


        if (doExport)
        {
            // TODO: release the output video object
            cv::destroyAllWindows();
        }
    }

    return EXIT_SUCCESS;
}



