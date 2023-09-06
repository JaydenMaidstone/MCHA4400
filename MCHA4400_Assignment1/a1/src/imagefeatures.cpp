#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/aruco.hpp>
#include "imagefeatures.h"
#include <opencv2/core/hal/interface.h>
#include <opencv2/core/base.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <vector>
#include <array>
#include <algorithm> // Include the algorithm header for std::sort

// Custom comparator function for sorting in descending order
    bool compareResponseValues(const std::array<float, 3>& a, const std::array<float, 3>& b) {
    return a[2] > b[2]; // Compare the Harris response values
    }

cv::Mat detectAndDrawHarris(const cv::Mat & img, int maxNumFeatures)
{
    cv::Mat imgout = img.clone();

    // TODO

    int blockSize = 2;
    int apertureSize = 3;
    double k = 0.04;
    int thresh = 130;
    int max_thresh = 255;
    int NumFeatures = 0;
    const char* source_window = "Source image";
    const char* corners_window = "Corners detected";

    cv::Mat dst_norm, dst_norm_scaled, img_gray;
    cv::cvtColor( img, img_gray, cv::COLOR_BGR2GRAY );
    cv::Mat dst = cv::Mat::zeros( img.size(), CV_32FC1 );
    cv::cornerHarris( img_gray, dst, blockSize, apertureSize, k );
    std::vector<std::array<float, 3>> harrisValues; // Use std::vector instead of std::array
    cv::normalize( dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat() );
    cv::convertScaleAbs( dst_norm, dst_norm_scaled );
    for( int i = 0; i < dst_norm.rows ; i++ )
    {
        for( int j = 0; j < dst_norm.cols; j++ )
        {
            if( (int) dst_norm.at<float>(i,j) > thresh )
                {
                    harrisValues.push_back({static_cast<float>(j), static_cast<float>(i), dst.at<float>(i, j) }); // Use push_back with std::array
                    cv::circle(imgout, cv::Point(j, i), 5, cv::Scalar(0, 255, 0), 2, 8, 0); // Correct the circle function arguments                    NumFeatures++;
                    NumFeatures++;
                }
        }
    }

    // Sort the harrisValues vector using the custom comparator function
    std::sort(harrisValues.begin(), harrisValues.end(), compareResponseValues);

    std::cout << "Using Harris feature detector:" << std::endl;
    std::cout << "Image width:" << imgout.cols << std::endl;
    std::cout << "Image width:" << imgout.rows << std::endl;
    std::cout << "Features requested:" << maxNumFeatures << std::endl;
    std::cout << "Features detected:" << NumFeatures << std::endl;
    // handle if requested features are greater than detected
    if (maxNumFeatures > NumFeatures) {
        std::cout << "Requested features are greater than detected, using detected features." << std::endl;
    }
    // Print the sorted Harris corner values
    for (int i = 0; i < std::min(NumFeatures, maxNumFeatures) && i < harrisValues.size(); i++) {
        const auto& harrisCorner = harrisValues[i];
        cv::circle( imgout, cv::Point(harrisCorner[1],harrisCorner[0]), 5, cv::Scalar(0, 0, 255), 2, 8, 0 );
        std::cout << "idx:  " << i << " at point:  (" << harrisCorner[0] << "," << harrisCorner[1] << ")   Harris Score: " << harrisCorner[2] << std::endl;
    
        std::string text = "i = " + std::to_string(i);
        cv::Point textPosition(harrisCorner[1], harrisCorner[0]);
        cv::putText(imgout, text, textPosition, cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 2);
    }

    return imgout;
}

cv::Mat detectAndDrawShiAndTomasi(const cv::Mat & img, int maxNumFeatures)
{
    cv::Mat imgout = img.clone();

    // TODO
    int blockSize = 2;
    int apertureSize = 3;
    //double k = 0.04;
    double thresholdValue = 0.02; // Adjust this threshold based on your image and requirements
    //int max_thresh = 255;
    int NumFeatures = 0;
    //const char* source_window = "Source image";
    //const char* corners_window = "Corners detected";

    cv::Mat dst_norm, dst_norm_scaled, img_gray;
    cv::cvtColor( img, img_gray, cv::COLOR_BGR2GRAY );
    cv::Mat eigenValues;
    std::vector<std::array<float, 3>> responseValues; // Use std::vector instead of std::array
    cv::cornerMinEigenVal(img_gray, eigenValues, blockSize, apertureSize);
    cv::Mat corners;
    cv::threshold(eigenValues, dst_norm, thresholdValue, 255, cv::THRESH_BINARY);

    for( int i = 0; i < dst_norm.rows ; i++ )
    {
        for( int j = 0; j < dst_norm.cols; j++ )
        {
            if( (int) dst_norm.at<float>(i,j) > thresholdValue)
                {
                    responseValues.push_back({static_cast<float>(j), static_cast<float>(i), eigenValues.at<float>(i, j) }); // Use push_back with std::array
                    cv::circle(imgout, cv::Point(j, i), 5, cv::Scalar(0, 255, 0), 2, 8, 0); // Correct the circle function arguments                    NumFeatures++;
                    NumFeatures++;
                }
        }
    }

    // Sort the harrisValues vector using the custom comparator function
    std::sort(responseValues.begin(), responseValues.end(), compareResponseValues);

    std::cout << "Using ShiTomasi feature detector:" << std::endl;
    std::cout << "Image width:" << imgout.cols << std::endl;
    std::cout << "Image width:" << imgout.rows << std::endl;
    std::cout << "Features requested:" << maxNumFeatures << std::endl;
    std::cout << "Features detected:" << NumFeatures << std::endl;
    // Print the sorted Harris corner values
    for (int i = 0; i < maxNumFeatures && i < responseValues.size(); i++) {
        const auto& Corner = responseValues[i];
        cv::circle( imgout, cv::Point(Corner[1],Corner[0]), 5, cv::Scalar(0, 0, 255), 2, 8, 0 );
        std::cout << "idx:  " << i << " at point:  (" << Corner[0] << "," << Corner[1] << ")   Shi Score: " << Corner[2] << std::endl;
    
        std::string text = "i = " + std::to_string(i);
        cv::Point textPosition(Corner[1], Corner[0]);
        cv::putText(imgout, text, textPosition, cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 2);
    }

    return imgout;
}

cv::Mat detectAndDrawORB(const cv::Mat & img, int maxNumFeatures)
{
    cv::Mat imgout = img.clone();

    // TODO
    double thresholdValue = 0.01; // Adjust this threshold based on your image and requirements
    //int max_thresh = 255;
    int NumFeatures = 0;
    //const char* source_window = "Source image";
    //const char* corners_window = "Corners detected";


    cv::Mat dst_norm, dst_norm_scaled, img_gray;
    //cv::cvtColor( img, img_gray, cv::COLOR_BGR2GRAY );
    cv::Mat eigenValues;
    std::vector<std::array<float, 3>> responseValues; // Use std::vector instead of std::array
    // Create an ORB object
    cv::Ptr<cv::ORB> orb = cv::ORB::create(maxNumFeatures);
    // Detect keypoints
    std::vector<cv::KeyPoint> keypoints;
    orb->detect(img, keypoints);
    // Compute the ORB descriptors
    cv::Mat descriptors;
    orb->compute(img, keypoints, descriptors);
    // Draw keypoints on the image
    cv::drawKeypoints(img, keypoints,  imgout, cv::Scalar(0, 0, 255),
                      cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    
    // Sort the harrisValues vector using the custom comparator function
    std::sort(responseValues.begin(), responseValues.end(), compareResponseValues);
    
    std::cout << "Using ORB feature detector:" << std::endl;
    std::cout << "Image width:" << imgout.cols << std::endl;
    std::cout << "Image height:" << imgout.rows << std::endl;
    std::cout << "Descriptor Width:" << descriptors.cols << std::endl;
    std::cout << "Descriptor Height:" << descriptors.rows << std::endl;

    for (size_t i = 0; i < keypoints.size(); ++i) {
        const cv::KeyPoint& kp = keypoints[i];
        std::cout << "KeyPoint " << i << " descriptor: [";
        for (int j = 0; j < descriptors.cols; ++j) {
            std::cout << static_cast<int>(descriptors.at<uchar>(i, j));
            if (j < descriptors.cols - 1)
                std::cout << ", ";
        }
        std::cout << "]" << std::endl;
    }

    return imgout;
}

cv::Mat detectAndDrawArUco(const cv::Mat & img, int maxNumFeatures)
{
    cv::Mat imgout = img.clone();

    // TODO
    std::vector<std::array<float, 3>> responseValues; // Use std::vector instead of std::array
    cv::aruco::DetectorParameters detectorParams = cv::aruco::DetectorParameters();
    cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    cv::aruco::ArucoDetector detector(dictionary, detectorParams);

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners, rejected;
    detector.detectMarkers(img, corners, ids, rejected);
    cv::aruco::drawDetectedMarkers(imgout, corners, ids);

    std::cout << "Using ShiTomasi feature detector:" << std::endl;
    std::cout << "Image width:" << imgout.cols << std::endl;
    std::cout << "Image width:" << imgout.rows << std::endl;

    // Sort the IDs in ascending order
    std::sort(ids.begin(), ids.end());

    // Print detected markers with their IDs and corner coordinates
    for (size_t i = 0; i < ids.size(); ++i) {
        std::cout << "ID: " << ids[i] << " with corners: ";
        for (size_t j = 0; j < corners[i].size(); ++j) {
            std::cout << "(" << corners[i][j].x << "," << corners[i][j].y << ")";
            if (j < corners[i].size() - 1)
                std::cout << " ";
        }
        std::cout << std::endl;
    }

    return imgout;
}