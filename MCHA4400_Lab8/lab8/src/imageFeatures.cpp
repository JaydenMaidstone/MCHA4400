#include <iostream>
#include <vector>
#include <algorithm>

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>

#include "imageFeatures.h"

#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/aruco.hpp>
//#include "imagefeatures.h"
#include <opencv2/core/hal/interface.h>
#include <opencv2/core/base.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <vector>
#include <array>
#include <algorithm>

// Custom comparator function for sorting in descending order
    bool compareResponseValues(const std::array<float, 3>& a, const std::array<float, 3>& b) {
    return a[2] > b[2]; // Compare the Harris response values
    }

PointFeature::PointFeature()
    : score(0)
    , x(0)
    , y(0)
{}

PointFeature::PointFeature(const double & score_, const double & x_, const double & y_)
    : score(score_)
    , x(x_)
    , y(y_)
{}

bool PointFeature::operator<(const PointFeature & other) const
{
    return (score > other.score);
}

std::vector<PointFeature> detectFeatures(const cv::Mat & img, const int & maxNumFeatures)
{
    std::vector<PointFeature> features;
    // TODO: Lab 8
    // Choose a suitable feature detector
    // Save features above a certain texture threshold
    // Sort features by texture
    // Cap number of features to maxNumFeatures

    cv::Mat imgout = img.clone();

    int blockSize = 3;
    int apertureSize = 5;
    double k = 0.00001;
    int thresh = 10;
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
                    //harrisValues.push_back({ i, j, dst.at<float>(i, j) }); // Use push_back with std::array
                    harrisValues.push_back({ static_cast<float>(j), static_cast<float>(i), dst.at<float>(i, j) });

                    //cv::circle(imgout, cv::Point(j, i), 5, cv::Scalar(0, 255, 0), 2, 8, 0); // Correct the circle function arguments                    NumFeatures++;
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
        //cv::circle( imgout, cv::Point(harrisCorner[1],harrisCorner[0]), 5, cv::Scalar(0, 0, 255), 2, 8, 0 );
        PointFeature feature(harrisCorner[2], harrisCorner[0], harrisCorner[1]);
        features.push_back(feature);
        //std::cout << "idx:  " << i << " at point:  (" << harrisCorner[0] << "," << harrisCorner[1] << ")   Harris Score: " << harrisCorner[2] << std::endl;
    
        // std::string text = "i = " + std::to_string(i);
        // cv::Point textPosition(harrisCorner[1], harrisCorner[0]);
        // cv::putText(imgout, text, textPosition, cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 2);
    }

    return features;
}
