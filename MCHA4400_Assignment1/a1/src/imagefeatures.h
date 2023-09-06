#ifndef IMAGEFEATURES_H
#define IMAGEFEATURES_H 

#include <opencv2/core.hpp>

cv::Mat detectAndDrawHarris(const cv::Mat & img, int maxNumFeatures);
cv::Mat detectAndDrawShiAndTomasi(const cv::Mat & img, int maxNumFeatures);
cv::Mat detectAndDrawArUco(const cv::Mat & img, int maxNumFeatures);
cv::Mat detectAndDrawORB(const cv::Mat & img, int maxNumFeatures);

#endif