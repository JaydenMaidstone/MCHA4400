#ifndef IMAGEFEATURES_H
#define IMAGEFEATURES_H 

#include <opencv2/core.hpp>

cv::Mat detectAndDrawHarris(const cv::Mat & img, int maxNumFeatures);
cv::Mat detectAndDrawShiAndTomasi(const cv::Mat & img, int maxNumFeatures);
//cv::Mat detectAndDrawArUco(const cv::Mat & img, int maxNumFeatures);
cv::Mat detectAndDrawArUco(const cv::Mat & img, int maxNumFeatures, std::vector<std::vector<cv::Point2f>>& outCorners, std::vector<int>& outIds);
cv::Mat detectAndDrawORB(const cv::Mat & img, int maxNumFeatures);

#endif