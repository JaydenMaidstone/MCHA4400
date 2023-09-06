#include <cassert>
#include <cstddef>
#include <cmath>
#include <limits>
#include <iostream>
#include <vector>
#include <filesystem>
#include <regex>
#include <opencv2/core/types.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/persistence.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/calib3d.hpp>
#include "Camera.h"
#include <opencv2/core.hpp>
#include <vector>
#include <cstdlib>
#include <iostream>
#include <string>  
#include <filesystem>
#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>


void Chessboard::write(cv::FileStorage& fs) const
{
    fs << "{"
       << "grid_width"  << boardSize.width
       << "grid_height" << boardSize.height
       << "square_size" << squareSize
       << "}";
}

void Chessboard::read(const cv::FileNode& node)
{
    node["grid_width"]  >> boardSize.width;
    node["grid_height"] >> boardSize.height;
    node["square_size"] >> squareSize;
}

std::vector<cv::Point3f> Chessboard::gridPoints() const
{
    std::vector<cv::Point3f> rPNn_all;
    rPNn_all.reserve(boardSize.height*boardSize.width);
    for (int i = 0; i < boardSize.height; ++i)
        for (int j = 0; j < boardSize.width; ++j)
            rPNn_all.push_back(cv::Point3f(j*squareSize, i*squareSize, 0));   
    return rPNn_all; 
}

std::ostream & operator<<(std::ostream & os, const Chessboard & chessboard)
{
    return os << "boardSize: " << chessboard.boardSize << ", squareSize: " << chessboard.squareSize;
}

ChessboardImage::ChessboardImage(const cv::Mat & image_, const Chessboard & chessboard, const std::filesystem::path & filename_)
    : image(image_)
    , filename(filename_)
    , isFound(false)
{
    // TODO:
    //  - Detect chessboard corners in image and set the corners member
    cv::Mat grayImage;
    cv::cvtColor(image, grayImage, cv::COLOR_BGR2GRAY);
    // Detect chessboard corners in the image
    std::vector<cv::Point2f> corners;
    isFound = cv::findChessboardCorners(image, chessboard.boardSize, corners, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);

    //  - (optional) Do subpixel refinement of detected corners
    if (isFound)
    {
        // Refine the corners using subpixel refinement
        cv::TermCriteria criteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1);
        cv::cornerSubPix(grayImage, corners, cv::Size(11, 11), cv::Size(-1, -1), criteria);
        this->corners = corners;
    }
}


void ChessboardImage::drawCorners(const Chessboard & chessboard)
{
    cv::drawChessboardCorners(image, chessboard.boardSize, corners, isFound);
}

/*template <typename T>
std::vector<T> linspace(T start, T end, size_t num)
{
    std::vector<T> result;
    if (num == 0)
        return result;

    T step = (end - start) / static_cast<T>(num - 1);
    for (size_t i = 0; i < num - 1; ++i)
    {
        result.push_back(start + static_cast<T>(i) * step);
    }
    result.push_back(end); // Include the 'end' value
    return result;
}*/

void interpolateLine(const cv::Point3d & startPoint, const cv::Point3d & endPoint, const Chessboard & chessboard, const Camera & camera, cv::Mat &image, const Pose & cameraPose, const cv::Scalar & colour)
{
    // Clear the worldVertices vector to start with an empty vector
    std::vector<cv::Vec2d> imageVertices;
    std::vector<cv::Vec3d> worldVertices;
    worldVertices.clear();
    worldVertices.push_back(cv::Vec3d(startPoint));

    // Calculate the number of intervals between the points (excluding the start and end points)
    int numIntervals = 10;

    // Calculate the step size in each dimension
    double stepX = (endPoint.x - startPoint.x) / (numIntervals + 1);
    double stepY = (endPoint.y - startPoint.y) / (numIntervals + 1);
    double stepZ = (endPoint.z - startPoint.z) / (numIntervals + 1);

    // Generate the points between the start and end points
    for (int i = 1; i <= numIntervals; ++i)
    {
        double x = startPoint.x + i * stepX;
        double y = startPoint.y + i * stepY;
        double z = startPoint.z + i * stepZ;
        //std::cout << x << y << z << std::endl;
        worldVertices.push_back(cv::Vec3d(cv::Point3d(x, y, z)));
    }
    worldVertices.push_back(cv::Vec3d(endPoint));
    //std::cout << worldVertices[0] << std::endl;

    for (size_t i = 0; i < worldVertices.size() - 1; ++i)
    {
         if (camera.isWorldWithinFOV(worldVertices[i], cameraPose) && camera.isWorldWithinFOV(worldVertices[i + 1], cameraPose))
        {
            cv::Vec2d rQOi = camera.worldToPixel(worldVertices[i], cameraPose);
            cv::Vec2d rQOi1 = camera.worldToPixel(worldVertices[i + 1], cameraPose);
            cv::Point currentPoint(rQOi[0],rQOi[1]);
            cv::Point nextPoint(rQOi1[0],rQOi1[1]);
            //std::cout << currentPoint << nextPoint << std::endl;
            cv::line(image,currentPoint, nextPoint, colour, 3);
        }
        else
        {
            //std::cout << "failed" << worldVertices[i] << worldVertices[i+1] << std::endl;
        }
    }
    //std::cout << "worldVerticessize " << worldVertices.size() << "  imageVerticessize " << imageVertices.size() << std::endl;
}

void ChessboardImage::drawBox(const Chessboard & chessboard, const Camera & camera)
{
    // Define the height of the rectangular prism (0.23 meters as mentioned in the task)
    double boxHeight = -0.23;
    
        interpolateLine(cv::Point3f(0, 0, 0), cv::Point3f(0, 6*0.022, 0), chessboard, camera, image, cameraPose, cv::Scalar(255 , 0, 0));
        interpolateLine(cv::Point3f(0, 0, 0), cv::Point3f(9*0.022, 0, 0), chessboard, camera, image, cameraPose, cv::Scalar(0, 255, 0));
        interpolateLine(cv::Point3f(0, 6*0.022, 0), cv::Point3f(9*0.022, 6*0.022, 0), chessboard, camera, image, cameraPose, cv::Scalar(0 , 255, 0));
        interpolateLine(cv::Point3f(9*0.022, 0, 0), cv::Point3f(9*0.022, 6*0.022, 0), chessboard, camera, image, cameraPose, cv::Scalar(255 , 0, 0));
        
        interpolateLine(cv::Point3f(0, 0, boxHeight), cv::Point3f(0, 6*0.022, boxHeight), chessboard, camera, image, cameraPose, cv::Scalar(255 , 0, 0));
        interpolateLine(cv::Point3f(0, 0, boxHeight), cv::Point3f(9*0.022, 0, boxHeight), chessboard, camera, image, cameraPose, cv::Scalar(0 , 255, 0));
        interpolateLine(cv::Point3f(0, 6*0.022, boxHeight), cv::Point3f(9*0.022, 6*0.022, boxHeight), chessboard, camera, image, cameraPose, cv::Scalar(0 , 255, 0));
        interpolateLine(cv::Point3f(9*0.022, 0, boxHeight), cv::Point3f(9*0.022, 6*0.022, boxHeight), chessboard, camera, image, cameraPose, cv::Scalar(255 , 0, 0));
        
        interpolateLine(cv::Point3f(0, 0, 0), cv::Point3f(0, 0, boxHeight), chessboard, camera, image, cameraPose, cv::Scalar(0 , 0, 255));
        interpolateLine(cv::Point3f(0, 6*0.022, 0), cv::Point3f(0, 6*0.022, boxHeight), chessboard, camera, image, cameraPose, cv::Scalar(0 , 255, 255));
        interpolateLine(cv::Point3f(9*0.022, 0, 0), cv::Point3f(9*0.022, 0, boxHeight), chessboard, camera, image, cameraPose, cv::Scalar(0 , 0, 255));
        interpolateLine(cv::Point3f(9*0.022, 6*0.022, 0), cv::Point3f(9*0.022, 6*0.022, boxHeight), chessboard, camera, image, cameraPose, cv::Scalar(0 , 255, 255));
}


void ChessboardImage::recoverPose(const Chessboard & chessboard, const Camera & camera)
{
    std::vector<cv::Point3f> rPNn_all = chessboard.gridPoints();

    cv::Mat Thetacn, rNCc;
    cv::solvePnP(rPNn_all, corners, camera.cameraMatrix, camera.distCoeffs, Thetacn, rNCc);

    cv::Mat Rcn;
    cv::Rodrigues(Thetacn, Rcn);
    cameraPose.Rnc = cv::Mat(Rcn.t());
    cameraPose.rCNn = cv::Mat(-Rcn.t()*rNCc);
}

ChessboardData::ChessboardData(const std::filesystem::path & configPath)
{
    assert(std::filesystem::exists(configPath));
    cv::FileStorage fs(configPath.string(), cv::FileStorage::READ);
    assert(fs.isOpened());
    cv::FileNode node = fs["chessboard_data"];
    node["chessboard"] >> chessboard;
    std::cout << "Chessboard: " << chessboard << std::endl;

    std::string pattern;
    node["file_regex"] >> pattern;
    fs.release();
    std::regex re(pattern, std::regex_constants::basic | std::regex_constants::icase);
    
    // Populate chessboard images from regex
    std::filesystem::path root = configPath.parent_path();
    std::cout << "Scanning directory " << root.string() << " for file pattern \"" << pattern << "\"" << std::endl;
    chessboardImages.clear();
    if (std::filesystem::exists(root) && std::filesystem::is_directory(root))
    {
        for (const auto & p : std::filesystem::recursive_directory_iterator(root))
        {
            if (std::filesystem::is_regular_file(p))
            {
                if (std::regex_match(p.path().filename().string(), re))
                {
                    std::cout << "Loading " << p.path().filename().string() << "..." << std::endl;
                    std::cout << "p.path().filename() " << p.path().filename() << "..." << std::endl;
                    std::cout << "p.path().string() " << p.path().string() << "..." << std::endl;

                    cv::VideoCapture cap( p.path().string());
                    bool isVideo = (cap.get(cv::CAP_PROP_FRAME_COUNT)>1); // TODO
                    bool isImage = !isVideo; // TODO

                    // if (!isImage && !isVideo)
                    // {
                    //     std::cout << "Could not read file: " << p.path().string() << std::endl;
                    //     return EXIT_FAILURE;
                    // }

                    if (isVideo)
                    {
                        cv::VideoWriter writer;

                            // Get the input video's frame width, height, and FPS
                            int frameWidth = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH));
                            int frameHeight = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT));
                            double fps = cap.get(cv::CAP_PROP_FPS);
                            int fourcc = cv::VideoWriter::fourcc('m', 'p', '4', 'v');

                        // if (doExport)
                        // {
                        //     // TODO: Open output video for writing using the same fps as the input video
                        //     //       and the codec set to cv::VideoWriter::fourcc('m', 'p', '4', 'v')
                            
                        //     if (!cap.isOpened())
                        //     {
                        //         std::cout << "Error opening the video file." << std::endl;
                        //         return -1;
                        //     }

                        //     // Create the output video writer
                        //     writer.open(outputPath.string(), fourcc, fps, cv::Size(cap.get(cv::CAP_PROP_FRAME_WIDTH), cap.get(cv::CAP_PROP_FRAME_HEIGHT)));
                        //     // Check if the output video writer is opened successfully
                        //     if (!writer.isOpened())
                        //     {
                        //         std::cout << "Error opening the output video file." << std::endl;
                        //         return -1;
                        //     }
                        // }

                        int frameCounter = 0;
                        while (true)
                        {
                            // TODO: Get next frame from input video
                            cv::Mat frame;
                            cap >> frame;


                            // TODO: If frame is empty, break out of the while loop
                            if (frame.empty()) {
                                break;
                            }

                            frameCounter++; // Increment the frame counter

                            // Check if it's the 30th frame
                            if (frameCounter == 30) 
                            {
                                frameCounter = 0; // Reset the frame counter

                                imshow("Frame", frame);
                                    
                                // TODO: Call one of the detectAndDraw functions from imagefeatures.cpp according to the detector option specified at the command line
                                // Using Harris detector
                                // if(detector == "harris")
                                // {
                                //     printf("Using Harris feature detector\n");
                                //     printf("Features requested: %d", maxNumFeatures);
                                //     imgout = detectAndDrawHarris(frame, maxNumFeatures);
                                // }
                                
                                //cv::Mat image = cv::imread(p.path().string());
                                std::cout << "done, detecting chessboard..." << std::flush;
                                ChessboardImage ci(frame, chessboard, p.path().filename());
                                std::cout << (ci.isFound ? "found" : "not found") << std::endl;
                                if (ci.isFound)
                                {
                                    chessboardImages.push_back(ci);
                                }


                                // if (doExport)
                                // {
                                //     // TODO: Write image returned from detectAndDraw to frame of output video
                                //     writer.write(imgout);


                                // }
                                // else
                                // {
                                //     // TODO: Display image returned from detectAndDraw on screen and wait for 1000/fps milliseconds
                                //     cv::imshow("The THING", imgout);
                                //     cv::waitKey(1000 / fps);
                                // }
                            }
                        }


                        // TODO: release the input video object
                        cap.release();


                        // if (doExport)
                        // {
                        //     // TODO: release the output video object
                        //     cv::destroyAllWindows();
                        // }
                    }
                }
            }
        }
    }
}

void ChessboardData::drawCorners()
{
    for (auto & chessboardImage : chessboardImages)
    {
        chessboardImage.drawCorners(chessboard);
    }
}

void ChessboardData::drawBoxes(const Camera & camera)
{
    for (auto & chessboardImage : chessboardImages)
    {
        chessboardImage.drawBox(chessboard, camera);
    }
}

void ChessboardData::recoverPoses(const Camera & camera)
{
    for (auto & chessboardImage : chessboardImages)
    {
        chessboardImage.recoverPose(chessboard, camera);
    }
}

void Camera::calibrate(ChessboardData & chessboardData)
{
    std::vector<cv::Point3f> rPNn_all = chessboardData.chessboard.gridPoints();

    std::vector<std::vector<cv::Point2f>> rQOi_all;
    for (const auto & chessboardImage : chessboardData.chessboardImages)
    {
        rQOi_all.push_back(chessboardImage.corners);
    }
    assert(!rQOi_all.empty());

    imageSize = chessboardData.chessboardImages[0].image.size();
    
    flags = cv::CALIB_RATIONAL_MODEL | cv::CALIB_THIN_PRISM_MODEL;

    // Find intrinsic and extrinsic camera parameters
    cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    distCoeffs = cv::Mat::zeros(12, 1, CV_64F);
    std::vector<cv::Mat> Thetacn_all, rNCc_all;
    double rms;
    std::cout << "Calibrating camera..." << std::flush;
    
    // Prepare a vector of vector of 3D points for cv::calibrateCamera
    std::vector<std::vector<cv::Point3f>> objectPoints(chessboardData.chessboardImages.size(), rPNn_all);
    // TODO: Calibrate camera from detected chessboard corners
    // Calibrate the camera
    rms = cv::calibrateCamera(objectPoints, rQOi_all, imageSize, cameraMatrix, distCoeffs, Thetacn_all, rNCc_all, flags);
    std::cout << "done" << std::endl;
    
    // Calculate horizontal, vertical and diagonal field of view
    calcFieldOfView();
    // Write extrinsic parameters for each chessboard image
    assert(chessboardData.chessboardImages.size() == rNCc_all.size());
    assert(chessboardData.chessboardImages.size() == Thetacn_all.size());
    for (std::size_t i = 0; i < chessboardData.chessboardImages.size(); ++i)
    {
        cv::Mat Rcn;
        cv::Rodrigues(Thetacn_all[i], Rcn);
        Pose & cameraPose = chessboardData.chessboardImages[i].cameraPose;
        
        
        // TODO: Set the camera orientation and position (extrinsic camera parameters)
        // cameraPose.Rnc;
        cameraPose.Rnc = cv::Matx33d(Rcn).t();
        // cameraPose.rCNn;
        cameraPose.rCNn = -cameraPose.Rnc * cv::Vec3d(rNCc_all[i]);
    }
    
    printCalibration();
    std::cout << std::setw(30) << "RMS reprojection error: " << rms << std::endl;

    assert(cv::checkRange(cameraMatrix));
    assert(cv::checkRange(distCoeffs));
}

void Camera::printCalibration() const
{
    std::bitset<8*sizeof(flags)> bitflag(flags);
    std::cout << std::endl << "Calibration data:" << std::endl;
    std::cout << std::setw(30) << "Bit flags: " << bitflag << std::endl;
    std::cout << std::setw(30) << "cameraMatrix:\n" << cameraMatrix << std::endl;
    std::cout << std::setw(30) << "distCoeffs:\n" << distCoeffs.t() << std::endl;
    std::cout << std::setw(30) << "Focal lengths: " 
              << "(fx, fy) = "
              << "("<< cameraMatrix.at<double>(0, 0) << ", "<< cameraMatrix.at<double>(1, 1) << ")"
              << std::endl;       
    std::cout << std::setw(30) << "Principal point: " 
              << "(cx, cy) = "
              << "("<< cameraMatrix.at<double>(0, 2) << ", "<< cameraMatrix.at<double>(1, 2) << ")"
              << std::endl;     
    std::cout << std::setw(30) << "Field of view (horizontal): " << 180.0/CV_PI*hFOV << " deg" << std::endl;
    std::cout << std::setw(30) << "Field of view (vertical): " << 180.0/CV_PI*vFOV << " deg" << std::endl;
    std::cout << std::setw(30) << "Field of view (diagonal): " << 180.0/CV_PI*dFOV << " deg" << std::endl;
}

void Camera::calcFieldOfView()
{
    assert(cameraMatrix.rows == 3);
    assert(cameraMatrix.cols == 3);
    assert(cameraMatrix.type() == CV_64F);

    // TODO:
    hFOV = 0.0;
    vFOV = 0.0;
    dFOV = 0.0;

    // TODO:
    // Get the center, corner, and edge points in the image
   cv::Vec2d centerPoint(imageSize.width / 2.0, imageSize.height / 2.0);
    cv::Vec2d cornerPoint1(0, 0);
    cv::Vec2d cornerPoint2(imageSize.width,imageSize.height);
    cv::Vec2d edgePointX1(imageSize.width, imageSize.height / 2.0);
    cv::Vec2d edgePointX2(0, imageSize.height / 2.0);
    cv::Vec2d edgePointY1(imageSize.width / 2.0, imageSize.height);
    cv::Vec2d edgePointY2(imageSize.width / 2.0, 0);

    // Compute the direction vectors for these points
    //cv::Vec3d centerVector = pixelToVector(centerPoint);
    cv::Vec3d cornerVector1 = pixelToVector(cornerPoint1);
    cv::Vec3d cornerVector2 = pixelToVector(cornerPoint2);
    cv::Vec3d edgeVectorX1 = pixelToVector(edgePointX1);
    cv::Vec3d edgeVectorY1 = pixelToVector(edgePointY1);
    cv::Vec3d edgeVectorX2 = pixelToVector(edgePointX2);
    cv::Vec3d edgeVectorY2 = pixelToVector(edgePointY2);
    // Compute the dot product between the center vector and other vectors
    
    double CornerCornerDotProduct = cornerVector1.dot(cornerVector2);
    double EdgeEdgeDotProductX = edgeVectorX1.dot(edgeVectorX2);
    double EdgeEdgeDotProductY = edgeVectorY1.dot(edgeVectorY2);

    // Compute the magnitudes of the vectors
    
    double edgeVectorX1Mag = cv::norm(edgeVectorX1);
    double edgeVectorX2Mag = cv::norm(edgeVectorX2);

    double edgeVectorY1Mag = cv::norm(edgeVectorY1);
    double edgeVectorY2Mag = cv::norm(edgeVectorY2);
    
    double cornerVector1Mag = cv::norm(cornerVector1);
    double cornerVector2Mag = cv::norm(cornerVector2);


    // Compute the field of view angles using the dot product formula: dot(v1, v2) = ||v1|| ||v2|| cos(theta)
    hFOV = std::acos(EdgeEdgeDotProductX / (edgeVectorX1Mag * edgeVectorX2Mag));
    vFOV = std::acos(EdgeEdgeDotProductY / (edgeVectorY1Mag * edgeVectorY2Mag));
    dFOV = std::acos(CornerCornerDotProduct / (cornerVector1Mag * cornerVector2Mag));
}

cv::Vec3d Camera::worldToVector(const cv::Vec3d & rPNn, const Pose & pose) const
{
    cv::Vec3d uPCc;
    // TODO: Compute the unit vector uPCc from the world position rPNn and camera pose
    // Convert the world point to a camera point
    cv::Vec3d rPCc = pose.Rnc.t() * (rPNn - pose.rCNn);

    // Normalize the vector
    uPCc = rPCc / cv::norm(rPCc);
    
    return uPCc;
}

cv::Vec2d Camera::worldToPixel(const cv::Vec3d & rPNn, const Pose & pose) const
{
    return vectorToPixel(worldToVector(rPNn, pose));
}

cv::Vec2d Camera::vectorToPixel(const cv::Vec3d & rPCc) const
{
    cv::Vec2d rQOi;
    // TODO: Compute the pixel location (rQOi) for the given unit vector (uPCc)
    cv::Vec3d uPCc = rPCc/cv::norm(rPCc);

    // Form a vector of object points from the unit vector
    std::vector<cv::Vec3d> objectPoints{uPCc};

    // Vector to hold output image points
    std::vector<cv::Vec2d> imagePoints;

    // Project the 3D points to 2D image points
    cv::projectPoints(objectPoints, cv::Vec3d::zeros(), cv::Vec3d::zeros(), cameraMatrix, distCoeffs, imagePoints);

    // Extract the first (and only) projected image point
    rQOi = imagePoints[0];
    return rQOi;
}

cv::Vec3d Camera::pixelToVector(const cv::Vec2d & rQOi) const
{
    cv::Vec3d uPCc;
    // TODO: Compute unit vector (uPCc) for the given pixel location (rQOi)
    // Form a vector of image points from the given pixel coordinate
    std::vector<cv::Vec2d> imagePoints{rQOi};
    
    // Vector to hold output object points
    std::vector<cv::Vec2d> objectPoints;

    // Convert the 2D image points to normalized 2D points
    cv::undistortPoints(imagePoints, objectPoints, cameraMatrix, distCoeffs);

    // Convert the 2D point to a 3D unit vector
    uPCc = cv::Vec3d(objectPoints[0][0], objectPoints[0][1], 1.0);

    // Normalize the vector
    uPCc = uPCc / cv::norm(uPCc);
    
    return uPCc;
}

bool Camera::isVectorWithinFOV(const cv::Vec3d & rPCc) const
{
   // cv::Vec3d uPCc = rPCc/cv::norm(rPCc);
    //assert(std::abs(cv::norm(uPCc) - 1.0) < 100*std::numeric_limits<double>::epsilon());
    double angleDiagonal = std::atan2(std::hypot(rPCc[0], rPCc[1] ), rPCc[2]);

    // Check if the pixel is within the image bounds
    if (angleDiagonal < dFOV/2 /*&& uPCc[0]!=1 && uPCc[1]!=1*/)
    {
        return true;
    }
    else
    {
        return false;
    }

    return false;
}

bool Camera::isWorldWithinFOV(const cv::Vec3d & rPNn, const Pose & pose) const
{
    return isVectorWithinFOV(worldToVector(rPNn, pose));
}

void Camera::write(cv::FileStorage & fs) const
{
    fs << "{"
       << "camera_matrix"           << cameraMatrix
       << "distortion_coefficients" << distCoeffs
       << "flags"                   << flags
       << "imageSize"               << imageSize
       << "}";
}

void Camera::read(const cv::FileNode & node)
{
    node["camera_matrix"]           >> cameraMatrix;
    node["distortion_coefficients"] >> distCoeffs;
    node["flags"]                   >> flags;
    node["imageSize"]               >> imageSize;

    calcFieldOfView();

    assert(cameraMatrix.cols == 3);
    assert(cameraMatrix.rows == 3);
    assert(cameraMatrix.type() == CV_64F);
    assert(distCoeffs.cols == 1);
    assert(distCoeffs.type() == CV_64F);
}
