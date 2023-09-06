#ifndef CAMERA_H
#define CAMERA_H

#include <vector>
#include <filesystem>
#include <Eigen/Core>
#include <opencv2/core/types.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/persistence.hpp>
#include "serialisation.hpp"
#include <iostream>

struct Pose
{
    cv::Matx33d Rnc;
    cv::Vec3d rCNn;
};

struct Chessboard
{
    cv::Size boardSize;
    float squareSize;

    void write(cv::FileStorage & fs) const;                 // OpenCV serialisation
    void read(const cv::FileNode & node);                   // OpenCV serialisation

    std::vector<cv::Point3f> gridPoints() const;
    friend std::ostream & operator<<(std::ostream &, const Chessboard &);
};

struct Camera;

struct ChessboardImage
{
    ChessboardImage(const cv::Mat &, const Chessboard &, const std::filesystem::path & = "");
    cv::Mat image;
    std::filesystem::path filename;
    Pose cameraPose;                                        // Extrinsic camera parameters
    std::vector<cv::Point2f> corners;                       // Chessboard corners in image [rQOi]
    bool isFound;
    void drawCorners(const Chessboard &);
    void drawBox(const Chessboard &, const Camera &);
    void recoverPose(const Chessboard &, const Camera &);
};

struct ChessboardData
{
    explicit ChessboardData(const std::filesystem::path &); // Load from config file

    Chessboard chessboard;
    std::vector<ChessboardImage> chessboardImages;

    void drawCorners();
    void drawBoxes(const Camera &);
    void recoverPoses(const Camera &);
};

namespace Eigen {
using Matrix23d = Eigen::Matrix<double, 2, 3>;
using Matrix26d = Eigen::Matrix<double, 2, 6>;
using Vector6d = Eigen::Matrix<double, 6, 1>;
template <typename Scalar> using Vector6 = Eigen::Matrix<Scalar, 6, 1>;
}

struct Camera
{
    void calibrate(ChessboardData &);                       // Calibrate camera from chessboard data
    void printCalibration() const;

    cv::Vec3d worldToVector(const cv::Vec3d & rPNn, const Pose & pose) const;
    template <typename Scalar> Eigen::Vector3<Scalar> worldToVector(const Eigen::Vector3<Scalar> & rPNn, const Eigen::Vector6<Scalar> & eta) const;

    cv::Vec2d worldToPixel(const cv::Vec3d &, const Pose &) const;
    template <typename Scalar> Eigen::Vector2<Scalar> worldToPixel(const Eigen::Vector3<Scalar> & rPNn, const Eigen::Vector6<Scalar> & eta) const;
    Eigen::Vector2d worldToPixel(const Eigen::Vector3d & rPNn, const Eigen::Vector6d & eta, Eigen::Matrix23d & JrPNn, Eigen::Matrix26d & Jeta) const;

    cv::Vec2d vectorToPixel(const cv::Vec3d &) const;
    template <typename Scalar> Eigen::Vector2<Scalar> vectorToPixel(const Eigen::Vector3<Scalar> &) const;
    Eigen::Vector2d vectorToPixel(const Eigen::Vector3d &, Eigen::Matrix23d &) const;

    cv::Vec3d pixelToVector(const cv::Vec2d &) const;

    bool isWorldWithinFOV(const cv::Vec3d & rPNn, const Pose & pose) const;
    bool isVectorWithinFOV(const cv::Vec3d & rPCc) const;

    void calcFieldOfView();
    void write(cv::FileStorage &) const;                    // OpenCV serialisation
    void read(const cv::FileNode &);                        // OpenCV serialisation

    cv::Mat cameraMatrix;                                   // Camera matrix
    cv::Mat distCoeffs;                                     // Lens distortion coefficients
    int flags = 0;                                          // Calibration flags
    cv::Size imageSize;                                     // Image size
    double hFOV = 0.0;                                      // Horizonal field of view
    double vFOV = 0.0;                                      // Vertical field of view
    double dFOV = 0.0;                                      // Diagonal field of view

    Eigen::Vector3d rCBb = Eigen::Vector3d::Zero();         // TODO: Assignment(s)
    Eigen::Matrix3d Rbc = Eigen::Matrix3d::Identity();      // TODO: Assignment(s)
};

template <typename Scalar>
Eigen::Vector3<Scalar> Camera::worldToVector(const Eigen::Vector3<Scalar> & rPNn, const Eigen::Vector6<Scalar> & eta) const
{
    Eigen::Vector3<Scalar> rCNn = eta.template head<3>();
    Eigen::Vector3<Scalar> Thetanc = eta.template tail<3>();
    Eigen::Matrix3<Scalar> Rnc = rpy2rot(Thetanc);
    Eigen::Vector3<Scalar> rPCc = Rnc.transpose()*(rPNn - rCNn);
    return rPCc;
}

template <typename Scalar>
Eigen::Vector2<Scalar> Camera::worldToPixel(const Eigen::Vector3<Scalar> & rPNn, const Eigen::Vector6<Scalar> & eta) const
{
    return vectorToPixel(worldToVector(rPNn, eta));
}

#include <cmath>
#include <opencv2/calib3d.hpp>

template <typename Scalar>
Eigen::Vector2<Scalar> Camera::vectorToPixel(const Eigen::Vector3<Scalar> & rPCc) const
{
    
    bool isRationalModel    = (flags & cv::CALIB_RATIONAL_MODEL) == cv::CALIB_RATIONAL_MODEL;
    bool isThinPrismModel   = (flags & cv::CALIB_THIN_PRISM_MODEL) == cv::CALIB_THIN_PRISM_MODEL;
    assert(isRationalModel && isThinPrismModel);

    Eigen::Vector2<Scalar> rQOi;
    
    Scalar x = rPCc[0];
    Scalar y = rPCc[1];
    Scalar z = rPCc[2];
    Scalar u = x / z;
    Scalar v = y / z;

    Scalar r2 = u*u + v*v;

    Scalar k1 = distCoeffs.at<double>(0,0);
    Scalar k2 = distCoeffs.at<double>(1,0);
    Scalar P1 = distCoeffs.at<double>(2,0);
    Scalar P2 = distCoeffs.at<double>(3,0);
    Scalar k3 = distCoeffs.at<double>(4,0);
    Scalar k4 = distCoeffs.at<double>(5,0);
    Scalar k5 = distCoeffs.at<double>(6,0);
    Scalar k6 = distCoeffs.at<double>(7,0);
    Scalar s1 = distCoeffs.at<double>(8,0);
    Scalar s2 = distCoeffs.at<double>(9,0);
    Scalar s3 = distCoeffs.at<double>(10,0);
    Scalar s4 = distCoeffs.at<double>(11,0);

    Scalar fx = cameraMatrix.at<double>(0,0);
    Scalar fy = cameraMatrix.at<double>(1,1);
    Scalar cx = cameraMatrix.at<double>(0,2);
    Scalar cy = cameraMatrix.at<double>(1,2);

    //std::cout << "fx inside camera.h " << fx << fy << cx  << cy << std::endl;




    Scalar alpha = k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2;
    Scalar beta = k4 * r2 + k5 * r2 * r2 + k6 * r2 * r2 * r2;

    // Apply the distortion model
    Scalar c = (1 + alpha) / (1 + beta);
    //Eigen::Vector3<Scalar> distorted_point = radial_distortion * uPCc;
    //Extract the first (and only) projected image point

    Scalar udash = c*u + 2*P1*u*v + P2*(r2+2*u*u) + s1*r2+s2*r2*r2;
    Scalar vdash = c*v + P1*(r2+2*v*v) + 2*P2*u*v + s3*r2+s4*r2*r2;

    rQOi[0] = fx*udash + cx;
    rQOi[1] = fy*vdash + cy;

    //std::cout << "udash inside camera.h " << udash << std::endl;
    //std::cout << "vdash inside camera.h " << vdash << std::endl;



    return rQOi;
}

#endif

