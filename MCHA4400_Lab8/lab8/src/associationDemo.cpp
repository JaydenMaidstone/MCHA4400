#include <cassert>
#include <cstddef>
#include <algorithm>
#include <iostream>
#include <vector>
#include <numeric>

#include <Eigen/Core>

#include <opencv2/core/eigen.hpp>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "Camera.h"
#include "Gaussian.hpp"
#include "imageFeatures.h"
#include "StateSLAM.h"
#include "StateSLAMPointLandmarks.h"
#include "plot.h"
#include "rotation.hpp"
#include "dataAssociation.h"
#include "associationDemo.h"

// Forward declarations
static StateSLAMPointLandmarks exampleStateFromChessboardImage(const ChessboardImage & chessboardImage);

// ------------------------------------------------------------------------------------------
// 
// associationDemo
// 
// ------------------------------------------------------------------------------------------
cv::Mat associationDemo(const Camera & camera, const ChessboardImage & chessboardImage)
{
    StateSLAMPointLandmarks state = exampleStateFromChessboardImage(chessboardImage);

    // ------------------------------------------------------------------------
    // Feature detector
    // ------------------------------------------------------------------------
    int maxNumFeatures = 10000;
    std::vector<PointFeature> features = detectFeatures(chessboardImage.image, maxNumFeatures);
    std::cout << features.size() << " features found in " << chessboardImage.filename.string() << std::endl;
    assert(features.size() > 0);
    assert(features.size() <= maxNumFeatures);
    
    // ------------------------------------------------------------------------
    // Populate measurement set Y with elements of the features vector
    // ------------------------------------------------------------------------
    Eigen::Matrix<double, 2, Eigen::Dynamic> Y(2, features.size());
    for (std::size_t i = 0; i < features.size(); ++i)
    {
        Y.col(i) << features[i].x, features[i].y;
    }
    //std::cout << Y << std::endl;
    
    // ------------------------------------------------------------------------
    // Select landmarks expected to be within the field of view of the camera
    // ------------------------------------------------------------------------
    std::vector<std::size_t> idxLandmarks;
    idxLandmarks.reserve(state.numberLandmarks());  // Reserve maximum possible size to avoid reallocation
    for (std::size_t j = 0; j < state.numberLandmarks(); ++j)
    {
        Eigen::Vector3d murPNn = state.landmarkPositionDensity(j).mean();
        cv::Vec3d rPNn;
        cv::eigen2cv(murPNn, rPNn);
        if (camera.isWorldWithinFOV(rPNn, chessboardImage.cameraPose))
        {
            std::cout << "Landmark " << j << " is expected to be within camera FOV" << std::endl;
            idxLandmarks.push_back(j);
        }
        else
        {
            std::cout << "Landmark " << j << " is NOT expected to be within camera FOV" << std::endl;
        }
    }
    
    // ------------------------------------------------------------------------
    // Run surprisal nearest neighbours
    // ------------------------------------------------------------------------
    std::vector<int> idx;
    
    double surprisal = snn(state, idxLandmarks, Y, camera, idx);
    std::cout << "Surprisal = " << surprisal << std::endl;
    // ------------------------------------------------------------------------
    // Visualisation and console output
    // ------------------------------------------------------------------------
    plotAllFeatures(state.view(), Y);
    for (std::size_t jj = 0; jj < idx.size(); ++jj)
    {
        int j           = idxLandmarks[jj];     // Index of landmark in state vector
        int i           = idx[jj];              // Index of feature
        bool isMatch    = i >= 0;

        Gaussian<double> featureDensity = state.predictFeatureDensity(camera, j);
        const Eigen::VectorXd & murQOi = featureDensity.mean();

        // Plot confidence ellipse and landmark index text
        Eigen::Vector3d colour = isMatch ? Eigen::Vector3d(0, 0, 255) : Eigen::Vector3d(255, 0, 0);
        plotGaussianConfidenceEllipse(state.view(), featureDensity, colour);
        plotLandmarkIndex(state.view(), murQOi, colour, j);

        if (isMatch)
            std::cout << "Feature " << i << " located at [" << Y.col(i).transpose() << "] in image matches landmark " << j <<"." << std::endl;
        else
            std::cout << "No feature associated with landmark "<< j << "." << std::endl;
    }
    plotMatchedFeatures(state.view(), idx, Y);
    std::cout << std::endl;

    return state.view();
}

// ----------------------------------------------------------------------------
// Helper functions (shouldn't need to edit)
// ----------------------------------------------------------------------------
StateSLAMPointLandmarks exampleStateFromChessboardImage(const ChessboardImage & chessboardImage)
{
    // Body velocity mean
    Eigen::Vector6d munu;
    munu.setZero();

    // Camera pose
    Eigen::Map<const Eigen::Vector3d> rCNn(chessboardImage.cameraPose.rCNn.val);
    Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> Rnc(chessboardImage.cameraPose.Rnc.val);

    // Pose of camera w.r.t body (for Lab 7/8)
    Eigen::Vector3d rCBb = Eigen::Vector3d::Zero();
    Eigen::Matrix3d Rbc = Eigen::Matrix3d::Identity();

    // Obtain body pose from camera pose
    Eigen::Matrix3d Rnb = Rnc*Rbc.transpose();  // Rnb = Rnc*Rcb = Rnc*Rbc.'
    Eigen::Vector3d Thetanb = rot2rpy(Rnb);
    Eigen::Vector3d rBNn = rCNn - Rnb*rCBb;     // rBNn = rCNn + rBCn = rCNn - rCBn = rCNn - Rnb*rCBb

    // Body pose mean
    Eigen::Vector6d mueta;
    mueta << rBNn, Thetanb;

    // Landmark mean
    Eigen::VectorXd murPNn(27);
    murPNn <<
                        -0.2,
                         0.1,
                           0,
                           0,
                        -0.3,
                           0,
                           0,
                         0.6,
                           0,
                         0.4,
                         0.6,
                           0,
                         0.4,
                        -0.4,
                           0,
                       0.088,
                       0.066,
                           0,
                       0.022,
                       0.132,
                           0,
                       0.297,
                           0,
                           0,
                     0.29029,
                     0.09502,
                    -0.14843;

    // State mean
    Eigen::VectorXd mu(6 + 6 + 27);
    mu << munu, mueta, murPNn;

    // State square-root covariance
    Eigen::MatrixXd S(39, 39);
    S.setZero();

    S.bottomRightCorner(33, 33) <<
         0.001427830283,    0.001229097682,    0.003320690394,   -0.002585376784,    0.004103269664,   -0.002795866555,                 0,                -0,                 0,                 0,                 0,                 0,                 0,                -0,                 0,                -0,                 0,                -0,                 0,                -0,                 0,                 0,                 0,                -0,                 0,                -0,                 0,                -0,                 0,                -0,                 0,                 0,                 0,
                      0,   0.0003924364969,    0.001926923125,    0.004464152406,    -0.00231243553,   0.0001452205508,                 0,                 0,                -0,                -0,                -0,                 0,                 0,                 0,                 0,                 0,                -0,                 0,                 0,                 0,                -0,                 0,                -0,                 0,                 0,                 0,                -0,                 0,                 0,                 0,                 0,                -0,                -0,
                      0,                 0,   0.0005341369799,   0.0006053459295,   0.0009182405056,    -0.00099283847,                 0,                 0,                 0,                 0,                 0,                 0,                -0,                 0,                 0,                 0,                 0,                 0,                -0,                 0,                 0,                -0,                 0,                 0,                -0,                 0,                 0,                 0,                -0,                 0,                -0,                 0,                 0,
                      0,                 0,                 0,    0.001328048581,     0.00308475703,  -0.0007088613796,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                -0,                 0,                -0,                 0,                -0,                 0,                -0,                 0,                 0,                 0,
                      0,                 0,                 0,                 0,     0.00300904338,   -0.001734500276,                 0,                 0,                 0,                -0,                -0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                -0,                 0,                -0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                -0,                 0,
                      0,                 0,                 0,                 0,                 0,    0.002043219303,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,
                      0,                 0,                 0,                 0,                 0,                 0,            0.0015,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,
                      0,                 0,                 0,                 0,                 0,                 0,                 0,            0.0015,                 0,                 0,                 0,                 0,                 0,                -0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,
                      0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,            0.0015,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,
                      0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,            0.0045,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,
                      0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,            0.0015,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                -0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,
                      0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,            0.0015,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,
                      0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,            0.0015,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,
                      0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,             0.006,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,
                      0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,            0.0015,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,
                      0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,            0.0015,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,
                      0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,            0.0015,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,
                      0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,            0.0045,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,
                      0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,            0.0015,                 0,                 0,                 0,                 0,                 0,                -0,                 0,                 0,                 0,                -0,                 0,                -0,                 0,                 0,
                      0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,            0.0015,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,
                      0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,            0.0015,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,
                      0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,            0.0045,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,
                      0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,            0.0015,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,
                      0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,            0.0015,                 0,                 0,                 0,                 0,                 0,                -0,                 0,                 0,                 0,
                      0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,            0.0015,                 0,                 0,                 0,                 0,                 0,                -0,                 0,                 0,
                      0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,             0.006,                 0,                -0,                 0,                 0,                 0,                 0,                 0,
                      0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,            0.0015,                 0,                 0,                 0,                 0,                 0,                -0,
                      0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,            0.0015,                 0,                 0,                 0,                 0,                 0,
                      0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,            0.0015,                 0,                 0,                 0,                 0,
                      0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,            0.0045,                 0,                 0,                 0,
                      0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,            0.0015,                 0,                 0,
                      0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,            0.0015,                 0,
                      0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,                 0,            0.0015;

    StateSLAMPointLandmarks state(Gaussian(mu, S));
    state.view() = chessboardImage.image.clone();
    return state;
}
