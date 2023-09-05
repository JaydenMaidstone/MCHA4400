#include <cstddef>
#include <cmath>
#include <Eigen/Core>
#include <opencv2/core/mat.hpp>
#include "Gaussian.hpp"
#include "State.h"
#include "StateSLAM.h"

StateSLAM::StateSLAM(const Gaussian<double> & density)
    : State(density)
{}

// Evaluate f(x) from the SDE dx = f(x)*dt + dw
Eigen::VectorXd StateSLAM::dynamics(const Eigen::VectorXd & x) const
{
    assert(size() == x.size());
    //
    //  dnu/dt =          0 + dwnu/dt
    // deta/dt = JK(eta)*nu +       0
    //   dm/dt =          0 +       0
    // \_____/   \________/   \_____/
    //  dx/dt  =    f(x)    +  dw/dt
    //
    //        [          0 ]
    // f(x) = [ JK(eta)*nu ]
    //        [          0 ] for all map states
    //
    //        [                    0 ]
    //        [                    0 ]
    // f(x) = [    Rnb(thetanb)*vBNb ]
    //        [ TK(thetanb)*omegaBNb ]
    //        [                    0 ] for all map states
    //
    Eigen::VectorXd f(x.size());
    f.setZero();
    // TODO: Implement in Assignment(s)

    return f;
}

// Evaluate f(x) and its Jacobian J = df/fx from the SDE dx = f(x)*dt + dw
Eigen::VectorXd StateSLAM::dynamics(const Eigen::VectorXd & x, Eigen::MatrixXd & J) const
{
    Eigen::VectorXd f = dynamics(x);

    // TODO: Implement in Assignment(s)

    // Jacobian J = df/dx
    //    
    //     [  0                  0 0 ]
    // J = [ JK d(JK(eta)*nu)/deta 0 ]
    //     [  0                  0 0 ]
    //
    J.resize(f.size(), x.size());
    J.setZero();
    // TODO: Implement in Assignment(s)

    return f;
}

cv::Mat & StateSLAM::view()
{
    return view_;
};

const cv::Mat & StateSLAM::view() const
{
    return view_;
};

Gaussian<double> StateSLAM::bodyPositionDensity() const
{
    return density.marginal(Eigen::seqN(6, 3));
}

Gaussian<double> StateSLAM::bodyOrientationDensity() const
{
    return density.marginal(Eigen::seqN(9, 3));
}

Gaussian<double> StateSLAM::bodyTranslationalVelocityDensity() const
{
    return density.marginal(Eigen::seqN(0, 3));
}

Gaussian<double> StateSLAM::bodyAngularVelocityDensity() const
{
    return density.marginal(Eigen::seqN(3, 3));
}

#include <autodiff/forward/dual.hpp>
#include <autodiff/forward/dual/eigen.hpp>

Eigen::Vector3d StateSLAM::cameraPosition(const Camera & cam, const Eigen::VectorXd & x, Eigen::MatrixXd & J)
{
    Eigen::Vector3<autodiff::dual> rCNn_dual;
    Eigen::VectorX<autodiff::dual> x_dual = x.cast<autodiff::dual>();
    J = jacobian(cameraPosition<autodiff::dual>, wrt(x_dual), at(cam, x_dual), rCNn_dual);
    return rCNn_dual.cast<double>();
};

Eigen::Vector3d StateSLAM::cameraOrientation(const Camera & cam, const Eigen::VectorXd & x, Eigen::MatrixXd & J)
{
    Eigen::Vector3<autodiff::dual> Thetanc_dual;
    Eigen::VectorX<autodiff::dual> x_dual = x.cast<autodiff::dual>();
    J = jacobian(cameraOrientation<autodiff::dual>, wrt(x_dual), at(cam, x_dual), Thetanc_dual);
    return Thetanc_dual.cast<double>();
};

Gaussian<double> StateSLAM::cameraPositionDensity(const Camera & cam) const
{
    auto f = [&](const Eigen::VectorXd & x, Eigen::MatrixXd & J) {return cameraPosition(cam, x, J); };
    return density.transform(f);
}

Gaussian<double> StateSLAM::cameraOrientationDensity(const Camera & cam) const
{
    auto f = [&](const Eigen::VectorXd & x, Eigen::MatrixXd & J) {return cameraOrientation(cam, x, J); };
    return density.transform(f);    
}

Gaussian<double> StateSLAM::landmarkPositionDensity(std::size_t idxLandmark) const
{
    assert(idxLandmark < numberLandmarks());
    std::size_t idx = landmarkPositionIndex(idxLandmark);
    return density.marginal(Eigen::seqN(idx, 3));
}

#include "Camera.h"
#include "rotation.hpp"

// Image feature location for a given landmark and Jacobian
Eigen::Vector2d StateSLAM::predictFeature(const Eigen::VectorXd & x, Eigen::MatrixXd & J, const Camera & cam, std::size_t idxLandmark) const
{
    // Set elements of J
    // TODO: Lab 7 (optional)
    return predictFeature(x, cam, idxLandmark);
    // Note: If you use autodiff, return the evaluated function value (cast with double scalar type) instead of calling predictFeature as above
}

// Density of image feature location for a given landmark
Gaussian<double> StateSLAM::predictFeatureDensity(const Camera & cam, std::size_t idxLandmark) const
{
    const auto func = [&](const Eigen::VectorXd & x, Eigen::MatrixXd & J) { return predictFeature(x, J, cam, idxLandmark); };
    return density.transform(func);
}

// Density of image feature location for a given landmark with added noise
Gaussian<double> StateSLAM::predictFeatureDensity(const Camera & cam, std::size_t idxLandmark, const Gaussian<double> & noise) const
{
    const auto func = [&](const Eigen::VectorXd & x, Eigen::MatrixXd & J) { return predictFeature(x, J, cam, idxLandmark); };
    return density.transform(func, noise);
}

// Image feature locations for a bundle of landmarks
Eigen::VectorXd StateSLAM::predictFeatureBundle(const Eigen::VectorXd & x, Eigen::MatrixXd & J, const Camera & cam, const std::vector<std::size_t> & idxLandmarks) const
{
    const std::size_t & nL = idxLandmarks.size();
    const std::size_t & nx = size();
    assert(x.size() == nx);

    Eigen::VectorXd h(2*nL);
    J.resize(2*nL, nx);
    for (std::size_t i = 0; i < nL; ++i)
    {
        Eigen::MatrixXd Jfeature;
        Eigen::Vector2d rQOi = predictFeature(x, Jfeature, cam, idxLandmarks[i]);
        // Set pair of elements of h
        // TODO: Lab 8
        // Set pair of rows of J
        // TODO: Lab 8
    }
    return h;
}

// Density of image features for a set of landmarks
Gaussian<double> StateSLAM::predictFeatureBundleDensity(const Camera & cam, const std::vector<std::size_t> & idxLandmarks) const
{
    auto func = [&] (const Eigen::VectorXd & x, Eigen::MatrixXd & J) { return predictFeatureBundle(x, J, cam, idxLandmarks); };
    return density.transform(func);
}

// Density of image features for a set of landmarks
Gaussian<double> StateSLAM::predictFeatureBundleDensity(const Camera & cam, const std::vector<std::size_t> & idxLandmarks, const Gaussian<double> & noise) const
{
    auto func = [&] (const Eigen::VectorXd & x, Eigen::MatrixXd & J) { return predictFeatureBundle(x, J, cam, idxLandmarks); };
    return density.transform(func, noise);
}
