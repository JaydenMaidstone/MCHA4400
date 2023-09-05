#include <cmath>
#include <Eigen/Core>
#include <unsupported/Eigen/CXX11/Tensor>

#define DERIVATIVE_ANALYTICAL 1
#define DERIVATIVE_FORWARD_AUTODIFF 2
#define DERIVATIVE_REVERSE_AUTODIFF 3

#ifndef DERIVATIVE_MODE
#define DERIVATIVE_MODE DERIVATIVE_ANALYTICAL // Set the derivative mode here to easily switch between methods
#endif

// If using autodiff, include either the forward-mode headers or the
// reverse-mode headers, but not both (https://github.com/autodiff/autodiff/issues/179).
#if DERIVATIVE_MODE == DERIVATIVE_FORWARD_AUTODIFF
#include <autodiff/forward/dual.hpp>
#include <autodiff/forward/dual/eigen.hpp>
#elif DERIVATIVE_MODE == DERIVATIVE_REVERSE_AUTODIFF
#include <autodiff/reverse/var.hpp>
#include <autodiff/reverse/var/eigen.hpp>
#endif

#include "Gaussian.hpp"
#include "State.h"
#include "MeasurementRADAR.h"

const double MeasurementRADAR::r1 = 5000;    // Horizontal position of sensor [m]
const double MeasurementRADAR::r2 = 5000;    // Vertical position of sensor [m]

MeasurementRADAR::MeasurementRADAR(double time, const Eigen::VectorXd & y)
    : Measurement(time, y)
{
    // SR is an upper triangular matrix such that SR.'*SR = R is the measurement noise covariance
    Eigen::MatrixXd SR(1, 1);
    SR << 50.0;
    noise_ = Gaussian(SR);

    // useQuasiNewton = false;
}


// Evaluate h(x) from the measurement model y = h(x) + v
Eigen::VectorXd MeasurementRADAR::predict(const Eigen::VectorXd & x) const
{
    return predictImpl(x);
}

// Evaluate h(x) from the measurement model y = h(x) + v
// Eigen::VectorXd MeasurementRADAR::predict(const Eigen::VectorXd & x) const
// {
//     Eigen::VectorXd h(1);
//     // TODO: Set h
//     // Calculate the horizontal and vertical position of the target
//     double px = x(0);  // Horizontal position
    
//     // Calculate the range measurement based on the RADAR measurement model
//     h << std::hypot(r1, px - r2);

//     return predictImpl(x);
// }

// // Evaluate h(x) and its Jacobian J = dh/fx from the measurement model y = h(x) + v
// Eigen::VectorXd MeasurementRADAR::predict(const Eigen::VectorXd & x, Eigen::MatrixXd & J) const
// {
//     Eigen::VectorXd h = predict(x);

//     J.resize(h.size(), x.size());
//     J.setZero();
//     // TODO: Set non-zero elements of J

//     return h
// }

// Evaluate h(x) and its Jacobian J = dh/fx from the measurement model y = h(x) + v
Eigen::VectorXd MeasurementRADAR::predict(const Eigen::VectorXd & x, Eigen::MatrixXd & J) const
{
    Eigen::VectorXd h = predict(x);

    J.resize(h.size(), x.size());
    // TODO: Set J
    J.setZero();
    // Compute the Jacobian matrix J = dh/dx
    double px = x(0);  // Horizontal position
    double range = h(0);  // Predicted range
    
    // Partial derivatives based on the range measurement model
    J << (px - r1) / range, 0, 0;  // The measurement model only depends on px


    return h;
}

Eigen::VectorXd MeasurementRADAR::predict(const Eigen::VectorXd & x, Eigen::MatrixXd & J, Eigen::Tensor<double, 3> & H) const
{
    Eigen::VectorXd h = predict(x, J);

    //               d^2 h_i     dJ_{ij}
    // H(i, j, k) = --------- = ---------
    //              dx_j dx_k     dx_k
    H.resize(h.size(), x.size(), x.size());
    H.setZero();
    // TODO: Set non-zero elements of H
    double px = x(0);  // Horizontal position
    
    // Hessian elements based on the range measurement model
    H(0, 0, 0) = (1 / std::hypot(r1, px - r2)) - (px - r1) * (px - r2) / std::pow(std::hypot(r1, px - r2),3);  // Second derivative with respect to px
    


    return h;
}

double MeasurementRADAR::logLikelihood(const Eigen::VectorXd & x) const
{
#if DERIVATIVE_MODE == DERIVATIVE_ANALYTICAL
    return Measurement::logLikelihood(x);           // use the base class implementation
#else
    return logLikelihoodImpl(x);
#endif
}

double MeasurementRADAR::logLikelihood(const Eigen::VectorXd & x, Eigen::VectorXd & g) const
{
#if DERIVATIVE_MODE == DERIVATIVE_ANALYTICAL
    // i) Analytical derivatives
    return Measurement::logLikelihood(x, g);        // use the base class implementation
#elif DERIVATIVE_MODE == DERIVATIVE_FORWARD_AUTODIFF
    // ii) Forward-mode autodifferentiation
    // TODO
#elif DERIVATIVE_MODE == DERIVATIVE_REVERSE_AUTODIFF
    // iii) Reverse-mode autodifferentiation
    // TODO
#endif
}

double MeasurementRADAR::logLikelihood(const Eigen::VectorXd & x, Eigen::VectorXd & g, Eigen::MatrixXd & H) const
{
#if DERIVATIVE_MODE == DERIVATIVE_ANALYTICAL
    // i) Analytical derivatives
    return Measurement::logLikelihood(x, g, H);     // use the base class implementation
#elif DERIVATIVE_MODE == DERIVATIVE_FORWARD_AUTODIFF
    // ii) Forward-mode autodifferentiation
    // TODO
#elif DERIVATIVE_MODE == DERIVATIVE_REVERSE_AUTODIFF
    // iii) Reverse-mode autodifferentiation
    // TODO
#endif
}

