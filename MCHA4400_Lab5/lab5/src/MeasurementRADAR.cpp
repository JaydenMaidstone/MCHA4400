#include <cmath>
#include <Eigen/Core>
#include "Gaussian.h"
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
}

// Evaluate h(x) from the measurement model y = h(x) + v
Eigen::VectorXd MeasurementRADAR::predict(const Eigen::VectorXd & x) const
{
    Eigen::VectorXd h(1);
    // TODO: Set h
    // Calculate the horizontal and vertical position of the target
    double px = x(0);  // Horizontal position
    
    // Calculate the range measurement based on the RADAR measurement model
    h << std::hypot(r1, px - r2);

    return h;
}

// Evaluate h(x) and its Jacobian J = dh/fx from the measurement model y = h(x) + v
Eigen::VectorXd MeasurementRADAR::predict(const Eigen::VectorXd & x, Eigen::MatrixXd & J) const
{
    Eigen::VectorXd h = predict(x);

    J.resize(h.size(), x.size());
    // TODO: Set J
    // Compute the Jacobian matrix J = dh/dx
    double px = x(0);  // Horizontal position
    double range = h(0);  // Predicted range
    
    // Partial derivatives based on the range measurement model
    J << (px - r1) / range, 0, 0;  // The measurement model only depends on px


    return h;
}
