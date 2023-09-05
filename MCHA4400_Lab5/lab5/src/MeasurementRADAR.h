#ifndef MEASUREMENTRADAR_H
#define MEASUREMENTRADAR_H

#include <Eigen/Core>
#include "Measurement.h"

class MeasurementRADAR : public Measurement
{
public:
    MeasurementRADAR(double time, const Eigen::VectorXd & y);
    virtual Eigen::VectorXd predict(const Eigen::VectorXd & x) const override;
    virtual Eigen::VectorXd predict(const Eigen::VectorXd & x, Eigen::MatrixXd & J) const override;
protected:
    static const double r1, r2;
};

#endif
