#ifndef MEASUREMENTPOINTBUNDLE_H
#define MEASUREMENTPOINTBUNDLE_H

#include <Eigen/Core>
#include "State.h"
#include "Camera.h"
#include "Measurement.h"

class MeasurementPointBundle : public Measurement
{
public:
    MeasurementPointBundle(double time, const Eigen::VectorXd & y, const Camera & camera);
    virtual double logLikelihood(const State & state, const Eigen::VectorXd & x) const override;
    virtual double logLikelihood(const State & state, const Eigen::VectorXd & x, Eigen::VectorXd & g) const override;
    virtual double logLikelihood(const State & state, const Eigen::VectorXd & x, Eigen::VectorXd & g, Eigen::MatrixXd & H) const override;
protected:
    virtual void update(State & state) override;
    const Camera & camera_;
};

#endif