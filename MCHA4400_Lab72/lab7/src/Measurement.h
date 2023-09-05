#ifndef MEASUREMENT_H
#define MEASUREMENT_H

#include <Eigen/Core>
#include "Gaussian.hpp"
#include "Event.h"
#include "State.h"

class Measurement : public Event
{
public:
    Measurement(double time, const Eigen::VectorXd & y);
    Measurement(double time, const Eigen::VectorXd & y, const Gaussian<double> & noise);
    virtual double logLikelihood(const State & state, const Eigen::VectorXd & x) const = 0;
    virtual double logLikelihood(const State & state, const Eigen::VectorXd & x, Eigen::VectorXd & g) const = 0;
    virtual double logLikelihood(const State & state, const Eigen::VectorXd & x, Eigen::VectorXd & g, Eigen::MatrixXd & H) const = 0;
protected:
    double costJointDensity(const Eigen::VectorXd & x, const State & state);
    double costJointDensity(const Eigen::VectorXd & x, const State & state, Eigen::VectorXd & g);
    double costJointDensity(const Eigen::VectorXd & x, const State & state, Eigen::VectorXd & g, Eigen::MatrixXd & H);
    virtual void update(State & state) override;
    bool useQuasiNewton;
    Eigen::VectorXd y_;
    Gaussian<double> noise_;
};

#endif
