#ifndef MEASUREMENT_H
#define MEASUREMENT_H

#include <Eigen/Core>
#include <unsupported/Eigen/CXX11/Tensor>
#include "Event.h"
#include "State.h"

class Measurement : public Event
{
public:
    Measurement(double time, const Eigen::VectorXd & y);
    Measurement(double time, const Eigen::VectorXd & y, const Gaussian<double> & noise);
    virtual Eigen::VectorXd predict(const Eigen::VectorXd & x) const = 0;
    virtual Eigen::VectorXd predict(const Eigen::VectorXd & x, Eigen::MatrixXd & J) const = 0;
    virtual Eigen::VectorXd predict(const Eigen::VectorXd & x, Eigen::MatrixXd & J, Eigen::Tensor<double, 3> & H) const = 0;
    virtual Eigen::VectorXd simulate(const Eigen::VectorXd & x) const;
    virtual double logLikelihood(const Eigen::VectorXd & x) const;
    virtual double logLikelihood(const Eigen::VectorXd & x, Eigen::VectorXd & g) const;
    virtual double logLikelihood(const Eigen::VectorXd & x, Eigen::VectorXd & g, Eigen::MatrixXd & H) const;
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
