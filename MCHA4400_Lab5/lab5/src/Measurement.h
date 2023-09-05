#ifndef MEASUREMENT_H
#define MEASUREMENT_H

#include <Eigen/Core>
#include "Event.h"
#include "State.h"
#include "Gaussian.h"

class Measurement : public Event
{
public:
    Measurement(double time, const Eigen::VectorXd & y);
    Measurement(double time, const Eigen::VectorXd & y, const Gaussian & noise);
    virtual Eigen::VectorXd predict(const Eigen::VectorXd & x) const = 0;
    virtual Eigen::VectorXd predict(const Eigen::VectorXd & x, Eigen::MatrixXd & J) const = 0;
    virtual Eigen::VectorXd simulate(const Eigen::VectorXd & x) const;
protected:
    Eigen::MatrixXd augmentedPredict(const Eigen::MatrixXd & x, Eigen::MatrixXd & Ja) const;
    virtual void update(State & state) override;
    Eigen::VectorXd y_;
    Gaussian noise_;
};

#endif
