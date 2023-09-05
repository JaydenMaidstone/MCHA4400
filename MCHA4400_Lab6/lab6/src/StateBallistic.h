#ifndef STATEBALLISTIC_H
#define STATEBALLISTIC_H

#include <Eigen/Core>
#include "Gaussian.hpp"
#include "State.h"

class StateBallistic : public State
{
public:
    explicit StateBallistic(const Gaussian<double> & density);
    virtual Eigen::VectorXd dynamics(const Eigen::VectorXd & x) const override;
    virtual Eigen::VectorXd dynamics(const Eigen::VectorXd & x, Eigen::MatrixXd & J) const override;
protected:
    static const double p0, M, R, L, T0, g;
};

#endif
