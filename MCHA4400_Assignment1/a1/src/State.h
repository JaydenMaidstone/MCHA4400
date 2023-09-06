#ifndef STATE_H
#define STATE_H

#include <cstddef>
#include <Eigen/Core>
#include "Gaussian.hpp"

class State
{
public:
    Gaussian<double> density;
    virtual ~State();
    explicit State(std::size_t n);
    explicit State(const Gaussian<double> & density);

    std::size_t size() const;
    void predict(double time);
    virtual Eigen::VectorXd dynamics(const Eigen::VectorXd & x) const = 0;
    virtual Eigen::VectorXd dynamics(const Eigen::VectorXd & x, Eigen::MatrixXd & J) const = 0;
protected:
    Eigen::MatrixXd augmentedDynamics(const Eigen::MatrixXd & X) const;
    Eigen::VectorXd RK4SDEHelper(const Eigen::VectorXd & xdw, double dt, Eigen::MatrixXd & J) const;
    Eigen::MatrixXd SQ_;
private:
    double time_;
};

#endif
