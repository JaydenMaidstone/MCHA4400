#include <Eigen/Core>
#include "rosenbrock.hpp"

// Functor for Rosenbrock function and its derivatives
double RosenbrockAnalytical::operator()(const Eigen::VectorXd &x)
{
    return rosenbrock(x);
}

double RosenbrockAnalytical::operator()(const Eigen::VectorXd &x, Eigen::VectorXd &g)
{
    g.resize(2, 1);
    // TODO: Write gradient to g

    double x1 = x[0];
    double x2 = x[1];
    
    // Compute gradient
    g[0] = -2 * (1 - x1) - 400 * x1 * (x2 - x1 * x1);
    g[1] = 200 * (x2 - x1 * x1);

    return operator()(x);
}

double RosenbrockAnalytical::operator()(const Eigen::VectorXd &x, Eigen::VectorXd &g, Eigen::MatrixXd &H)
{
    H.resize(2, 2);
    // TODO: Write Hessian to H
    double x1 = x[0];
    double x2 = x[1];
    
    //H(0, 0) = 2 + 1200 * x1 * x1 - 400 * x2;
    H(0, 0) = 800*x1*x1-400*(x2-x1*x1) + 2;
    H(0, 1) = -400 * x1;
    H(1, 0) = -400 * x1;
    H(1, 1) = 200;

    return operator()(x, g);
}
