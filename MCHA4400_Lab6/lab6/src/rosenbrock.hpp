#ifndef ROSENBROCK_H
#define ROSENBROCK_H

#include <Eigen/Core>

// Templated version of Rosenbrock function
template <typename Scalar>
static Scalar rosenbrock(const Eigen::VectorX<Scalar> & x)
{   
    Scalar x2 = x(0)*x(0);
    Scalar ymx2 = x(1) - x2;
    Scalar xm1 = x(0) - 1;
    return (xm1*xm1 + 100*ymx2*ymx2);
}

// Functors for Rosenbrock function and its derivatives

// Functor for Rosenbrock function and its analytical derivatives
struct RosenbrockAnalytical
{
    double operator()(const Eigen::VectorXd & x);
    double operator()(const Eigen::VectorXd & x, Eigen::VectorXd & g);
    double operator()(const Eigen::VectorXd & x, Eigen::VectorXd & g, Eigen::MatrixXd & H);
};

// Functor for Rosenbrock function and its derivatives using forward-mode autodifferentiation
struct RosenbrockFwdAutoDiff
{
    double operator()(const Eigen::VectorXd & x);
    double operator()(const Eigen::VectorXd & x, Eigen::VectorXd & g);
    double operator()(const Eigen::VectorXd & x, Eigen::VectorXd & g, Eigen::MatrixXd & H);
};

// Functor for Rosenbrock function and its derivatives using reverse-mode autodifferentiation
struct RosenbrockRevAutoDiff
{
    double operator()(const Eigen::VectorXd & x);
    double operator()(const Eigen::VectorXd & x, Eigen::VectorXd & g);
    double operator()(const Eigen::VectorXd & x, Eigen::VectorXd & g, Eigen::MatrixXd & H);
};

#endif
