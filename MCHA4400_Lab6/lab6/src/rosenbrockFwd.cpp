#include <Eigen/Core>
#include <autodiff/forward/dual.hpp>
#include <autodiff/forward/dual/eigen.hpp>
#include "rosenbrock.hpp"

// Note: The forward-mode and reverse-mode autodiff implementations are in
//       separate compilation units due to the following known issue with autodiff v0.6:
//       https://github.com/autodiff/autodiff/issues/179

// Functor for Rosenbrock function and its derivatives using forward-mode autodifferentiation
double RosenbrockFwdAutoDiff::operator()(const Eigen::VectorXd & x)
{   
    return rosenbrock(x);
}

double RosenbrockFwdAutoDiff::operator()(const Eigen::VectorXd & x, Eigen::VectorXd & g)
{
    // Forward-mode autodifferentiation
    Eigen::VectorX<autodiff::dual> xdual = x.cast<autodiff::dual>();
    autodiff::dual fdual;
    g = gradient(rosenbrock<autodiff::dual>, wrt(xdual), at(xdual), fdual);
    return val(fdual);
}

double RosenbrockFwdAutoDiff::operator()(const Eigen::VectorXd & x, Eigen::VectorXd & g, Eigen::MatrixXd & H)
{
    // Forward-mode autodifferentiation
    Eigen::VectorX<autodiff::dual2nd> xdual = x.cast<autodiff::dual2nd>();
    autodiff::dual2nd fdual;
    H = hessian(rosenbrock<autodiff::dual2nd>, wrt(xdual), at(xdual), fdual, g);
    return val(fdual);
}
