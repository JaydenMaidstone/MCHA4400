#include <cstddef>
#include <cassert>
#include <cmath>
#include <Eigen/Core>
#include "Gaussian.hpp"
#include "State.h"

State::~State() = default;

State::State(std::size_t n)
    : density(n)
    , SQ_(n,n)
    , time_(0)
{
    SQ_.fill(0);
}

State::State(const Gaussian<double> & density)
    : density(density)
    , SQ_(density.size(),density.size())
    , time_(0)
{
    SQ_.fill(0);
}

std::size_t State::size() const
{
    return density.size();
}

// Evaluate F(X) from dX = F(X)*dt + dW
Eigen::MatrixXd State::augmentedDynamics(const Eigen::MatrixXd & X) const
{
    assert(X.size() > 0);
    int nx = X.rows();
    assert(X.cols() == 2*nx + 1);

    Eigen::VectorXd x = X.col(0);
    Eigen::MatrixXd J;
    Eigen::VectorXd f = dynamics(x, J);
    assert(f.rows() == nx);
    assert(J.rows() == nx);

    Eigen::MatrixXd dX(nx, 2*nx + 1);
    dX << f, J*X.block(0, 1, nx, 2*nx);
    return dX;
}

// Map [x[k]; dw[k]] to x[k+1] using RK4
Eigen::VectorXd State::RK4SDEHelper(const Eigen::VectorXd & xdw, double dt, Eigen::MatrixXd &J) const
{
    const std::size_t nxdw = xdw.size();
    assert(nxdw > 0);
    assert(nxdw % 2 == 0);
    const std::size_t nx = nxdw/2;
    Eigen::VectorXd x(nx), dw(nx);

    x       = xdw.head(nx);
    dw      = xdw.tail(nx);

    typedef Eigen::MatrixXd Matrix;

    Matrix X(nx, nxdw+1), dW(nx, nxdw+1);
    X  <<  x, Matrix::Identity(nx, nx), Matrix::Zero(nx, nx);
    dW << dw, Matrix::Zero(nx, nx),     Matrix::Identity(nx, nx);

    Matrix F1, F2, F3, F4, Xnext;
    F1 = augmentedDynamics(                 X);
    F2 = augmentedDynamics(X + (F1*dt + dW)/2);
    F3 = augmentedDynamics(X + (F2*dt + dW)/2);
    F4 = augmentedDynamics(    X + F3*dt + dW);

    Xnext = X + (F1 + 2*F2 + 2*F3 + F4)*dt/6 + dW;
    J     = Xnext.block(0, 1, nx, 2*nx);
    Eigen::VectorXd xnext = Xnext.col(0);
    return xnext;
}

void State::predict(double time)
{
    double dt = time - time_;
    assert(dt >= 0);
    if (dt == 0.0) return;

    // Augment state density with independent noise increment dw ~ N(0, Q*dt)
    // [ x] ~ N([mu] , [P,    0])
    // [dw]    ([ 0]   [0, Q*dt])

    // p(x, dw) = p(x)*p(dw)
    Gaussian augmentedDensity = density*Gaussian((SQ_*std::sqrt(dt)).eval());

    auto func = [&](const Eigen::VectorXd & x, Eigen::MatrixXd & J){ return RK4SDEHelper(x, dt, J); };
    density = augmentedDensity.transform(func);

    time_ = time;
}
