#include <cstddef>
#include <cmath>
#include <ctime>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/normal_distribution.hpp>
#include <Eigen/Core>
#include <Eigen/QR>
#include "Gaussian.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846264338327950288
#endif

Gaussian::Gaussian() {}

Gaussian::Gaussian(std::size_t n)
    : mu_(n)
    , S_(n, n)
{}

Gaussian::Gaussian(const Eigen::MatrixXd & S)
    : mu_(Eigen::VectorXd::Zero(S.cols()))
    , S_(S)
{
    assert(S_.rows() == S_.cols());
}

Gaussian::Gaussian(const Eigen::VectorXd & mu, const Eigen::MatrixXd & S)
    : mu_(mu)
    , S_(S)
{
    assert(S_.rows() == S_.cols());
    assert(mu_.rows() == S_.cols());
}

std::size_t Gaussian::size() const
{
    assert(mu_.rows() == S_.cols());
    return mu_.rows();
}

const Eigen::VectorXd & Gaussian::mean() const
{
    return mu_;
}

const Eigen::MatrixXd & Gaussian::sqrtCov() const
{
    return S_;
}

Eigen::MatrixXd Gaussian::cov() const
{
    return S_.transpose()*S_;
}

Gaussian Gaussian::operator*(const Gaussian & other) const
{
    const std::size_t & n1 = size();
    const std::size_t & n2 = other.size();
    Gaussian out(n1 + n2);
    out.mu_ << mu_, other.mu_;
    out.S_ << S_,                            Eigen::MatrixXd::Zero(n1, n2),
              Eigen::MatrixXd::Zero(n2, n1), other.S_;
    return out;
}

Eigen::VectorXd Gaussian::simulate() const
{
    static boost::random::mt19937 rng(std::time(0));    // Initialise and seed once
    boost::random::normal_distribution<> dist;

    // Draw w ~ N(0, I)
    Eigen::VectorXd w(size());
    for (std::size_t i = 0; i < size(); i++)
    {
        w(i) = dist(rng);
    }

    return mu_ + S_.transpose()*w;
}
