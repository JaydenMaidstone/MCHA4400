#include <Eigen/Core>
#include "Event.h"
#include "State.h"
#include "Gaussian.h"
#include "Measurement.h"

Measurement::Measurement(double time, const Eigen::VectorXd & y)
    : Event(time)
    , y_(y)
{}

Measurement::Measurement(double time, const Eigen::VectorXd & y, const Gaussian & noise)
    : Event(time)
    , y_(y)
    , noise_(noise)
{}

// Augmented measurement model
// [ y ] = [ h(x) ] + [ v ]
// [ x ]   [ x    ]   [ 0 ]
// \___/   \______/   \___/
//   ya  =   ha     +   va

// Evaluate ha(x) and its Jacobian Ja = dha/fx
Eigen::MatrixXd Measurement::augmentedPredict(const Eigen::MatrixXd & x, Eigen::MatrixXd & Ja) const
{
    Eigen::MatrixXd J;
    Eigen::VectorXd h = predict(x, J);

    const std::size_t ny = h.size();
    const std::size_t nx = x.size();

    Eigen::VectorXd ha(ny + nx);
    ha << h,
          x;

    Ja.resize(ny + nx, nx);
    Ja << J,
          Eigen::MatrixXd::Identity(nx, nx);

    return ha;
}

void Measurement::update(State & state)
{
    const std::size_t & ny = y_.size();
    const std::size_t & nx = state.size();
    
    auto func = [&](const Eigen::VectorXd & x, Eigen::MatrixXd & J){ return augmentedPredict(x, J); };

    // p(va) = p(v, 0) = p(v)*p(0)
    Gaussian augmentedNoise = noise_*Gaussian(Eigen::MatrixXd::Zero(nx, nx));

    Gaussian augmentedDensity = state.density.transformWithAdditiveNoise(func, augmentedNoise);
    state.density = augmentedDensity.conditional(Eigen::lastN(nx), Eigen::seqN(0, ny), y_);
}

Eigen::VectorXd Measurement::simulate(const Eigen::VectorXd & x) const
{
    Gaussian p(predict(x), noise_.sqrtCov());
    return p.simulate();
}
