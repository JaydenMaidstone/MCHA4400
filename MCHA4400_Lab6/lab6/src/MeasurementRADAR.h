#ifndef MEASUREMENTRADAR_H
#define MEASUREMENTRADAR_H

#include <Eigen/Core>
#include <unsupported/Eigen/CXX11/Tensor>
#include "State.h"
#include "Measurement.h"

class MeasurementRADAR : public Measurement
{
public:
    MeasurementRADAR(double time, const Eigen::VectorXd & y);
    virtual Eigen::VectorXd predict(const Eigen::VectorXd & x) const override;
    virtual Eigen::VectorXd predict(const Eigen::VectorXd & x, Eigen::MatrixXd & J) const override;
    virtual Eigen::VectorXd predict(const Eigen::VectorXd & x, Eigen::MatrixXd & J, Eigen::Tensor<double, 3> & H) const override;
    virtual double logLikelihood(const Eigen::VectorXd & x) const override;
    virtual double logLikelihood(const Eigen::VectorXd & x, Eigen::VectorXd & g) const override;
    virtual double logLikelihood(const Eigen::VectorXd & x, Eigen::VectorXd & g, Eigen::MatrixXd & H) const override;
protected:
    static const double r1, r2;
    template <typename Scalar> Eigen::VectorX<Scalar> predictImpl(const Eigen::VectorX<Scalar> & x) const;
    template <typename Scalar> Scalar logLikelihoodImpl(const Eigen::VectorX<Scalar> & x) const;
};

// Note: To enable autodiff to be used without duplicating functions for all the
//       different scalar types (e.g., double, autodiff::real, autodiff::dual,
//       autodiff::dual2nd, autodiff::val), we template out the scalar type.
//       Since the functions we want to autodiff are member functions, and we
//       can't template virtual member functions (the vtable would be infinite),
//       we create some helper non-virtual template member functions below.

template <typename Scalar>
Eigen::VectorX<Scalar> MeasurementRADAR::predictImpl(const Eigen::VectorX<Scalar> & x) const
{
    using std::hypot;      // Bring into global namespace, which is where autodiff provides its functions
    Eigen::VectorX<Scalar> h(1);
    h(0) = hypot(r1, x(0) - r2);
    return h;
}

template <typename Scalar>
Scalar MeasurementRADAR::logLikelihoodImpl(const Eigen::VectorX<Scalar> & x) const
{
    Eigen::VectorX<Scalar> y = y_.cast<Scalar>();
    Eigen::VectorX<Scalar> h = predictImpl<Scalar>(x);
    Eigen::MatrixX<Scalar> SR = noise_.sqrtCov().cast<Scalar>();
    Gaussian<Scalar> likelihood(h, SR);
    return likelihood.log(y);
}

#endif
