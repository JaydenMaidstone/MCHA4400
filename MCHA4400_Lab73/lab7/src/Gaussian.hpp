#ifndef GAUSSIAN_HPP
#define GAUSSIAN_HPP

#include <cstddef>
#include <cmath>
#include <ctime>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/math/special_functions/gamma.hpp>
#include <boost/math/special_functions/erf.hpp>
#include <Eigen/Core>
#include <Eigen/QR>

#ifndef M_PI
#define M_PI 3.14159265358979323846264338327950288
#endif

template <typename Scalar = double>
class Gaussian
{
public:
    Gaussian()
    {}

    explicit Gaussian(std::size_t n)
        : mu_(n)
        , S_(n, n)
    {}

    // template <typename OtherScalar>
    explicit Gaussian(const Eigen::MatrixX<Scalar> & S)
        : mu_(Eigen::VectorX<Scalar>::Zero(S.cols()))
        , S_(S)
    {
        assert(S_.rows() == S_.cols());
    }

    template <typename OtherScalar>
    Gaussian(const Eigen::VectorX<OtherScalar> & mu, const Eigen::MatrixX<OtherScalar> & S)
        : mu_(mu.template cast<Scalar>())
        , S_(S.template cast<Scalar>())
    {
        assert(S_.rows() == S_.cols());
        assert(mu_.rows() == S_.cols());
    }

    template <typename OtherScalar> friend class Gaussian;

    template <typename OtherScalar>
    Gaussian(const Gaussian<OtherScalar> & p)
        : mu_(p.mu_.template cast<Scalar>())
        , S_(p.S_.template cast<Scalar>())
    {
        assert(S_.rows() == S_.cols());
        assert(mu_.rows() == S_.cols());
    }

    template <typename OtherScalar>
    Gaussian<OtherScalar> cast() const
    {
        return Gaussian<OtherScalar>(*this);
    }

    Eigen::Index size() const
    {
        return mu_.size();
    }

    Eigen::VectorX<Scalar> & mean()
    {
        return mu_;
    }

    Eigen::MatrixX<Scalar> & sqrtCov()
    {
        return S_;
    }

    const Eigen::VectorX<Scalar> & mean() const
    {
        return mu_;
    }

    const Eigen::MatrixX<Scalar> & sqrtCov() const
    {
        return S_;
    }

    Eigen::MatrixX<Scalar> cov() const
    {
        return S_.transpose()*S_;
    }

    // Given joint density p(x), return marginal density p(x(idx))
    template <typename IndexType>
    Gaussian marginal(const IndexType & idx) const
    {
        Gaussian out;
        // TODO: Merge from Lab 6
        return out;
    }

    // Given joint density p(x), return conditional density p(x(idxA) | x(idxB) = xB)
    template <typename IndexTypeA, typename IndexTypeB>
    Gaussian conditional(const IndexTypeA & idxA, const IndexTypeB & idxB, const Eigen::VectorX<Scalar> & xB) const
    {
        Gaussian out;
        // TODO: Merge from Lab 6
        return out;
    }

    template <typename Func>
    Gaussian transform(Func h) const
    {
        Gaussian out;
        Eigen::MatrixX<Scalar> C;
        out.mu_ = h(mu_, C);
        const std::size_t & ny = out.mu_.rows();
        Eigen::MatrixX<Scalar> SS = S_*C.transpose();
        Eigen::HouseholderQR<Eigen::Ref<Eigen::MatrixX<Scalar>>> qr(SS);   // In-place QR decomposition
        out.S_ = SS.topRows(ny).template triangularView<Eigen::Upper>();
        return out;
    }

    template <typename Func>
    Gaussian transform(Func h, const Gaussian & noise) const
    {
        assert(noise.mean().isZero());
        Gaussian out;
        Eigen::MatrixX<Scalar> C;
        out.mu_ = h(mu_, C) /*+ noise.mean()*/;
        const std::size_t & nx = mu_.rows();
        const std::size_t & ny = out.mu_.rows();
        Eigen::MatrixX<Scalar> SS(nx + ny, ny);
        SS << S_*C.transpose(), noise.sqrtCov();
        Eigen::HouseholderQR<Eigen::Ref<Eigen::MatrixX<Scalar>>> qr(SS);   // In-place QR decomposition
        out.S_ = SS.topRows(ny).template triangularView<Eigen::Upper>();
        return out;
    }

    // log likelihood and derivatives
    Scalar log(const Eigen::VectorX<Scalar> & x) const
    {
        // TODO: Merge from Lab 6
        return 0;
    }

    Scalar log(const Eigen::VectorX<Scalar> & x, Eigen::VectorX<Scalar> & g) const
    {
        // TODO: Merge from Lab 6
        return log(x);
    }

    Scalar log(const Eigen::VectorX<Scalar> & x, Eigen::VectorX<Scalar> & g, Eigen::MatrixX<Scalar> & H) const
    {
        // TODO: Merge from Lab 6
        return log(x, g);
    }

    Gaussian operator*(const Gaussian & other) const
    {
        const std::size_t & n1 = size();
        const std::size_t & n2 = other.size();
        Gaussian out(n1 + n2);
        out.mu_ << mu_, other.mu_;
        out.S_ << S_,                                  Eigen::MatrixXd::Zero(n1, n2),
                Eigen::MatrixX<Scalar>::Zero(n2, n1), other.S_;
        return out;
    }

    Eigen::VectorX<Scalar> simulate() const
    {
        static boost::random::mt19937 rng(std::time(0));    // Initialise and seed once
        boost::random::normal_distribution<> dist;

        // Draw w ~ N(0, I)
        Eigen::VectorX<Scalar> w(size());
        for (Eigen::Index i = 0; i < size(); ++i)
        {
            w(i) = dist(rng);
        }

        return mu_ + S_.transpose()*w;
    }

    // https://en.wikipedia.org/wiki/Inverse-chi-squared_distribution
    static double chi2inv(double p, double nu)
    {
        assert(p >= 0);
        assert(p < 1);
        // TODO: Lab 7
        return 0.0;
    }

    static double normcdf(double w)
    {
        // TODO: Lab 7
        return 0.0;
    }

    bool isWithinConfidenceRegion(const Eigen::VectorX<Scalar> & x, double nSigma = 3.0)
    {
        const Eigen::Index & n = size();
        // TODO: Lab 7
        return false;
    }

    // Points on boundary of confidence ellipse for a given number of standard deviations
    Eigen::Matrix<Scalar, 2, Eigen::Dynamic> confidenceEllipse(double nSigma = 3.0, int nSamples = 100) const
    {
        const Eigen::Index & n = size();
        assert(n == 2);

        Eigen::Matrix<Scalar, 2, Eigen::Dynamic> X(2, nSamples);
        // TODO: Lab 7
        assert(X.cols() == nSamples);
        assert(X.rows() == 2);
        return X;
    }

    // Quadric surface coefficients for a given number of standard deviations
    Eigen::Matrix4<Scalar> quadricSurface(double nSigma = 3.0) const
    {
        const Eigen::Index & n = size();
        assert(n == 3);
        
        Eigen::Matrix4<Scalar> Q;
        // TODO: Lab 7
        return Q;
    }

protected:
    Eigen::VectorX<Scalar> mu_;
    Eigen::MatrixX<Scalar> S_;
};

#endif
