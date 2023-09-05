#ifndef GAUSSIAN_HPP
#define GAUSSIAN_HPP

#include <cstddef>
#include <cmath>
#include <ctime>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/normal_distribution.hpp>
#include <Eigen/Core>
#include <Eigen/QR>
#include <Eigen/LU> // for .determinant() and .inverse(), which you should never use

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
        // out.mu_ = ???
        // out.S_ = ???
        out.mu_ = mu_(idx);
        //Eigen::MatrixXd temp = S_(Eigen::all,idxVec.array());
        // Perform QR decomposition
        Eigen::HouseholderQR<Eigen::MatrixXd> qr(S_(Eigen::all,idx));
        // Extract upper triangular matrix
        out.S_ = qr.matrixQR().triangularView<Eigen::Upper>();
        return out;
    }

    // Given joint density p(x), return conditional density p(x(idxA) | x(idxB) = xB)
    template <typename IndexTypeA, typename IndexTypeB>
    Gaussian conditional(const IndexTypeA & idxA, const IndexTypeB & idxB, const Eigen::VectorX<Scalar> & xB) const
    {
        // FIXME: The following implementation is in error, but it does pass some of the unit tests
        Gaussian out;
        Eigen::MatrixXd S_con(S_.rows(), idxA.size() + idxB.size());
        // Concatenate S_A and S_B horizontally
        S_con.block(0, 0, S_.rows(), idxB.size()) = S_(Eigen::all,idxB);
        S_con.block(0, idxB.size(), S_.rows(), idxA.size()) = S_(Eigen::all,idxA);

        Eigen::MatrixXd R;
        Eigen::HouseholderQR<Eigen::MatrixXd> qr(S_con);

        R = qr.matrixQR().triangularView<Eigen::Upper>();;

        int ny = xB.size();
        int nx = mu_.size() - ny;

        Eigen::MatrixXd R1 = R.topLeftCorner(ny, ny);
        Eigen::MatrixXd R2 = R.topRightCorner(ny, nx);
        Eigen::MatrixXd R3 = R.bottomRightCorner(nx, nx);

        out.mu_ = mu_(idxA) + (R2.transpose())*(R1.transpose()).triangularView<Eigen::Lower>().solve<Eigen::OnTheLeft>(xB - mu_(idxB));
        out.S_ = R3;

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
    Gaussian transformWithAdditiveNoise(Func h, const Gaussian & noise) const
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
        assert(x.cols() == 1);
        assert(x.size() == size());

        
        // TODO: Numerically stable version
        Eigen::VectorX<Scalar> y = S_.transpose().template triangularView<Eigen::Lower>().solve(x - mu_);
        return -0.5 * y.squaredNorm() - 0.5 * (x.size() * std::log(2.0 * M_PI) + 2.0 * S_.diagonal().array().abs().log().sum());
    }

    Scalar log(const Eigen::VectorX<Scalar> & x, Eigen::VectorX<Scalar> & g) const
    {
        // TODO: Compute gradient of log N(x; mu, P) w.r.t x and write it to g
    //    assert(x.cols() == 1);
    // assert(x.size() == size());

    // Eigen::VectorX<Scalar> diff = x - mu_;
    // Eigen::VectorX<Scalar> P_inv_diff = S_.transpose().template triangularView<Eigen::Lower>().solve(diff);

    // Gradient
        //g = -P_inv_diff;
        g.resize(x.size(), 1);
        Eigen::VectorXd diff = -(x - mu_);

        // // Solve for y
        Eigen::VectorXd P_inv_diff = S_.transpose().template triangularView<Eigen::Lower>().solve(diff);
        g = S_.template triangularView<Eigen::Upper>().solve(P_inv_diff);
        return log(x);
    }

    Scalar log(const Eigen::VectorX<Scalar> & x, Eigen::VectorX<Scalar> & g, Eigen::MatrixX<Scalar> & H) const
    {
        // TODO: Compute Hessian of log N(x; mu, P) w.r.t x and write it to H
        H.resize(x.size(), x.size());

        // Eigen::MatrixXd P_inv = S_.inverse() * S_.transpose().inverse();
        // P_inv = -P_inv;
        // H = P_inv;

        Eigen::MatrixXd S_inv = S_.template triangularView<Eigen::Upper>().solve(Eigen::MatrixX<Scalar>::Identity(size(), size()));
        H = -S_inv * S_inv.transpose();

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
protected:
    Eigen::VectorX<Scalar> mu_;
    Eigen::MatrixX<Scalar> S_;
};

#endif
