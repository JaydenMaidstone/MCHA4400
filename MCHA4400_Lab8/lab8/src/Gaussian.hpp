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
        Gaussian out(idx.size());
        // TODO: Merge from Lab 6
        out.mu_ = mu_(idx);

         // Check if covariance matrix is square
        //Eigen::MatrixXd temp = S_(Eigen::all,idxVec.array());
        // Perform QR decomposition
        Eigen::MatrixXd temp = S_(Eigen::all,idx);
        Eigen::HouseholderQR<Eigen::MatrixXd> qr(temp);
        Eigen::MatrixXd qr_matrix = qr.matrixQR();


        if (qr_matrix.rows() != qr_matrix.cols()) 
        {
        //std::cout << "S not square" << std::endl;
        int required_rows = qr_matrix.cols();
        qr_matrix.conservativeResize(required_rows, qr_matrix.cols()); // Resize to square by removing rows
        assert(qr_matrix.rows() == qr_matrix.cols());
        }
        assert(qr_matrix.rows() == qr_matrix.cols());
        // Extract upper triangular matrix
        out.S_ = qr_matrix.triangularView<Eigen::Upper>();
        //std::cout << S_ << std::endl;
        return out;
    }

    // Given joint density p(x), return conditional density p(x(idxA) | x(idxB) = xB)
    template <typename IndexTypeA, typename IndexTypeB>
    Gaussian conditional(const IndexTypeA & idxA, const IndexTypeB & idxB, const Eigen::VectorX<Scalar> & xB) const
    {
        Gaussian out;
        // TODO: Merge from Lab 6
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
        assert(x.cols() == 1);
        //assert(x.size() == size());

        
        // TODO: Numerically stable version
        Eigen::VectorX<Scalar> y = S_.transpose().template triangularView<Eigen::Lower>().solve(x - mu_);
        return -0.5 * y.squaredNorm() - 0.5 * (x.size() * std::log(2.0 * M_PI) + 2.0 * S_.diagonal().array().abs().log().sum());
    
        //return 0;
    }

    Scalar log(const Eigen::VectorX<Scalar> & x, Eigen::VectorX<Scalar> & g) const
    {
        // TODO: Merge from Lab 6
        g.resize(x.size(), 1);
        Eigen::VectorXd diff = -(x - mu_);

        // // Solve for y
        Eigen::VectorXd P_inv_diff = S_.transpose().template triangularView<Eigen::Lower>().solve(diff);
        g = S_.template triangularView<Eigen::Upper>().solve(P_inv_diff);
        
        return log(x);
    }

    Scalar log(const Eigen::VectorX<Scalar> & x, Eigen::VectorX<Scalar> & g, Eigen::MatrixX<Scalar> & H) const
    {
        // TODO: Merge from Lab 6
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

    // https://en.wikipedia.org/wiki/Inverse-chi-squared_distribution
    static double chi2inv(double p, double nu)
    {
        assert(p >= 0);
        assert(p < 1);
        // TODO: Lab 7
        double halfNu = nu * 0.5;
        double gamma_p_inv_value = boost::math::gamma_p_inv(halfNu, p);
        return 2.0 * gamma_p_inv_value;
    }

    static double normcdf(double w)
    {
        // TODO: Lab 7
        double erfc_value = boost::math::erfc(-w / std::sqrt(2.0));
        return 0.5 * erfc_value;
    }

    bool isWithinConfidenceRegion(const Eigen::VectorX<Scalar> & x, double nSigma = 3.0)
    {
        const Eigen::Index & n = size();
        // TODO: Lab 7

        double c = 2*normcdf(nSigma)-1;
        //Eigen::VectorX<Scalar> y = S_.transpose().template triangularView<Eigen::Lower>().solve(x - mu_);
        //Eigen::VectorX<Scalar> w = y.squaredNorm();
        double wTw = (S_.transpose().template triangularView<Eigen::Lower>().solve(x - mu_)).squaredNorm();

        // Check if each element of w is within the confidence interval
        bool isWithin = false;
            if (wTw <= chi2inv(c,n))
            {
                isWithin = true;
            }
        

        return isWithin;
        //return false;
    }

    // Points on boundary of confidence ellipse for a given number of standard deviations
    Eigen::Matrix<Scalar, 2, Eigen::Dynamic> confidenceEllipse(double nSigma = 3.0, int nSamples = 100) const
    {
        const Eigen::Index & n = size();
        assert(n == 2);

        Eigen::Matrix<Scalar, 2, Eigen::Dynamic> X(2, nSamples);
        // TODO: Lab 7
        double c = 2*normcdf(nSigma)-1;
        double chi2_value = chi2inv(c, 2);

        Eigen::VectorXd t = Eigen::VectorXd::LinSpaced(100, 0, 2 * M_PI);
        //double r = std::sqrt(chi2_value);
        //std::cout << std::sqrt(chi2_value) << std::endl;
        //std::cout << S_ << std::endl;
        double r = std::sqrt(chi2_value);

        for (int i = 0; i < nSamples; ++i)
        {
            double w1 = r * std::cos(t(i));
            double w2 = r * std::sin(t(i));

            Eigen::VectorX<Scalar> w(2);
            w << w1, w2;

            Eigen::VectorX<Scalar> y = mu_ + S_.transpose() * w;
            X.col(i) = y;
        }

        
        assert(X.cols() == nSamples);
        assert(X.rows() == 2);
        return X;
    }

    // Quadric surface coefficients for a given number of standard deviations
    Eigen::Matrix4<Scalar> quadricSurface(double nSigma = 3.0) const
    {
        const Eigen::Index & n = size();
        assert(n == 3);
        
        //Eigen::Matrix4<Scalar> Q;
        Eigen::Matrix4<Scalar> Q;
        // TODO: Lab 7
        double c = 2*normcdf(nSigma)-1;
        double chi2_value = chi2inv(c, n);
        //Eigen::MatrixXd S_inv = S_.template triangularView<Eigen::Upper>().solve(Eigen::MatrixX<Scalar>::Identity(size(), size()));
        Eigen::MatrixX<Scalar> S_inv = S_.template triangularView<Eigen::Upper>().solve(Eigen::MatrixX<Scalar>::Identity(n, n));

        Eigen::VectorX<Scalar> z = S_.transpose().template triangularView<Eigen::Lower>().solve(mu_);

        Eigen::VectorX<Scalar> y = S_.template triangularView<Eigen::Upper>().solve(z);

        // Define the identity matrix
        //Eigen::MatrixXd I = Eigen::MatrixXd::Identity(n, n);
        // Compute the "inverse" of S using the solve method
        //Eigen::MatrixXd S_inv = S_.template triangularView<Eigen::Lower>().solve(I);

        Eigen::MatrixX<Scalar> A = S_inv * S_inv.transpose();

        //Eigen::MatrixX<Scalar> z = mu_* S_inv_T;

        Eigen::MatrixX<Scalar> B = -y;

        Eigen::MatrixX<Scalar> C = B.transpose();

        Scalar z_squaredNorm = z.squaredNorm();
        //std::cout << Q << std::endl;
        //Q << A, B, C, D;
        Q.topLeftCorner(3,3) = A;
        Q.topRightCorner(3,1) = B;
        Q.bottomLeftCorner(1,3) = C;
        Q(3,3) = z_squaredNorm - chi2_value;

        //Q.block(1,1) = A;


        return Q;
    }

protected:
    Eigen::VectorX<Scalar> mu_;
    Eigen::MatrixX<Scalar> S_;
};

#endif
