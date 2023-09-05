#ifndef GAUSSIAN_H
#define GAUSSIAN_H

#include <cstddef>
#include <cassert>
#include <Eigen/Core>
#include <Eigen/QR>
#include <cassert>



class Gaussian
{
public:
    Gaussian();
    explicit Gaussian(std::size_t n);
    explicit Gaussian(const Eigen::MatrixXd & S);
    Gaussian(const Eigen::VectorXd & mu, const Eigen::MatrixXd & S);

    std::size_t size() const;
    const Eigen::VectorXd & mean() const;
    const Eigen::MatrixXd & sqrtCov() const;
    Eigen::MatrixXd cov() const;

    // Joint distribution from product of independent Gaussians
    Gaussian operator*(const Gaussian & other) const;

    // Simulate (generate samples)
    Eigen::VectorXd simulate() const;

    // Marginal distribution
    template <typename IndexType> Gaussian marginal(const IndexType & idx) const;

    // Conditional distribution
    template <typename IndexTypeA, typename IndexTypeB> Gaussian conditional(const IndexTypeA & idxA, const IndexTypeB & idxB, const Eigen::VectorXd &xB) const;

    // Affine transform
    template <typename Func> Gaussian transform(Func h) const;
    template <typename Func> Gaussian transformWithAdditiveNoise(Func h, const Gaussian & noise) const;
protected:
    Eigen::VectorXd mu_;
    Eigen::MatrixXd S_;
};

// Given joint density p(x), return marginal density p(x(idx))
template <typename IndexType>
Gaussian Gaussian::marginal(const IndexType & idx) const
{
    Gaussian out(idx.size());

    // TODO
    //Eigen::Map<const Eigen::VectorXi> idxVec(idx.data(), idx.size());
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
Gaussian Gaussian::conditional(const IndexTypeA & idxA, const IndexTypeB & idxB, const Eigen::VectorXd & xB) const
{
    // FIXME: The following implementation is in error, but it does pass some of the unit tests
    Gaussian out;
    // out.mu_ = mu_(idxA) +
    //     S_(idxB, idxA).transpose()*
    //     S_(idxB, idxB).eval().template triangularView<Eigen::Upper>().transpose().solve(xB - mu_(idxB));
   
    //Eigen::MatrixXd S_B = S_(Eigen::all,idxB);
    //Eigen::MatrixXd S_A = S_(Eigen::all,idxA);

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

    // Extract elements from muyxjoint
    //Eigen::VectorXd mu_y = mu_(idxB);
    //Eigen::VectorXd mu_x = mu_(idxA);

    //Eigen::VectorXd y_diff = xB - mu_(idxB);

    //muxcond = muyxjoint.tail(nx) + S2.transpose() * S1_inv.transpose() * (y - muyxjoint.head(ny));

    // Compute the S2.transpose()*S1.-transpose() term
    //Eigen::MatrixXd R1_invT = (R1.transpose()).triangularView<Eigen::Lower>().solve<Eigen::OnTheLeft>(xB - mu_(idxB));

    out.mu_ = mu_(idxA) + (R2.transpose())*(R1.transpose()).triangularView<Eigen::Lower>().solve<Eigen::OnTheLeft>(xB - mu_(idxB));
    //std::cout << "muxcond = \n" << muxcond << "\n" << std::endl;

    out.S_ = R3;

    //out.S_ = S_(idxA, idxA);
    return out;
}

template <typename Func>
Gaussian Gaussian::transform(Func h) const
{
    Gaussian out;
    Eigen::MatrixXd C;
    out.mu_ = h(mu_, C);
    const std::size_t & ny = out.mu_.rows();
    Eigen::MatrixXd SS = S_*C.transpose();
    Eigen::HouseholderQR<Eigen::Ref<Eigen::MatrixXd>> qr(SS);   // In-place QR decomposition
    out.S_ = SS.topRows(ny).triangularView<Eigen::Upper>();
    return out;
}

template <typename Func>
Gaussian Gaussian::transformWithAdditiveNoise(Func h, const Gaussian & noise) const
{
    assert(noise.mean().isZero());
    Gaussian out;
    Eigen::MatrixXd C;
    out.mu_ = h(mu_, C) /*+ noise.mean()*/;
    const std::size_t & nx = mu_.rows();
    const std::size_t & ny = out.mu_.rows();
    Eigen::MatrixXd SS(nx + ny, ny);
    SS << S_*C.transpose(), noise.sqrtCov();
    Eigen::HouseholderQR<Eigen::Ref<Eigen::MatrixXd>> qr(SS);   // In-place QR decomposition
    out.S_ = SS.topRows(ny).triangularView<Eigen::Upper>();
    return out;
}

#endif
