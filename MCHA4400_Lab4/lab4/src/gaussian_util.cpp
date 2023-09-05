// Tip: Only include headers needed to parse this implementation only
#include <cassert>
#include <Eigen/Core>
#include <Eigen/QR>
#include <Eigen/Cholesky>
#include <Eigen/Dense>

#include "gaussian_util.h"

// TODO: Function implementations
void pythagoreanQR(const Eigen::MatrixXd & S1, const Eigen::MatrixXd & S2, Eigen::MatrixXd & S)
{    
    // Implementation of the pythagoreanQR function will go here
    // Concatenate S1 and S2 vertically to form S
    S.resize(S1.rows() + S2.rows(), S1.cols());
    S << S1, S2;

    // Perform QR decomposition on matrix S
    Eigen::HouseholderQR<Eigen::MatrixXd> qr(S);
    qr.compute(S);

    // S will now hold the upper triangular factor of the QR decomposition
    S = qr.matrixQR().triangularView<Eigen::Upper>();
}

void conditionGaussianOnMarginal(const Eigen::VectorXd & muyxjoint, const Eigen::MatrixXd & Syxjoint, const Eigen::VectorXd & y, Eigen::VectorXd & muxcond, Eigen::MatrixXd & Sxcond)
{
    int ny = y.size();
    int nx = muyxjoint.size() - ny;

    Eigen::MatrixXd S1 = Syxjoint.topLeftCorner(ny, ny);
    Eigen::MatrixXd S2 = Syxjoint.topRightCorner(ny, nx);
    Eigen::MatrixXd S3 = Syxjoint.bottomRightCorner(nx, nx);

    // Extract elements from muyxjoint
    Eigen::VectorXd mu_y = muyxjoint.head(ny);
    Eigen::VectorXd mu_x = muyxjoint.tail(nx);

    Eigen::VectorXd y_diff = y - mu_y;
    //muxcond = muyxjoint.tail(nx) + S2.transpose() * S1_inv.transpose() * (y - muyxjoint.head(ny));

    // Compute the S2.transpose()*S1.-transpose() term
    Eigen::MatrixXd S2T_S1_invT = (S1.transpose()).triangularView<Eigen::Lower>().solve<Eigen::OnTheRight>(S2.transpose());

    muxcond = mu_x + (S2T_S1_invT)*y_diff;
    //std::cout << "muxcond = \n" << muxcond << "\n" << std::endl;

    Sxcond = S3;
}
