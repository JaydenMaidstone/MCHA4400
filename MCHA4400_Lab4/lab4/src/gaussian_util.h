#ifndef GAUSSIAN_UTIL_H
#define GAUSSIAN_UTIL_H

// Tip: Only include headers needed to parse this header only
#include <Eigen/Core>

// TODO: Function prototypes
void pythagoreanQR(const Eigen::MatrixXd & S1, const Eigen::MatrixXd & S2, Eigen::MatrixXd & S);

void conditionGaussianOnMarginal(const Eigen::VectorXd & muyxjoint, const Eigen::MatrixXd & Syxjoint, const Eigen::VectorXd & y, Eigen::VectorXd & muxcond, Eigen::MatrixXd & Sxcond);


#endif
