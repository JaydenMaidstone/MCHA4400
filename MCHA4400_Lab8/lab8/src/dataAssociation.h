#ifndef DATAASSOCIATION_H
#define DATAASSOCIATION_H

#include <cstddef>
#include <Eigen/Core>
#include <vector>
#include "Camera.h"
#include "StateSLAM.h"

double snn(const StateSLAM & state, const std::vector<std::size_t> & idxLandmarks, const Eigen::Matrix<double, 2, Eigen::Dynamic> & Y, const Camera & camera, std::vector<int> & idx, bool enforceJointCompatibility = false);
bool individualCompatibility(const int & i, const int & j, const Eigen::Matrix<double, 2, Eigen::Dynamic> & Y, const Gaussian<double> & density, const double & nstd);
bool jointCompatibility(const std::vector<int> & idx, const double & sU, const Eigen::Matrix<double, 2, Eigen::Dynamic> & Y, const Gaussian<double> & density, const double & nstd, double & surprisal);

#endif
