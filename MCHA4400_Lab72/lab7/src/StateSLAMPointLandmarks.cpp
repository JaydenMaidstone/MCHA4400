#include <cmath>
#include <Eigen/Core>
#include "Gaussian.hpp"
#include "StateSLAM.h"
#include "StateSLAMPointLandmarks.h"

StateSLAMPointLandmarks::StateSLAMPointLandmarks(const Gaussian<double> & density)
    : StateSLAM(density)
{

}

StateSLAM * StateSLAMPointLandmarks::clone() const
{
    return new StateSLAMPointLandmarks(*this);
}

std::size_t StateSLAMPointLandmarks::numberLandmarks() const
{
    return (size() - 12)/3;
}

std::size_t StateSLAMPointLandmarks::landmarkPositionIndex(std::size_t idxLandmark) const
{
    assert(idxLandmark < numberLandmarks());
    return 12 + 3*idxLandmark;    
}