#include <cmath>
#include <Eigen/Core>
#include "Gaussian.hpp"
#include "StateSLAM.h"
#include "StateSLAMPoseLandmarks.h"

StateSLAMPoseLandmarks::StateSLAMPoseLandmarks(const Gaussian<double> & density)
    : StateSLAM(density)
{

}

StateSLAM * StateSLAMPoseLandmarks::clone() const
{
    return new StateSLAMPoseLandmarks(*this);
}

std::size_t StateSLAMPoseLandmarks::numberLandmarks() const
{
    return (size() - 12)/6;
}

std::size_t StateSLAMPoseLandmarks::landmarkPositionIndex(std::size_t idxLandmark) const
{
    assert(idxLandmark < numberLandmarks());
    return 12 + 6*idxLandmark;    
}