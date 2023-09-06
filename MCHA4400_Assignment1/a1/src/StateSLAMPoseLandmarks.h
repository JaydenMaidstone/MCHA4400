#ifndef STATESLAMPOSELANDMARKS_H
#define STATESLAMPOSELANDMARKS_H

#include <Eigen/Core>
#include "Gaussian.hpp"
#include "StateSLAM.h"

/*
 * State containing body velocities, body pose and landmark poses
 *
 *     [ vBNb     ]  Body translational velocity (body-fixed)
 *     [ omegaBNb ]  Body angular velocity (body-fixed)
 *     [ rBNn     ]  Body position (world-fixed)
 *     [ Thetanb  ]  Body orientation (world-fixed)
 * x = [ rL1Nn     ]  Landmark 1 position (world-fixed)
 *     [ omegaL1Nc ]  Landmark 1 orientation (world-fixed)
 *     [ rL2Nn     ]  Landmark 2 position (world-fixed)
 *     [ omegaL2Nc ]  Landmark 2 orientation (world-fixed)
 *     [ ...       ]  ...
 *
 */
class StateSLAMPoseLandmarks : public StateSLAM
{
public:
    explicit StateSLAMPoseLandmarks(const Gaussian<double> & density);
    StateSLAM * clone() const override;
    virtual std::size_t numberLandmarks() const override;
    virtual std::size_t landmarkPositionIndex(std::size_t idxLandmark) const override;
};

#endif
