#ifndef STATESLAMPOINTLANDMARKS_H
#define STATESLAMPOINTLANDMARKS_H

#include <Eigen/Core>
#include "Gaussian.hpp"
#include "StateSLAM.h"

/*
 * State containing body velocities, body pose and landmark positions
 *
 *     [ vBNb     ]  Body translational velocity (body-fixed)
 *     [ omegaBNb ]  Body angular velocity (body-fixed)
 *     [ rBNn     ]  Body position (world-fixed)
 * x = [ Thetanb  ]  Body orientation (world-fixed)
 *     [ rL1Nn    ]  Landmark 1 position (world-fixed)
 *     [ rL2Nn    ]  Landmark 2 position (world-fixed)
 *     [ ...      ]  ...
 *
 */
class StateSLAMPointLandmarks : public StateSLAM
{
public:
    explicit StateSLAMPointLandmarks(const Gaussian<double> & density);
    StateSLAM * clone() const override;
    virtual std::size_t numberLandmarks() const override;
    virtual std::size_t landmarkPositionIndex(std::size_t idxLandmark) const override;
};

#endif
