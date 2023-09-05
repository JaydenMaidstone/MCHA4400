#include <cmath>
#include <Eigen/Core>
#include "Gaussian.h"
#include "State.h"
#include "StateBallistic.h"

const double StateBallistic::p0 = 101.325e3;            // Air pressure at sea level [Pa]
const double StateBallistic::M  = 0.0289644;            // Molar mass of dry air [kg/mol]
const double StateBallistic::R  = 8.31447;              // Gas constant [J/(mol.K)]
const double StateBallistic::L  = 0.0065;               // Temperature gradient [K/m]
const double StateBallistic::T0 = 288.15;               // Temperature at sea level [K]
const double StateBallistic::g  = 9.81;                 // Acceleration due to gravity [m/s^2]

StateBallistic::StateBallistic(const Gaussian & density)
    : State(density)
{
    // SQ_ is an upper triangular matrix such that SQ_.'*SQ_ = Q is the power spectral density of the continuous time process noise
    SQ_ << 0,     0,    0,
           0, 1e-10,    0,
           0,     0, 5e-6;
}

// Evaluate f(x) from the SDE dx = f(x)*dt + dw
Eigen::VectorXd StateBallistic::dynamics(const Eigen::VectorXd & x) const
{
    Eigen::VectorXd f(x.size());
    // TODO: Set f
    using std::pow;
    f << x(1),
         (0.5*(M*p0/R) * (1/(T0 - L*x(0))) * pow(1-(L*x(0)/T0), g*M/(R*L)) * x(1)*x(1)*x(2)) - g,
         0;

    return f;
}

// Evaluate f(x) and its Jacobian J = df/fx from the SDE dx = f(x)*dt + dw
Eigen::VectorXd StateBallistic::dynamics(const Eigen::VectorXd & x, Eigen::MatrixXd & J) const
{
    Eigen::VectorXd f = dynamics(x);
    using std::pow;
    J.resize(x.size(), x.size());
    
    J << 0, 1, 0,
        (L*M*x(2)*p0*pow(x(1),2)*pow( (1 - (L*x(0))/T0), ((M*g)/(L*R))))/(2*R*pow((T0 - L*x(0)),2)) - (pow(M,2)*x(2)*g*p0*pow(x(1),2)*pow( (1 - (L*x(0))/T0),((M*g)/(L*R)) - 1))/(2*pow(R,2)*T0*(T0 - L*x(0))), (M*x(2)*p0*x(1)*pow( (1 - (L*x(0))/T0), ((M*g)/(L*R))))/(R*(T0 - L*x(0))), (M*p0*pow(x(1),2)*pow( (1 - (L*x(0))/T0), ((M*g)/(L*R))))/(2*R*(T0 - L*x(0))),
        0, 0, 0;

    return f;
}
