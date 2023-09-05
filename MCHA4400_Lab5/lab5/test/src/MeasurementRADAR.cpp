#include <doctest/doctest.h>
#include <cstddef>
#include <Eigen/Core>
#include "../../src/MeasurementRADAR.h"
#include "../../src/Gaussian.h"

#ifndef CAPTURE_EIGEN
#define CAPTURE_EIGEN(x) INFO(#x " = \n", x.format(Eigen::IOFormat(Eigen::FullPrecision, 0, ", ", ";\n", "", "", "[", "]")));
#endif

SCENARIO("MeasurementRADAR")
{
    const std::size_t ny = 1;
    Eigen::VectorXd y(ny);
    y.setZero();
    double t = 0.0;
    MeasurementRADAR measurement(t, y);

    GIVEN("A state vector")
    {
        const std::size_t nx = 3;
        Eigen::VectorXd x(nx);
        x <<    13955,
             -449.745,
               0.0005;

        WHEN("Evaluating h = measurement.predict(x)")
        {
            Eigen::VectorXd h = measurement.predict(x);

            THEN("h has expected dimensions")
            {
                REQUIRE(h.rows() == ny);
                REQUIRE(h.cols() == 1);

                AND_THEN("h has expected values")
                {
                    Eigen::VectorXd h_expected(ny);
                    h_expected << 10256.3163465252;
                    CAPTURE_EIGEN(h);
                    CAPTURE_EIGEN(h_expected);
                    CHECK(h.isApprox(h_expected));
                }
            }
        }

        WHEN("Evaluating h = measurement.predict(x, J)")
        {
            Eigen::MatrixXd J;
            Eigen::VectorXd h = measurement.predict(x, J);

            THEN("h has expected dimensions")
            {
                REQUIRE(h.rows() == ny);
                REQUIRE(h.cols() == 1);

                AND_THEN("h has expected values")
                {
                    Eigen::VectorXd h_expected(ny);
                    h_expected << 10256.3163465252;
                    CAPTURE_EIGEN(h);
                    CAPTURE_EIGEN(h_expected);
                    CHECK(h.isApprox(h_expected));
                }
            }

            THEN("J has expected dimensions")
            {
                REQUIRE(J.rows() == ny);
                REQUIRE(J.cols() == nx);

                AND_THEN("J has expected values")
                {
                    Eigen::MatrixXd J_expected(ny,nx);
                    J_expected << 0.873120494477915,                 0,                 0;
                    CAPTURE_EIGEN(J);
                    CAPTURE_EIGEN(J_expected);
                    CHECK(J.isApprox(J_expected));
                }
            }
        }
    }
}
