#include <doctest/doctest.h>
#include <cstddef>
#include <Eigen/Core>
#include "../../src/Gaussian.hpp"

#ifndef CAPTURE_EIGEN
#define CAPTURE_EIGEN(x) INFO(#x " = \n", x.format(Eigen::IOFormat(Eigen::FullPrecision, 0, ", ", ";\n", "", "", "[", "]")));
#endif

SCENARIO("Gaussian density")
{
    GIVEN("A Gaussian density")
    {
        const std::size_t n = 5;
        Eigen::VectorXd mu(n);
        mu << 1, 2, 3, 4, 5;
        Eigen::MatrixXd S(n,n);
        S <<
            10, 11, 12, 13, 14,
             0, 15, 16, 17, 18,
             0,  0, 19, 20, 21,
             0,  0,  0, 22, 23,
             0,  0,  0,  0, 24;
        REQUIRE_MESSAGE(S.isUpperTriangular(), "S must be upper triangular");
        Eigen::MatrixXd P = S.transpose()*S;

        Gaussian p(mu, S);
        REQUIRE(p.size() == n);

        WHEN("Extracting mean")
        {
            Eigen::VectorXd mean = p.mean();

            THEN("Mean has expected dimensions")
            {
                REQUIRE(mean.rows() == n);
                REQUIRE(mean.cols() == 1);

                AND_THEN("Mean matches expected values")
                {
                    CAPTURE_EIGEN(mean);
                    CAPTURE_EIGEN(mu);
                    CHECK(mean.isApprox(mu));
                }
            }
        }

        WHEN("Extracting cov")
        {
            Eigen::MatrixXd cov = p.cov();

            THEN("Cov has expected dimensions")
            {
                REQUIRE(cov.rows() == n);
                REQUIRE(cov.cols() == n);

                AND_THEN("Cov matches expected values")
                {
                    CAPTURE_EIGEN(cov);
                    CAPTURE_EIGEN(P);
                    CHECK(cov.isApprox(P));
                }
            }
        }

        WHEN("Extracting sqrt cov")
        {
            Eigen::MatrixXd sqrtCov = p.sqrtCov();

            THEN("sqrtCov() has expected dimensions")
            {
                REQUIRE(sqrtCov.rows() == n);
                REQUIRE(sqrtCov.cols() == n);

                AND_THEN("sqrtCov() is upper triangular")
                {
                    CAPTURE_EIGEN(sqrtCov);
                    CHECK(sqrtCov.isUpperTriangular());
                }

                AND_THEN("sqrtCov().'*sqrtCov() == cov()")
                {
                    Eigen::MatrixXd cov = sqrtCov.transpose()*sqrtCov;
                    Eigen::MatrixXd cov_expected = p.cov();
                    CAPTURE_EIGEN(cov);
                    CAPTURE_EIGEN(cov_expected);
                    CHECK(cov.isApprox(cov_expected));
                }
            }
        }
    }
}
