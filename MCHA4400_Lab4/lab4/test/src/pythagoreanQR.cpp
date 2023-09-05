#include <doctest/doctest.h>
#include <Eigen/Core>
#include "../../src/gaussian_util.h"
#define _USE_MATH_DEFINES
//#include <math.h>
#include <cmath>
#include <iostream>

#ifndef CAPTURE_EIGEN
#define CAPTURE_EIGEN(x) INFO(#x " = \n", x.format(Eigen::IOFormat(Eigen::FullPrecision, 0, ", ", ";\n", "", "", "[", "]")));
#endif

SCENARIO("PythagoreanQR")
{
    GIVEN("S1 = I and S2 = 0")
    {
        //Eigen::MatrixXd S1, S2;
        // TODO
        Eigen::MatrixXd S1 = Eigen::MatrixXd::Identity(3, 3);
        Eigen::MatrixXd S2 = Eigen::MatrixXd::Zero(3, 3);
        REQUIRE(S1.cols() == S2.cols());

        WHEN("pythagoreanQR(S1, S2, S) is called")
        {
            Eigen::MatrixXd S;
            pythagoreanQR(S1, S2, S);

            THEN("S has expected dimensions")
            {
                // TODO
                REQUIRE(S.rows() == (S1.rows() + S2.rows()));
                REQUIRE(S.cols() == S1.cols());

                double pi = std::acos(-1.0);
                double approx = 22/7;
                CHECK(Eigen::internal::isApprox(pi, approx, 1e-1));

                AND_THEN("S is upper triangular")
                {
                    // TODO
                    CAPTURE_EIGEN(S); // Capture S to be printed on test failure
                    CHECK(S.isUpperTriangular());
                }

                AND_THEN("S satisfies S.'*S = S1.'*S1 + S2.'*S2")
                {
                    Eigen::MatrixXd LHS = S.transpose()*S;
                    Eigen::MatrixXd RHS = S1.transpose()*S1 + S2.transpose()*S2;
                    // TODO
                    CAPTURE_EIGEN(LHS); // Capture M to be printed on test failure
                    CAPTURE_EIGEN(RHS); // Capture M_exp to be printed on test failure
                    CHECK(LHS.isApprox(RHS));
                }
            }
        }
    }

    GIVEN("S1 = 0 and S2 = I")
    {
        Eigen::MatrixXd S1, S2;
        // TODO
        S1 = Eigen::MatrixXd::Zero(3, 3);
        S2 = Eigen::MatrixXd::Identity(3, 3);
        REQUIRE(S1.cols() == S2.cols());

        WHEN("pythagoreanQR(S1, S2, S) is called")
        {
            Eigen::MatrixXd S;
            pythagoreanQR(S1, S2, S);
            // pythagoreanQR(S1, S2, S);


            THEN("S has expected dimensions")
            {
                // TODO
                REQUIRE(S.rows() == (S1.rows() + S2.rows()));
                REQUIRE(S.cols() == S1.cols());
                REQUIRE(1 == 1);

                AND_THEN("S is upper triangular")
                {
                    // TODO
                    CAPTURE_EIGEN(S); // Capture S to be printed on test failure
                    CHECK(S.isUpperTriangular());
                }

                AND_THEN("S satisfies S.'*S = S1.'*S1 + S2.'*S2")
                {
                    Eigen::MatrixXd LHS = S.transpose()*S;
                    Eigen::MatrixXd RHS = S1.transpose()*S1 + S2.transpose()*S2;
                    // TODO
                    CAPTURE_EIGEN(LHS); // Capture M to be printed on test failure
                    CAPTURE_EIGEN(RHS); // Capture M_exp to be printed on test failure
                    CHECK(LHS.isApprox(RHS));
                }
            }
        }
    }

    GIVEN("S1 = 3I and S2 = 4I")
    {
        //Eigen::MatrixXd S1, S2;
        // TODO
        Eigen::MatrixXd S1 = Eigen::MatrixXd::Identity(3, 3);
        S1 *= 3;
        Eigen::MatrixXd S2 = Eigen::MatrixXd::Zero(3, 3);
        S2 *= 4;
        REQUIRE(S1.cols() == S2.cols());
        

        WHEN("pythagoreanQR(S1, S2, S) is called")
        {
            Eigen::MatrixXd S;
            pythagoreanQR(S1, S2, S);
            // pythagoreanQR(S1, S2, S);


            THEN("S has expected dimensions")
            {
                // TODO
                REQUIRE(S.rows() == (S1.rows() + S2.rows()));
                REQUIRE(S.cols() == S1.cols());
                REQUIRE(1 == 1);

                AND_THEN("S is upper triangular")
                {
                    // TODO
                    CAPTURE_EIGEN(S); // Capture S to be printed on test failure
                    CHECK(S.isUpperTriangular());
                }

                AND_THEN("S satisfies S.'*S = S1.'*S1 + S2.'*S2")
                {
                    Eigen::MatrixXd LHS = S.transpose()*S;
                    Eigen::MatrixXd RHS = S1.transpose()*S1 + S2.transpose()*S2;
                    // TODO
                    CAPTURE_EIGEN(LHS); // Capture M to be printed on test failure
                    CAPTURE_EIGEN(RHS); // Capture M_exp to be printed on test failure
                    CHECK(LHS.isApprox(RHS));
                }
            }
        }
    }

    GIVEN("S1 and S2 with different numbers of rows")
    {
        Eigen::MatrixXd S1(4,4), S2(1,4);
        // TODO
        S1 <<   1, 2, 3, 4,
                0, 5, 6, 7,
                0, 0, 8, 9,
                0, 0, 0, 16;

        S2 <<   0, 10, -1, -3;
        REQUIRE(S1.cols() == S2.cols());
        

        WHEN("pythagoreanQR(S1, S2, S) is called")
        {
            Eigen::MatrixXd S;
            pythagoreanQR(S1, S2, S);
            // pythagoreanQR(S1, S2, S);


            THEN("S has expected dimensions")
            {
                // TODO
                REQUIRE(S.rows() == (S1.rows() + S2.rows()));
                REQUIRE(S.cols() == S1.cols());
                REQUIRE(1 == 1);

                AND_THEN("S is upper triangular")
                {
                    // TODO
                    CAPTURE_EIGEN(S); // Capture S to be printed on test failure
                    CHECK(S.isUpperTriangular());
                }

                AND_THEN("S satisfies S.'*S = S1.'*S1 + S2.'*S2")
                {
                    Eigen::MatrixXd LHS = S.transpose()*S;
                    Eigen::MatrixXd RHS = S1.transpose()*S1 + S2.transpose()*S2;
                    // TODO
                    CAPTURE_EIGEN(LHS); // Capture M to be printed on test failure
                    CAPTURE_EIGEN(RHS); // Capture M_exp to be printed on test failure
                    CHECK(LHS.isApprox(RHS));
                }
            }
        }
    }
}
