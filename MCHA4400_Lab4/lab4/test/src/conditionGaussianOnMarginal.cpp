#include <doctest/doctest.h>
#include <Eigen/Core>
#include "../../src/gaussian_util.h"

#ifndef CAPTURE_EIGEN
#define CAPTURE_EIGEN(x) INFO(#x " = \n", x.format(Eigen::IOFormat(Eigen::FullPrecision, 0, ", ", ";\n", "", "", "[", "]")));
#endif

SCENARIO("conditionGaussianOnMarginal")
{
    GIVEN("muyxjoint, Syxjoint and y (ny = 1, nx = 1)")
    {
        const int ny = 1;
        const int nx = 1;

        Eigen::VectorXd muyxjoint(ny + nx);
        muyxjoint << 1,
                     1;

        Eigen::MatrixXd S1(ny, ny), S2(ny, nx), S3(nx, nx);
        S1 <<                  1;
        S2 << -0.649013765191241;
        S3 <<                  1;
        Eigen::MatrixXd Syxjoint(ny + nx, ny + nx);
        Syxjoint << S1,                            S2, 
                    Eigen::MatrixXd::Zero(nx, ny), S3;
        REQUIRE_MESSAGE(Syxjoint.isUpperTriangular(), "Syxjoint must be upper triangular");

        Eigen::VectorXd y(ny);
        y << 0;

        WHEN("conditionGaussianOnMarginal(muyxjoint, Syxjoint, y, muxcond, Sxcond) is called")
        {
            Eigen::VectorXd muxcond;
            Eigen::MatrixXd Sxcond;
            conditionGaussianOnMarginal(muyxjoint, Syxjoint, y, muxcond, Sxcond);

            THEN("muxcond and Sxcond have correct dimensions")
            {
                // TODO
                CHECK(muxcond.size() == nx);
                CHECK(Sxcond.rows() == nx);
                CHECK(Sxcond.cols() == nx);

                AND_THEN("muxcond matches oracle")
                {
                    Eigen::VectorXd muxcond_exp(nx);
                    muxcond_exp << 1.64901376519124;
                    // TODO
                    CHECK(muxcond.isApprox(muxcond_exp));
                }

                AND_THEN("Sxcond is upper triangular")
                {
                    // TODO
                    CAPTURE_EIGEN(Sxcond);
                    CHECK(Sxcond.isUpperTriangular());
                }

                AND_THEN("Sxcond.'*Sxcond matches oracle")
                {
                    Eigen::MatrixXd Pxcond_exp(nx, nx);
                    Pxcond_exp << 1.0;
                    Eigen::MatrixXd Pxcond = Sxcond.transpose()*Sxcond;
                    // TODO
                    CAPTURE_EIGEN(Pxcond);
                    CAPTURE_EIGEN(Pxcond_exp);
                    CHECK(Pxcond.isApprox(Pxcond_exp));
                }
            }
        }
    }

    GIVEN("muyxjoint, Syxjoint and y (ny = 3, nx = 1)")
    {
        const int ny = 3;
        const int nx = 1;

        Eigen::VectorXd muyxjoint(ny + nx);
        muyxjoint << 1,
                     1,
                     1,
                     1;

        Eigen::MatrixXd S1(ny, ny), S2(ny, nx), S3(nx, nx);
        S1 <<   -0.649013765191241,   -1.10961303850152,  -0.558680764473972,
                                 0,  -0.845551240007797,   0.178380225849766,
                                 0,                   0,  -0.196861446475943;
        S2 <<    0.586442621667069,
                -0.851886969622469,
                 0.800320709801823;
        S3 <<    -1.50940472473439;
        Eigen::MatrixXd Syxjoint(ny + nx, ny + nx);
        Syxjoint << S1,                            S2, 
                    Eigen::MatrixXd::Zero(nx, ny), S3;
        REQUIRE_MESSAGE(Syxjoint.isUpperTriangular(), "Syxjoint must be upper triangular");

        Eigen::VectorXd y(ny);
        y << 0.875874147834533,
             -0.24278953633334,
             0.166813439453503;

        WHEN("conditionGaussianOnMarginal(muyxjoint, Syxjoint, y, muxcond, Sxcond) is called")
        {
            Eigen::VectorXd muxcond;
            Eigen::MatrixXd Sxcond;
            conditionGaussianOnMarginal(muyxjoint, Syxjoint, y, muxcond, Sxcond);

            THEN("muxcond and Sxcond have correct dimensions")
            {
                // TODO
                REQUIRE(muxcond.size() == nx);
                REQUIRE(Sxcond.rows() == nx);
                REQUIRE(Sxcond.cols() == nx);

                AND_THEN("muxcond matches oracle")
                {
                    Eigen::VectorXd muxcond_exp(nx);
                    muxcond_exp << 3.91058676524518;
                    // TODO
                    CHECK(muxcond.isApprox(muxcond_exp));
                }


                AND_THEN("Sxcond is upper triangular")
                {
                    // TODO
                    CAPTURE_EIGEN(Sxcond);
                    CHECK(Sxcond.isUpperTriangular());
                }

                AND_THEN("Sxcond.'*Sxcond matches oracle")
                {
                    Eigen::MatrixXd Pxcond_exp(nx, nx);
                    Pxcond_exp << 2.2783026230505;
                    Eigen::MatrixXd Pxcond = Sxcond.transpose()*Sxcond;
                    // TODO
                    CAPTURE_EIGEN(Pxcond);
                    CAPTURE_EIGEN(Pxcond_exp);
                    CHECK(Pxcond.isApprox(Pxcond_exp));
                }
            }
        }
    }

    GIVEN("muyxjoint, Syxjoint and y (ny = 1, nx = 3)")
    {
        const int ny = 1;
        const int nx = 3;

        Eigen::VectorXd muyxjoint(ny + nx);
        muyxjoint << 1,
                     1,
                     1,
                     1;

        Eigen::MatrixXd S1(ny, ny), S2(ny, nx), S3(nx, nx);
        S1 <<   -0.649013765191241;
        S2 <<     1.18116604196553,  -0.758453297283692,   -1.10961303850152;
        S3 <<   -0.845551240007797,   0.178380225849766,  -0.851886969622469,
                                 0,  -0.196861446475943,   0.800320709801823,
                                 0,                   0,   -1.50940472473439;

        Eigen::MatrixXd Syxjoint(ny + nx, ny + nx);
        Syxjoint << S1,                            S2, 
                    Eigen::MatrixXd::Zero(nx, ny), S3;
        REQUIRE_MESSAGE(Syxjoint.isUpperTriangular(), "Syxjoint must be upper triangular");

        Eigen::VectorXd y(ny);
        y << 0.875874147834533;

        WHEN("conditionGaussianOnMarginal(muyxjoint, Syxjoint, y, muxcond, Sxcond) is called")
        {
            Eigen::VectorXd muxcond;
            Eigen::MatrixXd Sxcond;
            conditionGaussianOnMarginal(muyxjoint, Syxjoint, y, muxcond, Sxcond);

            THEN("muxcond and Sxcond have correct dimensions")
            {
                // TODO
                REQUIRE(muxcond.size() == nx);
                REQUIRE(Sxcond.rows() == nx);
                REQUIRE(Sxcond.cols() == nx);

                AND_THEN("muxcond matches oracle")
                {
                    Eigen::VectorXd muxcond_exp(nx);
                    muxcond_exp << 1.22590159002973,
                                   0.854943505203921,
                                   0.787783139025826;
                    // TODO
                    CHECK(muxcond.isApprox(muxcond_exp));
                }

                AND_THEN("Sxcond is upper triangular")
                {
                    // TODO
                    CAPTURE_EIGEN(Sxcond);
                    CHECK(Sxcond.isUpperTriangular());
                }

                AND_THEN("Sxcond.'*Sxcond matches oracle")
                {
                    Eigen::MatrixXd Pxcond_exp(nx, nx);
                    Pxcond_exp <<   0.714956899478723,       -0.150829621160141,        0.720314083510763,
                                   -0.150829621160141,       0.0705739340828141,       -0.309512082615969,
                                    0.720314083510763,       -0.309512082615969,         3.64452727060075;
                    Eigen::MatrixXd Pxcond = Sxcond.transpose()*Sxcond;
                    // TODO
                    CAPTURE_EIGEN(Pxcond);
                    CAPTURE_EIGEN(Pxcond_exp);
                    CHECK(Pxcond.isApprox(Pxcond_exp));
                }
            }
        }
    }

    GIVEN("muyxjoint, Syxjoint and y (ny = 3, nx = 3)")
    {
        const int ny = 3;
        const int nx = 3;

        Eigen::VectorXd muyxjoint(ny + nx);
        muyxjoint << 1,
                     1,
                     1,
                     1,
                     1,
                     1;

        Eigen::MatrixXd S1(ny, ny), S2(ny, nx), S3(nx, nx);
        S1 <<   -0.649013765191241,   -1.10961303850152,  -0.558680764473972,
                                 0,  -0.845551240007797,   0.178380225849766,
                                 0,                   0,  -0.196861446475943;
        S2 <<    0.586442621667069,   -1.50940472473439,   0.166813439453503,
                -0.851886969622469,   0.875874147834533,   -1.96541870928278,
                 0.800320709801823,   -0.24278953633334,   -1.27007139263854;
        S3 <<     1.17517126546302,   0.603658445825815,   -1.86512257453063,
                                 0,     1.7812518932425,   -1.05110705924059,
                                 0,                   0,  -0.417382047996795;

        Eigen::MatrixXd Syxjoint(ny + nx, ny + nx);
        Syxjoint << S1,                            S2, 
                    Eigen::MatrixXd::Zero(nx, ny), S3;
        REQUIRE_MESSAGE(Syxjoint.isUpperTriangular(), "Syxjoint must be upper triangular");

        Eigen::VectorXd y(ny);
        y <<   1.40216228633781,
              -1.36774699097611,
             -0.292534999151873;

        WHEN("conditionGaussianOnMarginal(muyxjoint, Syxjoint, y, muxcond, Sxcond) is called")
        {
            Eigen::VectorXd muxcond;
            Eigen::MatrixXd Sxcond;
            conditionGaussianOnMarginal(muyxjoint, Syxjoint, y, muxcond, Sxcond);

            THEN("muxcond and Sxcond have correct dimensions")
            {
                // TODO
                REQUIRE(muxcond.size() == nx);
                REQUIRE(Sxcond.rows() == nx);
                REQUIRE(Sxcond.cols() == nx);

                AND_THEN("muxcond matches oracle")
                {
                    Eigen::VectorXd muxcond_exp(nx);
                    muxcond_exp <<  6.84085527167069,
                                    2.28421796254053,
                                   -20.9360494953007;
                    // TODO
                    CHECK(muxcond.isApprox(muxcond_exp));
                }

                AND_THEN("Sxcond is upper triangular")
                {
                    // TODO
                    CAPTURE_EIGEN(Sxcond);
                    CHECK(Sxcond.isUpperTriangular());
                }

                AND_THEN("Sxcond.'*Sxcond matches oracle")
                {
                    Eigen::MatrixXd Pxcond_exp(nx, nx);
                    Pxcond_exp <<    1.38102750316996,        0.709402059688563,        -2.19183845615481,
                                    0.709402059688563,         3.53726182639683,        -2.99818343388866,
                                    -2.19183845615481,        -2.99818343388866,         4.75771604199917;
                    Eigen::MatrixXd Pxcond = Sxcond.transpose()*Sxcond;
                    // TODO
                    CAPTURE_EIGEN(Pxcond);
                    CAPTURE_EIGEN(Pxcond_exp);
                    CHECK(Pxcond.isApprox(Pxcond_exp));
                }
            }
        }
    }
}
