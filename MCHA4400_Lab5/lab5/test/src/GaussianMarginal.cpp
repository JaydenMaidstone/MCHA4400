#include <doctest/doctest.h>
#include <cstddef>
#include <vector>
#include <Eigen/Core>
#include "../../src/Gaussian.h"

#ifndef CAPTURE_EIGEN
#define CAPTURE_EIGEN(x) INFO(#x " = \n", x.format(Eigen::IOFormat(Eigen::FullPrecision, 0, ", ", ";\n", "", "", "[", "]")));
#endif

// Helper function for Gaussian marginal density unit tests
template <typename IndexType>
static void testMarginal(const Gaussian & p, const IndexType & idx)
{
    const std::size_t n = idx.size();
    Gaussian m = p.marginal(idx);
    REQUIRE(m.size() == n);

    THEN("Marginal mean matches expected result")
    {
        Eigen::VectorXd mum = m.mean();
        REQUIRE(mum.rows() == n);
        REQUIRE(mum.cols() == 1);
        Eigen::VectorXd mu = p.mean();
        Eigen::VectorXd mum_expected = mu(idx);
        CAPTURE_EIGEN(mum);
        CAPTURE_EIGEN(mum_expected);
        CHECK(mum.isApprox(mum_expected));
    }

    THEN("Marginal sqrt cov is upper triangular")
    {
        Eigen::MatrixXd Sm = m.sqrtCov();
        CAPTURE_EIGEN(Sm);
        REQUIRE(Sm.isUpperTriangular());
    }

    THEN("Marginal cov matches expected result")
    {
        Eigen::MatrixXd Pm = m.cov();
        REQUIRE(Pm.rows() == n);
        REQUIRE(Pm.cols() == n);
        Eigen::MatrixXd P = p.cov();
        Eigen::MatrixXd Pm_expected = P(idx, idx);
        CAPTURE_EIGEN(Pm);
        CAPTURE_EIGEN(Pm_expected);
        CHECK(Pm.isApprox(Pm_expected));
    }
}

SCENARIO("Gaussian marginal density")
{
    GIVEN("A Gaussian density (size = 5)")
    {
        Eigen::VectorXd mu(5);
        mu << 1, 2, 3, 4, 5;
        Eigen::MatrixXd S(5, 5);
        S <<
            10, 11, 12, 13, 14,
             0, 15, 16, 17, 18,
             0,  0, 19, 20, 21,
             0,  0,  0, 22, 23,
             0,  0,  0,  0, 24;
        REQUIRE_MESSAGE(S.isUpperTriangular(), "S must be upper triangular");

        Gaussian p(mu, S);
        REQUIRE(p.size() == 5);

        WHEN("Extracting marginal head (size = 2)")
        {
            std::vector<int> idx = {0, 1};
            testMarginal(p, idx);
        }

        WHEN("Extracting marginal tail (size = 2)")
        {
            std::vector<int> idx = {3, 4};
            testMarginal(p, idx);
        }

        WHEN("Extracting marginal segment (size = 3)")
        {
            std::vector<int> idx = {1, 2, 3};
            testMarginal(p, idx);
        }

        WHEN("Extracting marginal non-continguous elements (size = 3)")
        {
            std::vector<int> idx = {0, 2, 4};
            testMarginal(p, idx);
        }

        WHEN("Extracting marginal non-continguous elements in non-ascending order (size = 3)")
        {
            std::vector<int> idx = {4, 2, 0};
            testMarginal(p, idx);
        }
    }
}
