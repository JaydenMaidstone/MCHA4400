#include <Eigen/Core>
#include "Event.h"
#include "State.h"
#include "funcmin.hpp"
#include "Measurement.h"

Measurement::Measurement(double time, const Eigen::VectorXd & y)
    : Event(time)
    , useQuasiNewton(true)
    , y_(y)
{}

Measurement::Measurement(double time, const Eigen::VectorXd & y, const Gaussian<double> & noise)
    : Event(time)
    , useQuasiNewton(true)
    , y_(y)
    , noise_(noise)
{}

double Measurement::costJointDensity(const Eigen::VectorXd & x, const State & state)
{
    double logprior = state.density.log(x);
    double loglik = logLikelihood(x);
    return -(logprior + loglik);
}

double Measurement::costJointDensity(const Eigen::VectorXd & x, const State & state, Eigen::VectorXd & g)
{
    Eigen::VectorXd logpriorGrad(x.size());
    double logprior = state.density.log(x, logpriorGrad);

    Eigen::VectorXd loglikGrad(x.size());
    double loglik = logLikelihood(x, loglikGrad);

    g = -(logpriorGrad + loglikGrad);
    return -(logprior + loglik);
}

double Measurement::costJointDensity(const Eigen::VectorXd & x, const State & state, Eigen::VectorXd & g, Eigen::MatrixXd & H)
{
    Eigen::VectorXd logpriorGrad(x.size());
    Eigen::MatrixXd logpriorHess(x.size(),x.size());
    double logprior = state.density.log(x, logpriorGrad, logpriorHess);

    Eigen::VectorXd loglikGrad(x.size());
    Eigen::MatrixXd loglikHess(x.size(),x.size());
    double loglik = logLikelihood(x, loglikGrad, loglikHess);

    g = -(logpriorGrad + loglikGrad);
    H = -(logpriorHess + loglikHess);
    return -(logprior + loglik);
}

#include <Eigen/SVD>

void Measurement::update(State & state)
{
    const Eigen::Index n = state.size();
    Eigen::MatrixXd Q(n,n);
    Eigen::VectorXd v(n);
    Eigen::VectorXd g(n);
    Eigen::VectorXd x = state.density.mean(); // Set initial decision variable to prior mean
    Eigen::MatrixXd & S = state.density.sqrtCov();

    constexpr int verbosity = 1; // 0:none, 1:dots, 2:summary, 3:iter
    if (useQuasiNewton)
    {
        // Generate eigendecomposition of initial Hessian (inverse of prior covariance)
        // via an SVD of S = U*D*V.', i.e., (S.'*S)^{-1} = (V*D*U.'*U*D*V.')^{-1} = V*D^{-2}*V.'
        // This avoids the loss of precision associated with directly computing the eigendecomposition of (S.'*S)^{-1}
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(S, Eigen::ComputeFullV);
        v = svd.singularValues().array().square().inverse();
        Q = svd.matrixV();

        assert(Q.rows() == n);
        assert(Q.cols() == n);
        assert(v.size() == n);

        // Foreshadowing: If we were doing landmark SLAM with a quasi-Newton method,
        //                we can purposely introduce negative eigenvalues for newly
        //                initialised landmarks to force the Hessian and hence initial
        //                covariance to be approximated correctly.

        // Create cost function with prototype V = costFunc(x, g)
        auto costFunc = [&](const Eigen::VectorXd & x, Eigen::VectorXd & g){ return costJointDensity(x, state, g); };

        // Minimise cost
        int ret = funcmin::SR1TrustEig(costFunc, x, g, Q, v, verbosity);
        assert(ret == 0);
    }
    else
    {
        // Create cost function with prototype V = costFunc(x, g, H)
        auto costFunc = [&](const Eigen::VectorXd & x, Eigen::VectorXd & g, Eigen::MatrixXd & H){ return costJointDensity(x, state, g, H); };

        // Minimise cost
        int ret = funcmin::NewtonTrustEig(costFunc, x, g, Q, v, verbosity);
        assert(ret == 0);
    }

    // Set posterior mean to maximum a posteriori (MAP) estimate
    state.density.mean() = x;

    // Post-calculate posterior square-root covariance from Hessian eigendecomposition
    S = v.array().rsqrt().matrix().asDiagonal()*Q.transpose();
    Eigen::HouseholderQR<Eigen::Ref<Eigen::MatrixXd>> qr(S);    // In-place QR decomposition
    S = S.triangularView<Eigen::Upper>();                       // Safe aliasing
}

Eigen::VectorXd Measurement::simulate(const Eigen::VectorXd & x) const
{
    Gaussian p(predict(x), noise_.sqrtCov());
    return p.simulate();
}

double Measurement::logLikelihood(const Eigen::VectorXd & x) const
{
    Eigen::VectorXd h = predict(x);
    Gaussian likelihood(h, noise_.sqrtCov());
    
    // Evaluate log N(y; h(x), R)
    double logLik = likelihood.log(y_);
    return logLik;
}

double Measurement::logLikelihood(const Eigen::VectorXd & x, Eigen::VectorXd & g) const
{
    // i) Analyical derivatives
    Eigen::MatrixXd dhdx;
    Eigen::VectorXd h = predict(x, dhdx);
    Gaussian likelihood(h, noise_.sqrtCov());

    // Evaluate log N(y; h(x), R) and d/dy log N(y; h(x), R)
    Eigen::VectorXd loglikGrad;
    double logLik = likelihood.log(y_, loglikGrad);
    // Note:
    //  d                        d     
    // -- log N(y; h(x), R) = - -- log N(y; h(x), R)
    // dh                       dy

    // Gradient of log likelihood:
    //
    //         d 
    // g_i = ---- log N(y; h(x), R)
    //       dx_i
    //
    //             dh_k     d
    // g_i = sum_k ---- * ---- log N(y; h(x), R)
    //             dx_i   dh_k
    //
    //               dh_k     d
    // g_i = - sum_k ---- * ---- log N(y; h(x), R)
    //               dx_i   dy_k
    //

    // TODO: g = ???;

    g = -dhdx.transpose() * loglikGrad;

    return logLik;
}

double Measurement::logLikelihood(const Eigen::VectorXd & x, Eigen::VectorXd & g, Eigen::MatrixXd & H) const
{
    // i) Analyical derivatives
    Eigen::MatrixXd dhdx;
    Eigen::Tensor<double, 3> d2hdx2;
    Eigen::VectorXd h = predict(x, dhdx, d2hdx2);
    Gaussian likelihood(h, noise_.sqrtCov());

    // Evaluate log N(y; h(x), R), d/dy log N(y; h(x), R) and d^2/dy^2 log N(y; h(x), R)
    Eigen::VectorXd loglikGrad;
    Eigen::MatrixXd logLikHess;
    double logLik = likelihood.log(y_, loglikGrad, logLikHess);
    // Note:
    //  d                        d     
    // -- log N(y; h(x), R) = - -- log N(y; h(x), R)
    // dh                       dy
    //
    //  d^2                      d^2                  
    // ---- log N(y; h(x), R) = ---- log N(y; h(x), R)
    // dh^2                     dy^2                  

    // Gradient of log likelihood:
    //
    //         d 
    // g_i = ---- log N(y; h(x), R)
    //       dx_i
    //
    //             dh_k     d
    // g_i = sum_k ---- * ---- log N(y; h(x), R)
    //             dx_i   dh_k
    //
    //               dh_k     d
    // g_i = - sum_k ---- * ---- log N(y; h(x), R)
    //               dx_i   dy_k
    //

    // TODO: g = ???;
    g = -dhdx.transpose() * loglikGrad;
    // Hessian of log likelihood:
    //
    //              d                                 d  ( dh_k     d                    )
    // H_{ij} = --------- log N(y; h(x), R) = sum_k ---- ( ---- * ---- log N(y; h(x), R) )
    //          dx_i dx_j                           dx_j ( dx_i   dh_k                   )
    //
    //                      dh_k   d^2 log N(y; h(x), R)   dh_k          d^2 h_k      d                   
    // H_{ij} = sum_k sum_l ---- * --------------------- * ---- + sum_k --------- * ---- log N(y; h(x), R) 
    //                      dx_i         dh_k dh_l         dx_j         dx_i dx_j   dh_k                   
    //
    //                      dh_k   d^2 log N(y; h(x), R)   dh_k          d^2 h_k      d                  
    // H_{ij} = sum_k sum_l ---- * --------------------- * ---- - sum_k --------- * ---- log N(y; h(x), R)
    //                      dx_i         dy_k dy_l         dx_j         dx_i dx_j   dy_k   
    //

    // TODO: H = ???;

    //Eigen::MatrixXd myMatrix = d2hdx2.cast<double>();
    //H = -dhdx.transpose() * loglikHess * dhdx;

    // Calculate the multiplication of the transpose of d2hdx2 with loglikGrad
    //Eigen::MatrixXd d2hdx2_transpose_mult_loglikGrad = Eigen::MatrixXd::Zero(nx, nh);

    // for (int i=0; i < nx ; ++i)
    // {
    //     for (int k = 0; k < nh; ++k) {
    //         for (int k)
            
    //     d2hdx2_transpose_mult_loglikGrad += d2hdx2.chip(k, 2) * loglikGrad(k);

    //     }
    // }

    int nh = h.size();
    int nx = x.size();


        // Compute the second term in the Hessian
        Eigen::MatrixXd d2hdx2_transpose_mult_loglikGrad(nx, nx);
        for (int i = 0; i < nx; ++i) 
        {
            for (int j = 0; j < nx; ++j) 
            {
                double sum = 0.0;
                for (int k = 0; k < nh; ++k) 
                {
                    sum += d2hdx2(k, i, j) * loglikGrad(k);
                }
                d2hdx2_transpose_mult_loglikGrad(i, j) = sum;
            }
        }
        
        H = dhdx.transpose() * logLikHess * dhdx;
        H = H - d2hdx2_transpose_mult_loglikGrad;
       
    //H = (dhdx.transpose() * logLikHess * dhdx) - (myMatrix.transpose() * loglikGrad);

    // Compute the Hessian matrix using the given formula
    

    // Hint: In Matlab, this operation would look like the following:
    //       nh = length(h);
    //       nx = length(x);
    //       H = dhdx.'*logLikHess*dhdx - reshape(sum(d2hdx2 .* reshape(loglikGrad, [nh, 1, 1]), 1), [nx, nx]);
    
    return logLik;
}
