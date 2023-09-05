#include <cstdlib>
#include <fstream>
#include <sstream>
#include <iostream>
#include <random>
#include <Eigen/Core>
#include "Gaussian.hpp"
#include "StateBallistic.h"
#include "MeasurementRADAR.h"
#include "ballistic_plot.h"
#include "rosenbrock.hpp"
#include "funcmin.hpp"

int main(int argc, char *argv[])
{
    // ------------------------------------------------------
    // PROBLEM 1: Optimisation
    // ------------------------------------------------------
    
    Eigen::VectorXd x(2);
    x << 10.0, 10.0;
    std::cout << "Initial x =\n" << x << "\n" << std::endl;

    RosenbrockAnalytical func;
    // TODO: Call funcmin::NewtonTrust
    // Call the Newton trust region method
    //int retval = funcmin::NewtonTrust(func, x);

    int verbosity = 3;  // Set the verbosity level

    Eigen::VectorXd g;
    Eigen::MatrixXd H;
    double f;

    double x1 = x[0];
    double x2 = x[1];
    
    f = (1 - x[0])*(1 - x[0]) + 100*(x[1] - x[0]*x[0])*(x[1] - x[0]*x[0]);

    g.resize(2, 1);
    g[0] = -2 * (1 - x1) - 400 * x1 * (x2 - x1 * x1);
    g[1] = 200 * (x2 - x1 * x1);

    H.resize(2, 2);
    H(0, 0) = 800*x1*x1-400*(x2-x1*x1) + 2;
    H(0, 1) = -400 * x1;
    H(1, 0) = -400 * x1;
    H(1, 1) = 200;

    std::cout << "f =\n" << f << "\n" << std::endl;
    std::cout << "g =\n" << g << "\n" << std::endl;
    std::cout << "H =\n" << H << "\n" << std::endl;

    //int retval = funcmin::SR1Trust(func, x, g, H, verbosity);
    int retval = funcmin::NewtonTrust(func, x, g, H, verbosity);

    // TODO: Display results
     if (retval == 0)
    {
        std::cout << "Newton trust region method converged to a minimum." << std::endl;
        std::cout << "Optimal x =\n" << x << "\n" << std::endl;
        f = (1 - x[0])*(1 - x[0]) + 100*(x[1] - x[0]*x[0])*(x[1] - x[0]*x[0]);
        std::cout << "f =\n" << f << "\n" << std::endl;
        std::cout << "g =\n" << g << "\n" << std::endl;
        std::cout << "H =\n" << H << "\n" << std::endl;
    }
    else if (retval == 1)
    {
        std::cout << "WARNING: Newton trust region method reached the maximum number of iterations." << std::endl;
        std::cout << "Current x =\n" << x << "\n" << std::endl;
    }
    else
    {
        std::cout << "ERROR: Newton trust region method encountered an issue." << std::endl;
    }

    // Comment out the following line to move on to Problem 3
    //return EXIT_SUCCESS;

    // ------------------------------------------------------
    // PROBLEM 3: Iterated EKF
    // ------------------------------------------------------

    std::string fileName   = "../data/estimationdata.csv";
    
    // Dimensions of state and measurement vectors for recording results
    const std::size_t nx = 3;
    const std::size_t ny = 1;

    Eigen::VectorXd x0(nx);
    Eigen::VectorXd u;
    Eigen::VectorXd tHist;
    Eigen::MatrixXd xHist, hHist, yHist;

    // Read from CSV
    std::fstream input;
    input.open(fileName, std::fstream::in);
    if (!input.is_open())
    {
        std::cout << "Could not open input file \"" << fileName << "\"! Exiting" << std::endl;
        return EXIT_FAILURE;
    }
    std::cout << "Reading data from " << fileName << std::endl;

    // Determine number of time steps
    std::size_t rows = 0;
    std::string line;
    while (std::getline(input, line))
    {
        rows++;
    }
    std::cout << "Found " << rows << " rows within " << fileName << std::endl << std::endl;
    std::size_t nsteps = rows - 1;  // Disregard header row

    tHist.resize(nsteps);
    xHist.resize(nx, nsteps);
    hHist.resize(ny, nsteps);
    yHist.resize(ny, nsteps);

    // Read each row of data
    rows = 0;
    input.clear();
    input.seekg(0);
    std::vector<std::string> row;
    std::string csvElement;
    while (std::getline(input, line))
    {
        if (rows > 0)
        {
            std::size_t i = rows - 1;
            
            row.clear();

            std::stringstream s(line);
            while (std::getline(s, csvElement, ','))
            {
                row.push_back(csvElement);
            }
            
            tHist(i) = stof(row[0]);
            xHist(0, i) = stof(row[1]);
            xHist(1, i) = stof(row[2]);
            xHist(2, i) = stof(row[3]);
            hHist(0, i) = stof(row[4]);
            yHist(0, i) = stof(row[5]);
        }
        rows++;
    }

    Eigen::MatrixXd muHist(nx, nsteps);
    Eigen::MatrixXd sigmaHist(nx, nsteps);

    // Initial state estimate
    Eigen::MatrixXd S0(nx, nx);
    Eigen::VectorXd mu0(nx);
    S0.fill(0);
    S0.diagonal() << 2200, 100, 1e-3;

    mu0 << 14000, // Initial height
            -450, // Initial velocity
          0.0005; // Ballistic coefficient

    StateBallistic state(Gaussian(mu0, S0));

    std::cout << "Initial state estimate" << std::endl;
    std::cout << "mu[0] = \n" << mu0 << std::endl;
    std::cout << "P[0] = \n" << S0.transpose()*S0 << std::endl;

    std::cout << "Run filter with " << nsteps << " steps. " << std::endl;
    for (std::size_t k = 0; k < nsteps; ++k)
    {
        // Create RADAR measurement
        double t = tHist(k);
        Eigen::VectorXd y = yHist.col(k);
        MeasurementRADAR measurementRADAR(t, y);

        // Process measurement event (do time update and measurement update)
        std::cout << "[k=" << k << "] Processing event:";
        measurementRADAR.process(state);
        std::cout << "done" << std::endl;

        // Save results for plotting
        muHist.col(k)       = state.density.mean();
        sigmaHist.col(k)    = state.density.cov().diagonal().cwiseSqrt();
    }

    std::cout << std::endl;
    std::cout << "Final state estimate" << std::endl;
    std::cout << "mu[end] = \n" << state.density.mean() << std::endl;
    std::cout << "P[end] = \n" << state.density.cov() << std::endl;

    // Plot results
    plot_simulation(tHist, xHist, muHist, sigmaHist, hHist, yHist);

    return EXIT_SUCCESS;
}
