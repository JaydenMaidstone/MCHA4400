#ifndef  BALLISTIC_PLOT_H
#define  BALLISTIC_PLOT_H


#include <Eigen/Core>

void plot_simulation(
    const Eigen::VectorXd & tHist, 
    const Eigen::MatrixXd & xHist, 
    const Eigen::MatrixXd & muHist, 
    const Eigen::MatrixXd & sigmaHist, 
    const Eigen::MatrixXd & hHist, 
    const Eigen::MatrixXd & yHist);
    
#endif 