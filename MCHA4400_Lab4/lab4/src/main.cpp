#include <cassert>
#include <cstdlib>
#include <iostream>
#include <Eigen/Core>

int main(int argc, char *argv[])
{
    std::cout << "Eigen version: ";
    // TODO
    std::cout << "\n" << std::endl;

    std::cout << "Create a column vector:" << std::endl;
    Eigen::VectorXd x(3);
    // TODO
    x << 1, 3.2, 0.01;
    std::cout << "x = \n" << x << "\n" << std::endl;

    std::cout << "Create a matrix:" << std::endl;
    //Eigen::MatrixXd A(4,3);
    // TODO: Don't just use a for loop or hardcode all the elements
    //       Try and be creative :)
     Eigen::MatrixXd A = Eigen::MatrixXd::NullaryExpr(4, 3, [](int i, int j) {
        return (i + 1) * (j + 1);
    });

    std::cout << "A.size() = " << A.size() << std::endl;
    std::cout << "A.rows() = " << A.rows() << std::endl;
    std::cout << "A.cols() = " << A.cols() << std::endl;
    std::cout << "A = \n" << A << "\n" << std::endl;
    std::cout << "A.transpose() = \n" << A.transpose() << "\n" << std::endl;

    std::cout << "Matrix multiplication:" << std::endl;
    Eigen::VectorXd Ax = A * x;
    // TODO
    std::cout << "A*x = \n" << Ax << "\n" << std::endl;

    std::cout << "Matrix concatenation:" << std::endl;
    Eigen::MatrixXd B(4, 6);
    B << A, A;
    // TODO
    std::cout << "B = \n" << B << "\n" << std::endl;
    Eigen::MatrixXd C(8, 3);
    C << A, A;
    // TODO
    std::cout << "C = \n" << C << "\n" << std::endl;

    std::cout << "Submatrix via block:" << std::endl;
    Eigen::MatrixXd D = B.block(1, 2, 1, 3);
    // TODO
    std::cout << "D = \n" << D << "\n" << std::endl;
    std::cout << "Submatrix via slicing:" << std::endl;
    // TODO
    D = B.row(1).segment(2, 3);
    std::cout << "D = \n" << D << "\n" << std::endl;

    std::cout << "Broadcasting:" << std::endl;
    Eigen::VectorXd v(6);
    v << 1, 3, 5, 7, 4, 6;
    Eigen::MatrixXd E = B.rowwise() + v.transpose();
    // TODO
    std::cout << "E = \n" << E << "\n" << std::endl;

    std::cout << "Index subscripting:" << std::endl;

    Eigen::VectorXi r(4);
    r << 1, 3, 2, 4;
    Eigen::VectorXi c(6);
    c << 1, 4, 2, 5, 3, 6;

    Eigen::MatrixXd F = B(r.array() - 1, c.array() - 1);
    
    // Eigen::MatrixXd F(r.size(), c.size());
    // for (int i = 0; i < r.size(); ++i) {
    //     for (int j = 0; j < c.size(); ++j) {
    //         F(i, j) = B(r[i] - 1, c[j] - 1);
    //     }
    // }
    // TODO
    std::cout << "F = \n" << F << "\n" << std::endl;

    std::cout << "Memory mapping:" << std::endl;
    float array[9] = {1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f};
    //Eigen::Matrix3f G;              // TODO: Replace this with an Eigen::Map
    Eigen::Map<Eigen::Matrix<float, 3, 3, Eigen::RowMajor>> G(array);
    array[2] = -3.0f;               // Change an element in the raw storage
    assert(array[2] == G(0,2));     // Ensure the change is reflected in the view
    G(2,0) = -7.0f;                 // Change an element via the view
    assert(G(2,0) == array[6]);     // Ensure the change is reflected in the raw storage
    std::cout << "G = \n" << G << "\n" << std::endl;

    return EXIT_SUCCESS;
}
