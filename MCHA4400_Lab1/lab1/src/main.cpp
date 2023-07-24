#include <iostream>
#include <vector>
#include <opencv2/core/mat.hpp>
#include <Eigen/Core>

int main(int argc, char *argv[])
{
    std::cout << "Hello world!" << std::endl;


    // Undefined behaviour using a raw array
    double stack_array[10];
    double temp = stack_array[1000000];         // Maybe a segmentation fault
    // stack_array[1000000] = 1.0;                 // Maybe a segmentation fault

    // Undefined behaviour using a null pointer
    double *p = nullptr;
    // p[10] = 1.0;                                // Maybe a segmentation violation

    // Undefined behaviour using a C++ STL container
    std::vector<double> v(10);
    // v[1000000] = 1.0;                           // Maybe a segmentation fault
    // v.at(1000000) = 1.0;                        // Note that std::vector::at has bounds checking and throws a std::out_of_range exception

    // Compile-time assertion
    // static_assert(1 == 2, "A message that is included if the condition is false at compile time");      // A build error

    // Run-time assertion
    int one = 1;
    int two = one + one;
    // assert(one == two && "A cheeky hack to include a message in a run-time assertion failure");         // Assertion failure in debug build

    // Undefined behaviour using an OpenCV matrix
    cv::Mat M(10, 10, CV_64FC1);                // A 10x10 matrix of double-precision (64-bit) floating point numbers
    // M.at<double>(1000, 1000) = 1.0;             // Maybe a segmentation fault

    // Undefined behaviour using an eigen3 matrix
    Eigen::MatrixXd A(10, 10);                  // A 10x10 matrix of double-precision (64-bit) floating point numbers
    // A(1000, 1000) = 1.0;                        // Assertion failure in debug build, maybe a segmentation fault in release build

    return 0;
}
