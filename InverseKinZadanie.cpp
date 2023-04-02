//#include "InverseKinematics_2DOF.h"
//#include <iostream>
//#include <fstream>
//#include <vector>
//#include <cmath>
//#include <chrono>
//using namespace std;
//
//int main()
//{
//    double P0[2] = {400, 0};
//    double P2[2] = {-400, 0};
//
//    // Define the coefficients of the quadratic equation
//    const double a = -0.0025;
//    const double b = 0.0;
//    const double c = 400.0;
//
//    // Generate n_points points on the parabolic trajectory
//    const long n_points = 2000000001;
//    vector<double> x(n_points), y(n_points);
//    const double dx = (P2[0] - P0[0]) / (n_points - 1);
//    for (int i = 0; i < n_points; i++) {
//        x[i] = P0[0] + i * dx;
//        y[i] = a * x[i] * x[i] + b * x[i] + c;
//    }
//
//    // Calculate joint angles for each point along the trajectory
//    auto begin = chrono::high_resolution_clock::now();
//    vector<pair<double, double>> joint_angles;
//    for (int i = 0; i < n_points; i++) {
//
//        double theta1, theta2;
//
//        inverseKinematics(x[i], y[i], theta1, theta2);
//        joint_angles.emplace_back(theta1, theta2);
//    }
//
//    // Save joint angles to a file for plotting in Python
////    ofstream outfile("joint_angles.txt");
////    for (const auto& angles : joint_angles) {
////        outfile << angles.first << " " << angles.second << endl;
////    }
////    outfile.close();
//
//    auto end = chrono::high_resolution_clock::now();
//    auto elapsed = chrono::duration_cast<chrono::nanoseconds>(end - begin);
//    printf("\nCzas realizacji algorytmu SEKWENCYJNEGO wynosi: %.0f [ms].\n", elapsed.count() * 1e-6);
//    double seconds = elapsed.count() / 1e9;
//    printf("\nCzas realizacji algorytmu SEKWENCYJNEGO wynosi: %.6f [s].\n", seconds);
//
//
//    return 0;
//}
#include "InverseKinematics_2DOF.h"
#include <iostream>
#include <fstream>
#include <cmath>
#include <chrono>
using namespace std;

int main()
{
    double P0[2] = {400, 0};
    double P2[2] = {-400, 0};

    // Define the coefficients of the quadratic equation
    const double a = -0.0025;
    const double b = 0.0;
    const double c = 400.0;

    // Generate n_points points on the parabolic trajectory
    const long long n_points = 20000000001;
    double x=0;
    double y=0;
    const double dx = (P2[0] - P0[0]) / (n_points - 1);

    // Calculate joint angles for each point along the trajectory
    auto begin = chrono::high_resolution_clock::now();
  //  double** joint_angles = new double*[n_points];
    for (long long i = 0; i < n_points; i++) {
    //    joint_angles[i] = new double[2];

        double theta1, theta2;
        x = P0[0] + i * dx;
        y = a * x * x + b * x + c;
        inverseKinematics(x, y, theta1, theta2);
        //joint_angles[i][0] = theta1;
        //joint_angles[i][1] = theta2;
    }

    // Save joint angles to a file for plotting in Python
//    ofstream outfile("joint_angles.txt");
//    for (int i = 0; i < n_points; i++) {
//        outfile << joint_angles[i][0] << " " << joint_angles[i][1] << endl;
//    }
//    outfile.close();

    auto end = chrono::high_resolution_clock::now();
    auto elapsed = chrono::duration_cast<chrono::nanoseconds>(end - begin);
    printf("\nCzas realizacji algorytmu SEKWENCYJNEGO wynosi: %.0f [ms].\n", elapsed.count() * 1e-6);
    double seconds = elapsed.count() / 1e9;
    printf("\nCzas realizacji algorytmu SEKWENCYJNEGO wynosi: %.6f [s].\n", seconds);

    // Free dynamically allocated memory
//    for (int i = 0; i < n_points; i++) {
//        delete[] joint_angles[i];
//    }
//    delete[] joint_angles;

    return 0;
}
