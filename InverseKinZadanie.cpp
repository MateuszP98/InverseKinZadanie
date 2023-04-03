#include "InverseKinematics_2DOF.h"
#include <iostream>
#include <fstream>
#include <cmath>
#include <chrono>

using namespace std;

int main() {
    // Define the start and end points of the parabolic trajectory
    double P0[2] = {400, 0};
    double P2[2] = {-400, 0};

    // Define the coefficients of the quadratic equation
    const double a = -0.0025;
    const double b = 0.0;
    const double c = 400.0;

    // Generate n_points points on the parabolic trajectory
    const long long n_points = 6000000001;
    double x, y;
    const double dx = (P2[0] - P0[0]) / (n_points - 1);

    // Calculate joint angles for each point along the trajectory
    auto begin = chrono::high_resolution_clock::now();
    for (long long i = 0; i < n_points; i++) {
        x = P0[0] + i * dx;
        y = a * x * x + b * x + c;
        double theta1, theta2;
        inverseKinematics(x, y, theta1, theta2);
        // Do something with the joint angles, such as save them to a file
    }

    auto end = chrono::high_resolution_clock::now();
    auto elapsed = chrono::duration_cast<chrono::nanoseconds>(end - begin);
    double milliseconds = elapsed.count() * 1e-6;
    double seconds = elapsed.count() / 1e9;
    printf("\nCzas realizacji algorytmu SEKWENCYJNEGO wynosi: %.0f [ms].\n", milliseconds);
    printf("\nCzas realizacji algorytmu SEKWENCYJNEGO wynosi: %.6f [s].\n", seconds);

    return 0;
}
