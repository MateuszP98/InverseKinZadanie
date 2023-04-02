#include "InverseKinematics_2DOF.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <chrono>
using namespace std;

int main()
{
    double start_x = 0, start_y = 400.0;
    double end_x = 400, end_y = 0;
    long long num_points = 10000000000;

    double step_x = (end_x - start_x) / num_points;
    double step_y = (end_y - start_y) / num_points;

    // Calculate joint angles for each point along the trajectory
    auto begin = chrono::high_resolution_clock::now();
    vector<pair<double, double>> joint_angles;
    for (int i = 0; i < num_points; i++) {


        double x = start_x + i * step_x;
        double y = start_y + i * step_y;
        double theta1, theta2;

        inverseKinematics(x, y, theta1, theta2);
        joint_angles.emplace_back(theta1, theta2);
    }
    auto end = chrono::high_resolution_clock::now();
    auto elapsed = chrono::duration_cast<chrono::nanoseconds>(end - begin);
    printf("\nCzas realizacji algorytmu SEKWENCYJNEGO wynosi: %.0f [ms].\n", elapsed.count() * 1e-6);
    double seconds = elapsed.count() / 1e9;
    printf("\nCzas realizacji algorytmu SEKWENCYJNEGO wynosi: %.6f [s].\n", seconds);

    // Save joint angles to a file for plotting in Python
//    ofstream outfile("joint_angles.txt");
//    for (const auto& angles : joint_angles) {
//        outfile << angles.first << " " << angles.second << endl;
//    }
//    outfile.close();



    return 0;
}
