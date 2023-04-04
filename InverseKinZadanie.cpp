#include "InverseKinematics_2DOF.h"
#include <iostream>
#include <fstream>
#include <cmath>
#include <chrono>
#include <thread>
#include <iomanip>
#include <mutex>

using namespace std;
mutex mtx;
mutex m_thread;
const long long n_points = 50000000;
double* gtheta1 = new double[n_points];
double* gtheta2 = new double[n_points];

void calculateJointAngles(long long start, long long end, double dx, double P0, double a, double b, double c, double* theta1, double* theta2) {
    double x, y;
    for (long long i = start; i < end; i++) {
        x = P0 + i * dx;
        y = a * x * x + b * x + c;
        double t1, t2;
        inverseKinematics(x, y, t1, t2);
        // Store the joint angles in separate array
        theta1[i-start] = t1;
        theta2[i-start] = t2;
    }
    //mtx.lock();
    for (long long i = start; i < end; i++) {
        gtheta1[i] += theta1[i-start];
        gtheta2[i] += theta2[i-start];
    }
    //mtx.unlock();
}

int main() {

    //Choose the way you want to solve the issue
    int choice;
    cout << "Choose option:" << endl;
    cout << "1. Sequential" << endl;
    cout << "2. Thread" << endl;
    cout << "Enter choice: ";
    cin >> choice;

    // Define the start and end points of the parabolic trajectory
    double P0[2] = {400, 0};
    double P2[2] = {-400, 0};

    // Define the coefficients of the quadratic equation
    const double a = -0.0025;
    const double b = 0.0;
    const double c = 400.0;
    const int sd = 1;
    // Generate n_points points on the parabolic trajectory

    double x, y;
    const double dx = (P2[0] - P0[0]) / (n_points - 1);

    //Define number of threads and store them in an array
    const int n_threads = 50;
    thread threads[n_threads];
    if (choice == 1) {
        // Calculate joint angles for each point along the trajectory

        auto begin = chrono::high_resolution_clock::now();
        for (long long i = 0; i < n_points; i++) {
            x = P0[0] + i * dx;
            y = a * x * x + b * x + c;
            double theta1, theta2;
            inverseKinematics(x, y, theta1, theta2);
            // Do something with the joint angles, such as save them to a file
            gtheta1[i] += theta1;
            gtheta2[i] += theta2;

        }
        auto end = chrono::high_resolution_clock::now();
        auto elapsed = chrono::duration_cast<chrono::nanoseconds>(end - begin);
        double milliseconds = elapsed.count() * 1e-6;
        double seconds = elapsed.count() / 1e9;
        cout << "\nCzas realizacji algorytmu SEKWENCYJNEGO wynosi: " << fixed << setprecision(0) << milliseconds
             << " [ms].\n";
        cout << "\nCzas realizacji algorytmu SEKWENCYJNEGO wynosi: " << fixed << setprecision(6) << seconds
             << " [s].\n";
//        remove("joint_anglesThreadSeq.txt");
//        ofstream outfile("joint_anglesThreadSeq.txt", ios::app);
//        for (long long i = 0; i < n_points; i++) {
//            outfile << gtheta1[i] << " " << gtheta2[i] << endl;
//        }
//        outfile.close();
        delete[] gtheta2;
        delete[] gtheta1;

    } else if (choice == 2) {

        auto begin = chrono::high_resolution_clock::now();
        for (int i = 0; i < n_threads; i++) {
            const long long start = i * n_points / n_threads;
            const long long end = (i + 1) * n_points / n_threads;
            double* local_theta1 = new double[(end-start)];
            double* local_theta2 = new double[(end-start)];
            threads[i] = thread(calculateJointAngles, start, end, dx, P0[0], a, b, c,local_theta1,local_theta2);
        }

        for (int i = 0; i < n_threads; i++) {
            threads[i].join();
        }
        auto end = chrono::high_resolution_clock::now();
        auto elapsed = chrono::duration_cast<chrono::nanoseconds>(end - begin);
        double milliseconds = elapsed.count() * 1e-6;
        double seconds = elapsed.count() / 1e9;
        cout << "\nCzas realizacji algorytmu SEKWENCYJNEGO wynosi: " << fixed << setprecision(0) << milliseconds
             << " [ms].\n";
        cout << "\nCzas realizacji algorytmu SEKWENCYJNEGO wynosi: " << fixed << setprecision(6) << seconds
             << " [s].\n";
//        remove("joint_anglesThreadNoRace.txt");
//        ofstream outfile("joint_anglesThreadNoRace.txt", ios::app);
//        for (long long i = 0; i < n_points; i++) {
//            outfile << gtheta1[i] << " " << gtheta2[i] << endl;
//        }
//        outfile.close();
        delete[] gtheta2;
        delete[] gtheta1;
    }


    return 0;
}
