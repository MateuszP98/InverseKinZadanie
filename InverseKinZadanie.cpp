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

//Define global variables
const long long n_points = 500;
double* gtheta1 = (double*) malloc(n_points * sizeof(double));
double* gtheta2 = (double*) malloc(n_points * sizeof(double));

//Configuration variables
bool WRITE_FILE = TRUE;

void write_joint_angles(const std::string &filename, double *theta1, double *theta2, long long n_points) {
    // Remove the file if it exists
    remove(filename.c_str());

    // Open the file for writing
    std::ofstream outfile(filename, std::ios::app);

    // Write the joint angles to the file
    for (long long i = 0; i < n_points; i++) {
        outfile << theta1[i] << " " << theta2[i] << std::endl;
    }

    // Close the file
    outfile.close();
}

void
calculateJointAngles(long long start, long long end, double dx, double P0, double a, double b, double c, double *theta1,
                     double *theta2) {
    double x, y;
    for (long long i = start; i < end; i++) {
        x = P0 + i * dx;
        y = a * x * x + b * x + c;
        double t1, t2;
        inverseKinematics(x, y, t1, t2);
        // Store the joint angles in separate array
        theta1[i - start] = t1;
        theta2[i - start] = t2;
    }
    //mtx.lock();
    for (long long i = start; i < end; i++) {
        gtheta1[i] += theta1[i - start];
        gtheta2[i] += theta2[i - start];
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
    const double P0[2] = {400, 0};
    const double P2[2] = {-400, 0};

    // Define the coefficients of the quadratic equation
    const double a = -0.0025;
    const double b = 0.0;
    const double c = 400.0;

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

        // calculate the execution time

        auto end = chrono::high_resolution_clock::now();
        auto elapsed = chrono::duration_cast<chrono::nanoseconds>(end - begin);
        double milliseconds = elapsed.count() * 1e-6;
        double seconds = elapsed.count() / 1e9;

        cout << "\nCzas realizacji algorytmu SEKWENCYJNEGO wynosi: " << fixed << setprecision(0) << milliseconds
             << " [ms].\n";

        cout << "\nCzas realizacji algorytmu SEKWENCYJNEGO wynosi: " << fixed << setprecision(6) << seconds
             << " [s].\n";

        //write the values to a file
        if (WRITE_FILE) {
            write_joint_angles("joint_anglesThreadSeq.txt", gtheta1, gtheta2, n_points);
        }
        //free the gtheta arrays
        free(gtheta1);
        free(gtheta2);

    } else if (choice == 2) {

        //Start timer
        auto begin = chrono::high_resolution_clock::now();

        // Loop through the threads
        for (int i = 0; i < n_threads; i++) {
            const long long start = i * n_points / n_threads;
            const long long end = (i + 1) * n_points / n_threads;
            double* local_theta1 = (double*) malloc((end - start) * sizeof(double));
            double* local_theta2 = (double*) malloc((end - start) * sizeof(double));
            threads[i] = thread(calculateJointAngles, start, end, dx, P0[0], a, b, c, local_theta1, local_theta2);
        }

        // Wait for threads to finish
        for (int i = 0; i < n_threads; i++) {
            threads[i].join();
        }

        // calculate the execution time
        auto end = chrono::high_resolution_clock::now();
        auto elapsed = chrono::duration_cast<chrono::nanoseconds>(end - begin);
        double milliseconds = elapsed.count() * 1e-6;
        double seconds = elapsed.count() / 1e9;

        cout << "\nCzas realizacji algorytmu SEKWENCYJNEGO wynosi: " << fixed << setprecision(0) << milliseconds

             << " [ms].\n";
        cout << "\nCzas realizacji algorytmu SEKWENCYJNEGO wynosi: " << fixed << setprecision(6) << seconds
             << " [s].\n";
        if (WRITE_FILE) {
            //write the values to a file
            write_joint_angles("joint_anglesThreadNoRace.txt", gtheta1, gtheta2, n_points);
        }
        //free the gtheta arrays
        free(gtheta1);
        free(gtheta2);
    }


    return 0;
}
