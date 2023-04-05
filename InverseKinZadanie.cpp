#include "InverseKinematics_2DOF.h"
#include "AdditonalFunctions.h"
#include <iostream>
#include <fstream>
#include <cmath>
#include <chrono>
#include <thread>
#include <iomanip>
#include <mutex>
#include <omp.h>

using namespace std;
mutex mtx;

//Define global variables
const long long NUMBER_OF_POINTS = 5000;
const int NUMBER_OF_THREADS = 50;
double *theta1Combined = (double *) malloc(NUMBER_OF_POINTS * sizeof(double));
double *theta2Combined = (double *) malloc(NUMBER_OF_POINTS * sizeof(double));

//Configuration variables
bool WRITE_FILE = TRUE;

// additional functions defining
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
    mtx.lock();
    for (long long i = start; i < end; i++) {
        theta1Combined[i] += theta1[i - start];
        theta2Combined[i] += theta2[i - start];
    }
    mtx.unlock();
    free(theta1);
    free(theta2);

}

int main() {

    //Choose the way you want to solve the issue
    int choice;
    cout << "Choose option:" << endl;
    cout << "1. Sequential" << endl;
    cout << "2. Thread" << endl;
    cout << "3. OpenMP" << endl;
    cout << "Enter choice: ";
    cin >> choice;

    // Define the start and end points of the parabolic trajectory
    const double P0[2] = {400, 0};
    const double P2[2] = {-400, 0};

    // Define the coefficients of the quadratic equation
    const double Coeff_A = -0.0025;
    const double Coeff_B = 0.0;
    const double Coeef_B = 400.0;

    // Generate NUMBER_OF_POINTS points on the parabolic trajectory

    double x, y;
    const double dx = (P2[0] - P0[0]) / (NUMBER_OF_POINTS - 1);

    //Define number of threads and store them in an array

    thread threads[NUMBER_OF_THREADS];

    if (choice == 1) {
        // Start the timer
        auto begin = chrono::high_resolution_clock::now();

        // Calculate all the points of the trajectory
        for (long long i = 0; i < NUMBER_OF_POINTS; i++) {
            x = P0[0] + i * dx;
            y = Coeff_A * x * x + Coeff_B * x + Coeef_B;
            double theta1, theta2;
            inverseKinematics(x, y, theta1, theta2);

            // Write the data to the final Array
            theta1Combined[i] += theta1;
            theta2Combined[i] += theta2;
        }

        // calculate the execution time

        auto end = chrono::high_resolution_clock::now();
        auto elapsed = chrono::duration_cast<chrono::nanoseconds>(end - begin);
        double milliseconds = elapsed.count() * 1e-6;
        double seconds = elapsed.count() / 1e9;

        cout << "\nCzas realizacji algorytmu SEWKENCYJNEGO wynosi: " << fixed << setprecision(0) << milliseconds
             << " [ms], " << fixed << setprecision(6) << seconds << " [s].\n";

        //write the values to Coeff_A file
        if (WRITE_FILE) {
            WriteJointAngles("joint_anglesThreadSeq.txt", theta1Combined, theta2Combined, NUMBER_OF_POINTS);
        }
        //free the gtheta arrays
        free(theta1Combined);
        free(theta2Combined);

    } else if (choice == 2) {

        //Start timer
        auto begin = chrono::high_resolution_clock::now();

        // Loop through the threads
        for (int i = 0; i < NUMBER_OF_THREADS; i++) {
            const long long start = i * NUMBER_OF_POINTS / NUMBER_OF_THREADS;
            const long long end = (i + 1) * NUMBER_OF_POINTS / NUMBER_OF_THREADS;
            double *local_theta1 = (double *) malloc((end - start) * sizeof(double));
            double *local_theta2 = (double *) malloc((end - start) * sizeof(double));
            threads[i] = thread(calculateJointAngles, start, end, dx, P0[0], Coeff_A, Coeff_B, Coeef_B, local_theta1,
                                local_theta2);
        }
        // Wait for threads to finish
        for (int i = 0; i < NUMBER_OF_THREADS; i++) {
            threads[i].join();

        }

        // calculate the execution time
        auto end = chrono::high_resolution_clock::now();
        auto elapsed = chrono::duration_cast<chrono::nanoseconds>(end - begin);
        double milliseconds = elapsed.count() * 1e-6;
        double seconds = elapsed.count() / 1e9;

        cout << "\nCzas realizacji algorytmu WIELOWĄTKOWEGO wynosi: " << fixed << setprecision(0) << milliseconds
             << " [ms], " << fixed << setprecision(6) << seconds << " [s].\n";


        if (WRITE_FILE) {
            //write the values to Coeff_A file
            WriteJointAngles("joint_anglesThreadNoRace.txt", theta1Combined, theta2Combined, NUMBER_OF_POINTS);
        }
        //free the gtheta arrays
        free(theta1Combined);
        free(theta2Combined);
    } else if (choice == 3) {


        //Set the number of threads to use
        omp_set_num_threads(NUMBER_OF_THREADS);

        //Start timer
        auto begin = chrono::high_resolution_clock::now();

#pragma omp parallel for
        for (long long i = 0; i < NUMBER_OF_POINTS; i++) {
            double x = P0[0] + i * dx;
            double y = Coeff_A * x * x + Coeff_B * x + Coeef_B;
            double theta1, theta2;
            inverseKinematics(x, y, theta1, theta2);

            {
                // Write the data to the final Array
                theta1Combined[i] += theta1;
                theta2Combined[i] += theta2;
            }
        }
        // calculate the execution time
        auto end = chrono::high_resolution_clock::now();
        auto elapsed = chrono::duration_cast<chrono::nanoseconds>(end - begin);
        double milliseconds = elapsed.count() * 1e-6;
        double seconds = elapsed.count() / 1e9;

        cout << "\nCzas realizacji algorytmu OpenMP wynosi: " << fixed << setprecision(0) << milliseconds << " [ms], "
             << fixed << setprecision(6) << seconds << " [s].\n";

        if (WRITE_FILE) {
            // Write the values to Coeff_A file
            WriteJointAngles("joint_anglesOpenMP.txt", theta1Combined, theta2Combined, NUMBER_OF_POINTS);
        }

        // Free the gtheta arrays
        free(theta1Combined);
        free(theta2Combined);
    }
    return 0;
}
