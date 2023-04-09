#include "InverseKinematics_2DOF.h"
#include "AdditonalFunctions.h"
#include <iostream>
#include <cmath>
#include <chrono>
#include <thread>
#include <iomanip>
#include <omp.h>
#include <windows.h>


using namespace std;

//Configuration variables
bool WRITE_FILE = FALSE;
bool WRITE_ARRAY = TRUE;
bool THREAD_TEST = TRUE;

//Define global variables

const int NUMBER_OF_THREADS = 50;
long long MAX_NUMBER_OF_POINTS = 1700000000;

double *theta1Combined;// = (double *) malloc(MAX_NUMBER_OF_POINTS * sizeof(double));
double *theta2Combined;// = (double *) malloc(MAX_NUMBER_OF_POINTS * sizeof(double));
// other declarations

HANDLE h = GetStdHandle(STD_OUTPUT_HANDLE);

// additional functions defining
void calculateJointAngles(long long start, long long end, double dx, double P0, double a, double b, double c) {
    double x, y;
    for (long long i = start; i < end; i++) {
        x = P0 + i * dx;
        y = a * x * x + b * x + c;
        double t1, t2;
        inverseKinematics(x, y, t1, t2);
        // Store the joint angles in separate
        if (WRITE_ARRAY) {
            theta1Combined[i] += t1;
            theta2Combined[i] += t2;
        }
    }

}

int main() {

    //Choose the way you want to solve the issue
    int choice;
    cout << "Choose option:" << endl;
    cout << "1. Sequential" << endl;
    cout << "2. Thread" << endl;
    cout << "3. OpenMP" << endl;
    cout << "4. AutoMode" << endl;
    cout << "Enter choice: ";
    cin >> choice;

    // Define the start and end points of the parabolic trajectory
    const double P0[2] = {400, 0};
    const double P2[2] = {-400, 0};

    // Define the coefficients of the quadratic equation
    const double Coeff_A = -0.0025;
    const double Coeff_B = 0.0;
    const double Coeef_B = 400.0;

    // Predefine x and y for the trajectory points

    double x, y;

    // Declare time calculation for specific variants
    double millisecondsSeq = 0.0;
    double secondsSeq = 0.0;
    double millisecondsThread = 0.0;
    double secondsThread = 0.0;
    double millisecondsOpenMP = 0.0;
    double secondsOpenMP = 0.0;

    //Define number of threads and store them in an array

    //thread threads[NUMBER_OF_THREADS];


    //Declare number of points and iterations pointer variable

    int iterations = 1;
    long long numberOfPoints = 0;
    int numberOfThreads = 0;
    while (iterations <= 50) {

        if (choice == 4) {
            if (THREAD_TEST) {

                numberOfThreads = iterations;
                numberOfPoints = MAX_NUMBER_OF_POINTS;
            } else {
                numberOfPoints = MAX_NUMBER_OF_POINTS / 50 * iterations;
                numberOfThreads = NUMBER_OF_THREADS;
            }
        } else {
            numberOfPoints = MAX_NUMBER_OF_POINTS;
            numberOfThreads = NUMBER_OF_THREADS;
        }

        thread threads[numberOfThreads];

        // calculate the size of a step for the trajectory points
        double dx = (P2[0] - P0[0]) / (numberOfPoints - 1);

        // Pu the number of points and iterations
        SetConsoleTextAttribute(h, 5);
        cout << "\nWe are computing the iteration number: " << iterations << ", for points of number: "
             << numberOfPoints
             << endl;
        SetConsoleTextAttribute(h, 15);

        if ((choice == 1 || choice == 4) && THREAD_TEST == FALSE) {

            theta1Combined = (double *) malloc(numberOfPoints * sizeof(double));
            theta2Combined = (double *) malloc(numberOfPoints * sizeof(double));
            // Start the timer
            auto beginSeq = chrono::high_resolution_clock::now();

            // Calculate all the points of the trajectory
            for (long long i = 0; i < numberOfPoints; i++) {
                x = P0[0] + i * dx;
                y = Coeff_A * x * x + Coeff_B * x + Coeef_B;
                double theta1, theta2;
                inverseKinematics(x, y, theta1, theta2);

                // Write the data to the final Array
                if (WRITE_ARRAY) {
                    theta1Combined[i] += theta1;
                    theta2Combined[i] += theta2;
                }
            }

            // calculate the execution time

            auto endSeq = chrono::high_resolution_clock::now();
            auto elapsedSeq = chrono::duration_cast<chrono::nanoseconds>(endSeq - beginSeq);
            millisecondsSeq = elapsedSeq.count() * 1e-6;
            secondsSeq = elapsedSeq.count() / 1e9;

            cout << "\nExecution time of SEQUENTIAL algorithm is: " << fixed << setprecision(6) << millisecondsSeq
                 << " [ms], " << fixed << setprecision(6) << secondsSeq << " [s].\n";

            //write the values to Coeff_A file
            if (WRITE_FILE) {
                WriteJointAngles("joint_anglesThreadSeq.txt", theta1Combined, theta2Combined, numberOfPoints);
            }
            //free the gtheta arrays
            free(theta1Combined);
            free(theta2Combined);

        }
        if (choice == 2 || choice == 4) {

            theta1Combined = (double *) malloc(numberOfPoints * sizeof(double));
            theta2Combined = (double *) malloc(numberOfPoints * sizeof(double));
            //Start timer
            auto beginThread = chrono::high_resolution_clock::now();

            // Loop through the threads
            for (int i = 0; i < numberOfThreads; i++) {
                const long long start = i * numberOfPoints / numberOfThreads;
                const long long end = (i + 1) * numberOfPoints / numberOfThreads;
                threads[i] = thread(calculateJointAngles, start, end, dx, P0[0], Coeff_A, Coeff_B, Coeef_B);
            }

            // Wait for threads to finish
            for (int i = 0; i < numberOfThreads; i++) {
                threads[i].join();
            }

            // calculate the execution time
            auto endThread = chrono::high_resolution_clock::now();
            auto elapsedThread = chrono::duration_cast<chrono::nanoseconds>(endThread - beginThread);
            millisecondsThread = elapsedThread.count() * 1e-6;
            secondsThread = elapsedThread.count() / 1e9;

            cout << "\nExecution time of MULTITHREAD algorithm is: " << fixed << setprecision(6) << millisecondsThread
                 << " [ms], " << fixed << setprecision(6) << secondsThread << " [s].\n";


            if (WRITE_FILE) {
                //write the values to Coeff_A file
                WriteJointAngles("joint_anglesThreadNoRace.txt", theta1Combined, theta2Combined, numberOfPoints);
            }
            //free the gtheta arrays
            free(theta1Combined);
            free(theta2Combined);
        }
        if (choice == 3 || choice == 4) {

            //Set the number of threads to use
            omp_set_num_threads(numberOfThreads);
            theta1Combined = (double *) malloc(numberOfPoints * sizeof(double));
            theta2Combined = (double *) malloc(numberOfPoints * sizeof(double));
            //Start timer
            auto beginOpenMp = chrono::high_resolution_clock::now();

#pragma omp parallel for
            for (long long i = 0; i < numberOfPoints; i++) {
                double x = P0[0] + i * dx;
                double y = Coeff_A * x * x + Coeff_B * x + Coeef_B;
                double theta1, theta2;
                inverseKinematics(x, y, theta1, theta2);


                // Write the data to the final Array
                if (WRITE_ARRAY) {
                    theta1Combined[i] += theta1;
                    theta2Combined[i] += theta2;
                }

            }
            // calculate the execution time
            auto endOpenMp = chrono::high_resolution_clock::now();
            auto elapsedOpenMp = chrono::duration_cast<chrono::nanoseconds>(endOpenMp - beginOpenMp);
            millisecondsOpenMP = elapsedOpenMp.count() * 1e-6;
            secondsOpenMP = elapsedOpenMp.count() / 1e9;

            cout << "\nExecution time of OpenMP algorithm is: " << fixed << setprecision(6) << millisecondsOpenMP
                 << " [ms], " << fixed << setprecision(6) << secondsOpenMP << " [s].\n";


            if (WRITE_FILE) {
                // Write the values to Coeff_A file
                WriteJointAngles("joint_anglesOpenMP.txt", theta1Combined, theta2Combined, numberOfPoints);
            }

            // Free the gtheta arrays
            free(theta1Combined);
            free(theta2Combined);
        }

        //dont iterate if not auto mode
        if (choice != 4) {
            iterations = 99;
        } else {
            writeComputingTimes("ComputingTimesDifferentThreads.txt", millisecondsSeq, millisecondsThread,
                                millisecondsOpenMP, numberOfThreads);
            iterations++;
        }

    }
    return 0;
}
