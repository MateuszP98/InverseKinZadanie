#include <cmath>
#include <iomanip>
#include "AdditonalFunctions.h"

void WriteJointAngles(const string &filename, double *theta1, double *theta2, long long n_points) {
// Remove the file if it exists
    remove(filename.c_str());

// Open the file for writing
    ofstream outfile(filename);
    outfile << fixed << setprecision(4);
// Write the joint angles to the file
    for (long long i = 0; i < n_points; i++) {

        outfile << theta1[i] << " " << theta2[i] << std::endl;

    }

// Close the file
    outfile.close();
}

void writeComputingTimes(const std::string &filename, double timeSequence, double timeThread, long long TimeOpenMP,
                         int Threads) {

    std::ofstream outFile(filename.c_str());
    if (outFile.is_open()) {
        outFile << Threads << " " << timeSequence << " " << timeThread << " " << TimeOpenMP << std::endl;
        outFile.close();
    }

}