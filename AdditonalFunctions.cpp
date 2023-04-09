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

void writeComputingTimes(const std::string &filename, double timeSequence, double timeThread, double TimeOpenMP,
                         long long Threads) {

    // Open the file in append mode
    std::ofstream outFile(filename, std::ios_base::app);

    if (outFile.is_open()) { // Check if the file is opened successfully

        // Set output stream formatting
        outFile << std::fixed << std::setprecision(4);

        // Write data to the file
        outFile << Threads << " " << timeSequence << " " << timeThread << " " << TimeOpenMP << std::endl;

        // Close the file
        outFile.close();

    } else {
        std::cout << "Failed to open file for writing." << std::endl;
    }
}