#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <iomanip>

#ifndef INVERSEKINZADANIE_ADDITONALFUNCTIONS_H
#define INVERSEKINZADANIE_ADDITONALFUNCTIONS_H

using  namespace std;

void WriteJointAngles(const string &filename, double *theta1, double *theta2, long long n_points);
void writeComputingTimes(const string &filename, double timeSequence, double timeThread, double TimeOpenMP, long long Threads);

#endif //INVERSEKINZADANIE_ADDITONALFUNCTIONS_H
