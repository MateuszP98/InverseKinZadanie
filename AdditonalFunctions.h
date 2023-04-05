#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>

#ifndef INVERSEKINZADANIE_ADDITONALFUNCTIONS_H
#define INVERSEKINZADANIE_ADDITONALFUNCTIONS_H

using  namespace std;

void WriteJointAngles(const string &filename, double *theta1, double *theta2, long long n_points);
void writeComputingTimes(const string &filename, double timeSequence, double timeThread, long long TimeOpenMP, int Threads);
#endif //INVERSEKINZADANIE_ADDITONALFUNCTIONS_H
