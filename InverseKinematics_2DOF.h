#include <stdio.h>
#include <stdlib.h>
#include <cmath>

using namespace std;

#ifndef _InverseKinematics_2DOF_H
#define _InverseKinematics_2DOF_H

typedef enum {
    FALSE, TRUE
} BOOLEAN;

// Function prototypes

void inverseKinematics(double x, double y, double &theta1, double &theta2);

#endif