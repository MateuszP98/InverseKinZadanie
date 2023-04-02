#include "InverseKinematics_2DOF.h"


void inverseKinematics(double x, double y, double& theta1, double& theta2) {

	// L1 -> Length of the fiest elbow 
	// L2 -> Length of the second elbow 
	// r -> distance from the origin to the end effector

	int L1 = 100;
	int L2 = 150;
    double r = sqrt(x*x + y*y);
    // Calculate the angle between the first arm and the x-axis
    theta1 = atan2(y, x) - acos((L1*L1 + r*r - L2*L2) / (2*L1*r));

    // Calculate the angle between the second arm and the first arm
    theta2 = acos((L1*L1 + L2*L2 - r*r) / (2*L1*L2));

    // Convert angles to degrees
    theta1 = theta1 * 180.0 / M_PI;
    theta2 = theta2 * 180.0 / M_PI;


}