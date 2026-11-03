
#ifndef BALLBALANCING_KINEMATICS_HPP
#define BALLBALANCING_KINEMATICS_HPP

#include "TuningConstants.hpp"
#include "ExtraMath.hpp"

// These are declared here and defined once in src/Kinematics.cpp.

// calculates points a10, a20, b10, b20, c10, and c20
extern float a10[3];
extern float a20[3];
extern float b10[3];
extern float b20[3];
extern float c10[3];
extern float c20[3];

// calculates vectors a_10_20, b_10_20, c_10_20,
extern float a_10_20[3];
extern float b_10_20[3];
extern float c_10_20[3];

//calculates normal vectors na, nb, and nc for each side of the triangle
extern float na[3]; // pointing out
extern float nb[3]; // pointing out
extern float nc[3]; // pointing out

extern float t;
extern float w;

// calculated theta values a1, a2, b1, b2, c1, c2 respectively
extern float theta[6];
extern float theta_prev[6];

// compute rotations
void rotations(float* result, float hi[3], float Rz[3][3], float Ry[3][3], float Rx[3][3]);

// compute inverse kinematics
void InverseKinematics(float hx, float hy, float hz, float roll, float pitch, float yaw);

// stops all servos and blocks forever (defined in main.cpp)
void stop();

#endif //BALLBALANCING_KINEMATICS_HPP
