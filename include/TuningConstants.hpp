
#ifndef BALLBALANCING_TUNINGCONSTANTS_HPP
#define BALLBALANCING_TUNINGCONSTANTS_HPP

#include <Arduino.h>
#include <math.h>

// These are declared here and defined once in src/TuningConstants.cpp.

// --------------------------------------------------
// ------------------- START TUNE -------------------

// TUNE: ANGLE OFFSET FOR EACH MOTOR [NEGATIVE UP / POSITIVE DOWN] - DEGREES
// *should be less than 14 degrees since there are 25 teeth on the servo
extern float offset[6];

// TUNE: INPUTS FROM CAMERA CALIBRATION
extern float origin[2];  // (144, 108) X and Y co-ords of the  (in the camera frame)
extern float r_platform; // (121) distance from the center of the platform to the corner of the platform seen from the pixy2 cam


// TUNE: HOME hx, hy, hz, roll, pitch, yaw
extern float hx_home;
extern float hy_home;
extern float hz_home;
extern float roll_home;
extern float pitch_home;
extern float yaw_home;

// TUNE: MAX AVERAGE ANGLE CHANGE (PITCH AND ROLL) IN ONE STEP - DEGREES
extern float r_max;

// TUNE: PID CONSTANTS VALUES
extern float kp;         // proportional constant 6e-4
extern float kd;         // derivative constant 0.56
extern float ki;         // integral constant

// TUNE: Low pass filter constant T
extern float T;

// TUNE: MOTOR VELOCITIES AND ACCELERATIONS
extern int vel_home;
extern int acc_home;
extern int vel_control;
extern int acc_control;

// TUNE: INPUTS FROM MEASUREMENTS (mm)
extern float l0; // from h0 to a0
extern float d1; // from a0 to a01
extern float lf; // from hf to midle af1-cf2
extern float m; // from previous middle point to af1
extern float p1; // motor small arm
extern float p2; // motor long arm

// ------------------- END TUNE -------------------
// ------------------------------------------------

//**CONSTANTS**
extern float abs_0;                                     //ms position of absolute 0 degrees
extern float abs_90;                                    //ms position of absolute 90 degrees
extern float toDeg;                                     // radians to degrees conversion factor
extern float toRad;
extern int x, y, z;                                     // defines x, y, and z array indexes to be used
extern String ID[6];                                    //servo ID's

// ANGLE RANGE FOR EACH SERVO CCW
// each servo can move from absolute 0 to absolute 90 degrees
// this defines a different range such as 90 to 180 degrees
extern float range[6][2];  // a1, a2 / b1, b2 / c1, c2

#endif //BALLBALANCING_TUNINGCONSTANTS_HPP
