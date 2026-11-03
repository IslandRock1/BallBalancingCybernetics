
#include "TuningConstants.hpp"

// --------------------------------------------------
// ------------------- START TUNE -------------------

// TUNE: ANGLE OFFSET FOR EACH MOTOR [NEGATIVE UP / POSITIVE DOWN] - DEGREES
// *should be less than 14 degrees since there are 25 teeth on the servo
float offset[6] = { -18.0, -3.0,    // channel #0 and channel #1
                    5.0, 0.0,       // channel #2 and channel #3
                    3.0, -3.0 };    // channel #4 and channel #5

// TUNE: INPUTS FROM CAMERA CALIBRATION
float origin[2] = { 146, 100 };  // (144, 108) X and Y co-ords of the  (in the camera frame)
float r_platform = 88;           // (121) distance from the center of the platform to the corner of the platform seen from the pixy2 cam


// TUNE: HOME hx, hy, hz, roll, pitch, yaw
float hx_home = 0;
float hy_home = 0;
float hz_home = 120;// 118.19374266158451;
float roll_home = 0;
float pitch_home = 0;
float yaw_home = 0;

// TUNE: MAX AVERAGE ANGLE CHANGE (PITCH AND ROLL) IN ONE STEP - DEGREES
float r_max = 15;

// TUNE: PID CONSTANTS VALUES
float kp = 0.05;         // proportional constant 6e-4
float kd = 0.02;         // derivative constant 0.56
float ki = 0.005;       // integral constant

// TUNE: Low pass filter constant T (only used when the ball filter is disabled)
float T = 0.08;

// TUNE: BALL KALMAN FILTER
// R_MEAS: run the camera with the ball standing still and use the variance of
// the block position. Integer pixels alone give ~1 px^2.
// Q_ACCEL: raise it if the estimate lags a moving ball, lower it if the
// estimate is noisy.
float R_MEAS = 4.0;        // [px^2], sigma = 2 px
float Q_ACCEL = 1.0e6;     // [px^2/s^4], sigma = 1000 px/s^2
float KF_GATE = 9.0;       // 3 sigma per axis
int KF_COAST_MS = 200;     // ~12 camera frames
int KF_MAX_MISSES = 8;

// TUNE: MOTOR VELOCITIES AND ACCELERATIONS
int vel_home = 20;
int acc_home = 20;
int vel_control = 0;
int acc_control = 0;

// TUNE: INPUTS FROM MEASUREMENTS (mm)
float l0 = 73.025;  // from h0 to a0
float d1 = 36.8893; // from a0 to a01
float lf = 67.775;  // from hf to midle af1-cf2
float m = 12.7;     // from previous middle point to af1
float p1 = 31.75;   // motor small arm
float p2 = 129;     // motor long arm

// ------------------- END TUNE -------------------
// ------------------------------------------------

//**CONSTANTS**
float abs_0 = 4000;                                     //ms position of absolute 0 degrees
float abs_90 = 8000;                                    //ms position of absolute 90 degrees
float toDeg = 180 / PI;                                 // radians to degrees conversion factor
float toRad = 1 / toDeg;
int x = 0, y = 1, z = 2;                                // defines x, y, and z array indexes to be used
String ID[6] = { "a1", "a2", "b1", "b2", "c1", "c2" };  //servo ID's

// ANGLE RANGE FOR EACH SERVO CCW
// each servo can move from absolute 0 to absolute 90 degrees
// this defines a different range such as 90 to 180 degrees
float range[6][2] = { { -45, 45 }, { 45, -45 },  // a1, a2
                      { -45, 45 }, { 45, -45 },  // b1, b2
                      { -45, 45 }, { 45, -45 } }; // c1, c2
