/*               ****NOTES****
  Before using this, go to the Serial Settings
  tab in the Maestro Control Center and apply these settings:

   Serial mode: UART, fixed baud rate
   Baud rate: 9600
   CRC disabled

  Be sure to click "Apply Settings" after making any changes.

  Array Indexes
  -------------
  a1 = 0
  a2 = 1
  b1 = 2
  b2 = 3
  c1 = 4
  c2 = 5

  range for hz (105, 140)

*/

// libraries
#include "PololuMaestro.h"
#include "math.h"
#include <Pixy2SPI_SS.h>

#include "ExtraMath.hpp"
#include "Kinematics.hpp"
#include "TuningConstants.hpp"

#define maestroSerial SERIAL_PORT_HARDWARE_OPEN  //for teensy

// **OBJECTS**
Pixy2SPI_SS pixy;
MicroMaestro maestro(maestroSerial);  // if using the Maestro Micro controller

// **PIXY2 CAMERA**
float ball[2];// X and Y co-ords of the ball

// **PID**
float error[2];       // error of the ball
float error_prev[2];  // previous error value used to calculate derivative. Derivative = (error-previous_error)/(change in time)
float deriv[2] = {0,0};     // Initialize derivative of the error
float integral[2] = {0, 0}; // Initialize integral (x, y components)
float out[2];         // output values (pitch and roll) in degrees
float time_i;         // initial time
float time_f;         // final time
float delta_time;     // change in time (rename from 'time')

// move servo i to an input position at a certain speed and acceleration value
void moveServo(int i, float pos, int spd, int acc) {
  pos = pos + offset[i];                                    //adds offset amount to the input position (degrees)
  pos = map(pos, range[i][0], range[i][1], abs_0, abs_90);  //converts input pos to ms position
  maestro.setSpeed(i, spd);                                 //sets input speed to servo i
  maestro.setAcceleration(i, spd);                          //sets input acceleration to servo i
  maestro.setTarget(i, pos);                                //drives motor to calculated position
}

//moves all servos to their calculated positions at a certain speed and acceleration value
void moveServos(int spd, int acc) {
  float pos;
  for (int i = 0; i < 6; i++) {
    maestro.setSpeed(i, spd);                                 //sets input speed to servo i
    maestro.setAcceleration(i, acc);                          //sets input acceleration to servo i
    pos = theta[i] + offset[i];                               //adds offset amount to the calculated angle (degrees)
    pos = map(pos, range[i][0], range[i][1], abs_0, abs_90);  //converts input pos to ms position
    maestro.setTarget(i, pos);                                //drives motor to calculated position
  }
}

//ends the program and stops all of the servos
void stop() {
  for (int i = 0; i < 6; i++) {
    maestro.setTarget(i, 0);  //stops servo i
  }
  while (1) {}
}

// find the location of the ball using the pixy2 cam
void findBall() {
  // grab blocks!
  pixy.ccc.getBlocks();

  // If there is 1 ball detected then collect and print the data
  if (pixy.ccc.numBlocks == 1) {
    //sets current X and Y co-ords
    ball[x] = pixy.ccc.blocks[0].m_x;  // absolute X location of the ball
    ball[y] = pixy.ccc.blocks[0].m_y;  // absolute Y location of the ball
  }
  // If there are multiple balls detected, then print so
  else if (pixy.ccc.numBlocks > 1) {
    Serial.println("MULTIPLE BALLS DETECTED");
    ball[x] = 4004;  // X component of the ball
    ball[y] = 4004;  // Y component of the ball
  }
  // If there is no ball detected, then print so
  else {
    // Serial.println("NO BALL DETECTED");
    ball[x] = 4004;  // X component of the ball
    ball[y] = 4004;  // Y component of the ball
  }
}

// calculates the Proportional, Derivative an Integral values and moves the servos
void PID() {

  // the error is a vector pointing from the center of the platform to the center of the ball
  error[x] = ball[x] - origin[x];  // x component of error (origin x-axis and camera x-axis are the same)
  error[y] = -(ball[y] - origin[y]);  // y component of error (origin y-axis and camera y-axis are opposite)

  time_f = millis();             // sets final time
  delta_time = (time_f - time_i) / 1000;  // change in time (seconds)
  time_i = millis(); // sets initial time

  // x and y components of derivative with low-pass filter
  deriv[x] = deriv[x] + delta_time/T*( -deriv[x] + (error[x] - error_prev[x]) / delta_time );
  deriv[y] = deriv[y] + delta_time/T*( -deriv[y] + (error[y] - error_prev[y]) / delta_time );

  // Trapezoidal rule for integral values of x and y
  integral[x] += (error[x] + error_prev[x]) * delta_time / 2.0;
  integral[y] += (error[y] + error_prev[y]) * delta_time / 2.0;

  // checks if derivative is NaN or INF. If so, set to zero
  if (isnan(deriv[x]) || isinf(deriv[x])) {  // x component of derivative
    deriv[x] = 0;
  }
  if (isnan(deriv[y]) || isinf(deriv[y])) {  // y component of derivative
    deriv[y] = 0;
  }

  // sets previous error to current error
  error_prev[x] = error[x];  // x component of previous error
  error_prev[y] = error[y];  // x component of previous error


  float r_ball = sqrt(pow(error[x], 2) + pow(error[y], 2));  // calculates the distance from the center of the platfrom to the ball

  // if ball to far away from center, out of the platform -> move to home position
  if (r_ball > r_platform) {
    Serial.println("Detected ball out of platform");
    InverseKinematics(hx_home, hy_home, hz_home, roll_home, pitch_home, yaw_home);  //hx, hy, hz, roll, pitch, yaw
    moveServos(vel_home, acc_home);
  }
  // if ball is inside the platform -> PID control -> set thetas to move the motors
  else {
    out[x] = error[x] * kp + deriv[x] * kd + integral[x] * ki;
    out[y] = error[y] * kp + deriv[y] * kd + integral[y] * ki;

    // error prevention
    float r_out = sqrt(pow(out[x], 2) + pow(out[y], 2));  //calculates average of the out angle (pitch and roll)
    Serial.println(r_out);
    // if r_out is greater than r_max, then scale down out values
    if (r_out > r_max) {
      out[x] = out[x] * (r_max / r_out);
      out[y] = out[y] * (r_max / r_out);
    }

    // compute thetas with IK
    Serial.println("Performing PID control");
    // out[x] proportional to x-axis -> to achieve linear movement in x(+) -> pitch(+) rotation
    // out[y] proportional to y-axis -> to achieve linear movement in y(+) -> roll(-) rotation
    InverseKinematics(hx_home, hy_home, hz_home, roll_home - out[y], pitch_home + out[x], yaw_home);  //hx, hy, hz, roll, pitch, yaw

    // move platform
    moveServos(vel_control, acc_control);
  }
}

void setup() {
  Serial.begin(115200);
  maestroSerial.begin(9600);
  // maestroSerial.begin(115200);
  // maestroSerial.begin(250000);
  pixy.init();
}

void loop() {
  findBall();  // finds the location of the ball
  if (ball[x] == 4004 && ball[y] == 4004) {
    // sees if ball position (x and y) is 4004, if so then the ball is not detected and platform should be in home position
    InverseKinematics(hx_home, hy_home, hz_home, roll_home, pitch_home, yaw_home);  //hx, hy, hz, roll, pitch, yaw
    moveServos(vel_home, acc_home);
  } else {
    PID();  // calculates the proportional and derivative terms and outputs them to the platform
  }
}
