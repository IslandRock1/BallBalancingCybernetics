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
#include <math.h>
#include <Pixy2SPI_SS.h>

#include "BallFilter.hpp"
#include "Kinematics.hpp"
#include "TuningConstants.hpp"

#define maestroSerial SERIAL_PORT_HARDWARE_OPEN  //for teensy

// **MODES**
// Use the Kalman estimate (position and velocity) for the PID. Set to false to
// reproduce the original behaviour: raw camera position, low-pass filtered
// derivative, and home as soon as the ball is not seen.
static constexpr bool USE_BALL_FILTER = true;

// Longest control period we accept [s]. A longer one means the loop was
// blocked, so the ball state is unknown and the track is dropped.
static constexpr float DT_MAX = 0.5f;

// Print "t,x,y,vx,vy" every DEBUG_MS milliseconds (0 disables). Useful for
// tuning the filter: the data can be replayed offline.
static constexpr uint32_t DEBUG_MS = 0;

// **OBJECTS**
Pixy2SPI_SS pixy;
MicroMaestro maestro(maestroSerial);  // if using the Maestro Micro controller

// **BALL ESTIMATOR**
AxisFilter ballFilt[2];          // [x], [y] position and velocity estimates
static bool filterInit = false;  // false = nothing to predict from yet
static int missCount = 0;        // consecutive detections rejected by the gate
static uint32_t lastMeasureMs = 0;
static uint32_t tPrevUs = 0;

// **PID**
float error[2];             // ball position relative to the platform centre [px]
float error_prev[2];        // previous error, used by the integral and the fallback derivative
float deriv[2] = {0, 0};    // D input [px/s]: filter velocity, or low-pass filtered derivative
float integral[2] = {0, 0}; // integral of the error (x, y components)
float out[2];               // output values (pitch and roll) in degrees

// last speed/acceleration sent to the Maestro, so they only change when needed
static int lastSpd = -1;
static int lastAcc = -1;

//moves all servos to their calculated positions at a certain speed and acceleration value
void moveServos(int spd, int acc) {
  // Speed and acceleration are settings that persist in the Maestro, so they
  // only have to be sent when they change. Sending all six targets is what
  // keeps the control loop short (18 commands per cycle would take ~75 ms).
  if (spd != lastSpd || acc != lastAcc) {
    for (int i = 0; i < 6; i++) {
      maestro.setSpeed(i, spd);
      maestro.setAcceleration(i, acc);
    }
    lastSpd = spd;
    lastAcc = acc;
  }

  float pos;
  for (int i = 0; i < 6; i++) {
    pos = theta[i] + offset[i];                               //adds offset amount to the calculated angle (degrees)
    pos = map(pos, range[i][0], range[i][1], abs_0, abs_90);  //converts input pos to quarter-microseconds
    pos = constrain(pos, abs_0, abs_90);                      //never drive past the calibrated range
    maestro.setTarget(i, pos);                                //drives motor to calculated position
  }
}

//ends the program and stops all of the servos
void stop() {
  for (int i = 0; i < 6; i++) {
    maestro.setTarget(i, 0);  //stops sending pulses to servo i
  }
  while (1) {}
}

//moves the platform to the home position
void goHome() {
  InverseKinematics(hx_home, hy_home, hz_home, roll_home, pitch_home, yaw_home);  //hx, hy, hz, roll, pitch, yaw
  moveServos(vel_home, acc_home);
}

//forgets the PID history (used whenever control is interrupted)
void resetPid() {
  for (int i = 0; i < 2; i++) {
    error[i] = 0;
    error_prev[i] = 0;
    deriv[i] = 0;
    integral[i] = 0;
  }
}

// **CAMERA / BALL ESTIMATOR**

// pixy block -> coordinates centred on the platform, same axes as the PID
void blockToPlatform(int i, float& zx, float& zy) {
  zx = (float)pixy.ccc.blocks[i].m_x - origin[x];
  zy = origin[y] - (float)pixy.ccc.blocks[i].m_y;  // camera y-axis is opposite
}

// index of the largest block: the most likely ball when starting from scratch
int largestBlock() {
  int best = -1;
  uint32_t bestArea = 0;
  for (int i = 0; i < pixy.ccc.numBlocks; i++) {
    uint32_t area = (uint32_t)pixy.ccc.blocks[i].m_width * pixy.ccc.blocks[i].m_height;
    if (area > bestArea) {
      bestArea = area;
      best = i;
    }
  }
  return best;
}

//(re)start the estimate from block i
void acquireBall(int i) {
  if (i < 0) {
    return;
  }
  float zx, zy;
  blockToPlatform(i, zx, zy);
  ballFilt[x].reset(zx);
  ballFilt[y].reset(zy);
  filterInit = true;
  missCount = 0;
  lastMeasureMs = millis();
}

// Predict, then associate the camera blocks with the prediction and update on
// the plausible one. Returns true while the estimate may be used for control.
bool updateBallEstimator(float dt) {
  // prediction (also happens when no block is found, which is the point)
  if (filterInit) {
    ballFilt[x].predict(dt, Q_ACCEL);
    ballFilt[y].predict(dt, Q_ACCEL);
  }

  // grab blocks
  pixy.ccc.getBlocks();

  if (!filterInit) {
    if (pixy.ccc.numBlocks == 0) {
      return false;
    }
    acquireBall(largestBlock());
    return true;
  }

  // pick the block that is most consistent with the prediction. The distance
  // is normalised by the innovation variance, so it accounts for how uncertain
  // the prediction currently is.
  int best = -1;
  float bestD2 = KF_GATE;  // anything at or beyond the gate is rejected
  for (int i = 0; i < pixy.ccc.numBlocks; i++) {
    float zx, zy;
    blockToPlatform(i, zx, zy);
    float dx = zx - ballFilt[x].p;
    float dy = zy - ballFilt[y].p;
    float d2 = dx * dx / (ballFilt[x].P00 + R_MEAS) + dy * dy / (ballFilt[y].P00 + R_MEAS);
    if (d2 < bestD2) {
      bestD2 = d2;
      best = i;
    }
  }

  if (best >= 0) {
    float zx, zy;
    blockToPlatform(best, zx, zy);
    ballFilt[x].update(zx, R_MEAS);
    ballFilt[y].update(zy, R_MEAS);
    lastMeasureMs = millis();
    missCount = 0;
  } else if (pixy.ccc.numBlocks > 0) {
    // There are detections but none of them matches the prediction. The ball
    // was probably moved while it was out of view; after a few tries, restart
    // the track on the most plausible block.
    if (++missCount >= KF_MAX_MISSES) {
      acquireBall(largestBlock());
    }
  }

  return (millis() - lastMeasureMs) <= (uint32_t)KF_COAST_MS;
}

// original behaviour: exactly one block, no prediction, no coasting
bool readBallRaw() {
  pixy.ccc.getBlocks();
  if (pixy.ccc.numBlocks != 1) {
    return false;
  }
  blockToPlatform(0, error[x], error[y]);
  return true;
}

// calculates the Proportional, Derivative and Integral values and moves the servos
void PID(float dt) {
  // Fallback: low-pass filtered derivative of the measured position. Only
  // needed when the ball filter is disabled.
  if (!USE_BALL_FILTER) {
    deriv[x] = deriv[x] + dt / T * (-deriv[x] + (error[x] - error_prev[x]) / dt);
    deriv[y] = deriv[y] + dt / T * (-deriv[y] + (error[y] - error_prev[y]) / dt);
    if (isnan(deriv[x]) || isinf(deriv[x])) {
      deriv[x] = 0;
    }
    if (isnan(deriv[y]) || isinf(deriv[y])) {
      deriv[y] = 0;
    }
  }

  float r_ball = sqrt(pow(error[x], 2) + pow(error[y], 2));  // distance from the centre of the platform to the ball

  // if ball to far away from center, out of the platform -> move to home position
  if (r_ball > r_platform) {
    // the ball left the working area, so drop the accumulated integral instead
    // of applying it again when the ball comes back
    integral[x] = 0.0f;
    integral[y] = 0.0f;
    goHome();
  }
  // if ball is inside the platform -> PID control -> set thetas to move the motors
  else {
    // Trapezoidal rule, clamped so the integral alone can never exceed r_max.
    // Without this the integral keeps growing while the output is saturated.
    float i_max = (ki > 0.0f) ? r_max / ki : 0.0f;
    integral[x] = constrain(integral[x] + (error[x] + error_prev[x]) * dt / 2.0f, -i_max, i_max);
    integral[y] = constrain(integral[y] + (error[y] + error_prev[y]) * dt / 2.0f, -i_max, i_max);

    out[x] = error[x] * kp + deriv[x] * kd + integral[x] * ki;
    out[y] = error[y] * kp + deriv[y] * kd + integral[y] * ki;

    // error prevention
    float r_out = sqrt(pow(out[x], 2) + pow(out[y], 2));  // average of the out angle (pitch and roll)
    // if r_out is greater than r_max, then scale down out values
    if (r_out > r_max) {
      out[x] = out[x] * (r_max / r_out);
      out[y] = out[y] * (r_max / r_out);
    }

    // compute thetas with IK
    // out[x] proportional to x-axis -> to achieve linear movement in x(+) -> pitch(+) rotation
    // out[y] proportional to y-axis -> to achieve linear movement in y(+) -> roll(-) rotation
    InverseKinematics(hx_home, hy_home, hz_home, roll_home - out[y], pitch_home + out[x], yaw_home);  //hx, hy, hz, roll, pitch, yaw

    // move platform
    moveServos(vel_control, acc_control);
  }

  // sets previous error to current error
  error_prev[x] = error[x];
  error_prev[y] = error[y];
}

void setup() {
  Serial.begin(115200);
  maestroSerial.begin(9600);
  // maestroSerial.begin(115200);
  // maestroSerial.begin(250000);
  pixy.init();

  tPrevUs = micros();
  lastMeasureMs = millis();
}

void loop() {
  uint32_t now = micros();
  float dt = (float)(now - tPrevUs) * 1e-6f;  // unsigned difference, rollover-safe
  tPrevUs = now;

  if (dt <= 0.0f) {
    return;  // loop faster than the timer resolution
  }

  if (dt > DT_MAX) {
    // The loop was blocked (serial, camera, ...). What happened to the ball in
    // the meantime is unknown, so start over instead of extrapolating.
    filterInit = false;
    resetPid();
    goHome();
    return;
  }

  bool usable = USE_BALL_FILTER ? updateBallEstimator(dt) : readBallRaw();

  if (usable) {
    if (USE_BALL_FILTER) {
      // the estimate is the PID input; velocity replaces the filtered derivative
      error[x] = ballFilt[x].p;
      error[y] = ballFilt[y].p;
      deriv[x] = ballFilt[x].v;
      deriv[y] = ballFilt[y].v;
    }
    PID(dt);
  }
  // no usable measurement -> hold home position and start clean next time
  else {
    filterInit = false;
    resetPid();
    goHome();
  }

  if (DEBUG_MS > 0) {
    static uint32_t lastPrintMs = 0;
    if (millis() - lastPrintMs >= DEBUG_MS) {
      lastPrintMs = millis();
      Serial.print(millis());
      Serial.print(',');
      Serial.print(error[x]);
      Serial.print(',');
      Serial.print(error[y]);
      Serial.print(',');
      Serial.print(deriv[x]);
      Serial.print(',');
      Serial.println(deriv[y]);
    }
  }
}
