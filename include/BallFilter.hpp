
#ifndef BALLBALANCING_BALLFILTER_HPP
#define BALLBALANCING_BALLFILTER_HPP

// 2-state (position, velocity) Kalman filter for one axis.
// Model: constant velocity, white acceleration noise.
// The measurement only sees position, which keeps the update scalar and cheap
// enough to run inside the control loop.
struct AxisFilter {
  float p = 0.0f;    // position estimate [px]
  float v = 0.0f;    // velocity estimate [px/s]
  float P00 = 0.0f;  // covariance (position, position)
  float P01 = 0.0f;  // covariance (position, velocity)
  float P11 = 0.0f;  // covariance (velocity, velocity)

  // Restart the estimate at measurement z with unknown velocity.
  void reset(float z);

  // Time update. dt [s], q = acceleration noise variance [px^2/s^4].
  void predict(float dt, float q);

  // Measurement update. z [px], r = measurement variance [px^2].
  void update(float z, float r);
};

#endif //BALLBALANCING_BALLFILTER_HPP
