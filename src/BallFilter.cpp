#include <math.h>
#include "BallFilter.hpp"

namespace {
// Covariance right after a (re)acquisition. The position is trusted to about
// the same level as a single measurement; the velocity is unknown, so it gets
// a wide prior (500 px/s).
constexpr float P0_POS = 4.0f;      // [px^2]
constexpr float P0_VEL = 250000.0f; // [(px/s)^2]
}

// Restart the estimate at measurement z with unknown velocity.
void AxisFilter::reset(float z) {
  p = z;
  v = 0.0f;
  P00 = P0_POS;
  P01 = 0.0f;
  P11 = P0_VEL;
}

// Time update: p = F p, P = F P F^T + Q with F = [[1, dt], [0, 1]] and Q from a
// white acceleration model.
void AxisFilter::predict(float dt, float q) {
  p += v * dt;

  const float dt2 = dt * dt;
  const float p00 = P00 + 2.0f * dt * P01 + dt2 * P11 + q * dt2 * dt2 / 4.0f;
  const float p01 = P01 + dt * P11 + q * dt2 * dt / 2.0f;
  P11 += q * dt2;

  P00 = p00;
  P01 = p01;
}

// Measurement update with H = [1, 0]. For scalar H the covariance stays
// symmetric, so only the upper triangle has to be stored.
void AxisFilter::update(float z, float r) {
  const float S = P00 + r;           // innovation variance
  if (S <= 0.0f) {
    return;
  }

  const float k0 = P00 / S;
  const float k1 = P01 / S;
  const float innovation = z - p;

  p += k0 * innovation;
  v += k1 * innovation;

  P11 -= k1 * P01;
  P01 *= (1.0f - k0);
  P00 *= (1.0f - k0);
}
