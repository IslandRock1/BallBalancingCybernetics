//
// Created by killian on 03/11/2026.
//


#include "PID.hpp"
#include "TuningConstants.hpp"

#include <array>

#include "Kinematics.hpp"


bool PID::pid(float delta_time, bool USE_BALL_FILTER)
{
    bool gohome = false;
    std::array<float,2> out;
    if (!USE_BALL_FILTER)
    {
        deriv[0] = deriv[0] + delta_time/T*( -deriv[0] + (error[0] - error_prev[0]) / delta_time );
        deriv[1] = deriv[1] + delta_time/T*( -deriv[1] + (error[1] - error_prev[1]) / delta_time );
        if (isnan(deriv[0]) || isinf(deriv[0])) deriv[0] = 0.0;
        if (isnan(deriv[1]) || isinf(deriv[1])) deriv[1] = 0.0;
    }

    float r_ball = sqrt(pow(error[0], 2) + pow(error[1], 2));

    if (r_ball > r_platform)
    {
        integral = {0.0, 0.0};
        gohome = true;
    }
    // if ball is inside the platform -> PID control -> set thetas to move the motors
    else {
        // Trapezoidal rule, clamped so the integral alone can never exceed r_max.
        // Without this the integral keeps growing while the output is saturated.
        float i_max = (ki > 0.0f) ? r_max / ki : 0.0f;
        integral[0] = constrain(integral[0] + (error[0] + error_prev[0]) * delta_time / 2.0f, -i_max, i_max);
        integral[1] = constrain(integral[1] + (error[1] + error_prev[1]) * delta_time / 2.0f, -i_max, i_max);

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

    }

    // sets previous error to current error
    error_prev[0] = error[0];
    error_prev[1] = error[1];
    return gohome;
}

void PID::resetPid() {
    error_prev = {0,0};
    error = {0,0};
    deriv = {0,0};
    integral = {0,0};
}