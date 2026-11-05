//
// Created by killian on 03/11/2026.
//

#ifndef BALLBALANCINGCYBERNETICS_PID_H
#define BALLBALANCINGCYBERNETICS_PID_H

#include <array>

class PID
{
public:
    bool pid(float delta_time, bool USE_BALL_FILTER);
    void resetPid();
    std::array<float,2> error_prev;
    std::array<float,2> error;
    std::array<float,2> deriv;
    std::array<float,2> integral;
};

#endif //BALLBALANCINGCYBERNETICS_PID_H
