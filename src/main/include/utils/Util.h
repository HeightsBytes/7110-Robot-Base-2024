/**
 * @file Util.h
 * @author Nathan Correa
 * @brief A few utils commonly used across the robot's code
 * @date 2023-08-19
 */

#pragma once

#define LAMBDA(x) [this] {return x;}

namespace hb {
    
    inline int sgn(double x) {
        return x >= 0 ? 1 : -1;
    }

    inline bool InRange(double val, double target, double epsilon) {
        // Check to see if the val is within the bounded range created by [target - epsilon, target + epsilon]
        return (val > (target - epsilon) && val < (target + epsilon));
    }

} // namespace hb
