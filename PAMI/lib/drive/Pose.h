#pragma once

#include <math.h>

// Pose sur la table : x, y en mm, theta en radians (sens trigo).
struct Pose {
    float x = 0.0f;
    float y = 0.0f;
    float theta = 0.0f;
};

inline float wrapPi(float a) {
    a = fmodf(a + (float)M_PI, 2.0f * (float)M_PI);
    if (a < 0.0f) a += 2.0f * (float)M_PI;
    return a - (float)M_PI;
}

inline float distanceMm(const Pose& a, const Pose& b) {
    return hypotf(b.x - a.x, b.y - a.y);
}

inline float headingTo(const Pose& from, const Pose& to) {
    return atan2f(to.y - from.y, to.x - from.x);
}
