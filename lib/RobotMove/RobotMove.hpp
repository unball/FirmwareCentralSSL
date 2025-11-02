#ifndef ROBOT_MOVE_H
#define ROBOT_MOVE_H

#include <Arduino.h>
#include "Utils.hpp"
#include "../../include/pins.h"
#include "../../include/constants.h"

namespace RobotMove
{
    void setup(boolean isDebugModeActive);
    float* calculateWheelVelocities();
    void setRobotVelocities(float linearVelocity_x,float linearVelocity_y, float angularVelocity);

    // Wheel distance from robot center mass
    static float R = constants::ROBOT_WHEELS_RADIUS;

    // Wheel angle front
    static float phi = radians(constants::ROBOT_WHEELS_ANGLE_FRONT);

    // Wheel angle back
    static float theta = radians(constants::ROBOT_WHEELS_ANGLE_BACK);

    // Velocity Coupling Matrix
    static float D[4][3];

    static float robotVelocities[3];

    static boolean isModuleDebugModeActive;
    static char* moduleName = "RobotMove";
}

#endif // ROBOT_MOVE_H