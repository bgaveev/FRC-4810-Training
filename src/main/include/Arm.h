#pragma once

#include <frc/Timer.h>

#include "RobotIO.h"

namespace arm
{
    enum eState
    {
        STATE_START = 0,
        STATE_IDLE = 1,
        STATE_HOMING = 2,
        STATE_MANUAL_RAISE = 3,
        STATE_AUTO_RAISE = 5,
        STATE_ERROR = 99
    };

    enum eCommand
    {
        COMMAND_NONE,
        COMMAND_HOME,
        COMMAND_MANUAL_RAISE,
        COMMAND_MANUAL_LOWER,
        COMMAND_AUTO_RAISE,
        COMMAND_STOP
    }

    // Timeout Constants
    static constexpr double dHomingTimeout = 10.0;
    static constexpr double dManualRaiseTimeout = 15.0;
    static constexpr double dManualLowerTimeout = 15.0;
    static constexpr double dAutoRaiseTimeout = 10.0;

    // Motor Speed Constants
    static constexpr double dHomingSpeed = -0.25;
    static constexpr double dManualRaiseSpeed = 0.20;
    static constexpr double dManualLowerSpeed = -0.20;
    static constexpr double dAutoRaiseSpeed = 0.25;

    // Setpoint Constants
    static constexpr double dAutoRaiseSetpoint = 15.0;
    

}