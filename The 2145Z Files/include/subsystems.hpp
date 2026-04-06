#pragma once

/**
* @file subsystems.hpp
* @brief This file contains the definitions and constructors for the robot's subsystems.
* @details This includes the motors, smartwire devices, and three wire port devices.
* @author Ansh Rao - 2145Z
*/

#include <cstdint>  // IWYU pragma: keep
#include <type_traits>
#include "EZ-Template/piston.hpp"
#include "pros/adi.hpp" // IWYU pragma: keep
#include "pros/motors.hpp"
#include "pros/optical.hpp"
#include "api.h"    // IWYU pragma: keep
#include "EZ-Template/api.hpp"  // IWYU pragma: keep
#pragma region constants

// Defining three wire ports
#define PORT_LOADER         'H'
#define PORT_PISTON_SCORER  'C'
#define PORT_WING           'B'
#define PORT_PISTON_PARK    'E'
#define PORT_DESCORE        'D'

// Defining robot constants

#pragma endregion

#pragma region constructors 

