#include "lemlib/pose.hpp"
#include "subsystems.hpp"
#include "screen.hpp"
#include <string>

#pragma once

namespace lib {

enum class dir {
    Y_PLUS,
    Y_MINUS,
    X_PLUS,
    X_MINUS
};

const float WALL_DISTANCE = 70.5;
const float SENSOR_OFFSET = 5.75;


inline void wallReset(dir direction) {
    switch (direction) {
    
    case dir::Y_PLUS:
        chassis.setPose(chassis.getPose().x, 
                        WALL_DISTANCE - SENSOR_OFFSET - distance1.get_distance() * 0.0393701, 
                        chassis.getPose().theta);
        break;

    case dir::Y_MINUS:
        chassis.setPose(chassis.getPose().x, 
                        -WALL_DISTANCE + SENSOR_OFFSET + distance1.get_distance() * 0.0393701, 
                        chassis.getPose().theta);
        break;

    case dir::X_PLUS:
        chassis.setPose(WALL_DISTANCE - SENSOR_OFFSET - distance1.get_distance() * 0.0393701, 
                        chassis.getPose().y, 
                        chassis.getPose().theta);
        break;
    
    case dir::X_MINUS:
        chassis.setPose(-WALL_DISTANCE + SENSOR_OFFSET + distance1.get_distance() * 0.0393701, 
                        chassis.getPose().y, 
                        chassis.getPose().theta);
        break;
    }
    print(std::to_string(chassis.getPose().x) + ", " + std::to_string(chassis.getPose().y));
}}