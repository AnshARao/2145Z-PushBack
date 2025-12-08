#pragma once
#include "subsystems.hpp"
#include <math.h>

inline void doNothing() {
    // Intentionally empty
}

inline void measure_offsets() {
    // Number of times to test
    const int iterations = 10;
    // Final offsets
    double vert_offset = 0.0, horz_offset = 0.0;
    double prevdisttravelvert = 0;
    double prevdisttravelhoriz = 0;
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    
    for (int i = 0; i < iterations; i++) {
        // Reset robot position and sensors
        chassis.setPose(0, 0, 0);
        chassis.resetLocalPosition();
        
        // Get initial angle
        double start_angle = chassis.getPose().theta;
        
        // Alternate between turning 90 and 270 degrees
        double target = i % 2 == 0 ? -1 : 1;
        
        chassis.arcade(0, (127/2.0) * target);
        pros::delay(1000);
        chassis.arcade(0, 0);
        motorgroup_L.brake();
        motorgroup_R.brake();
        pros::delay(250);
        
        // Calculate angle change in radians
        double angle_delta = fabs(chassis.getPose().theta - start_angle) * (M_PI / 180.0);
        
        // Get tracker changes
        double vert_delta = odom_V.getDistanceTraveled() - prevdisttravelvert;
        double horz_delta = odom_H.getDistanceTraveled() - prevdisttravelhoriz;

        // Update previous distances
        prevdisttravelvert = odom_V.getDistanceTraveled();
        prevdisttravelhoriz = odom_H.getDistanceTraveled();
        
        // Calculate radius of turn for each tracker
        vert_offset += vert_delta / angle_delta;
        horz_offset += horz_delta / angle_delta;
        pros::delay(10);
    }
    
    // Average the offsets
    vert_offset /= iterations;
    horz_offset /= iterations;
}

inline void tune(){
  chassis.turnToHeading(180, 3000);
  chassis.waitUntilDone();
  
  chassis.turnToHeading(0, 3000);
  chassis.waitUntilDone();  
};

inline void skills() {
  
}