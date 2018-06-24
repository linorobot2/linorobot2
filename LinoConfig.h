    
#ifndef _LINOCONFIG_H_
#define _LINOCONFIG_H_

#include <math.h>

//uncomment the base you're building
#define DIFFERENTIAL_DRIVE  // 2WD and Tracked robot w/ 2 motors
// #define SKID_STEER       // 4WD robot
// #define ACKERMANN        // Car-like steering robot
// #define MECANUM          // Mecanum drive robot

#define MAX_RPM 330                 // motor's maximum RPM
#define WHEEL_DIAMETER 0.10         // wheel's diameter in meters
#define WHEELS_X_DISTANCE 0.235     // distance between front and rear wheels. Ignore this if you're on 2WD/ACKERMANN
#define WHEELS_y_DISTANCE 0.30      // distance between left and right wheels

const float WHEEL_CIRCUMFERENCE =  M_PI * WHEEL_DIAMETER;

#if defined(DIFFERENTIAL_DRIVE) || defined(ACKERMANN)
    #define WHEELS_X_DISTANCE 0
#endif

#endif