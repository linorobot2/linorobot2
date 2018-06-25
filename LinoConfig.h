    
#ifndef _LINOCONFIG_H_
#define _LINOCONFIG_H_

#include <math.h>

//uncomment the base you're building
#define LINO_BASE DIFFERENTIAL_DRIVE  // 2WD and Tracked robot w/ 2 motors
// #define LINO_BASE SKID_STEER       // 4WD robot
// #define LINO_BASE ACKERMANN        // Car-like steering robot
// #define LINO_BASE MECANUM          // Mecanum drive robot

#define K_P 0.6 // P constant
#define K_I 0.3 // I constant
#define K_D 0.5 // D constant

#define MAX_RPM 330                 // motor's maximum RPM
#define COUNTS_PER_REV 1550       // wheel encoder's no of ticks per rev
#define WHEEL_DIAMETER 0.10         // wheel's diameter in meters
#define PWM_BITS 8
#define LR_WHEELS_DISTANCE 0.235  // distance between left and right wheels
#define FR_WHEELS_DISTANCE 0.30   // distance between front and rear wheels. Ignore this if you're on 2WD/ACKERMANN

#define PWM_MAX pow(2, PWM_BITS) - 1
#define PWM_MIN -PWM_MAX

#endif