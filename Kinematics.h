/*
  Copyright (c) 2016, Juan Jimeno
  Source: http://research.ijcaonline.org/volume113/number3/pxc3901586.pdf
  All rights reserved.
  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
   Redistributions of source code must retain the above copyright notice,
  this list of conditions and the following disclaimer.
   Redistributions in binary form must reproduce the above copyright
  notice, this list of conditions and the following disclaimer in the
  documentation and/or other materials provided with the distribution.
   Neither the name of  nor the names of its contributors may be used to
  endorse or promote products derived from this software without specific
  prior written permission.
  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORTPPIPI (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef _KINEMATICS_H_
#define _KINEMATICS_H_

#include "LinoConfig.h"
#include "ArduinoShim.h"

struct rpm
{
    int motor1;
    int motor2;
    int motor3;
    int motor4;
};

struct velocities
{
    float linear_x;
    float linear_y;
    float angular_z;
};

struct rpm calculateRPM(float linear_x, float linear_y, float angular_z)
{
    float linear_vel_x_mins;
    float linear_vel_y_mins;
    float angular_vel_z_mins;
    float tangential_vel;
    float x_rpm;
    float y_rpm;
    float tan_rpm;

    //convert m/s to m/min
    linear_vel_x_mins = linear_x * 60;
    linear_vel_y_mins = linear_y * 60;

    //convert rad/s to rad/min
    angular_vel_z_mins = angular_z * 60;

    tangential_vel = angular_vel_z_mins * ((WHEELS_X_DISTANCE / 2) + (WHEELS_y_DISTANCE / 2));

    x_rpm = linear_vel_x_mins / WHEEL_CIRCUMFERENCE;
    y_rpm = linear_vel_y_mins / WHEEL_CIRCUMFERENCE;
    tan_rpm = tangential_vel / WHEEL_CIRCUMFERENCE;

    struct rpm rpm;

    //calculate for the target motor RPM and direction
    //front-left motor
    rpm.motor1 = x_rpm - y_rpm - tan_rpm;
    rpm.motor1 = constrain(rpm.motor1, -MAX_RPM, MAX_RPM);

    //front-right motor
    rpm.motor2 = x_rpm + y_rpm + tan_rpm;
    rpm.motor2 = constrain(rpm.motor2, -MAX_RPM, MAX_RPM);

    //rear-left motor
    rpm.motor3 = x_rpm + y_rpm - tan_rpm;
    rpm.motor3 = constrain(rpm.motor3, -MAX_RPM, MAX_RPM);

    //rear-right motor
    rpm.motor4 = x_rpm - y_rpm + tan_rpm;
    rpm.motor4 = constrain(rpm.motor4, -MAX_RPM, MAX_RPM);

    return rpm;
}

struct rpm getRPM(float linear_x, float linear_y, float angular_z)
{
    struct rpm rpm;

    #if defined(DIFFERENTIAL_DRIVE) || defined(SKID_STEER)
        rpm = calculateRPM(linear_x, 0.0 , angular_z);

    #elif defined(ACKERMANN)
        rpm = calculateRPM(linear_x, 0.0, 0.0);

    #elif defined(MECANUM)
        rpm = calculateRPM(linear_x, linear_y, angular_z);
    
    #endif
    
    return rpm;
}

struct velocities getVelocities(int motor1, int motor2, int motor3, int motor4)
{
    #if defined(DIFFERENTIAL_DRIVE) || defined(ACKERMANN)
        int total_motors = 2;
        motor3 = 0;
        motor4 = 0;
    
    #elif defined(SKID_STEER) || defined(MECANUM)
        int total_motors = 4;
    #endif

    struct velocities vel;

    float average_rpm_x = (motor1 + motor2 + motor3 + motor4) / total_motors; // RPM
    //convert revolutions per minute to revolutions per second
    float average_rps_x = average_rpm_x / 60; // RPS

    vel.linear_x = average_rps_x * WHEEL_CIRCUMFERENCE; // m/s

    float average_rpm_y = (-motor1 + motor2 + motor3 - motor4) / total_motors; // RPM
    //convert revolutions per minute in y axis to revolutions per second
    float average_rps_y = average_rpm_y / 60; // RPS

    vel.linear_y = average_rps_y * WHEEL_CIRCUMFERENCE; // m/s

    float average_rpm_a = (-motor1 + motor2 - motor3 + motor4) / total_motors;
    //convert revolutions per minute to revolutions per second
    float average_rps_a = average_rpm_a / 60;

    vel.angular_z =  (average_rps_a * WHEEL_CIRCUMFERENCE) / ((WHEELS_X_DISTANCE / 2) + (WHEELS_y_DISTANCE / 2)); //  rad/s

    return vel;
}

#endif