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
#include <math.h>

typedef enum base {DIFFERENTIAL_DRIVE, SKID_STEER, ACKERMANN, MECANUM}base;

typedef struct Kinematics
{
    base lino_base;
    int max_rpm;
    float wheels_x_distance;
    float wheels_y_distance;
    float circumference;    
}Kinematics;

typedef struct RPM
{
    int motor1;
    int motor2;
    int motor3;
    int motor4;
}RPM;

typedef struct Velocities
{
    float linear_x;
    float linear_y;
    float angular_z;
}Velocities;

void newKinematics(Kinematics *kinematics, base base_platform, int max_rpm, float wheel_diameter, 
    float wheels_x_distance, float wheels_y_distance)
{
    kinematics->lino_base = base_platform;
    kinematics->max_rpm = max_rpm;
    kinematics->wheels_x_distance = wheels_x_distance;
    kinematics->wheels_y_distance = wheels_y_distance;
    kinematics->circumference  =  M_PI * wheel_diameter;
}

RPM calculateRPM(Kinematics *kinematics, float linear_x, float linear_y, float angular_z)
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

    tangential_vel = angular_vel_z_mins * ((kinematics->wheels_x_distance / 2) + (kinematics->wheels_y_distance / 2));

    x_rpm = linear_vel_x_mins / kinematics->circumference;
    y_rpm = linear_vel_y_mins / kinematics->circumference;
    tan_rpm = tangential_vel / kinematics->circumference;

    RPM rpm;

    //calculate for the target motor RPM and direction
    //front-left motor
    rpm.motor1 = x_rpm - y_rpm - tan_rpm;
    rpm.motor1 = constrain(rpm.motor1, -kinematics->max_rpm, kinematics->max_rpm);

    //front-right motor
    rpm.motor2 = x_rpm + y_rpm + tan_rpm;
    rpm.motor2 = constrain(rpm.motor2, -kinematics->max_rpm, kinematics->max_rpm);

    //rear-left motor
    rpm.motor3 = x_rpm + y_rpm - tan_rpm;
    rpm.motor3 = constrain(rpm.motor3, -kinematics->max_rpm, kinematics->max_rpm);

    //rear-right motor
    rpm.motor4 = x_rpm - y_rpm + tan_rpm;
    rpm.motor4 = constrain(rpm.motor4, -kinematics->max_rpm, kinematics->max_rpm);

    return rpm;
}

RPM getRPM(Kinematics *kinematics, float linear_x, float linear_y, float angular_z)
{
    RPM rpm;

    if(kinematics->lino_base == DIFFERENTIAL_DRIVE || kinematics->lino_base == SKID_STEER)
        rpm = calculateRPM(kinematics, linear_x, 0.0 , angular_z);

    else if(kinematics->lino_base == ACKERMANN)
        rpm = calculateRPM(kinematics, linear_x, 0.0, 0.0);
    
    else if(kinematics->lino_base == MECANUM)
        rpm = calculateRPM(kinematics, linear_x, linear_y, angular_z);
        
    return rpm;
}

Velocities getVelocities(Kinematics *kinematics, int motor1, int motor2, int motor3, int motor4)
{
    int total_motors;

    if(kinematics->lino_base == DIFFERENTIAL_DRIVE || kinematics->lino_base == ACKERMANN)
    {
        total_motors = 2;
        motor3 = 0;
        motor4 = 0;
    }

    else if(kinematics->lino_base == SKID_STEER || kinematics->lino_base == MECANUM)
    {
        total_motors = 4;
    }

    Velocities vel;

    float average_rpm_x = (motor1 + motor2 + motor3 + motor4) / total_motors; // RPM
    //convert revolutions per minute to revolutions per second
    float average_rps_x = average_rpm_x / 60; // RPS

    vel.linear_x = average_rps_x * kinematics->circumference; // m/s

    float average_rpm_y = (-motor1 + motor2 + motor3 - motor4) / total_motors; // RPM
    //convert revolutions per minute in y axis to revolutions per second
    float average_rps_y = average_rpm_y / 60; // RPS

    vel.linear_y = average_rps_y * kinematics->circumference; // m/s

    float average_rpm_a = (-motor1 + motor2 - motor3 + motor4) / total_motors;
    //convert revolutions per minute to revolutions per second
    float average_rps_a = average_rpm_a / 60;

    vel.angular_z =  (average_rps_a * kinematics->circumference) / ((kinematics->wheels_x_distance / 2) + (kinematics->wheels_y_distance / 2)); //  rad/s

    return vel;
}

#endif