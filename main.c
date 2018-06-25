#include "HelloWorld.h"
#include <micrortps/client/xrce_client.h>

#include <stdio.h>
#include "Kinematics.h"
#include "PID.h"
#include "Motor.h"

#ifdef CONFIG_BUILD_KERNEL
int main(int args, FAR char *argv[])
#else
int linorobot2_main(int args, char *argv[])
#endif
{
    Kinematics kinematics;
    newKinematics(&kinematics, LINO_BASE,  MAX_RPM, WHEEL_DIAMETER, LR_WHEELS_DISTANCE, FR_WHEELS_DISTANCE);

    PID pid1, pid2 ,pid3 ,pid4;
    newPID(&pid1, PWM_MIN, PWM_MAX, K_P, K_I, K_D);
    newPID(&pid2, PWM_MIN, PWM_MAX, K_P, K_I, K_D);
    newPID(&pid3, PWM_MIN, PWM_MAX, K_P, K_I, K_D);
    newPID(&pid4, PWM_MIN, PWM_MAX, K_P, K_I, K_D);

    RPM target_rpm = getRPM(&kinematics, 1, 0, 0);

    int rpm1 = target_rpm.motor1;
    int rpm2 = target_rpm.motor2;
    int rpm3 = target_rpm.motor3;
    int rpm4 = target_rpm.motor4;

    printf( "Motor1 : %d\n", rpm1);
    printf( "Motor2 : %d\n", rpm2);
    printf( "Motor3 : %d\n", rpm3);
    printf( "Motor4 : %d\n", rpm4);

    Velocities current_vel = getVelocities(&kinematics, rpm1, rpm2, rpm3, rpm4);
    printf( "Linear Velocity X  : %3.6f\n", current_vel.linear_x);
    printf( "Linear Velocity Y  : %3.6f\n", current_vel.linear_y);
    printf( "Angular Velocity Z : %3.6f\n", current_vel.angular_z);

    spinMotor(computePID(&pid1, rpm1, 1));
    spinMotor(computePID(&pid2, rpm2, 1));
    spinMotor(computePID(&pid3, rpm3, 1));
    spinMotor(computePID(&pid4, rpm4, 1));

    return 0;
}
