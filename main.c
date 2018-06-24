#include "HelloWorld.h"
#include <micrortps/client/xrce_client.h>

#include <stdio.h>
#include "Kinematics.h"

void moveBase()
{
    struct rpm target_rpm = getRPM(1, 0, 0);

    int rpm1 = target_rpm.motor1;
    int rpm2 = target_rpm.motor2;
    int rpm3 = target_rpm.motor3;
    int rpm4 = target_rpm.motor4;

    printf( "Motor1 : %d\n", rpm1);
    printf( "Motor2 : %d\n", rpm2);
    printf( "Motor3 : %d\n", rpm3);
    printf( "Motor4 : %d\n", rpm4);

    struct velocities current_vel = getVelocities(rpm1, rpm2, rpm3, rpm4);
    printf( "Linear Velocity X  : %3.6f\n", current_vel.linear_x);
    printf( "Linear Velocity Y  : %3.6f\n", current_vel.linear_y);
    printf( "Angular Velocity Z : %3.6f\n", current_vel.angular_z);
}

#ifdef CONFIG_BUILD_KERNEL
int main(int args, FAR char *argv[])
#else
int linorobot2_main(int args, char *argv[])
#endif
{
    moveBase();
    return 0;
}
