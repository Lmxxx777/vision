#include "robot_status.h"

namespace robot_detection {
    
    void robot_state::updateData(float data[4],int color)
    {
        ab_roll = data[2];
        ab_pitch = data[0];
        ab_yaw = data[1];
        bullet_speed = data[3];
        enemy_color = color;
    }

    void robot_state::updateData(float data[4])
    {
        ab_roll = data[2];
        ab_pitch = data[0];
        ab_yaw = data[1];
        bullet_speed = data[3];
    }

    void robot_state::clone(robot_state &robot)
    {
        ab_roll = robot.ab_roll;
        ab_pitch = robot.ab_pitch;
        ab_yaw = robot.ab_yaw;
        bullet_speed = robot.bullet_speed;
        enemy_color = robot.enemy_color;
    }

}