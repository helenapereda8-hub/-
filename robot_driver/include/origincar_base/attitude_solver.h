#ifndef ATTITUDE_SOLVER_H_
#define ATTITUDE_SOLVER_H_

#include <sensor_msgs/msg/imu.hpp>
extern sensor_msgs::msg::Imu imu_global;

void solveAttitude(float gx, float gy, float gz, float ax, float ay, float az);

#endif