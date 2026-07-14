#include "robot_driver/attitude_solver.h"
#include <cmath>

#define FILTER_FREQ 20.0f

static float invSqrt(float x) {
    float half = 0.5f * x;
    int i = *(int*)&x;
    i = 0x5f375a86 - (i >> 1);
    x = *(float*)&i;
    x = x * (1.5f - half * x * x);
    return x;
}

static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
static float integral_fx = 0.0f, integral_fy = 0.0f, integral_fz = 0.0f;

void solveAttitude(float gx, float gy, float gz, float ax, float ay, float az)
{
    float recipNorm;
    float vx, vy, vz;
    float ex, ey, ez;

    if (!(ax == 0.0f && ay == 0.0f && az == 0.0f)) {
        recipNorm = invSqrt(ax*ax + ay*ay + az*az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // 估计重力方向
        vx = q1*q3 - q0*q2;
        vy = q0*q1 + q2*q3;
        vz = q0*q0 - 0.5f + q3*q3;

        // 误差
        ex = (ay*vz - az*vy);
        ey = (az*vx - ax*vz);
        ez = (ax*vy - ay*vx);

        // 积分项（Ki 设为 0，可调整）
        float Ki = 0.0f;
        if (Ki > 0.0f) {
            integral_fx += Ki * ex * (1.0f / FILTER_FREQ);
            integral_fy += Ki * ey * (1.0f / FILTER_FREQ);
            integral_fz += Ki * ez * (1.0f / FILTER_FREQ);
            gx += integral_fx;
            gy += integral_fy;
            gz += integral_fz;
        } else {
            integral_fx = integral_fy = integral_fz = 0.0f;
        }

        // 比例项 (Kp=1.0)
        gx += 1.0f * ex;
        gy += 1.0f * ey;
        gz += 1.0f * ez;
    }

    // 积分
    float dt = 1.0f / FILTER_FREQ;
    gx *= 0.5f * dt;
    gy *= 0.5f * dt;
    gz *= 0.5f * dt;

    float qa = q0, qb = q1, qc = q2;
    q0 += (-qb*gx - qc*gy - q3*gz);
    q1 += (qa*gx + qc*gz - q3*gy);
    q2 += (qa*gy - qb*gz + q3*gx);
    q3 += (qa*gz + qb*gy - qc*gx);

    // 归一化
    recipNorm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    imu_global.orientation.w = q0;
    imu_global.orientation.x = q1;
    imu_global.orientation.y = q2;
    imu_global.orientation.z = q3;
}