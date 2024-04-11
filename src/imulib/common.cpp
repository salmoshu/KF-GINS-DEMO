#include "imulib.h"

/**
 * @brief 将IMU数据转换为增量型输出
 *        English Notes
 * */
double rate2inc(double v1, double v2, double dt) {
    return (v1 + (v2 - v1)/2) * dt;
}