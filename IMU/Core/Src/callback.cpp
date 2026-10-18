#include "imu.hpp"
#include "main.h"
#include "tim.h"
#include "bmi088.hpp"

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
    if (htim!=&htim6) return;
    IMU_analyze_data();
    IMU_calibrate_bias();
    IMU_update_quaternion(0.001);
}