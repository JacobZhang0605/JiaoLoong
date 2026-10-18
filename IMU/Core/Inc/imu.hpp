#ifndef IMU_IMU_HPP
#define IMU_IMU_HPP
#include <stdint.h>
#include "imu_sensor.hpp"

#ifdef __cplusplus
class IMU {
public:
    IMU(IMU_SENSOR* sensor) : sensor_(sensor) {}

    void analyze_data();
    void change_flag();
    int get_flag();
    void calibrate_bias();
    void update_attitude(float dt);
    void update_quaternion(float dt);
    float roll = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;
    float q[4] = {1.0f, 0.0f, 0.0f, 0.0f}; //四元数

private:
    IMU_SENSOR* sensor_;

    struct {
        float accel[3];
        float gyro[3];
    } raw_data_ = {};

    void acc_calculate();
    void gyro_calculate();
    void sensorDataTransform();

    int analyze_flag = 0;

    uint8_t raw_range_accel = 0;
    uint8_t raw_range_gyro = 0;
    uint8_t rx_acc_data[6] = {};
    uint8_t rx_gyro_data[6] = {};
    //旋转矩阵
    float R_imu_[3][3] = {{-1,0,0},{0,1,0},{0,0,1}};
    //零偏
    float accel_bias_[3] = {};
    float gyro_bias_[3]  = {};
    //接受通道
    float accel_sensor_[3] = {0,0,0};
    float gyro_sensor_[3]  = {0,0,0};
    //量程系数
    float accel_sen_ = 0.0f;
    float gyro_sen_ = 0.0f;
    //低通滤波系数和上一次滤波值
    float acc_filter_k = 0.02f;
    float acc_filtered_[3] = {0.0f, 0.0f, 0.0f};
    //校准的一些系数
    int calib_count = 0;
    float gyro_sum[3] = {0, 0, 0};
    float accel_sum[3] = {0, 0, 0};
    bool is_calibrated = false;
    //互补滤波，但出现了欧拉角和世界角的bug
    const float comp_filter_k = 0.98f;
    float pitch_filtered_ = 0.0f;
    float roll_filtered_ = 0.0f;
    //文档里看到的两倍的系数，数学推导目前还在看
    const float two_kp = 0.1f;
};

extern IMU imu;
#endif

#ifdef __cplusplus
extern "C" {
#endif
void IMU_analyze_data(void);
void IMU_change_flag(void);
int IMU_get_flag(void);
void IMU_calibrate_bias(void);
void IMU_update_attitude(float dt);
void IMU_update_quaternion(float dt);
#ifdef __cplusplus
}
#endif

#endif //IMU_IMU_HPP