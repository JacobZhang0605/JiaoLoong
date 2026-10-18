#ifndef IMU_IMU_HPP
#define IMU_IMU_HPP
#include <stdint.h>
#include "imu_sensor.hpp" // 新增：引入你的接口

#ifdef __cplusplus
class IMU {
public:
    IMU(IMU_SENSOR* sensor) : sensor_(sensor) {}

    void analyze_data();
    void change_flag();
    int get_flag();
    void calibrate_bias();
    void update_attitude(float dt);
    float roll=0.0f;
    float pitch=0.0f;
    float yaw=0.0f;


private:
    IMU_SENSOR* sensor_;

    // raw data with 3 gyro and 3 accel
    struct {
        float accel[3];
        float gyro[3];
    } raw_data_ = {};

    void acc_calculate();
    void gyro_calculate();
    void sensorDataTransform();

    //flag to detect whether the data analysis is done
    int analyze_flag=0;

    // arrays that contain acc's range and data
    uint8_t raw_range_accel=0;
    uint8_t raw_range_gyro=0;
    uint8_t rx_acc_data[6] = {};
    uint8_t rx_gyro_data[6] = {};

    // rotating matrix
    float R_imu_[3][3] = {{-1,0,0},{0,1,0},{0,0,1}};

    // zero bias
    float accel_bias_[3] = {};
    float gyro_bias_[3]  = {};

    // data after processing
    float accel_sensor_[3] = {0,0,0};
    float gyro_sensor_[3]  = {0,0,0};

    // acceleration and gyroscope sensitivity
    float accel_sen_ = 0.0f;
    float gyro_sen_ = 0.0f;

    // filter
    float acc_filter_k=0.1f;
    float acc_filtered_[3]={0.0f,0.0f,0.0f};

    //zero-bias
    int calib_count = 0;
    float gyro_sum[3] = {0, 0, 0};
    float accel_sum[3] = {0, 0, 0};
    bool is_calibrated = false;

    //complementary filter
    const float comp_filter_k = 0.98f;
    float pitch_filtered_ = 0.0f;
    float roll_filtered_ = 0.0f;
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
#ifdef __cplusplus
}
#endif

#endif //IMU_IMU_HPP