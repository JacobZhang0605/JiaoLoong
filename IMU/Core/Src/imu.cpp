#include "imu.hpp"
#include "bmi088.hpp"
#include "imu_sensor.hpp"
#include <math.h>

BMI088 bmi088;
IMU imu(&bmi088);

void IMU::acc_calculate(){
    sensor_->accel_read(0x41, &raw_range_accel, 1);
    switch (raw_range_accel & 0x03) {
        case 0x00:accel_sen_=1.0f/10920.0f; break;
        case 0x01:accel_sen_=1.0f/5460.0f; break;
        case 0x02:accel_sen_=1.0f/2730.0f; break;
        case 0x03:accel_sen_=1.0f/1365.0f; break;
        default:accel_sen_=1.0f/5460.0f; break;
    }

    sensor_->accel_read(0x12, rx_acc_data, 6);
    raw_data_.accel[0] = static_cast<int16_t>(
    (static_cast<uint16_t>(rx_acc_data[1]) << 8) | rx_acc_data[0]
);
    raw_data_.accel[1] = static_cast<int16_t>(
        (static_cast<uint16_t>(rx_acc_data[3]) << 8) | rx_acc_data[2]
    );
    raw_data_.accel[2] = static_cast<int16_t>(
        (static_cast<uint16_t>(rx_acc_data[5]) << 8) | rx_acc_data[4]
    );

}

void IMU::gyro_calculate(){
    sensor_->gyro_read(0x0F, &raw_range_gyro, 1);
    switch (raw_range_gyro & 0x07) {
        case 0x00: gyro_sen_ = 16.384f;  break;
        case 0x01: gyro_sen_ = 32.768f;  break;
        case 0x02: gyro_sen_ = 65.536f;  break;
        case 0x03: gyro_sen_ = 131.072f; break;
        case 0x04: gyro_sen_ = 262.144f; break;
        default: gyro_sen_ = 16.384f; break;
    }

    sensor_->gyro_read(0x02, rx_gyro_data, 6);
    int16_t gyro_raw[3];
    gyro_raw[0] = static_cast<int16_t>((static_cast<uint16_t>(rx_gyro_data[1]) << 8) | rx_gyro_data[0]);
    gyro_raw[1] = static_cast<int16_t>((static_cast<uint16_t>(rx_gyro_data[3]) << 8) | rx_gyro_data[2]);
    gyro_raw[2] = static_cast<int16_t>((static_cast<uint16_t>(rx_gyro_data[5]) << 8) | rx_gyro_data[4]);

    raw_data_.gyro[0] = gyro_raw[0] / gyro_sen_;
    raw_data_.gyro[1] = gyro_raw[1] / gyro_sen_;
    raw_data_.gyro[2] = gyro_raw[2] / gyro_sen_;
}

void IMU::sensorDataTransform() {
    for (uint8_t i = 0; i < 3; i++) {
        gyro_sensor_[i] = (raw_data_.gyro[0] * R_imu_[i][0] +
                           raw_data_.gyro[1] * R_imu_[i][1] +
                           raw_data_.gyro[2] * R_imu_[i][2]) - gyro_bias_[i];

        float accel_temp = accel_sen_ * (raw_data_.accel[0] * R_imu_[i][0] +
                                         raw_data_.accel[1] * R_imu_[i][1] +
                                         raw_data_.accel[2] * R_imu_[i][2]);

        accel_sensor_[i] = acc_filter_k * accel_temp + (1 - acc_filter_k) * acc_filtered_[i];
        acc_filtered_[i] = accel_sensor_[i];
        accel_sensor_[i] -= accel_bias_[i];
    }
    analyze_flag = 1;
}

void IMU::calibrate_bias() {
    if (is_calibrated) return;
    for (int i = 0; i < 3; i++) {
        gyro_sum[i] += raw_data_.gyro[i];

        accel_sum[i] += raw_data_.accel[i] * accel_sen_;
    }
    calib_count++;

    if (calib_count >= 1000) {
        for (int i = 0; i < 3; i++) {
            gyro_bias_[i] = gyro_sum[i] / 1000.0f;
            accel_bias_[i] = accel_sum[i] / 1000.0f;
        }

        accel_bias_[2] = 0.0f;
        is_calibrated = true;
    }
}

void IMU::update_attitude(float dt) {
    float accel_pitch = atan2(-accel_sensor_[0], sqrt(accel_sensor_[1]*accel_sensor_[1] + accel_sensor_[2]*accel_sensor_[2])) * 57.29578f;
    float accel_roll  = atan2(accel_sensor_[1], accel_sensor_[2]) * 57.29578f;

    pitch_filtered_ = comp_filter_k * (pitch_filtered_ + gyro_sensor_[0] * dt) + (1.0f - comp_filter_k) * accel_pitch;
    roll_filtered_  = comp_filter_k * (roll_filtered_  + gyro_sensor_[1] * dt) + (1.0f - comp_filter_k) * accel_roll;

    pitch = pitch_filtered_;
    roll  = roll_filtered_;

    yaw += gyro_sensor_[2] * dt;

    if (yaw >= 180.0f) yaw -= 360.0f;
    if (yaw <= -180.0f) yaw += 360.0f;
}

void IMU::change_flag(){
    analyze_flag = !analyze_flag;
}

int IMU::get_flag() {
    return analyze_flag;
}

void IMU::analyze_data() {
    acc_calculate();
    gyro_calculate();
    sensorDataTransform();
}

extern "C" {
void IMU_analyze_data(void) {
    imu.analyze_data();
}
void IMU_change_flag(void) {
    imu.change_flag();
}
int IMU_get_flag(void) {
    int Flag=imu.get_flag();
    return Flag;
}
void IMU_calibrate_bias(void) {
    imu.calibrate_bias();
}
void IMU_update_attitude(float dt) {
    imu.update_attitude(dt);
}
}