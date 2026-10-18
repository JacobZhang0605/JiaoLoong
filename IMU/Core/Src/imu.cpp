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
        //accel_sensor_[i] -= accel_bias_[i];
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

//这个下午实测有欧拉角和世界角互通的bug，这个函数没用了，讨论下来应该四元数是最稳妥的
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


//四元数处理，0.01745是角换弧，57多的是弧换角
void IMU::update_quaternion(float dt) {
    if (!is_calibrated) return;

    float gyro_x = gyro_sensor_[0] * 0.01745329252f;
    float gyro_y = gyro_sensor_[1] * 0.01745329252f;
    float gyro_z = gyro_sensor_[2] * 0.01745329252f;

    float accel_x = accel_sensor_[0];
    float accel_y = accel_sensor_[1];
    float accel_z = accel_sensor_[2];

    float norm =sqrtf(accel_x*accel_x + accel_y*accel_y + accel_z*accel_z);
    accel_x /= norm; accel_y /= norm; accel_z /= norm;

    float virtual_x = 2.0f * (q[1] * q[3] - q[0] * q[2]);
    float virtual_y = 2.0f * (q[0] * q[1] + q[2] * q[3]);
    float virtual_z = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];

    float error_x = (accel_y * virtual_z - accel_z * virtual_y);
    float error_y = (accel_z * virtual_x - accel_x * virtual_z);
    float error_z = (accel_x * virtual_y - accel_y * virtual_x);
    //为什么是two_kp 还需要再想想，找的文档是这么写的
    gyro_x += two_kp * error_x;
    gyro_y += two_kp * error_y;
    gyro_z += two_kp * error_z;
    //参考$$\dot q=\frac12q\otimes(0,\boldsymbol\omega^b)$$ 把q前面的二分之一给dt
    float half_dt = 0.5f * dt;
    float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];
    //参考$$\tilde q_{k+1}=q_k+\frac12q_k\otimes(0,\boldsymbol\omega_k^b)\Delta t$$
    //这边的叉乘需要把第一个3乘1的虚部的向量变成3乘3的反对称矩阵
    q[0] += (-q1 * gyro_x - q2 * gyro_y - q3 * gyro_z) * half_dt;//实部
    q[1] += ( q0 * gyro_x + q2 * gyro_z - q3 * gyro_y) * half_dt;//虚部
    q[2] += ( q0 * gyro_y - q1 * gyro_z + q3 * gyro_x) * half_dt;//虚
    q[3] += ( q0 * gyro_z + q1 * gyro_y - q2 * gyro_x) * half_dt;//虚
    //下面这行不加的话波形会一直膨胀，问了ai，作用是消除积分误差
    norm =sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    q[0] /= norm; q[1] /= norm; q[2] /= norm; q[3] /= norm;
    //解算
    float sin_pitch = -2.0f * (q[1] * q[3] - q[0] * q[2]);
    //没有限幅有时会返回NAN，讨论得出可能是精度问题，超过了1
    if (sin_pitch > 1.0f)  sin_pitch = 1.0f;
    if (sin_pitch < -1.0f) sin_pitch = -1.0f;
    pitch = asinf(sin_pitch) * 57.29578f;

    roll = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), 1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2])) * 57.29578f;
    yaw = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3])) * 57.29578f;
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
void IMU_update_quaternion(float dt) {
    imu.update_quaternion(dt);
}
}