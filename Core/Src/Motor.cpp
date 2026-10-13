#include "main.h"
#include "Motor.hpp"
#include "PID.hpp"
#include <math.h>


float Motor::angle()const {
    return angle_;
}

float Motor::speedRpm()const {
    return speedRpm_;
}

float Motor::currentAmps()const {
    return currentA_;
}

float Motor::temperatureC()const {
    return tempC_;
}

bool Motor::hasFeedback() const{
    return received_;
}

float Motor::Get_last_ecd()const {
    return last_ecd_;
};

uint8_t* Motor::getTxData() const {
    return const_cast<uint8_t*>(tx_data_);
}

Motor motor(36.0f);
void Motor::canRxMsgCallback(const uint8_t rx_data[8]) {
    uint8_t* p = const_cast<uint8_t*>(rx_data);
    Cut(p);
}


Motor::Motor(float ratio)
    : ratio_(ratio){}

float Motor::control_loop(float target_angle) {
    float current_angle = angle();
    float current_speed = speedRpm();
    float target_speed = ppid.calc(target_angle, current_angle);
    float output = spid.calc(target_speed, current_speed *(2.0f * 3.1415926f / 60.0f));//都转化成rad/s，其实rpm也可以但是换成rad/s调参简单一点
    output += g_spid_kv_ff * target_speed;
    output += g_ff_gravity * sinf(current_angle * 0.0174532925f);//后面的是2派除以60
    return output;
}

void Motor::Cut(const uint8_t* data) {
    //分割回收的报文，根据的是c610的电调和m2006的电机
    ecd_angle_ =static_cast<int16_t>(data[0]<<8 | data[1]);
    speedRpm_ =static_cast<int16_t>(data[2]<<8 | data[3]);
    currentA_ =static_cast<int16_t>(data[4]<<8 | data[5])*10.0f/10000.0f;
    if (!received_) {
        last_ecd_ = ecd_angle_;
        received_ = true;
        return;
    }
    float diff = ecd_angle_ - last_ecd_;
    if (diff > 4096)       diff -= 8192;
    else if (diff < -4096) diff += 8192;
    angle_ += diff * 360.0f / 8192.0f/ ratio_;
    last_ecd_ = ecd_angle_;
}

void Motor::setTxCurrent(float amperes, uint8_t motor_id) {
    if (amperes >  10.0f) amperes =  10.0f;
    if (amperes < -10.0f) amperes = -10.0f;
    if (motor_id < 1 || motor_id > 4) return;
    int16_t raw = static_cast<int16_t>(amperes * 10000.0f / 10.0f);
    int idx = (motor_id - 1) * 2;
    tx_data_[idx] = (raw >> 8) & 0xFF;
    tx_data_[idx + 1] =  raw   & 0xFF;
    volatile uint8_t c0 = tx_data_[0];
    volatile uint8_t c1 = tx_data_[1];
    volatile int16_t r  = raw;
    volatile int i  = idx;
    (void)c0; (void)c1; (void)r; (void)i;
}

int Motor::change_stop_flag(int flag) {
    if (flag) flag=0;
    else if (!flag) flag=1;
    return flag;
}


extern "C" {
float Motor_control_loop(float target_angle) {
    return motor.control_loop(target_angle);
}
void Motor_SetCurrent(float amperes, uint8_t motor_id) {
    motor.setTxCurrent(amperes,motor_id);
}
int Motor_change_stop_flag(int flag) {
    return motor.change_stop_flag(flag);
}
}