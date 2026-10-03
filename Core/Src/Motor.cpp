#include "Motor.hpp"

float error_sum=0;

Motor motor(19.2f);
void Motor::canRxMsgCallback(const uint8_t rx_data[8]) {
    uint8_t* p = const_cast<uint8_t*>(rx_data);
    Cut(p);
}

Motor::Motor(float ratio) : ratio_(ratio) {}

void Motor::Cut(const uint8_t* data) {
    ecd_angle_ =(data[0]<<8 | data[1])*360.0f/8192.0f;
    speedRpm_ =static_cast<int16_t>(data[2]<<8 | data[3]);
    currentA_ =static_cast<int16_t>(data[4]<<8 | data[5])*20.0f/16384.0f;
    tempC_=static_cast<int8_t>(data[6]);
    if (!received_) {
        last_ecd_ = ecd_angle_;
        received_ = true;
        return;
    }
    float diff = ecd_angle_ - last_ecd_;
    if (diff >  180.0f) diff -= 360.0f;
    if (diff < -180.0f) diff += 360.0f;
    angle_ += diff / ratio_;
    last_ecd_ = ecd_angle_;
}

void Motor::setTxCurrent(float amperes, uint8_t motor_id) {
    if (amperes >  20.0f) amperes =  20.0f;
    if (amperes < -20.0f) amperes = -20.0f;
    if (motor_id < 1 || motor_id > 4) return;
    int16_t raw = static_cast<int16_t>(amperes * 16384.0f / 20.0f);
    int idx = (motor_id - 1) * 2;
    tx_data_[idx] = (raw >> 8) & 0xFF;
    tx_data_[idx + 1] =  raw   & 0xFF;
    volatile uint8_t c0 = tx_data_[0];
    volatile uint8_t c1 = tx_data_[1];  
    volatile int16_t r  = raw;           
    volatile int     i  = idx;          
    (void)c0; (void)c1; (void)r; (void)i; 
}

void Motor::PID(float targeted_amperes,uint8_t motor_id) {
    float kp=0.05,ki=0.1;
    float error=0;float out_amperes;
    error=-targeted_amperes+currentA_;
    error_sum+=error;
    if (error_sum>=3.0f) error_sum=0.0f;
    if (error_sum<-3.0f) error_sum=0.0f;
    out_amperes=kp*error+ki*error_sum+0.9*targeted_amperes;
    if (out_amperes >  targeted_amperes+0.5f) out_amperes = targeted_amperes;
    else if (out_amperes < targeted_amperes-0.5f) out_amperes = targeted_amperes;
    setTxCurrent(out_amperes,motor_id);
}

extern "C" {
void Motor_PID(float targeted_amperes,uint8_t motor_id) {
    motor.PID(targeted_amperes,motor_id);
}
}