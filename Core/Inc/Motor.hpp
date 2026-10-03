#ifndef CAN_MOTOR_HPP
#define CAN_MOTOR_HPP

#include "can.h"
#include "tim.h"
#include "main.h"
#include <stdint.h>

#ifdef __cplusplus

class Motor {
public:
    explicit Motor(float ratio);
    void canRxMsgCallback(const uint8_t rx_data[8]);
    float angle() const { return angle_; }
    float speedRpm() const { return speedRpm_; }
    float currentAmps() const { return currentA_; }
    float temperatureC() const { return tempC_; }
    bool hasFeedback() const { return received_; }
    uint8_t* getTxData() const { return const_cast<uint8_t*>(tx_data_); }
    void PID(float targeted_amperes, uint8_t motor_id);
    float Get_last_ecd()const{return last_ecd_;};
    float Get_angle()const{return angle_;};

private:
    const float ratio_;
    float ecd_angle_ = 0;
    float speedRpm_ = 0;
    float currentA_ = 0;
    float tempC_ = 0;
    float angle_ = 0;
    float last_ecd_ = 0;
    bool received_ = false;
    volatile uint8_t tx_data_[8] = {};
    void Cut(const uint8_t* data);
    void setTxCurrent(float amperes, uint8_t motor_id);
};

extern Motor motor;

#endif /* __cplusplus */


#ifdef __cplusplus
extern "C" {
#endif
extern float error_sum;
void Motor_PID(float targeted_amperes, uint8_t motor_id);
#ifdef __cplusplus
}
#endif

#endif /* CAN_MOTOR_HPP */





































