#ifndef CAN_MOTOR_HPP
#define CAN_MOTOR_HPP

#include "PID.hpp"
#include <stdint.h>

#ifdef __cplusplus

class Motor {
public:
    explicit Motor(float ratio);
    void canRxMsgCallback(const uint8_t rx_data[8]);
    float angle() const;
    float speedRpm() const ;
    float currentAmps() const ;
    float temperatureC() const ;
    bool hasFeedback() const;
    uint8_t* getTxData() const ;
    float Get_last_ecd()const;
    void setTxCurrent(float amperes, uint8_t motor_id);
    float control_loop(float target_angle);
    int change_stop_flag(int flag);
    PID spid;
    PID ppid;

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
};



extern Motor motor;

#endif /* __cplusplus */


#ifdef __cplusplus
extern "C" {
#endif
float Motor_control_loop(float target_angle);
void Motor_SetCurrent(float amperes, uint8_t motor_id);
void Motor_SetTargetAngle(float angle_);
int Motor_change_stop_flag(int flag);
#ifdef __cplusplus
}
#endif

#endif /* CAN_MOTOR_HPP */






























