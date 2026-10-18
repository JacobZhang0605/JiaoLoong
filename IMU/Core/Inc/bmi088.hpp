#ifndef BMI088_HPP
#define BMI088_HPP
#include <stdint.h>
#include "IMU_Sensor.hpp"

#ifdef __cplusplus
extern "C" {
#endif

void bmi088_write_byte(uint8_t tx_data);
void bmi088_read_byte(uint8_t *rx_data, uint8_t length);
void bmi088_write_reg(uint8_t reg, uint8_t data);

// 片选控制
void BMI088_ACCEL_NS_L(void);
void BMI088_ACCEL_NS_H(void);
void BMI088_GYRO_NS_L(void);
void BMI088_GYRO_NS_H(void);

// 读写寄存器
void bmi088_accel_write_single_reg(uint8_t reg, uint8_t tx_data);
void bmi088_accel_read_reg(uint8_t reg, uint8_t *rx_data, uint8_t length);
void bmi088_gyro_read_reg(uint8_t reg, uint8_t *rx_data, uint8_t length);
void bmi088_gyro_write_single_reg(uint8_t reg, uint8_t tx_data);

// 初始化
uint8_t bmi088_init(void);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

class BMI088 : public IMU_SENSOR {
public:
    void accel_read(uint8_t reg, uint8_t *data, uint8_t len) override {
        bmi088_accel_read_reg(reg, data, len); //override 父类的虚函数
    }

    void gyro_read(uint8_t reg, uint8_t *data, uint8_t len) override {
        bmi088_gyro_read_reg(reg, data, len);  //同上
    }
};

#endif // __cplusplus

#endif // BMI088_HPP