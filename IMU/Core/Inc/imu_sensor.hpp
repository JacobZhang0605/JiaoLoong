#ifndef IMU_IMU_SENSOR_HPP
#define IMU_IMU_SENSOR_HPP

#ifdef __cplusplus

class IMU_SENSOR {
    public:
    virtual void accel_read(uint8_t reg, uint8_t *data, uint8_t len)=0;
    virtual void gyro_read(uint8_t reg, uint8_t *data, uint8_t len)=0;
    virtual ~IMU_SENSOR() {}
};
#endif
#endif //IMU_IMU_SENSOR_HPP
