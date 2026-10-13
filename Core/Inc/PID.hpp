#ifndef CAN_PID_HPP
#define CAN_PID_HPP

#include "main.h"

#ifdef __cplusplus
class PID {
public:
    explicit PID(float kp = 0.0f, float ki = 0.0f, float kd = 0.0f,
        float i_max = 0.0f, float out_max = 0.0f, float d_filter_k = 0.0f);
    void reset();
    float calc(float ref, float fdb);
    void setParams(float kp, float ki, float kd,
        float i_max, float out_max, float d_filter_k);

    float kp, ki, kd;

private:
    float i_max, out_max, d_filter_k;
    float error_sum = 0.0f;
    float last_error = 0.0f;
    float error;
    float d_out_filter = 0.0f;
};

#endif

#ifdef __cplusplus
extern "C" {
#endif
void PID_reset();
void PID_calc(float ref, float fdb);
void PID_Update(void);

extern volatile float g_spid_kp, g_spid_ki, g_spid_kd, g_spid_i_max, g_spid_out_max, g_spid_d_filter_k;
extern volatile float g_spid_kv_ff;
extern volatile float g_ff_gravity;
extern volatile float g_ppid_kp, g_ppid_ki, g_ppid_kd, g_ppid_i_max, g_ppid_out_max, g_ppid_d_filter_k;
#ifdef __cplusplus
}
#endif

#endif //CAN_PID_HPP
