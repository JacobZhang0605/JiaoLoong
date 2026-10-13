#include "PID.hpp"
#include "Motor.hpp"

extern "C" {
//所有的全局变量，用来通过scope调试便于更改数据，其中g_spid_kv_ff是速度前馈，g_ff_gravity是重力前馈，不过实测下来速度前馈是零的时候就可以了
volatile float g_spid_kp          = 0.48f;
volatile float g_spid_ki          = 0.004f;
volatile float g_spid_kd          = 0.1f;
volatile float g_spid_i_max       = 35.0f;
volatile float g_spid_out_max     = 5.0f;
volatile float g_spid_d_filter_k  = 0.1f;
volatile float g_spid_kv_ff       = 0.0f;
volatile float g_ff_gravity       = 2.0f;
//accessible answer are as followed: 0.48 0.004 0.1 35.0 5.0 0.1 0.0 2.0
volatile float g_ppid_kp          = 20.0f;
volatile float g_ppid_ki          = 0.0f;
volatile float g_ppid_kd          = 200.0f;
volatile float g_ppid_i_max       = 1000.0f;
volatile float g_ppid_out_max     = 1000.0f;
volatile float g_ppid_d_filter_k  = 0.1f;
}

PID::PID(float kp, float ki, float kd, float i_max, float out_max, float d_filter_k)
    : kp(kp), ki(ki), kd(kd),
      i_max(i_max), out_max(out_max), d_filter_k(d_filter_k) {
}

void PID::setParams(float kp, float ki, float kd, float i_max, float out_max, float d_filter_k) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
    this->i_max = i_max;
    this->out_max = out_max;
    this->d_filter_k = d_filter_k;
}

float PID::calc(float ref, float fdb) {
    error = ref - fdb;

    float p_out = kp * error;

    float i_out = 0.0f;
    if (ki!= 0.0f) {
        error_sum += error;
        i_out = ki * error_sum;
        if (i_out >  i_max) { i_out = i_max; }
        else if (i_out < -i_max) { i_out = -i_max; }
    }

    float d_out_raw =  kd * (error - last_error);
    float d_out = d_out_filter * (1 - d_filter_k) + d_out_raw * d_filter_k;

    float output = p_out + i_out + d_out;
    d_out_filter = d_out;

    if (output >=  out_max) output =  out_max;
    else if (output <= -out_max) output = -out_max;

    last_error = error;
    return output;
}

void PID::reset() {
    error_sum = 0.0f;
    last_error = 0.0f;
    d_out_filter = 0.0f;
}

extern "C" {
void PID_Update(void) {
    //更新一下，方便全局函数用scope调参的时候更改能直接作用到pid的控制
    motor.spid.setParams(g_spid_kp, g_spid_ki, g_spid_kd, g_spid_i_max, g_spid_out_max, g_spid_d_filter_k);
    motor.ppid.setParams(g_ppid_kp, g_ppid_ki, g_ppid_kd, g_ppid_i_max, g_ppid_out_max, g_ppid_d_filter_k);
}

void PID_calc(float ref, float fdb) {
    motor.spid.calc(ref, fdb); //这个没啥用，主体函数挪到callback之后这个没用了，会直接写在control_loop里面
}

void PID_reset() {
    motor.spid.reset();//ppid的reset没写，我觉得应该reset一个就可以了
}
}
