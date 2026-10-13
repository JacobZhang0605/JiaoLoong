#include "main.h"
#include "can.h"
#include "tim.h"
#include "Motor.hpp"

volatile float angle = 90.0f;

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    uint8_t data[8];
    if (hcan != &hcan1) return;
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, data) != HAL_OK) return;
    motor.canRxMsgCallback(data);
}

volatile HAL_StatusTypeDef dbg_st;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim != &htim6) return;
    PID_Update();
    float output = motor.control_loop(angle);
    //如果stop_flag是1的话，停止发送报文，但是预估电机还是会因为上电抖动
    if (!stop_flag){
    motor.setTxCurrent(output, 1);
}
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0) dbg_st = HAL_CAN_AddTxMessage(&hcan1, &tx_header, motor.getTxData(), &can_tx_mailbox);

}

extern "C" void Motor_SetTargetAngle(float angle_) {
    angle = angle_;
}
