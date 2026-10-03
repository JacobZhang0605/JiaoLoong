#include "main.h"
#include "can.h"
#include "Motor.hpp"
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    uint8_t data[8];
    if (hcan != &hcan1)return;
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header,data)!=HAL_OK) return;
    motor.canRxMsgCallback(data);
}


volatile HAL_StatusTypeDef dbg_st;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim != &htim6) return;
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1)>0) {
       HAL_StatusTypeDef st=HAL_CAN_AddTxMessage(&hcan1,&tx_header,motor.getTxData(),&can_tx_mailbox);
       dbg_st = st;
    }
}