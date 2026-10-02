/**
******************************************************************************
 * @file    remote.cpp/h
 * @brief   Remote control. 遥控器
 ******************************************************************************
 * Copyright (c) 2026 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#include "remote.h"
#include <string.h>
#include "callback.h"
Remote remote(&huart3);
constexpr uint16_t REMOTE_CONNECT_TIMEOUT = 500u;
// Constructor 构造函数
Remote::Remote(UART_HandleTypeDef *huart): huart_(huart), connect_(REMOTE_CONNECT_TIMEOUT){
    switch_.l = RCSwitchState_e::DOWN;
    switch_.r = RCSwitchState_e::DOWN;
}

// Start UART(SBUS) receive. 打开UART接收
void Remote::init() {
    __HAL_UART_ENABLE_IT(huart_, UART_IT_IDLE);
    HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
    HAL_UART_Receive_DMA(huart_, rx_buf, RC_RX_BUF_SIZE);
}

// Reset RC data. 重置遥控器数据
void Remote::reset() {
    channel_.l_row=1024;
    channel_.r_row=1024;
    channel_.r_col=1024;
    channel_.l_col=1024;
    channel_.dial_wheel=1024;
    switch_.l=RCSwitchState_e::DOWN;
    switch_.r=RCSwitchState_e::DOWN;
}

// Check for uart correspondence. 检查串口是否匹配
void Remote::rxMsgCheck(UART_HandleTypeDef *huart) {
    if (huart != huart_) return;
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE)) {
        __HAL_UART_CLEAR_IDLEFLAG(huart);
        rxMsgCallback();
    }
}

// Update connect status, restart UART(SBUS) receive.
// 更新连接状态，重新打开UART(SBUS)接收
void Remote::rxMsgCallback() {
    HAL_UART_DMAStop(huart_);
    rx_len_ = RC_RX_BUF_SIZE - __HAL_DMA_GET_COUNTER(huart_->hdmarx);
    if (rx_len_ == RC_FRAME_LEN) {
        memcpy(rx_data_, rx_buf, RC_FRAME_LEN);
        handle();
        connect_.refresh();
    }
    HAL_UART_Receive_DMA(huart_, rx_buf, RC_RX_BUF_SIZE);
}

// Unpack data. 数据解包
void Remote::handle() {
    channel_.l_row = static_cast<uint16_t>((rx_data_[0] | (rx_data_[1] << 8)) & 0x07ff);
    channel_.l_col = static_cast<uint16_t>(((rx_data_[1] >> 3) | (rx_data_[2] << 5)) & 0x07ff);
    channel_.r_row = static_cast<uint16_t>(((rx_data_[2] >> 6) | (rx_data_[3] << 2) | (rx_data_[4] << 10)) & 0x07ff);
    channel_.r_col = static_cast<uint16_t>(((rx_data_[4] >> 1) | (rx_data_[5] << 7)) & 0x07ff);
    uint8_t s1 = (rx_data_[5] >> 4) & 0x03;
    uint8_t s2 = (rx_data_[5] >> 6) & 0x03;
    channel_.dial_wheel = static_cast<uint16_t>((rx_data_[16] | (rx_data_[17] << 8)) & 0x07ff);
    switch_.l = (s1 >= 1 && s1 <= 3) ? (RCSwitchState_e)s1 : RCSwitchState_e::MID;
    switch_.r = (s2 >= 1 && s2 <= 3) ? (RCSwitchState_e)s2 : RCSwitchState_e::MID;
}

void Remote::send(const uint8_t* data, uint16_t len) {
    HAL_UART_Transmit_DMA(&huart1,data, len);
}

extern "C" {
void Remote_Init(void) { remote.init(); }
void Remote_reset(void) { remote.reset(); }
void Remote_RxMsgCheck(UART_HandleTypeDef *huart) { remote.rxMsgCheck(huart); }
void Remote_Send(void) {
    remote.send((const uint8_t*)&remote.channel_, sizeof(remote.channel_));
}
uint8_t Remote_IsConnected(void) {
    if (remote.connect_.check()) return 1;
    return 0;
}
}




