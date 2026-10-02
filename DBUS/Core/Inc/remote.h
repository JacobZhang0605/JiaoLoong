#ifndef UART_REMOTE_H
#define UART_REMOTE_H

#include "usart.h"

#ifdef __cplusplus

#include "connect.hpp"

constexpr uint16_t RC_RX_BUF_SIZE = 36u;
constexpr uint16_t RC_FRAME_LEN = 18u;

class Remote {
    enum class RCSwitchState_e {
        UP = 1,
        DOWN = 2,
        MID = 3
    };

    UART_HandleTypeDef *huart_;
    uint8_t rx_buf[RC_RX_BUF_SIZE], rx_data_[RC_FRAME_LEN];
    volatile uint8_t rx_len_;

public:
    // connect state 遥控器连接状态
    Connect connect_;
    // remote channel 遥控器通道
    struct {
        uint16_t l_row;
        uint16_t l_col;
        uint16_t r_row;
        uint16_t r_col;
        uint16_t dial_wheel;
    } __packed channel_;
    // remote switch 遥控器拨挡
    struct {
        RCSwitchState_e l;
        RCSwitchState_e r;
    } __packed switch_;

    explicit Remote(UART_HandleTypeDef *huart);
    ~Remote() = default;

    void init(void);
    void reset(void);
    void rxMsgCallback(void);
    void rxMsgCheck(UART_HandleTypeDef* huart) ;
    void handle(void);
    void send(const uint8_t* data, uint16_t len);
};

extern Remote remote;

#endif  // __cplusplus

#ifdef __cplusplus
extern "C" {
#endif

void Remote_Init(void);
void Remote_reset(void);
void Remote_RxMsgCheck(UART_HandleTypeDef *huart);
void Remote_Send(void);
uint8_t Remote_IsConnected(void);

#ifdef __cplusplus
}
#endif

#endif  // UART_REMOTE_H