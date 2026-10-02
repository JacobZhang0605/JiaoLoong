#include "callback.h"

extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart3) {
        Remote_RxMsgCheck(huart);
    }
}