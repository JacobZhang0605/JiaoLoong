#include "main.h"
#include "tim.h"
#include "iwdg.h"
#include "gpio.h"
extern volatile uint8_t requested_mode;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	HAL_IWDG_Refresh(&hiwdg);
if (GPIO_Pin == KEY_Pin)
{
static uint32_t last_key_tick = 0;
static uint8_t has_last_key_tick = 0;
uint32_t now = HAL_GetTick();
if (!has_last_key_tick || (now - last_key_tick) >= 30U)
{
has_last_key_tick = 1;
last_key_tick = now;
requested_mode = (requested_mode + 1U) % 3U;
}
}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	HAL_IWDG_Refresh(&hiwdg);
    if (htim->Instance == TIM1)
    {
        HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin);
    }

}
