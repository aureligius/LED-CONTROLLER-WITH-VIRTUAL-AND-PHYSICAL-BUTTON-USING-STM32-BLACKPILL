#include "stm32f4xx_hal.h"
#include "stm32f4xx_it.h"

/* ================================================================
 *  External handles declared in main.c
 * ================================================================ */
extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef  htim3;

/* ================================================================
 *  Cortex-M4 core exception handlers
 * ================================================================ */
void NMI_Handler(void)
{
    while (1) {}
}

void HardFault_Handler(void)
{
    while (1) {}
}

void MemManage_Handler(void)
{
    while (1) {}
}

void BusFault_Handler(void)
{
    while (1) {}
}

void UsageFault_Handler(void)
{
    while (1) {}
}

void SVC_Handler(void)
{
}

void DebugMon_Handler(void)
{
}

void PendSV_Handler(void)
{
}

void SysTick_Handler(void)
{
    HAL_IncTick();
}

/* ================================================================
 *  STM32F4xx peripheral interrupt handlers
 * ================================================================ */

/* BTN1 — PA0 */
void EXTI0_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
}

/* BTN2 — PA1 */
void EXTI1_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
}

/* HC-05 Bluetooth UART */
void USART2_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart2);
}
