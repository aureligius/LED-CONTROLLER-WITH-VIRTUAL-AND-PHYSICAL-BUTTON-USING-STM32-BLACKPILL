#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* ================================================================
 *  PIN DEFINES
 * ================================================================ */

/* --- LEDs --- */
#define LED1_GPIO_PORT   GPIOB
#define LED1_PIN         GPIO_PIN_9
#define LED2_GPIO_PORT   GPIOB
#define LED2_PIN         GPIO_PIN_7
#define LED3_GPIO_PORT   GPIOB
#define LED3_PIN         GPIO_PIN_5
#define LED4_GPIO_PORT   GPIOB
#define LED4_PIN         GPIO_PIN_3
#define LED5_GPIO_PORT   GPIOA
#define LED5_PIN         GPIO_PIN_10
#define LED6_GPIO_PORT   GPIOA
#define LED6_PIN         GPIO_PIN_8
#define LED7_GPIO_PORT   GPIOB
#define LED7_PIN         GPIO_PIN_14
#define LED8_GPIO_PORT   GPIOB
#define LED8_PIN         GPIO_PIN_12

/* --- Buttons --- */
#define BTN1_GPIO_PORT   GPIOA
#define BTN1_PIN         GPIO_PIN_0
#define BTN1_IRQn        EXTI0_IRQn

#define BTN2_GPIO_PORT   GPIOA
#define BTN2_PIN         GPIO_PIN_1
#define BTN2_IRQn        EXTI1_IRQn

/* --- Potentiometer --- */
#define POT_GPIO_PORT    GPIOA
#define POT_PIN          GPIO_PIN_4
#define POT_ADC_CHANNEL  ADC_CHANNEL_4

/* --- Active Buzzer --- */
#define BUZZ_GPIO_PORT   GPIOB
#define BUZZ_PIN         GPIO_PIN_0

/* --- HC-05 / USART2 --- */
#define BT_USART         USART2
#define BT_TX_PORT       GPIOA
#define BT_TX_PIN        GPIO_PIN_2
#define BT_RX_PORT       GPIOA
#define BT_RX_PIN        GPIO_PIN_3
#define BT_GPIO_AF       GPIO_AF7_USART2

/* --- Note frequencies (Hz) --- */
#define NOTE_G4   392
#define NOTE_A4   440
#define NOTE_B4   494
#define NOTE_C5   523
#define NOTE_D5   587
#define NOTE_G5   784
#define NOTE_REST   0

/* ================================================================
 *  CONSTANTS
 * ================================================================ */
#define CNT_MAX_A        78
#define CNT_MAX_B        8
#define LED_SHIFT_DELAY  150u
#define BTN2_LED_ON_MS   5000u
#define BUZZ_BEEP_MS     100u
#define BTN_DEBOUNCE_MS  50u

/* ================================================================
 *  PERIPHERAL HANDLES
 * ================================================================ */
ADC_HandleTypeDef  hadc1;
UART_HandleTypeDef huart2;
TIM_HandleTypeDef  htim3;   /* PWM melody — PB1 / TIM3_CH4 */

/* ================================================================
 *  GLOBAL STATE
 * ================================================================ */
volatile uint32_t g_counter = 0;

typedef enum {
    MODE_SHIFT   = 1,
    MODE_COUNTER = 2,
    MODE_ADC     = 3,
    MODE_GACOR   = 4
} AppMode_t;

volatile AppMode_t g_appMode        = MODE_SHIFT;
volatile uint8_t   g_btn1PressCount = 1;
volatile uint8_t   g_btn2Active     = 0;
volatile uint8_t   g_modeChanged    = 0;
volatile uint32_t  g_btn1LastMs     = 0;
volatile uint32_t  g_btn2LastMs     = 0;

/* HC-05 RX (single-byte interrupt) */
volatile uint8_t   g_btRxByte = 0;
volatile uint8_t   g_btRxFlag = 0;

static uint8_t s_shiftPos     = 0;
static uint8_t s_counterPhase = 0;

/* ================================================================
 *  FORWARD DECLARATIONS
 * ================================================================ */
static void SystemClock_Config(void);
static void GPIO_Init(void);
static void ADC1_Init(void);
static void USART2_Init(void);
static void MX_TIM3_Init(void);

static void fn_SetLED(uint8_t index, GPIO_PinState state);
static void fn_SetAllLEDs(GPIO_PinState state);
static void fn_DoModeChange(void);
static void fn_ShiftLEDLeft(void);
static void fn_UpdateCounter(void);
static void fn_ADCReadLEDs(void);
static void fn_Buzzer(uint32_t durationMs);
static void fn_BTSendStatus(const char *msg);
static void fn_Button2LEDs(void);
static void fn_PlayTone(uint32_t freq, uint32_t durationMs);
static void fn_GacorNote(uint32_t freq, uint32_t durationMs);
static void fn_PlayJingleBells(void);
static void fn_HandleBTCommand(uint8_t cmd);

/* ================================================================
 *  LED LOOKUP TABLE
 * ================================================================ */
static GPIO_TypeDef *led_ports[8];
static uint16_t      led_pins[8];

static void fn_SetLED(uint8_t index, GPIO_PinState state)
{
    if (index < 8)
        HAL_GPIO_WritePin(led_ports[index], led_pins[index], state);
}

static void fn_SetAllLEDs(GPIO_PinState state)
{
    for (uint8_t i = 0; i < 8; i++)
        fn_SetLED(i, state);
}

/* ================================================================
 *  EXTI CALLBACK
 * ================================================================ */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    uint32_t now = HAL_GetTick();

    if (GPIO_Pin == BTN1_PIN)
    {
        if ((now - g_btn1LastMs) < BTN_DEBOUNCE_MS) return;
        g_btn1LastMs = now;

        g_btn1PressCount = (g_btn1PressCount % 4u) + 1u;
        g_appMode        = (AppMode_t)g_btn1PressCount;
        g_modeChanged    = 1;
        g_counter        = 0;
    }
    else if (GPIO_Pin == BTN2_PIN)
    {
        if ((now - g_btn2LastMs) < BTN_DEBOUNCE_MS) return;
        g_btn2LastMs = now;
        g_btn2Active = 1;
    }
}

/* ================================================================
 *  UART RX COMPLETE — HC-05 single-byte receive
 * ================================================================ */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == BT_USART)
    {
        g_btRxFlag = 1;
        HAL_UART_Receive_IT(&huart2, (uint8_t *)&g_btRxByte, 1);
    }
}

/* ================================================================
 *  MODE CHANGE — centralised teardown + announce
 * ================================================================ */
static void fn_DoModeChange(void)
{
    g_modeChanged  = 0;
    s_shiftPos     = 0;
    s_counterPhase = 0;
    g_counter      = 0;
    fn_SetAllLEDs(GPIO_PIN_RESET);
    HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);

    char buf[32];
    snprintf(buf, sizeof(buf), "MODE=%d\r\n", (int)g_appMode);
    fn_BTSendStatus(buf);
}

/* ================================================================
 *  MODE 1 — LED chases LEFT
 * ================================================================ */
static void fn_ShiftLEDLeft(void)
{
    fn_SetAllLEDs(GPIO_PIN_RESET);
    fn_SetLED(s_shiftPos, GPIO_PIN_SET);

    s_shiftPos = (s_shiftPos + 1u) % 8u;
    if (s_shiftPos == 0)
        fn_Buzzer(BUZZ_BEEP_MS);

    char buf[32];
    snprintf(buf, sizeof(buf), "SHIFT pos=%u\r\n", (unsigned)s_shiftPos);
    fn_BTSendStatus(buf);
}

/* ================================================================
 *  MODE 2 — Alternating sawtooth counter
 * ================================================================ */
static void fn_UpdateCounter(void)
{
    fn_SetAllLEDs(GPIO_PIN_RESET);

    uint32_t currentMax = (s_counterPhase == 0) ? CNT_MAX_A : CNT_MAX_B;

    g_counter++;

    if (g_counter >= currentMax)
    {
        g_counter      = 0;
        s_counterPhase = (s_counterPhase == 0) ? 1 : 0;
    }

    char buf[48];
    snprintf(buf, sizeof(buf), "CNT=%lu MAX=%lu\r\n",
             (unsigned long)g_counter, (unsigned long)currentMax);
    fn_BTSendStatus(buf);
}

/* ================================================================
 *  MODE 3 — Potentiometer → LED bar graph
 * ================================================================ */
static void fn_ADCReadLEDs(void)
{
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
    {
        uint32_t raw = HAL_ADC_GetValue(&hadc1);

        uint8_t ledsOn;
        if      (raw >= 3500) ledsOn = 8;
        else if (raw >= 3000) ledsOn = 7;
        else if (raw >= 2500) ledsOn = 6;
        else if (raw >= 2000) ledsOn = 5;
        else if (raw >= 1500) ledsOn = 4;
        else if (raw >= 1000) ledsOn = 3;
        else if (raw >=  500) ledsOn = 2;
        else if (raw >=  400) ledsOn = 1;
        else                  ledsOn = 0;

        for (uint8_t i = 0; i < 8; i++)
            fn_SetLED(i, (i < ledsOn) ? GPIO_PIN_SET : GPIO_PIN_RESET);

        char buf[40];
        snprintf(buf, sizeof(buf), "ADC=%lu LEDs=%u\r\n",
                 (unsigned long)raw, ledsOn);
        fn_BTSendStatus(buf);
    }
    HAL_ADC_Stop(&hadc1);
}

/* ================================================================
 *  ACTIVE BUZZER  (active-low: LOW = ON)
 * ================================================================ */
static void fn_Buzzer(uint32_t durationMs)
{
    HAL_GPIO_WritePin(BUZZ_GPIO_PORT, BUZZ_PIN, GPIO_PIN_RESET);
    HAL_Delay(durationMs);
    HAL_GPIO_WritePin(BUZZ_GPIO_PORT, BUZZ_PIN, GPIO_PIN_SET);
}

/* ================================================================
 *  HC-05 TX
 * ================================================================ */
static void fn_BTSendStatus(const char *msg)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)msg,
                      (uint16_t)strlen(msg), 100);
}

/* ================================================================
 *  HC-05 RX — command handler
 *  '1' = Mode 1 (Shift)
 *  '2' = Mode 2 (Counter)
 *  '3' = Mode 3 (ADC)
 *  '4' = Mode 4 (Gacor/Jingle Bells loop)
 *  'b' or 'B' = Virtual BTN2 (all LEDs 5s)
 * ================================================================ */
static void fn_HandleBTCommand(uint8_t cmd)
{
    if (cmd >= '1' && cmd <= '4')
    {
        uint8_t newMode = cmd - '0';
        g_btn1PressCount = newMode;
        g_appMode        = (AppMode_t)newMode;
        g_modeChanged    = 1;
        g_counter        = 0;

        char buf[32];
        snprintf(buf, sizeof(buf), "BT: MODE->%u\r\n", newMode);
        fn_BTSendStatus(buf);
    }
    else if (cmd == 'b' || cmd == 'B')
    {
        g_btn2Active = 1;
        fn_BTSendStatus("BT: BTN2 virtual\r\n");
    }
    else
    {
        fn_BTSendStatus("BT: ? send 1/2/3/4 or B\r\n");
    }
}

/* ================================================================
 *  PWM TONE  (PB1 / TIM3_CH4, 1 MHz tick)
 * ================================================================ */
static void fn_PlayTone(uint32_t freq, uint32_t durationMs)
{
    if (freq == NOTE_REST || durationMs == 0)
    {
        HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);
        uint32_t s = HAL_GetTick();
        while ((HAL_GetTick() - s) < durationMs)
        {
            if (g_modeChanged) return;
            if (g_btn2Active)  return;
        }
        return;
    }

    uint32_t period = 1000000UL / freq;
    __HAL_TIM_SET_AUTORELOAD(&htim3, period - 1);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, period / 2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

    uint32_t start = HAL_GetTick();
    while ((HAL_GetTick() - start) < durationMs)
    {
        if (g_modeChanged) { HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4); return; }
        if (g_btn2Active)  { HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4); return; }
    }
    HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);

    /* inter-note gap */
    start = HAL_GetTick();
    while ((HAL_GetTick() - start) < 23)
    {
        if (g_modeChanged) return;
        if (g_btn2Active)  return;
    }
}

/* ================================================================
 *  MODE 4 — Jingle Bells + random LED per note
 * ================================================================ */
static void fn_GacorNote(uint32_t freq, uint32_t durationMs)
{
    if (g_modeChanged) return;
    if (g_btn2Active)  return;
    fn_SetAllLEDs(GPIO_PIN_RESET);
    fn_SetLED((uint8_t)(rand() % 8), GPIO_PIN_SET);
    fn_PlayTone(freq, durationMs);
}

static void fn_PlayJingleBells(void)
{
    /* Verse 1 */
    fn_GacorNote(NOTE_B4, 208); fn_GacorNote(NOTE_B4, 208); fn_GacorNote(NOTE_B4, 416);
    fn_GacorNote(NOTE_B4, 208); fn_GacorNote(NOTE_B4, 208); fn_GacorNote(NOTE_B4, 416);
    fn_GacorNote(NOTE_B4, 208); fn_GacorNote(NOTE_D5, 208); fn_GacorNote(NOTE_G4, 312);
    fn_GacorNote(NOTE_A4, 104); fn_GacorNote(NOTE_B4, 833);
    fn_GacorNote(NOTE_C5, 208); fn_GacorNote(NOTE_C5, 208); fn_GacorNote(NOTE_C5, 312);
    fn_GacorNote(NOTE_C5, 104); fn_GacorNote(NOTE_C5, 208); fn_GacorNote(NOTE_B4, 208);
    fn_GacorNote(NOTE_B4, 208); fn_GacorNote(NOTE_B4, 104); fn_GacorNote(NOTE_B4, 104);
    fn_GacorNote(NOTE_B4, 208); fn_GacorNote(NOTE_A4, 208); fn_GacorNote(NOTE_A4, 208);
    fn_GacorNote(NOTE_B4, 208); fn_GacorNote(NOTE_A4, 416); fn_GacorNote(NOTE_D5, 416);

    if (g_modeChanged || g_btn2Active) return;

    /* Verse 2 */
    fn_GacorNote(NOTE_B4, 208); fn_GacorNote(NOTE_B4, 208); fn_GacorNote(NOTE_B4, 416);
    fn_GacorNote(NOTE_B4, 208); fn_GacorNote(NOTE_B4, 208); fn_GacorNote(NOTE_B4, 416);
    fn_GacorNote(NOTE_B4, 208); fn_GacorNote(NOTE_D5, 208); fn_GacorNote(NOTE_G4, 312);
    fn_GacorNote(NOTE_A4, 104); fn_GacorNote(NOTE_B4, 833);
    fn_GacorNote(NOTE_C5, 208); fn_GacorNote(NOTE_C5, 208); fn_GacorNote(NOTE_C5, 312);
    fn_GacorNote(NOTE_C5, 104); fn_GacorNote(NOTE_C5, 208); fn_GacorNote(NOTE_B4, 208);
    fn_GacorNote(NOTE_B4, 208); fn_GacorNote(NOTE_B4, 104); fn_GacorNote(NOTE_B4, 104);
    fn_GacorNote(NOTE_D5, 208); fn_GacorNote(NOTE_D5, 208); fn_GacorNote(NOTE_C5, 208);
    fn_GacorNote(NOTE_A4, 208); fn_GacorNote(NOTE_G4, 208); fn_GacorNote(NOTE_D5, 208);
    fn_GacorNote(NOTE_G5, 208);
}

/* ================================================================
 *  BUTTON-2 deferred handler (all LEDs on for 5 s)
 * ================================================================ */
static void fn_Button2LEDs(void)
{
    fn_SetAllLEDs(GPIO_PIN_SET);
    fn_Buzzer(BUZZ_BEEP_MS * 2u);
    fn_BTSendStatus("BTN2: ALL LEDs 5s\r\n");
    HAL_Delay(BTN2_LED_ON_MS);
    fn_SetAllLEDs(GPIO_PIN_RESET);
    fn_BTSendStatus("BTN2: DONE\r\n");
}

/* ================================================================
 *  MAIN
 * ================================================================ */
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    GPIO_Init();
    ADC1_Init();
    USART2_Init();
    MX_TIM3_Init();

    /* LED lookup table */
    led_ports[0] = LED1_GPIO_PORT; led_pins[0] = LED1_PIN;
    led_ports[1] = LED2_GPIO_PORT; led_pins[1] = LED2_PIN;
    led_ports[2] = LED3_GPIO_PORT; led_pins[2] = LED3_PIN;
    led_ports[3] = LED4_GPIO_PORT; led_pins[3] = LED4_PIN;
    led_ports[4] = LED5_GPIO_PORT; led_pins[4] = LED5_PIN;
    led_ports[5] = LED6_GPIO_PORT; led_pins[5] = LED6_PIN;
    led_ports[6] = LED7_GPIO_PORT; led_pins[6] = LED7_PIN;
    led_ports[7] = LED8_GPIO_PORT; led_pins[7] = LED8_PIN;

    /* Arm HC-05 RX interrupt */
    HAL_UART_Receive_IT(&huart2, (uint8_t *)&g_btRxByte, 1);

    fn_BTSendStatus("STM32 Ready - MODE=1\r\n");
    fn_Buzzer(BUZZ_BEEP_MS);

    uint32_t lastTick = HAL_GetTick();

    while (1)
    {
        /* --- Process BT input --- */
        if (g_btRxFlag) { g_btRxFlag = 0; fn_HandleBTCommand(g_btRxByte); }

        /* --- Mode change teardown --- */
        if (g_modeChanged) { fn_DoModeChange(); lastTick = HAL_GetTick(); continue; }

        /* --- BTN2 handler (runs in all modes, interrupts GACOR too) --- */
        if (g_btn2Active) { g_btn2Active = 0; fn_Button2LEDs(); }

        /* --- Mode dispatch --- */
        switch (g_appMode)
        {
            case MODE_SHIFT:
                if ((HAL_GetTick() - lastTick) >= LED_SHIFT_DELAY)
                    { lastTick = HAL_GetTick(); fn_ShiftLEDLeft(); }
                break;

            case MODE_COUNTER:
                if ((HAL_GetTick() - lastTick) >= 100u)
                    { lastTick = HAL_GetTick(); fn_UpdateCounter(); }
                break;

            case MODE_ADC:
                if ((HAL_GetTick() - lastTick) >= 50u)
                    { lastTick = HAL_GetTick(); fn_ADCReadLEDs(); }
                break;

            case MODE_GACOR:
                /* Loop Jingle Bells until BTN1 (mode change) or BTN2 */
                while (!g_modeChanged && !g_btn2Active)
                {
                    if (g_btRxFlag) { g_btRxFlag = 0; fn_HandleBTCommand(g_btRxByte); }
                    if (g_modeChanged || g_btn2Active) break;

                    fn_PlayJingleBells();

                    /* 500ms pause between replays, still interruptible */
                    uint32_t pauseStart = HAL_GetTick();
                    while ((HAL_GetTick() - pauseStart) < 500u)
                    {
                        if (g_btRxFlag) { g_btRxFlag = 0; fn_HandleBTCommand(g_btRxByte); }
                        if (g_modeChanged || g_btn2Active) break;
                        HAL_Delay(10);
                    }
                }
                break;
        }
    }
}   /* <-- end of main() */

/* ================================================================
 *  PERIPHERAL INITIALISATION
 * ================================================================ */

static void USART2_Init(void)
{
    __HAL_RCC_USART2_CLK_ENABLE();

    huart2.Instance          = BT_USART;
    huart2.Init.BaudRate     = 9600;
    huart2.Init.WordLength   = UART_WORDLENGTH_8B;
    huart2.Init.StopBits     = UART_STOPBITS_1;
    huart2.Init.Parity       = UART_PARITY_NONE;
    huart2.Init.Mode         = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart2);

    HAL_NVIC_SetPriority(USART2_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
}

static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM       = 25;
    RCC_OscInitStruct.PLL.PLLN       = 336;
    RCC_OscInitStruct.PLL.PLLP       = RCC_PLLP_DIV4;
    RCC_OscInitStruct.PLL.PLLQ       = 7;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { while(1); }

    RCC_ClkInitStruct.ClockType      = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
                                        RCC_CLOCKTYPE_PCLK1  | RCC_CLOCKTYPE_PCLK2);
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) { while(1); }
}

static void GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* --- LEDs: output push-pull --- */
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Pin = LED1_PIN; HAL_GPIO_Init(LED1_GPIO_PORT, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = LED2_PIN; HAL_GPIO_Init(LED2_GPIO_PORT, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = LED3_PIN; HAL_GPIO_Init(LED3_GPIO_PORT, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = LED4_PIN; HAL_GPIO_Init(LED4_GPIO_PORT, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = LED5_PIN; HAL_GPIO_Init(LED5_GPIO_PORT, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = LED6_PIN; HAL_GPIO_Init(LED6_GPIO_PORT, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = LED7_PIN; HAL_GPIO_Init(LED7_GPIO_PORT, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = LED8_PIN; HAL_GPIO_Init(LED8_GPIO_PORT, &GPIO_InitStruct);

    /* --- Buzzer: output, default OFF (HIGH) --- */
    GPIO_InitStruct.Pin = BUZZ_PIN;
    HAL_GPIO_Init(BUZZ_GPIO_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(BUZZ_GPIO_PORT, BUZZ_PIN, GPIO_PIN_SET);

    /* --- BTN1: falling-edge EXTI + pull-up --- */
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Pin  = BTN1_PIN;
    HAL_GPIO_Init(BTN1_GPIO_PORT, &GPIO_InitStruct);
    HAL_NVIC_SetPriority(BTN1_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(BTN1_IRQn);

    /* --- BTN2: falling-edge EXTI + pull-up --- */
    GPIO_InitStruct.Pin = BTN2_PIN;
    HAL_GPIO_Init(BTN2_GPIO_PORT, &GPIO_InitStruct);
    HAL_NVIC_SetPriority(BTN2_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(BTN2_IRQn);

    /* --- Potentiometer: analog --- */
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Pin  = POT_PIN;
    HAL_GPIO_Init(POT_GPIO_PORT, &GPIO_InitStruct);

    /* --- HC-05: USART2 alternate function --- */
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = BT_GPIO_AF;
    GPIO_InitStruct.Pin       = BT_TX_PIN; HAL_GPIO_Init(BT_TX_PORT, &GPIO_InitStruct);
    GPIO_InitStruct.Pin       = BT_RX_PIN; HAL_GPIO_Init(BT_RX_PORT, &GPIO_InitStruct);

    /* --- PB1: TIM3_CH4 AF2 for PWM melody --- */
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    GPIO_InitStruct.Pin       = GPIO_PIN_1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

static void MX_TIM3_Init(void)
{
    __HAL_RCC_TIM3_CLK_ENABLE();

    htim3.Instance               = TIM3;
    htim3.Init.Prescaler         = 83;
    htim3.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim3.Init.Period            = 999;
    htim3.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_PWM_Init(&htim3);

    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode     = TIM_OCMODE_PWM1;
    sConfigOC.Pulse      = 500;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4);
}

static void ADC1_Init(void)
{
    __HAL_RCC_ADC1_CLK_ENABLE();

    hadc1.Instance                   = ADC1;
    hadc1.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution            = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode          = DISABLE;
    hadc1.Init.ContinuousConvMode    = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion       = 1;
    hadc1.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
    HAL_ADC_Init(&hadc1);

    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel      = POT_ADC_CHANNEL;
    sConfig.Rank         = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);
}
