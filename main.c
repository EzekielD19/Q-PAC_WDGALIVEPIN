//
// Created by kanderson on 10/2/2024.
//

#include "compilers.h"
#include "stm32h5xx_hal.h"
#include "clock_config.h"
#include "gpio.h"
#include "msp_system.h"
#include "dma.h"
#include "spi.h"
#include "icache.h"
#include "uart.h"
#include "interrupts.h"
#include "common_definitions.h"
#include "msp_1704165_1.h"
#include "adc.h"
#include <stdio.h>
#include "adc_internal.h"
#include <stm32h563xx.h>
#include <stm32h5xx_hal_tim.h>
#include <string.h>



/*--------------------------------------------------------------------------------------------*/
TIM_HandleTypeDef htim2;

#define UC_ALIVE_PORT GPIOE
#define UC_ALIVE_PIN  GPIO_PIN_12


extern UART_Handle uart3;
extern UART_Handle uart1;
  extern uint8_t wdg_expired_flag;
extern const uint8_t wdg_pnt_mssg[];

void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_TIM2_Init(void);

uint8_t console_msg[UART_RX_BUFF] = "UCALIVE OFF!!!\r\n";
uint8_t rs485_msg[UART_RX_BUFF] = "MSP: UART1/RS485 Testing!!!\r\n";
char rs485Buffer[UART_RX_TEST_BUFF];
uint8_t SpiRx_Data[4]={0,0,0,0};

GPIO_Handle led_gpio = { .port = GPIO_PORT_B, .pin = 13 };
GPIO_Handle debug_usage_pin = { .port = GPIO_PORT_E, .pin = 12 };
GPIO_Setup digital_out_pushpull_setup =
{
    .type = GPIO_IO_TYPE_OUTPUT,
    .output_mode = GPIO_OUTPUT_MODE_PUSHPULL,
};

GPIO_Handle pt_cs = { .port = GPIO_PORT_B, .pin = 7 };

GPIO_Setup pt_cs_setup =
{
.type = GPIO_IO_TYPE_OUTPUT,
.output_mode = GPIO_OUTPUT_MODE_PUSHPULL,
.pull_type = GPIO_PULL_TYPE_NOPULL
};

GPIO_Handle dac_cs = { .port = GPIO_PORT_D, .pin = 10 };
GPIO_Setup dac_cs_setup =
{
.type = GPIO_IO_TYPE_OUTPUT,
.output_mode = GPIO_OUTPUT_MODE_PUSHPULL,
};

GPIO_Handle spd_mod_enable_gpio = { .port = GPIO_PORT_B, .pin = 2 };
GPIO_Handle spd_mod_fb_enable_gpio = { .port = GPIO_PORT_E, .pin = 7 };
GPIO_Handle spd_mod_feedback_pin = { .port = GPIO_PORT_B, .pin = 1 };
GPIO_Handle spd_mod_conditioned_pin = { .port = GPIO_PORT_A, .pin = 6 };

ADC_Handle adc1_ch3 =
{
    .adc_ID = ADC_1,
    .channel = MSP_ADC_CHANNEL_3,
    .gpio = &spd_mod_conditioned_pin,
};

ADC_Handle adc1_ch5 =
{
    .adc_ID = ADC_1,
    .channel = MSP_ADC_CHANNEL_5,
    .gpio = &spd_mod_feedback_pin,
};
UART_Setup console_settings =
{
    .rx_pin = &uart3_rx,
    .tx_pin = &uart3_tx,
    .cts_pin = NULL,
    .rts_pin = NULL,
    .baud_rate = SERIAL_BAUD_RATE_115200,
    .parity = PARITY_NONE,
    .stop_bits = SERIAL_STOP_BITS_ONE,
    .word_length = UART_WORDLENGTH_8B,
};

void uart_receive_to_idle_callback(UART_Handle* const uart)
{
    receive_uart_msg_to_idle(&uart1,rs485Buffer, sizeof(rs485Buffer));
}



UART_Setup rs485_settings =
{
    .rx_pin = &uart1_rx,
    .tx_pin = &uart1_tx,
    .rts_pin = &uart1_rts,
    .cts_pin = NULL,
    .baud_rate = SERIAL_BAUD_RATE_115200,
    .parity = PARITY_NONE,
    .stop_bits = SERIAL_STOP_BITS_ONE,
    .word_length = UART_WORDLENGTH_8B,
    .rx_to_idle_callback = &uart_receive_to_idle_callback,
};

void set_rts_pin(void)
{
    gpio_write(&uart1_rts, LOGIC_LOW);
    
}

/*--------------------------------------------------------------------------------------------*/

extern UART_Setup uart1_setup;
extern UART_Setup uart3_setup;

// --- Timer expiration callback ---

// --- Timer setup ---
void MX_TIM2_Init(void)
{
    __HAL_RCC_TIM2_CLK_ENABLE();

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 64000 - 1;  // 64 MHz / 64000 = 1 kHz
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 2000 - 1;      // 1 kHz / 2000 = 2 sec timeout
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_Base_Init(&htim2);

    HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

void MX_GPIO_Init_1(void)
{
    __HAL_RCC_GPIOE_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
 
}

// --- Interrupt handler ---
void TIM2_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim2);
}

void SystemClock_Config(void)
{
    // Placeholder — replace with actual CubeMX-generated clock config
}


int main(void)
{
    //* Interrupt handlers that must be implemented every executable ------------------------------*//
    // DO NOT DELETE THESE. THEY ARE REQUIRED FOR PROPER INTERRUPT HANDLER LINKING
    msp_init_interrupts(); // n.b. DON'T REMOVE. This is defined in the MSP static library. Include "interrupts.h"
    //* Interrupt handlers that must be implemented every executable ------------------------------*//

    msp_init();

    //////////////////////////////////////////////////////////////////////
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init_1();
    MX_TIM2_Init();
   

    
    HAL_TIM_Base_Start_IT(&htim2);
     uart_init(&uart3, &console_settings);
    send_uart_msg(&uart3, console_msg, sizeof(console_msg), 1000);
    HAL_Delay(500); 

   for (;;)
{
    HAL_GPIO_WritePin(UC_ALIVE_PORT, UC_ALIVE_PIN, GPIO_PIN_SET);      
    __HAL_TIM_SET_COUNTER(&htim2, 0);  
    HAL_Delay (500);
  
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        
        HAL_GPIO_WritePin(UC_ALIVE_PORT, UC_ALIVE_PIN, GPIO_PIN_RESET);  // Turn OFF pin
        send_uart_msg(&uart3, console_msg, sizeof(console_msg), 1000);
    }
}
}


