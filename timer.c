//
// Created by ssingapati on 9/17/2024.
//
#include "timer.h"
#include "timer_internal.h"

TIM_Setup timer5_setup = {
        .prescaler = TIM5_PRESCALER,      // Prescaler value = 23
        .period = TIMER5_PERIOD,          // Period = 65535
};
uint32_t tim5_get_tick_diff(){
    return tick_diff;
}

MSP_Failure timer_init(TIM_Handle* const timer, TIM_Setup* setup)
{
  TIM_TypeDef* stm_instance = get_tim_instance(timer->number);
  if (stm_instance == NULL)
    return MSP_FAILURE;

  TIM_HandleTypeDef* stm_tim = get_stm_tim(timer);
  if (stm_tim == NULL)
    return MSP_FAILURE;

  stm_tim->Instance = stm_instance;
  tim_init_internal(stm_tim, setup);

  //todo init gpio
  /*
  GPIO_Setup uart_rx_setup =
  {
      .type = GPIO_IO_TYPE_INPUT,
      .input_mode = //todo gpio needs help to make this work
  };
  gpio_init(&uart->rx,
  */

  return MSP_OK;
}

