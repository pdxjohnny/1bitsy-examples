/*
* This file is part of the libopencm3 project.
*
* Copyright (C) 2015-2016 Piotr Esden-Tempski <piotr@esden.net>
* Copyright (C) 2015 Jack Ziesing <jziesing@gmail.com>
*
* This library is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This library is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this library. If not, see <http://www.gnu.org/licenses/>.
*/

/*
 * Documentaion used: DocID022063 Rev 7 for STM32F4
 */

#include "../common/button_boot.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/exti.h>
#include <libopencmsis/core_cm3.h>

#define TIMER_CLOCK 81452800
#define TIMER_PERIOD ( TIMER_CLOCK /32/(1/0.020))

#define FRONT TIM4
#define FRONT_ROTATE TIM_OC1
#define FRONT_HIP TIM_OC2
#define FRONT_KNEE TIM_OC3

#define SERVO_0   ( TIMER_CLOCK /32/(1/0.001   ))
#define SERVO_45  ( TIMER_CLOCK /32/(1/0.00125 ))
#define SERVO_90  ( TIMER_CLOCK /32/(1/0.0015  ))
#define SERVO_135 ( TIMER_CLOCK /32/(1/0.00175 ))
#define SERVO_180 ( TIMER_CLOCK /32/(1/0.002   ))

#define FRONT_ROTATE_LEFT SERVO_135
#define FRONT_ROTATE_CENTER SERVO_90
#define FRONT_ROTATE_RIGHT SERVO_45

static void clock_setup(void)
{
    rcc_clock_setup_hse_3v3(&rcc_hse_25mhz_3v3[RCC_CLOCK_3V3_168MHZ]);
}

/*
 * Front three servos
 */
static void gpiob_setup(void)
{
    // Enable the clock for GPIOB outputs
    rcc_periph_clock_enable(RCC_GPIOB);

    // Rotation
    // Set GPIOB6 to use its alternate function mode with pull-up pull-down
    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO6);
    // AF2 for B6 is TIM4_CH1
    gpio_set_af(GPIOB, GPIO_AF2, GPIO6);

    // Hip
    // Set GPIOB7 to use its alternate function mode with pull-up pull-down
    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO7);
    // AF2 for B7 is TIM4_CH2
    gpio_set_af(GPIOB, GPIO_AF2, GPIO7);

    // Knee
    // Set GPIOB8 to use its alternate function mode with pull-up pull-down
    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO8);
    // AF2 for B8 is TIM4_CH3
    gpio_set_af(GPIOB, GPIO_AF2, GPIO8);
}

static void tim4_setup(void)
{
    /* Enable TIM4 clock. */
    rcc_periph_clock_enable(RCC_TIM4);
    /* Reset TIM4 peripheral. */
    timer_reset(FRONT);
    /* Timer global mode:
    * - No divider
    * - Alignment edge
    * - Direction up
    */
    timer_set_mode(FRONT, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
    timer_set_prescaler(FRONT, 32);
    /* Enable preload. */
    timer_disable_preload(FRONT);
    /* Continous mode. */
    timer_continuous_mode(FRONT);
    /* Period (36kHz). */
    timer_set_period(FRONT, TIMER_PERIOD);
    /* Disable outputs. */
    timer_disable_oc_output(FRONT, TIM_OC1);
    timer_disable_oc_output(FRONT, TIM_OC2);
    timer_disable_oc_output(FRONT, TIM_OC3);
    timer_disable_oc_output(FRONT, TIM_OC4);

    /* -- FRONT_ROTATE configuration -- */
    /* Configure global mode of line 1. */
    timer_disable_oc_clear(FRONT, FRONT_ROTATE);
    timer_enable_oc_preload(FRONT, FRONT_ROTATE);
    timer_set_oc_slow_mode(FRONT, FRONT_ROTATE);
    timer_set_oc_mode(FRONT, FRONT_ROTATE, TIM_OCM_PWM1);
    timer_set_oc_polarity_high(FRONT, FRONT_ROTATE);
    /* Set the capture compare value for OC1 to max value -1 for max duty cycle/brightness. */
    // timer_set_oc_value(FRONT, FRONT_ROTATE, (TIMER_PERIOD/timer_divide));
    timer_enable_oc_output(FRONT, FRONT_ROTATE);

    /* -- FRONT_KNEE configuration -- */
    /* Configure global mode of line 1. */
    timer_disable_oc_clear(FRONT, FRONT_HIP);
    timer_enable_oc_preload(FRONT, FRONT_HIP);
    timer_set_oc_slow_mode(FRONT, FRONT_HIP);
    timer_set_oc_mode(FRONT, FRONT_HIP, TIM_OCM_PWM1);
    timer_set_oc_polarity_high(FRONT, FRONT_HIP);
    /* Set the capture compare value for OC1 to max value -1 for max duty cycle/brightness. */
    // timer_set_oc_value(FRONT, FRONT_HIP, (TIMER_PERIOD/timer_divide));
    timer_enable_oc_output(FRONT, FRONT_HIP);
    timer_enable_preload(FRONT);

    /* -- FRONT_KNEE configuration -- */
    /* Configure global mode of line 1. */
    timer_disable_oc_clear(FRONT, FRONT_KNEE);
    timer_enable_oc_preload(FRONT, FRONT_KNEE);
    timer_set_oc_slow_mode(FRONT, FRONT_KNEE);
    timer_set_oc_mode(FRONT, FRONT_KNEE, TIM_OCM_PWM1);
    timer_set_oc_polarity_high(FRONT, FRONT_KNEE);
    /* Set the capture compare value for OC1 to max value -1 for max duty cycle/brightness. */
    // timer_set_oc_value(FRONT, FRONT_KNEE, (TIMER_PERIOD/timer_divide));
    timer_enable_oc_output(FRONT, FRONT_KNEE);
    timer_enable_preload(FRONT);

    /* Counter enable. */
    timer_enable_counter(FRONT);
}

static void button_setup(void)
{
	/* Enable GPIOC clock. */
	rcc_periph_clock_enable(RCC_GPIOC);

	/* Set GPIO1 (in GPIO port C) to 'input open-drain'. */
	gpio_mode_setup(GPIOC, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO1);
}

static void sleep(uint32_t time)
{
    uint32_t i;
    for (i = 0; i < time; ++i) {
        __asm("nop");
    }
}

static void forward_step(uint32_t direction)
{
  timer_set_oc_value(FRONT, FRONT_HIP, SERVO_180);
  sleep(10000000);

  timer_set_oc_value(FRONT, FRONT_ROTATE, direction);
  sleep(10000000);

  timer_set_oc_value(FRONT, FRONT_KNEE, SERVO_45);
  sleep(10000000);

  timer_set_oc_value(FRONT, FRONT_HIP, SERVO_45);
  sleep(10000000);

  timer_set_oc_value(FRONT, FRONT_KNEE, SERVO_90);
  sleep(10000000);
}

int main(void)
{
    uint32_t i;
    uint16_t exti_line_state;

    button_boot();

    clock_setup();
    button_setup();
    gpiob_setup();
    tim4_setup();

    while (1) {
        exti_line_state = GPIOC_IDR;
        if ((exti_line_state & (1 << 1)) == 0) {
          // Take a few steps forward on button press
          sleep(4000000);
          forward_step(FRONT_ROTATE_CENTER);
          sleep(4000000);
          forward_step(FRONT_ROTATE_CENTER);
          // forward_step(FRONT_ROTATE_LEFT);
          sleep(4000000);
          forward_step(FRONT_ROTATE_CENTER);
          // forward_step(FRONT_ROTATE_RIGHT);
          sleep(4000000);
          forward_step(FRONT_ROTATE_CENTER);
        } else {
          timer_set_oc_value(FRONT, FRONT_ROTATE, FRONT_ROTATE_CENTER);
          timer_set_oc_value(FRONT, FRONT_HIP, SERVO_90);
          timer_set_oc_value(FRONT, FRONT_KNEE, SERVO_90);
        }
    }
    return 0;
}
