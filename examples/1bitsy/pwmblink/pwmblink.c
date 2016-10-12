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

#define TIMER_PERIOD 50908

static void clock_setup(void)
{
    rcc_clock_setup_hse_3v3(&rcc_hse_25mhz_3v3[RCC_CLOCK_3V3_168MHZ]);
}

static void gpiob6_setup(void)
{
    // Enable the clock for GPIOB outputs
    rcc_periph_clock_enable(RCC_GPIOB);
    // Set GPIOB6 to use its alternate function mode with pull-up pull-down
    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO6);
    // AF2 for B6 is TIM4_CH1
    gpio_set_af(GPIOB, GPIO_AF2, GPIO6);
}

static void gpiob1_setup(void)
{
    // Enable the clock for GPIOB outputs
    rcc_periph_clock_enable(RCC_GPIOB);
    // Set GPIOB9 to use its alternate function mode with pull-up pull-down
    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9);
    // AF2 for B9 is TIM4_CH4
    gpio_set_af(GPIOB, GPIO_AF2, GPIO9);
}

static void tim4_setup(void)
{
    /* Enable TIM4 clock. */
    rcc_periph_clock_enable(RCC_TIM4);
    /* Reset TIM4 peripheral. */
    timer_reset(TIM4);
    /* Timer global mode:
    * - No divider
    * - Alignment edge
    * - Direction up
    */
    timer_set_mode(TIM4, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
    timer_set_prescaler(TIM4, 32);
    /* Enable preload. */
    timer_disable_preload(TIM4);
    /* Continous mode. */
    timer_continuous_mode(TIM4);
    /* Period (36kHz). */
    timer_set_period(TIM4, TIMER_PERIOD);
    /* Disable outputs. */
    timer_disable_oc_output(TIM4, TIM_OC1);
    timer_disable_oc_output(TIM4, TIM_OC2);
    timer_disable_oc_output(TIM4, TIM_OC3);
    timer_disable_oc_output(TIM4, TIM_OC4);

    /* -- OC1 configuration -- */
    /* Configure global mode of line 1. */
    timer_disable_oc_clear(TIM4, TIM_OC1);
    timer_enable_oc_preload(TIM4, TIM_OC1);
    timer_set_oc_slow_mode(TIM4, TIM_OC1);
    timer_set_oc_mode(TIM4, TIM_OC1, TIM_OCM_PWM1);
    timer_set_oc_polarity_high(TIM4, TIM_OC1);
    /* Set the capture compare value for OC1 to max value -1 for max duty cycle/brightness. */
    // timer_set_oc_value(TIM4, TIM_OC1, (TIMER_PERIOD/timer_divide));
    timer_enable_oc_output(TIM4, TIM_OC1);

    /* -- OC4 configuration -- */
    /* Configure global mode of line 1. */
    timer_disable_oc_clear(TIM4, TIM_OC4);
    timer_enable_oc_preload(TIM4, TIM_OC4);
    timer_set_oc_slow_mode(TIM4, TIM_OC4);
    timer_set_oc_mode(TIM4, TIM_OC4, TIM_OCM_PWM1);
    timer_set_oc_polarity_high(TIM4, TIM_OC4);
    /* Set the capture compare value for OC1 to max value -1 for max duty cycle/brightness. */
    // timer_set_oc_value(TIM4, TIM_OC4, (TIMER_PERIOD/timer_divide));
    timer_enable_oc_output(TIM4, TIM_OC4);
    timer_enable_preload(TIM4);

    /* Counter enable. */
    timer_enable_counter(TIM4);
}

static void button_setup(void)
{
	/* Enable GPIOC clock. */
	rcc_periph_clock_enable(RCC_GPIOC);

	/* Set GPIO1 (in GPIO port C) to 'input open-drain'. */
	gpio_mode_setup(GPIOC, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO1);
}

int main(void)
{
    uint16_t exti_line_state;

    button_boot();

    clock_setup();
    button_setup();
    gpiob6_setup();
    gpiob1_setup();
    tim4_setup();

    while (1) {
        /* Upon button press, change rotation. */
        exti_line_state = GPIOC_IDR;
        if ((exti_line_state & (1 << 1)) == 0) {
          timer_set_oc_value(TIM4, TIM_OC1, (TIMER_PERIOD/14));
          timer_set_oc_value(TIM4, TIM_OC4, (TIMER_PERIOD/10));
        } else {
          timer_set_oc_value(TIM4, TIM_OC1, (TIMER_PERIOD/10));
          timer_set_oc_value(TIM4, TIM_OC4, (TIMER_PERIOD/14));
        }
    }
    return 0;
}
