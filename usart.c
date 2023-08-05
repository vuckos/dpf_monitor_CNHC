/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>,
 * Copyright (C) 2011 Piotr Esden-Tempski <piotr@esden.net>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "main.h"
#include <libopencm3/stm32/dma.h>

//#define USART2_SPEED 115200
#define USART2_SPEED 2000000

void print(const char * str) {
	for (; *str; str++) {
		usart_send_blocking(USART3, *str);
	}
}


/*
print("hello world!" "\r\n");
*/



void usart3_setup(void)
{
    /* Enable the USART2 interrupt. */
    //nvic_enable_irq(NVIC_USART2_IRQ);

    /* Setup GPIO pin GPIO_USART2_TX on GPIO port A for transmit. */
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
        GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART3_TX);

    /* Setup GPIO pin GPIO_USART2_RX on GPIO port A for receive. */
    gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO_USART3_RX);

    /* Setup UART parameters. */
    usart_set_baudrate(USART3, USART2_SPEED);
    usart_set_databits(USART3, 8);
    usart_set_stopbits(USART3, USART_STOPBITS_1);
    usart_set_parity(USART3, USART_PARITY_NONE);
    /* TODO use hardware handshaking */
    usart_set_flow_control(USART3, USART_FLOWCONTROL_NONE);
    usart_set_mode(USART3, USART_MODE_TX_RX);

    /* Enable USART2 Receive interrupt. */
    USART_CR1(USART3) |= USART_CR1_RXNEIE;

    /* Finally enable the USART. */
    usart_enable(USART3);

    //dma_read();
}

void usart2_setup(void)
{
    /* Enable the USART2 interrupt. */
    //nvic_enable_irq(NVIC_USART2_IRQ);

    /* Setup GPIO pin GPIO_USART2_TX on GPIO port A for transmit. */
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
        GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART2_TX);

    /* Setup GPIO pin GPIO_USART2_RX on GPIO port A for receive. */
    gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO_USART2_RX);

    /* Setup UART parameters. */
    usart_set_baudrate(USART2, USART2_SPEED);
    usart_set_databits(USART2, 8);
    usart_set_stopbits(USART2, USART_STOPBITS_1);
    usart_set_parity(USART2, USART_PARITY_NONE);
    /* TODO use hardware handshaking */
    usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);
    usart_set_mode(USART2, USART_MODE_TX_RX);

    /* Enable USART2 Receive interrupt. */
    USART_CR1(USART2) |= USART_CR1_RXNEIE;

    /* Finally enable the USART. */
    usart_enable(USART2);

    //dma_read();
}