/*
 * hal_uart.h
 *
 *  Created on: May 1, 2025
 *      Author: c.monange
 */

#ifndef SRC_HAL_UART_H_
#define SRC_HAL_UART_H_


#include "stdint.h"
#include "stm32f0xx_hal.h"

	void 				hal_uart_init ();
	HAL_StatusTypeDef 	hal_uart_transmit_IT (const uint8_t* const li_u8_ptr_tab, const uint16_t li_u16_size);
	uint8_t  			hal_uart_USART1_get_char ();
	void 				hal_uart_USART1_IRQHandler(void);




#endif /* SRC_HAL_UART_H_ */
