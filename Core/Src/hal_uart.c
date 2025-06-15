/*
 * hal_uart.c
 *
 *  Created on: May 1, 2025
 *      Author: c.monange
 */


#include "hal_uart.h"
#include <string.h> // Pour memcpy ou strlen




// Variables de gestion de la transmission par interruption
typedef struct {
    uint8_t *pTxBuffer;					// buffer de data à transmettre
    uint16_t TxXferSize;				// number of byte to send
    uint16_t TxXferCount;				// number of byte sent
    volatile uint8_t is_transmitting; // Flag pour indiquer si une transmission est en cours
} UART_TxBuffer_t;

UART_TxBuffer_t uart1_tx_data;
UART_TxBuffer_t uart2_tx_data;


#define UART_TXEIE_POS		7
#define UART_TCIE_POS		6



//------------------------------------------------------------------------
//
//------------------------------------------------------------------------
HAL_StatusTypeDef hal_uart1_transmit_IT ( const uint8_t* const li_u8_ptr_tab, const uint16_t li_u16_size){

	if (uart1_tx_data.is_transmitting) {
		return HAL_BUSY; // Une transmission est déjà en cours
	}

	uart1_tx_data.pTxBuffer = (uint8_t *) li_u8_ptr_tab;
	uart1_tx_data.TxXferSize = li_u16_size;
	uart1_tx_data.TxXferCount = li_u16_size;
	uart1_tx_data.is_transmitting = 1;

	// Active l'interruption de registre de transmission vide (TXEIE)
	// C'est cette interruption qui déclenchera l'envoi du premier octet
	USART1->CR1 |= (0x1UL << UART_TXEIE_POS);		//__HAL_UART_ENABLE_IT(huart, UART_IT_TXE);

	return HAL_OK;
}

HAL_StatusTypeDef hal_uart2_transmit_IT ( const uint8_t* const li_u8_ptr_tab, const uint16_t li_u16_size){

	if (uart2_tx_data.is_transmitting) {
		return HAL_BUSY; // Une transmission est déjà en cours
	}

	uart2_tx_data.pTxBuffer = (uint8_t *) li_u8_ptr_tab;
	uart2_tx_data.TxXferSize = li_u16_size;
	uart2_tx_data.TxXferCount = li_u16_size;
	uart2_tx_data.is_transmitting = 1;

	// Active l'interruption de registre de transmission vide (TXEIE)
	// C'est cette interruption qui déclenchera l'envoi du premier octet
	USART2->CR1 |= (0x1UL << UART_TXEIE_POS);		//__HAL_UART_ENABLE_IT(huart, UART_IT_TXE);

	return HAL_OK;
}



//------------------------------------------------------------------------
// -- Gestionnaire d'interruption USART --
//------------------------------------------------------------------------
void hal_uart_USART1_IRQHandler(void) {
	//drv_modbus_slave_rxtUartHandler ( (uint8_t) huart1.Instance->RDR );

	//- RXNE : received byte, read RDR to clear
	if ( 0!= ( USART1->ISR & UART_FLAG_RXNE ) ){
		wrapper_modbus1_drv_modbus_slave_rxtUartHandler ( (uint8_t) USART1->RDR );
	}

	// Vérifie si l'interruption TXE est active et si le drapeau TXE est levé
	// clear par un write dans le registre TDR
	if ( 0!= ( USART1->ISR & UART_FLAG_TXE ) )
	{
        // Envoie l'octet si des données restent à envoyer
        if (uart1_tx_data.TxXferCount > 0) {
        	USART1->TDR = (*uart1_tx_data.pTxBuffer++ & (uint8_t)0xFF);
            uart1_tx_data.TxXferCount--;
            // Si c'est le dernier octet, active l'interruption TCIE pour la fin complète de la transmission
            if (uart1_tx_data.TxXferCount == 0) {
            	USART1->CR1 &= ~(0x1UL << UART_TXEIE_POS);		//	__HAL_UART_DISABLE_IT(&huart1, UART_IT_TXE); // Désactive TXE
            	USART1->CR1 |= (0x1UL << UART_TCIE_POS);		// __HAL_UART_ENABLE_IT(&huart1, UART_IT_TC);   // Active TC
            }
        }
    }

    // Vérifie si l'interruption TC est active et si le drapeau TC est levé
	// clear par soft en écrivant un1  dans TCCF dans le registre  ICR
	if ( 0!= ( USART1->ISR & UART_FLAG_TC ) )
	{
        // La transmission est complètement terminée (dernier octet envoyé hors du registre de décalage)
		USART1->ICR = USART_ICR_TCCF;			 		//__HAL_UART_CLEAR_FLAG(&huart1, UART_FLAG_TC); // Efface le drapeau TC
        USART1->CR1 &= ~(0x1UL << UART_TCIE_POS);   	//__HAL_UART_DISABLE_IT(&huart1, UART_IT_TC);   // Désactive TC
        uart1_tx_data.is_transmitting = 0;           	// Marque la fin de la transmission

        // end of transmission of all bytes
        //drv_modbus_handler_tx_ended ();	// mettre la patte d'activation du driver en IDLE
        wrapper_modbus1_drv_modbus_handler_tx_ended ();
    }
}
void hal_uart_USART2_IRQHandler(void) {
	//drv_modbus_slave_rxtUartHandler ( (uint8_t) huart1.Instance->RDR );

	//- RXNE : received byte, read RDR to clear
	if ( 0!= ( USART2->ISR & UART_FLAG_RXNE ) ){
		wrapper_modbus2_drv_modbus_slave_rxtUartHandler ( (uint8_t) USART2->RDR );
	}

	// Vérifie si l'interruption TXE est active et si le drapeau TXE est levé
	// clear par un write dans le registre TDR
	if ( 0!= ( USART2->ISR & UART_FLAG_TXE ) )
	{
        // Envoie l'octet si des données restent à envoyer
        if (uart2_tx_data.TxXferCount > 0) {
        	USART2->TDR = (*uart2_tx_data.pTxBuffer++ & (uint8_t)0xFF);
            uart2_tx_data.TxXferCount--;
            // Si c'est le dernier octet, active l'interruption TCIE pour la fin complète de la transmission
            if (uart2_tx_data.TxXferCount == 0) {
            	USART2->CR1 &= ~(0x1UL << UART_TXEIE_POS);		//	__HAL_UART_DISABLE_IT(&huart1, UART_IT_TXE); // Désactive TXE
            	USART2->CR1 |= (0x1UL << UART_TCIE_POS);		// __HAL_UART_ENABLE_IT(&huart1, UART_IT_TC);   // Active TC
            }
        }
    }

    // Vérifie si l'interruption TC est active et si le drapeau TC est levé
	// clear par soft en écrivant un1  dans TCCF dans le registre  ICR
	if ( 0!= ( USART2->ISR & UART_FLAG_TC ) )
	{
        // La transmission est complètement terminée (dernier octet envoyé hors du registre de décalage)
		USART2->ICR = USART_ICR_TCCF;			 		//__HAL_UART_CLEAR_FLAG(&huart1, UART_FLAG_TC); // Efface le drapeau TC
        USART2->CR1 &= ~(0x1UL << UART_TCIE_POS);   	//__HAL_UART_DISABLE_IT(&huart1, UART_IT_TC);   // Désactive TC
        uart2_tx_data.is_transmitting = 0;           	// Marque la fin de la transmission

        // end of transmission of all bytes
        wrapper_modbus2_drv_modbus_handler_tx_ended ();		// mettre la patte d'activation du driver en IDLE
    }
}


//------------------------------------------------------------------------
//
//------------------------------------------------------------------------
uint8_t  hal_uart_USART1_get_char (){
	return USART1->RDR;
}
uint8_t  hal_uart_USART2_get_char (){
	return USART2->RDR;
}
//------------------------------------------------------------------------
//
//------------------------------------------------------------------------
void hal_uart_USART1_init (){
	/*
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_TXINVERT_INIT;
    huart1.AdvancedInit.TxPinLevelInvert = UART_ADVFEATURE_TXINV_ENABLE;


    if (HAL_UART_Init(&huart1) != HAL_OK) {
        Error_Handler(); // Gérer l'erreur d'initialisation
    }
    */
}













