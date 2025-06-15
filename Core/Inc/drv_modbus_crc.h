/*
 * drv_modbus_crc.h
 *
 *  Created on: May 1, 2025
 *      Author: c.monange
 */

//-----------------------------------------------------------------------
//
//
// Utilization
//
// uint16_t u16_crc = 0;
//
//
// 	u16_crc = crc16_update ( u16_crc, char_value);
// 	u16_crc = crc16_update ( u16_crc, char_value);
// 	u16_crc = crc16_update ( u16_crc, char_value);
// 	u16_crc = crc16_update ( u16_crc, char_value);
//
//
//
//-----------------------------------------------------------------------

#ifndef SRC_DRV_MODBUS_CRC_H_
#define SRC_DRV_MODBUS_CRC_H_



	#include <stdint.h>

	#define DRV_MODBUS_CRC_INIT_VALUE	0xFFFF


	uint16_t 	drv_modbus_crc_CRC16			(const uint8_t* liu8_PtrBuffer, uint16_t liu8_Length);
	uint16_t 	drv_modbus_crc_crc16_update		(uint16_t crc, uint8_t a);
	uint8_t 	drv_modbus_crc_valid 			();


#endif /* SRC_DRV_MODBUS_CRC_H_ */
