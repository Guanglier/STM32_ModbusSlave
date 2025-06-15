/*
 * drv_modbus_slave_valid.c
 *
 *  Created on: May 3, 2025
 *      Author: c.monange
 */



#include "stdint.h"
#include "drv_modbus_slave_valid.h"
#include "drv_modbus_crc.h"


#ifdef VALID_MODBUS

#define DRV_MODBUS_SLAVE_VALID_BUFF		20
uint8_t	g_buff_frame_rx[DRV_MODBUS_SLAVE_VALID_BUFF];
uint8_t	g_buff_frame_tx[DRV_MODBUS_SLAVE_VALID_BUFF];
uint8_t	Gu8_CntBytesToCheck;

#define DRV_MODBUS_SLAVE_VALID_ADR		0x12




extern uint8_t drv_modbus_slave_rxtUartHandler ( const uint8_t li_u8_char );


static uint8_t g_buff_frame_ReqToSend[DRV_MODBUS_SLAVE_VALID_BUFF];
static uint8_t g_buff_frame_ExpectedAnswer[DRV_MODBUS_SLAVE_VALID_BUFF];
static uint8_t u8_IndexBuff_ReqToSend;
static uint8_t u8_IndexBuff_ExpectedAnswer;


//-----------------------------------------------------
//		Simulate the sending of the frame
//-----------------------------------------------------
void drv_modbus_slave_valid_SendFrame ( uint8_t *lu8ptr, const uint8_t liu8_NbBytes ){
	uint8_t cnt;

	for ( cnt=0 ; cnt<liu8_NbBytes ; cnt++){
		drv_modbus_slave_rxtUartHandler ( *lu8ptr++ );
	}
}


//-----------------------------------------------------
//
//			Read coils 0x01
//			read reg 006B 2 values slave adress 0x12
//
//			12  01  00  6B  00  02  CE  B4
//			12  01  01  01  94  CC
//
//-----------------------------------------------------
static uint8_t drv_modbus_slave_valid_CRC (){
	uint16_t u16_crc = 0xFFFF;
	uint16_t u16_crc_expected;

	u8_IndexBuff_ReqToSend = 0;
	g_buff_frame_ReqToSend[u8_IndexBuff_ReqToSend++] = 0x12;
	g_buff_frame_ReqToSend[u8_IndexBuff_ReqToSend++] = 0x01;
	g_buff_frame_ReqToSend[u8_IndexBuff_ReqToSend++] = 0x00;
	g_buff_frame_ReqToSend[u8_IndexBuff_ReqToSend++] = 0x6B;
	g_buff_frame_ReqToSend[u8_IndexBuff_ReqToSend++] = 0x00;
	g_buff_frame_ReqToSend[u8_IndexBuff_ReqToSend++] = 0x02;
	g_buff_frame_ReqToSend[u8_IndexBuff_ReqToSend++] = 0xCE;
	g_buff_frame_ReqToSend[u8_IndexBuff_ReqToSend++] = 0xB4;

	for ( uint8_t i=0 ; i<u8_IndexBuff_ReqToSend ; i++){
		u16_crc = drv_modbus_crc_crc16_update ( u16_crc, g_buff_frame_ReqToSend[i] );
	}

	u16_crc_expected = (uint16_t)g_buff_frame_ReqToSend[u8_IndexBuff_ReqToSend-2] << 8;
	u16_crc_expected += (uint16_t)g_buff_frame_ReqToSend[u8_IndexBuff_ReqToSend-2];

	if ( u16_crc == u16_crc_expected ){
		return 0;
	}
	return 1;
}




//-----------------------------------------------------
//
//			Read coils 0x01
//			read reg 006B 2 values slave adress 0x12
//
//			12  01  00  6B  00  02  CE  B4
//			12  01  01  01  94  CC
//
//-----------------------------------------------------
static uint8_t drv_modbus_slave_valid_ReadCoil (){

	u8_IndexBuff_ReqToSend = 0;
	g_buff_frame_ReqToSend[u8_IndexBuff_ReqToSend++] = 0x12;
	g_buff_frame_ReqToSend[u8_IndexBuff_ReqToSend++] = 0x01;
	g_buff_frame_ReqToSend[u8_IndexBuff_ReqToSend++] = 0x00;
	g_buff_frame_ReqToSend[u8_IndexBuff_ReqToSend++] = 0x6B;
	g_buff_frame_ReqToSend[u8_IndexBuff_ReqToSend++] = 0x00;
	g_buff_frame_ReqToSend[u8_IndexBuff_ReqToSend++] = 0x02;
	g_buff_frame_ReqToSend[u8_IndexBuff_ReqToSend++] = 0xCE;
	g_buff_frame_ReqToSend[u8_IndexBuff_ReqToSend++] = 0xB4;

	u8_IndexBuff_ExpectedAnswer = 0;
	g_buff_frame_ExpectedAnswer[u8_IndexBuff_ExpectedAnswer++] = 0x12;
	g_buff_frame_ExpectedAnswer[u8_IndexBuff_ExpectedAnswer++] = 0x01;
	g_buff_frame_ExpectedAnswer[u8_IndexBuff_ExpectedAnswer++] = 0x01;
	g_buff_frame_ExpectedAnswer[u8_IndexBuff_ExpectedAnswer++] = 0x01;
	g_buff_frame_ExpectedAnswer[u8_IndexBuff_ExpectedAnswer++] = 0x94;
	g_buff_frame_ExpectedAnswer[u8_IndexBuff_ExpectedAnswer++] = 0xCC;

	return 0;
}

//-----------------------------------------------------
//
//		Read discrete inputs 0x02
//		read reg 006B 2 values slave adress 0x12
//
//		12  02  00  6B  00  02  8A  B4
//		12  02  01  00  A5  0C
//
//-----------------------------------------------------
static uint8_t drv_modbus_slave_valid_ReadDiscreteInput (){

	u8_IndexBuff_ReqToSend = 0;
	g_buff_frame_ReqToSend[u8_IndexBuff_ReqToSend++] = 0x12;
	g_buff_frame_ReqToSend[u8_IndexBuff_ReqToSend++] = 0x02;
	g_buff_frame_ReqToSend[u8_IndexBuff_ReqToSend++] = 0x00;
	g_buff_frame_ReqToSend[u8_IndexBuff_ReqToSend++] = 0x6B;
	g_buff_frame_ReqToSend[u8_IndexBuff_ReqToSend++] = 0x00;
	g_buff_frame_ReqToSend[u8_IndexBuff_ReqToSend++] = 0x02;
	g_buff_frame_ReqToSend[u8_IndexBuff_ReqToSend++] = 0x8A;
	g_buff_frame_ReqToSend[u8_IndexBuff_ReqToSend++] = 0xB4;

	u8_IndexBuff_ExpectedAnswer = 0;
	g_buff_frame_ExpectedAnswer[u8_IndexBuff_ExpectedAnswer++] = 0x12;
	g_buff_frame_ExpectedAnswer[u8_IndexBuff_ExpectedAnswer++] = 0x02;
	g_buff_frame_ExpectedAnswer[u8_IndexBuff_ExpectedAnswer++] = 0x01;
	g_buff_frame_ExpectedAnswer[u8_IndexBuff_ExpectedAnswer++] = 0x00;
	g_buff_frame_ExpectedAnswer[u8_IndexBuff_ExpectedAnswer++] = 0xA5;
	g_buff_frame_ExpectedAnswer[u8_IndexBuff_ExpectedAnswer++] = 0x0C;

	return 0;
}



//-----------------------------------------------------
//
//		read holding registers 0x03
//		read reg 006B 2 values slave adress 0x12
//
//			12  03  00  6B  00  02  B7  74
//			12  03  04  35  A0  35  A1  01  F4
//
//-----------------------------------------------------
static uint8_t drv_modbus_slave_valid_ReadHoldingReg (){

	u8_IndexBuff_ReqToSend = 0;
	g_buff_frame_ReqToSend[u8_IndexBuff_ReqToSend++] = 0x12;
	g_buff_frame_ReqToSend[u8_IndexBuff_ReqToSend++] = 0x03;
	g_buff_frame_ReqToSend[u8_IndexBuff_ReqToSend++] = 0x00;
	g_buff_frame_ReqToSend[u8_IndexBuff_ReqToSend++] = 0x6B;
	g_buff_frame_ReqToSend[u8_IndexBuff_ReqToSend++] = 0x00;
	g_buff_frame_ReqToSend[u8_IndexBuff_ReqToSend++] = 0x02;
	g_buff_frame_ReqToSend[u8_IndexBuff_ReqToSend++] = 0xB7;
	g_buff_frame_ReqToSend[u8_IndexBuff_ReqToSend++] = 0x74;

	u8_IndexBuff_ExpectedAnswer = 0;
	g_buff_frame_ExpectedAnswer[u8_IndexBuff_ExpectedAnswer++] = 0x12;
	g_buff_frame_ExpectedAnswer[u8_IndexBuff_ExpectedAnswer++] = 0x03;
	g_buff_frame_ExpectedAnswer[u8_IndexBuff_ExpectedAnswer++] = 0x04;
	g_buff_frame_ExpectedAnswer[u8_IndexBuff_ExpectedAnswer++] = 0x35;
	g_buff_frame_ExpectedAnswer[u8_IndexBuff_ExpectedAnswer++] = 0xA0;
	g_buff_frame_ExpectedAnswer[u8_IndexBuff_ExpectedAnswer++] = 0x35;
	g_buff_frame_ExpectedAnswer[u8_IndexBuff_ExpectedAnswer++] = 0xA1;
	g_buff_frame_ExpectedAnswer[u8_IndexBuff_ExpectedAnswer++] = 0x01;
	g_buff_frame_ExpectedAnswer[u8_IndexBuff_ExpectedAnswer++] = 0xF4;

	return 0;
}



//-----------------------------------------------------
//
//		read input registers 0x04
//		read reg 006B 2 values slave adress 0x12
//
//			12  04  00  6B  00  02  02  B4
//			12  04  04  45  A0  45  A1  3F  43
//
//-----------------------------------------------------
static uint8_t drv_modbus_slave_valid_ReadInputReg (){

	u8_IndexBuff_ReqToSend = 0;
	g_buff_frame_ReqToSend[u8_IndexBuff_ReqToSend++] = 0x12;
	g_buff_frame_ReqToSend[u8_IndexBuff_ReqToSend++] = 0x04;
	g_buff_frame_ReqToSend[u8_IndexBuff_ReqToSend++] = 0x00;
	g_buff_frame_ReqToSend[u8_IndexBuff_ReqToSend++] = 0x6B;
	g_buff_frame_ReqToSend[u8_IndexBuff_ReqToSend++] = 0x00;
	g_buff_frame_ReqToSend[u8_IndexBuff_ReqToSend++] = 0x02;
	g_buff_frame_ReqToSend[u8_IndexBuff_ReqToSend++] = 0x02;
	g_buff_frame_ReqToSend[u8_IndexBuff_ReqToSend++] = 0xB4;

	u8_IndexBuff_ExpectedAnswer = 0;
	g_buff_frame_ExpectedAnswer[u8_IndexBuff_ExpectedAnswer++] = 0x12;
	g_buff_frame_ExpectedAnswer[u8_IndexBuff_ExpectedAnswer++] = 0x04;
	g_buff_frame_ExpectedAnswer[u8_IndexBuff_ExpectedAnswer++] = 0x04;
	g_buff_frame_ExpectedAnswer[u8_IndexBuff_ExpectedAnswer++] = 0x45;
	g_buff_frame_ExpectedAnswer[u8_IndexBuff_ExpectedAnswer++] = 0xA0;
	g_buff_frame_ExpectedAnswer[u8_IndexBuff_ExpectedAnswer++] = 0x45;
	g_buff_frame_ExpectedAnswer[u8_IndexBuff_ExpectedAnswer++] = 0xA1;
	g_buff_frame_ExpectedAnswer[u8_IndexBuff_ExpectedAnswer++] = 0x3F;
	g_buff_frame_ExpectedAnswer[u8_IndexBuff_ExpectedAnswer++] = 0x43;

	return 0;
}


//-----------------------------------------------------
//
//		write single coil 0x05
//		read reg 006B value 1 slave adress 0x12
//
//			12  05  00  6B  FF  00  FF  45
//			12  05  00  6B  FF  00  FF  45
//			to do : verify send value different from 0xff00 and 0x0000 -> error !
//-----------------------------------------------------
static uint8_t drv_modbus_slave_valid_WriteSingleCoil (){

	u8_IndexBuff_ReqToSend = 0;
	g_buff_frame_ReqToSend[u8_IndexBuff_ReqToSend++] = 0x12;
	g_buff_frame_ReqToSend[u8_IndexBuff_ReqToSend++] = 0x05;
	g_buff_frame_ReqToSend[u8_IndexBuff_ReqToSend++] = 0x00;
	g_buff_frame_ReqToSend[u8_IndexBuff_ReqToSend++] = 0x6B;
	g_buff_frame_ReqToSend[u8_IndexBuff_ReqToSend++] = 0xFF;
	g_buff_frame_ReqToSend[u8_IndexBuff_ReqToSend++] = 0x00;
	g_buff_frame_ReqToSend[u8_IndexBuff_ReqToSend++] = 0xFF;
	g_buff_frame_ReqToSend[u8_IndexBuff_ReqToSend++] = 0x45;

	u8_IndexBuff_ExpectedAnswer = 0;
	g_buff_frame_ExpectedAnswer[u8_IndexBuff_ExpectedAnswer++] = 0x12;
	g_buff_frame_ExpectedAnswer[u8_IndexBuff_ExpectedAnswer++] = 0x05;
	g_buff_frame_ExpectedAnswer[u8_IndexBuff_ExpectedAnswer++] = 0x00;
	g_buff_frame_ExpectedAnswer[u8_IndexBuff_ExpectedAnswer++] = 0x6B;
	g_buff_frame_ExpectedAnswer[u8_IndexBuff_ExpectedAnswer++] = 0xFF;
	g_buff_frame_ExpectedAnswer[u8_IndexBuff_ExpectedAnswer++] = 0x00;
	g_buff_frame_ExpectedAnswer[u8_IndexBuff_ExpectedAnswer++] = 0xFF;
	g_buff_frame_ExpectedAnswer[u8_IndexBuff_ExpectedAnswer++] = 0x45;

	return 0;
}

//-----------------------------------------------------
//
//		write single register 0x06
//		read reg 006B slave adress 0x12
//
//			12  06  00  6B  12  34  F7  C2
//			12  06  00  6B  12  34  F7  C2
//			to do : verify send value different from 0xff00 and 0x0000 -> error !
//-----------------------------------------------------
static uint8_t drv_modbus_slave_valid_WriteSingleRegister (){

	u8_IndexBuff_ReqToSend = 0;
	g_buff_frame_ReqToSend[u8_IndexBuff_ReqToSend++] = 0x12;
	g_buff_frame_ReqToSend[u8_IndexBuff_ReqToSend++] = 0x06;
	g_buff_frame_ReqToSend[u8_IndexBuff_ReqToSend++] = 0x00;
	g_buff_frame_ReqToSend[u8_IndexBuff_ReqToSend++] = 0x6B;
	g_buff_frame_ReqToSend[u8_IndexBuff_ReqToSend++] = 0x12;
	g_buff_frame_ReqToSend[u8_IndexBuff_ReqToSend++] = 0x34;
	g_buff_frame_ReqToSend[u8_IndexBuff_ReqToSend++] = 0xF7;
	g_buff_frame_ReqToSend[u8_IndexBuff_ReqToSend++] = 0xC2;

	u8_IndexBuff_ExpectedAnswer = 0;
	g_buff_frame_ExpectedAnswer[u8_IndexBuff_ExpectedAnswer++] = 0x12;
	g_buff_frame_ExpectedAnswer[u8_IndexBuff_ExpectedAnswer++] = 0x06;
	g_buff_frame_ExpectedAnswer[u8_IndexBuff_ExpectedAnswer++] = 0x00;
	g_buff_frame_ExpectedAnswer[u8_IndexBuff_ExpectedAnswer++] = 0x6B;
	g_buff_frame_ExpectedAnswer[u8_IndexBuff_ExpectedAnswer++] = 0x12;
	g_buff_frame_ExpectedAnswer[u8_IndexBuff_ExpectedAnswer++] = 0x34;
	g_buff_frame_ExpectedAnswer[u8_IndexBuff_ExpectedAnswer++] = 0xF7;
	g_buff_frame_ExpectedAnswer[u8_IndexBuff_ExpectedAnswer++] = 0xC2;

	return 0;
}



//-----------------------------------------------------
//
//		write multiple coils 0x0F
//		read reg 006B slave adress 0x0F
//		values = 101110111
//
//			12  0F  00  6B  00  09  02  DD  01  AC  07
//			12  0F  00  6B  00  09  E6  B2
//			to do : verify send value different from 0xff00 and 0x0000 -> error !
//-----------------------------------------------------
static uint8_t drv_modbus_slave_valid_WriteMultipleCoils (){

	u8_IndexBuff_ReqToSend = 0;
	g_buff_frame_ReqToSend[u8_IndexBuff_ReqToSend++] = 0x12;
	g_buff_frame_ReqToSend[u8_IndexBuff_ReqToSend++] = 0x0F;
	g_buff_frame_ReqToSend[u8_IndexBuff_ReqToSend++] = 0x00;
	g_buff_frame_ReqToSend[u8_IndexBuff_ReqToSend++] = 0x6B;
	g_buff_frame_ReqToSend[u8_IndexBuff_ReqToSend++] = 0x00;
	g_buff_frame_ReqToSend[u8_IndexBuff_ReqToSend++] = 0x09;
	g_buff_frame_ReqToSend[u8_IndexBuff_ReqToSend++] = 0x02;
	g_buff_frame_ReqToSend[u8_IndexBuff_ReqToSend++] = 0xDD;
	g_buff_frame_ReqToSend[u8_IndexBuff_ReqToSend++] = 0x01;
	g_buff_frame_ReqToSend[u8_IndexBuff_ReqToSend++] = 0xAC;
	g_buff_frame_ReqToSend[u8_IndexBuff_ReqToSend++] = 0x07;

	u8_IndexBuff_ExpectedAnswer = 0;
	g_buff_frame_ExpectedAnswer[u8_IndexBuff_ExpectedAnswer++] = 0x12;
	g_buff_frame_ExpectedAnswer[u8_IndexBuff_ExpectedAnswer++] = 0x0F;
	g_buff_frame_ExpectedAnswer[u8_IndexBuff_ExpectedAnswer++] = 0x00;
	g_buff_frame_ExpectedAnswer[u8_IndexBuff_ExpectedAnswer++] = 0x6B;
	g_buff_frame_ExpectedAnswer[u8_IndexBuff_ExpectedAnswer++] = 0x00;
	g_buff_frame_ExpectedAnswer[u8_IndexBuff_ExpectedAnswer++] = 0x09;
	g_buff_frame_ExpectedAnswer[u8_IndexBuff_ExpectedAnswer++] = 0xE6;
	g_buff_frame_ExpectedAnswer[u8_IndexBuff_ExpectedAnswer++] = 0xB2;

	return 0;
}

//-----------------------------------------------------
//
//		write multiple registers 0x10
//		read reg 006B slave adress 0x0F
//		values = 101110111
//
//	Req:	12  10  00  6B  00  09  12  12  34  54  68  78  94  78
//				21  48  23  14  50  10  02  35  87  04  56  BC  C2
//	values are : 0x1234 0x5468 0x7894 0x7821 0x4823 0x1450 0x1002 0x3587 0x0456
//
//	Aswr:	12  10  00  6B  00  09  73  70
//
//-----------------------------------------------------
static uint8_t drv_modbus_slave_valid_WriteMultipleRegister (){

	u8_IndexBuff_ReqToSend = 0;
	g_buff_frame_ReqToSend[u8_IndexBuff_ReqToSend++] = 0x12;
	g_buff_frame_ReqToSend[u8_IndexBuff_ReqToSend++] = 0x10;
	g_buff_frame_ReqToSend[u8_IndexBuff_ReqToSend++] = 0x00;
	g_buff_frame_ReqToSend[u8_IndexBuff_ReqToSend++] = 0x6B;
	g_buff_frame_ReqToSend[u8_IndexBuff_ReqToSend++] = 0x00;
	g_buff_frame_ReqToSend[u8_IndexBuff_ReqToSend++] = 0x09;
	g_buff_frame_ReqToSend[u8_IndexBuff_ReqToSend++] = 0x12;
	g_buff_frame_ReqToSend[u8_IndexBuff_ReqToSend++] = 0x12;
	g_buff_frame_ReqToSend[u8_IndexBuff_ReqToSend++] = 0x34;
	g_buff_frame_ReqToSend[u8_IndexBuff_ReqToSend++] = 0x54;
	g_buff_frame_ReqToSend[u8_IndexBuff_ReqToSend++] = 0x68;
	g_buff_frame_ReqToSend[u8_IndexBuff_ReqToSend++] = 0x78;
	g_buff_frame_ReqToSend[u8_IndexBuff_ReqToSend++] = 0x94;
	g_buff_frame_ReqToSend[u8_IndexBuff_ReqToSend++] = 0x78;
	g_buff_frame_ReqToSend[u8_IndexBuff_ReqToSend++] = 0x21;
	g_buff_frame_ReqToSend[u8_IndexBuff_ReqToSend++] = 0x48;
	g_buff_frame_ReqToSend[u8_IndexBuff_ReqToSend++] = 0x23;
	g_buff_frame_ReqToSend[u8_IndexBuff_ReqToSend++] = 0x14;
	g_buff_frame_ReqToSend[u8_IndexBuff_ReqToSend++] = 0x50;
	g_buff_frame_ReqToSend[u8_IndexBuff_ReqToSend++] = 0x10;
	g_buff_frame_ReqToSend[u8_IndexBuff_ReqToSend++] = 0x02;
	g_buff_frame_ReqToSend[u8_IndexBuff_ReqToSend++] = 0x35;
	g_buff_frame_ReqToSend[u8_IndexBuff_ReqToSend++] = 0x87;
	g_buff_frame_ReqToSend[u8_IndexBuff_ReqToSend++] = 0x04;
	g_buff_frame_ReqToSend[u8_IndexBuff_ReqToSend++] = 0x56;
	g_buff_frame_ReqToSend[u8_IndexBuff_ReqToSend++] = 0xBC;
	g_buff_frame_ReqToSend[u8_IndexBuff_ReqToSend++] = 0xC2;

	u8_IndexBuff_ExpectedAnswer = 0;
	g_buff_frame_ExpectedAnswer[u8_IndexBuff_ExpectedAnswer++] = 0x12;
	g_buff_frame_ExpectedAnswer[u8_IndexBuff_ExpectedAnswer++] = 0x10;
	g_buff_frame_ExpectedAnswer[u8_IndexBuff_ExpectedAnswer++] = 0x00;
	g_buff_frame_ExpectedAnswer[u8_IndexBuff_ExpectedAnswer++] = 0x6B;
	g_buff_frame_ExpectedAnswer[u8_IndexBuff_ExpectedAnswer++] = 0x00;
	g_buff_frame_ExpectedAnswer[u8_IndexBuff_ExpectedAnswer++] = 0x09;
	g_buff_frame_ExpectedAnswer[u8_IndexBuff_ExpectedAnswer++] = 0x73;
	g_buff_frame_ExpectedAnswer[u8_IndexBuff_ExpectedAnswer++] = 0x70;

	return 0;
}




//---------------------------------------------------------------------------------
//	READ Holding 3 register
//
//	Request : 06 03   00 6B   00 03   CR CR
//	Answer :  06 03   06   xx xx   xx xx   xx xx   CR CR
//---------------------------------------------------------------------------------
static uint8_t u8_drv_modbus_slave_valid__scernario_1 (){
	uint8_t 	u8_IndexBuff = 0;
	uint8_t 	u8_IndexBuffTX = 0;
	uint16_t	u16_reg_adr = 0x006B;
	uint16_t	u16_reg_cnt = 0x0003;
	uint16_t	u16_crc = 0xFFFF;


	// prepare the check of response
	g_buff_frame_tx[u8_IndexBuffTX++] = DRV_MODBUS_SLAVE_VALID_ADR;		//address
	g_buff_frame_tx[u8_IndexBuffTX++] = 0x03;			//function code
	g_buff_frame_tx[u8_IndexBuffTX++] = 0x06;			// adr lsb
	g_buff_frame_tx[u8_IndexBuffTX++] = 0x00;
	g_buff_frame_tx[u8_IndexBuffTX++] = 0xAE;
	g_buff_frame_tx[u8_IndexBuffTX++] = 0x00;
	g_buff_frame_tx[u8_IndexBuffTX++] = 0x12;
	g_buff_frame_tx[u8_IndexBuffTX++] = 0x00;
	g_buff_frame_tx[u8_IndexBuffTX++] = 0x32;
	for ( uint8_t i=0 ; i<u8_IndexBuffTX ; i++){
		u16_crc = drv_modbus_crc_crc16_update ( u16_crc, g_buff_frame_tx[i] );
	}
	g_buff_frame_tx[u8_IndexBuffTX++] = (uint8_t) (u16_crc >> 8);
	g_buff_frame_tx[u8_IndexBuffTX++] = (uint8_t) (u16_crc & 0x00FF);
	Gu8_CntBytesToCheck = u8_IndexBuffTX;

	// preprae the frame to be sent
	u16_crc = 0xFFFF;
	g_buff_frame_rx[u8_IndexBuff++] = DRV_MODBUS_SLAVE_VALID_ADR;
	g_buff_frame_rx[u8_IndexBuff++] = 0x03;
	g_buff_frame_rx[u8_IndexBuff++] = (uint8_t) (u16_reg_adr >> 8);
	g_buff_frame_rx[u8_IndexBuff++] = (uint8_t) (u16_reg_adr & 0x00FF);

	g_buff_frame_rx[u8_IndexBuff++] = (uint8_t) (u16_reg_cnt >> 8);
	g_buff_frame_rx[u8_IndexBuff++] = (uint8_t) (u16_reg_cnt & 0x00FF);

	for ( uint8_t i=0 ; i<u8_IndexBuff ; i++){
		u16_crc = drv_modbus_crc_crc16_update ( u16_crc, g_buff_frame_rx[i] );
	}
	g_buff_frame_rx[u8_IndexBuff++] = (uint8_t) (u16_crc >> 8);
	g_buff_frame_rx[u8_IndexBuff++] = (uint8_t) (u16_crc & 0x00FF);

	// send the frame request
	for ( uint8_t cnt=0 ; cnt<u8_IndexBuff ; cnt++){
		drv_modbus_slave_rxtUartHandler ( g_buff_frame_rx[cnt] );
	}



	return 0;
}





//---------------------------------------------------------------------------------
//	READ Holding 3 register
//
//	Request : 06 03   00 6B   00 03   CR CR
//	Answer :  06 03   06   xx xx   xx xx   xx xx   CR CR
//---------------------------------------------------------------------------------
static uint8_t u8_drv_modbus_slave_valid__scernario_2 (){
	uint8_t 	u8_IndexBuff = 0;
	uint8_t 	u8_IndexBuffTX = 0;
	uint16_t	u16_reg_adr = 0x01C4;
	uint16_t	u16_reg_cnt = 0x0002;
	uint16_t	u16_crc = 0xFFFF;

/*
	// prepare the check of response
	g_buff_frame_tx[u8_IndexBuffTX++] = DRV_MODBUS_SLAVE_VALID_ADR;
	g_buff_frame_tx[u8_IndexBuffTX++] = 0x03;
	g_buff_frame_tx[u8_IndexBuffTX++] = 0xC4;
	g_buff_frame_tx[u8_IndexBuffTX++] = 0x01;
	g_buff_frame_tx[u8_IndexBuffTX++] = 0x02;
	g_buff_frame_tx[u8_IndexBuffTX++] = 0x00;
	g_buff_frame_tx[u8_IndexBuffTX++] = 0xA9;
	g_buff_frame_tx[u8_IndexBuffTX++] = 0x86;
	for ( uint8_t i=0 ; i<u8_IndexBuffTX ; i++){
		u16_crc = drv_modbus_crc_crc16_update ( u16_crc, g_buff_frame_tx[i] );
	}
	g_buff_frame_tx[u8_IndexBuffTX++] = (uint8_t) (u16_crc >> 8);
	g_buff_frame_tx[u8_IndexBuffTX++] = (uint8_t) (u16_crc & 0x00FF);
	Gu8_CntBytesToCheck = u8_IndexBuffTX;
	*/

	// preprae the frame to be sent
	u16_crc = 0xFFFF;
	g_buff_frame_rx[u8_IndexBuff++] = DRV_MODBUS_SLAVE_VALID_ADR;
	g_buff_frame_rx[u8_IndexBuff++] = 0x03;
	g_buff_frame_rx[u8_IndexBuff++] = (uint8_t) (u16_reg_adr >> 8);
	g_buff_frame_rx[u8_IndexBuff++] = (uint8_t) (u16_reg_adr & 0x00FF);

	g_buff_frame_rx[u8_IndexBuff++] = (uint8_t) (u16_reg_cnt >> 8);
	g_buff_frame_rx[u8_IndexBuff++] = (uint8_t) (u16_reg_cnt & 0x00FF);

	for ( uint8_t i=0 ; i<u8_IndexBuff ; i++){
		u16_crc = drv_modbus_crc_crc16_update ( u16_crc, g_buff_frame_rx[i] );
	}
	g_buff_frame_rx[u8_IndexBuff++] = (uint8_t) (u16_crc >> 8);
	g_buff_frame_rx[u8_IndexBuff++] = (uint8_t) (u16_crc & 0x00FF);

	// send the frame request
	for ( uint8_t cnt=0 ; cnt<u8_IndexBuff ; cnt++){
		drv_modbus_slave_rxtUartHandler ( g_buff_frame_rx[cnt] );
	}



	return 0;
}






uint8_t pl_send_uart_tab ( const uint8_t *PtrTab, const uint8_t liu8_cnt){

	for ( uint8_t cnt=0; (cnt<Gu8_CntBytesToCheck)&&(cnt<DRV_MODBUS_SLAVE_VALID_BUFF); cnt++ ){
		if ( g_buff_frame_tx[cnt] != PtrTab [cnt] ){
			return 1;
		}
	}
	return 0;
}







//---------------------------------------------------------------------------------
//		GLOBAL validation function for modbus
//
//---------------------------------------------------------------------------------
uint8_t u8_drv_modbus_slave_valid (){

	uint8_t	lu8_err_code;



	lu8_err_code = drv_modbus_slave_valid_CRC ();
	if ( 0 != lu8_err_code ){
		return lu8_err_code;
	}
/*
	lu8_err_code = u8_drv_modbus_slave_valid__scernario_2 ();
	if ( 0 != lu8_err_code ){
		return lu8_err_code;
	}
*/

	lu8_err_code = drv_modbus_slave_valid_WriteMultipleRegister ();
	if ( 0 != lu8_err_code ){
		return lu8_err_code;
	}
	lu8_err_code = drv_modbus_slave_valid_WriteMultipleCoils ();
	if ( 0 != lu8_err_code ){
		return lu8_err_code;
	}
	lu8_err_code = drv_modbus_slave_valid_WriteSingleRegister ();
	if ( 0 != lu8_err_code ){
		return lu8_err_code;
	}
	lu8_err_code = drv_modbus_slave_valid_WriteSingleCoil ();
	if ( 0 != lu8_err_code ){
		return lu8_err_code;
	}
	lu8_err_code = drv_modbus_slave_valid_ReadInputReg ();
	if ( 0 != lu8_err_code ){
		return lu8_err_code;
	}
	lu8_err_code = drv_modbus_slave_valid_ReadHoldingReg ();
	if ( 0 != lu8_err_code ){
		return lu8_err_code;
	}
	lu8_err_code = drv_modbus_slave_valid_ReadDiscreteInput ();
	if ( 0 != lu8_err_code ){
		return lu8_err_code;
	}
	lu8_err_code = drv_modbus_slave_valid_ReadCoil ();
	if ( 0 != lu8_err_code ){
		return lu8_err_code;
	}



/*
	lu8_err_code = u8_drv_modbus_slave_valid__scernario_1 ();
	if ( 0 != lu8_err_code ){
		return lu8_err_code;
	}
	*/

	return 0;

}


#endif







