/*
 * app_modbus.c
 *
 *  Created on: Jun 6, 2025
 *      Author: user
 */

#include "stm32f0xx_hal.h"
#include "drv_modbus_slave.h"




MOSBUS_SLVAE_INSTANCE_t		G_ModbusSlave_instance;
MOSBUS_SLVAE_INSTANCE_t		G_ModbusSlave_instance2;

MODBUS_SLAVE_UPDATED_REG_VALUE_t	reg_cfg;
MODBUS_SLAVE_UPDATED_REG_VALUE_t	reg_cfg2;


void app_wrapper_modbuslavetimeoutcallback (){
	drv_modbus_slave_TimeoutTimer_callback (&G_ModbusSlave_instance);
	drv_modbus_slave_TimeoutTimer_callback (&G_ModbusSlave_instance2);
}
void wrapper_modbus1_drv_modbus_slave_rxtUartHandler ( uint8_t li_data ){
	drv_modbus_slave_rxtUartHandler ( &G_ModbusSlave_instance, li_data );
}
void  wrapper_modbus1_drv_modbus_handler_tx_ended (){
	 drv_modbus_handler_tx_ended (&G_ModbusSlave_instance);
}


void wrapper_modbus2_drv_modbus_slave_rxtUartHandler ( uint8_t li_data ){
	drv_modbus_slave_rxtUartHandler ( &G_ModbusSlave_instance2, li_data );
}
void  wrapper_modbus2_drv_modbus_handler_tx_ended (){
	 drv_modbus_handler_tx_ended (&G_ModbusSlave_instance2);
}



DRV_MODBUS_SLAVE_REGDEF_t		Mb1_TableReg_OutputRegs[]=
{
		{ 0x006B, 0, 0},
		{ 0x006C, 0, 0},
		{ 0x006D, 0, 0},
		{ 0x006E, 0, 0},
		{ 0x006F, 0, 0},
		{ 0x0070, 0, 0},
		{ 0x0071, 0, 0},
		{ 0x0072, 0, 0},
		{ 0x0073, 0, 0}
};

DRV_MODBUS_SLAVE_REGDEF_t		Mb1_TableReg_IntputRegs[]=
{
		{ 0x0080, 0, 0},
		{ 0x0081, 0, 0},
		{ 0x0082, 0, 0},
		{ 0x0083, 0, 0},
		{ 0x0084, 0, 0},
		{ 0x0085, 0, 0}
};

DRV_MODBUS_SLAVE_REGDEF_t		Mb1_TableReg_OutputCoils[]=
{
		{ 0x006D, 0, 0},
		{ 0x0090, 0, 0},
		{ 0x0091, 0, 0},
		{ 0x0092, 0, 0},
		{ 0x0093, 0, 0},
		{ 0x0094, 0, 0},
		{ 0x0095, 0, 0},
		{ 0x0096, 0, 0},
		{ 0x0097, 0, 0},
		{ 0x0098, 0, 0},
		{ 0x0099, 0, 0},
		{ 0x009A, 0, 0}
};

DRV_MODBUS_SLAVE_REGDEF_t		Mb1_TableReg_Inputcoils[]=
{
		{ 0x00A0, 0, 0},
		{ 0x00A1, 0, 0},
		{ 0x00A2, 0, 0},
		{ 0x00A3, 0, 0},
		{ 0x00A4, 0, 0},
		{ 0x00A5, 0, 0},
		{ 0x00A6, 0, 0},
		{ 0x00A7, 0, 0},
		{ 0x00A8, 0, 0},
		{ 0x00A9, 0, 0},
		{ 0x00AA, 0, 0}
};

DRV_MODBUS_SLAVE_REGBASE_t	Mb1__ModbusCfg = {
		{ Mb1_TableReg_OutputRegs,	sizeof(Mb1_TableReg_OutputRegs) / sizeof(DRV_MODBUS_SLAVE_REGDEF_t)	} ,
		{ Mb1_TableReg_OutputCoils, sizeof(Mb1_TableReg_OutputCoils) / sizeof(DRV_MODBUS_SLAVE_REGDEF_t)	} ,
		{ Mb1_TableReg_IntputRegs, 	sizeof(Mb1_TableReg_IntputRegs) / sizeof(DRV_MODBUS_SLAVE_REGDEF_t)	} ,
		{ Mb1_TableReg_Inputcoils, 	sizeof(Mb1_TableReg_Inputcoils) / sizeof(DRV_MODBUS_SLAVE_REGDEF_t)	} ,
};



DRV_MODBUS_SLAVE_REGDEF_t		Mb2_TableReg_OutputRegs[]=
{
		{ 0x006B, 0, 0},
		{ 0x006C, 0, 0},
		{ 0x006D, 0, 0},
		{ 0x006E, 0, 0},
};
DRV_MODBUS_SLAVE_REGDEF_t		Mb2_TableReg_IntputRegs[]=
{
		{ 0x0080, 0, 0},
		{ 0x0081, 0, 0},
		{ 0x0082, 0, 0},
		{ 0x0083, 0, 0},
};
DRV_MODBUS_SLAVE_REGDEF_t		Mb2_TableReg_OutputCoils[]=
{
		{ 0x0090, 0, 0},
		{ 0x0091, 0, 0},
		{ 0x0092, 0, 0},
		{ 0x0093, 0, 0},

};
DRV_MODBUS_SLAVE_REGDEF_t		Mb2_TableReg_Inputcoils[]=
{
		{ 0x00A0, 0, 0},
		{ 0x00A1, 0, 0},
		{ 0x00A2, 0, 0},
		{ 0x00A3, 0, 0},
};

DRV_MODBUS_SLAVE_REGBASE_t	Mb2__ModbusCfg = {
		{ Mb2_TableReg_OutputRegs,	sizeof(Mb2_TableReg_OutputRegs) / sizeof(DRV_MODBUS_SLAVE_REGDEF_t)	} ,
		{ Mb2_TableReg_OutputCoils, sizeof(Mb2_TableReg_OutputCoils) / sizeof(DRV_MODBUS_SLAVE_REGDEF_t)	} ,
		{ Mb2_TableReg_IntputRegs, 	sizeof(Mb2_TableReg_IntputRegs) / sizeof(DRV_MODBUS_SLAVE_REGDEF_t)	} ,
		{ Mb2_TableReg_Inputcoils, 	sizeof(Mb2_TableReg_Inputcoils) / sizeof(DRV_MODBUS_SLAVE_REGDEF_t)	} ,
};




void app_modbus_init (){

	G_ModbusSlave_instance.config.RegistersStrPtr = &Mb1__ModbusCfg;
	G_ModbusSlave_instance.config.u8_ModbusAddress = 0x12;
	G_ModbusSlave_instance.config.u8_uart_id = 1;

	drv_modbus_slave_init ( &G_ModbusSlave_instance );

	G_ModbusSlave_instance2.config.RegistersStrPtr = &Mb2__ModbusCfg;
	G_ModbusSlave_instance2.config.u8_ModbusAddress = 0x13;
	G_ModbusSlave_instance2.config.u8_uart_id = 2;
	drv_modbus_slave_init ( &G_ModbusSlave_instance2 );


	drv_modbus_slave_SetNewRegValue (&G_ModbusSlave_instance, MODBUS_SLAVE_REG_TYPE_INPUTREG, 0x0080, 0x1234 );
	drv_modbus_slave_SetNewRegValue (&G_ModbusSlave_instance, MODBUS_SLAVE_REG_TYPE_INPUTREG, 0x0081, 0x5678 );
	drv_modbus_slave_SetNewRegValue (&G_ModbusSlave_instance2, MODBUS_SLAVE_REG_TYPE_INPUTREG, 0x0080, 0x9ABC );
	drv_modbus_slave_SetNewRegValue (&G_ModbusSlave_instance2, MODBUS_SLAVE_REG_TYPE_INPUTREG, 0x0081, 0xDEF0 );
}


void app_modbus_autom (){


	  // if holding register 0x006B has been updated (written by modbus),
	  // take the value, add 1, and write it on INPUT register
	  // at adress 0x6C, one can read this register to see the new value
	  while ( 0 != drv_modbus_slave_GetUpdatedRegValue (&G_ModbusSlave_instance, &reg_cfg) )
	  {
		  // depending on the type of register
		  switch ( reg_cfg.reg_type )
		  {
			  case MODBUS_SLAVE_REG_TYPE_OUTPUTREG:

				  // depending on the address of the register
				  switch ( reg_cfg.u16_reg_adr )
				  {
					  case 0x006B:
						  reg_cfg2.reg_type = MODBUS_SLAVE_REG_TYPE_INPUTREG;
						  reg_cfg2.u16_reg_adr = 0x6D;
						  reg_cfg2.u16_reg_new_val = reg_cfg.u16_reg_new_val + 1;
						  drv_modbus_slave_SetNewRegValue (&G_ModbusSlave_instance, MODBUS_SLAVE_REG_TYPE_INPUTREG, 0x006D, reg_cfg2.u16_reg_new_val );
						  break;
					  default:
						  break;
				  }
				  break;

			  //-- un registre coil output a été mis à jour
			  case MODBUS_SLAVE_REG_TYPE_OUTPUTCOIL:
				  switch ( reg_cfg.u16_reg_adr )
				  {
					  case 0x006C:
						  reg_cfg2.reg_type = MODBUS_SLAVE_REG_TYPE_INPUTREG;
						  reg_cfg2.u16_reg_adr = 0x6D;

						  if (0 == reg_cfg.u16_reg_new_val ){
							  reg_cfg2.u16_reg_new_val = 0;
						  }else{
							  reg_cfg2.u16_reg_new_val = 123;
						  }
						  break;
					  default:
						  break;
				  }
				  break;

			//-- these types are not written by modbus master, so no update on these, but put there for compiler warning
			case MODBUS_SLAVE_REG_TYPE_INPUTCOIL:
			case MODBUS_SLAVE_REG_TYPE_INPUTREG:
			case MODBUS_SLAVE_REG_TYPE_ERROR:
			  default:
				  break;

		  }

	  }



	  while ( 0 != drv_modbus_slave_GetUpdatedRegValue (&G_ModbusSlave_instance2, &reg_cfg) )
	  {
		  // depending on the type of register
		  switch ( reg_cfg.reg_type )
		  {
			  case MODBUS_SLAVE_REG_TYPE_OUTPUTREG:

				  // depending on the address of the register
				  switch ( reg_cfg.u16_reg_adr )
				  {
					  case 0x006B:
						  reg_cfg2.reg_type = MODBUS_SLAVE_REG_TYPE_INPUTREG;
						  reg_cfg2.u16_reg_adr = 0x6D;
						  reg_cfg2.u16_reg_new_val = reg_cfg.u16_reg_new_val + 10;
						  drv_modbus_slave_SetNewRegValue (&G_ModbusSlave_instance, MODBUS_SLAVE_REG_TYPE_INPUTREG, 0x006D, reg_cfg2.u16_reg_new_val );
						  break;
					  default:
						  break;
				  }
				  break;

			  //-- un registre coil output a été mis à jour
			  case MODBUS_SLAVE_REG_TYPE_OUTPUTCOIL:
				  switch ( reg_cfg.u16_reg_adr )
				  {
					  case 0x006C:
						  reg_cfg2.reg_type = MODBUS_SLAVE_REG_TYPE_INPUTREG;
						  reg_cfg2.u16_reg_adr = 0x6D;

						  if (0 == reg_cfg.u16_reg_new_val ){
							  reg_cfg2.u16_reg_new_val = 0;
						  }else{
							  reg_cfg2.u16_reg_new_val = 789;
						  }
						  break;
					  default:
						  break;
				  }
				  break;

			//-- these types are not written by modbus master, so no update on these, but put there for compiler warning
			case MODBUS_SLAVE_REG_TYPE_INPUTCOIL:
			case MODBUS_SLAVE_REG_TYPE_INPUTREG:
			case MODBUS_SLAVE_REG_TYPE_ERROR:
			  default:
				  break;

		  }

	  }


}









