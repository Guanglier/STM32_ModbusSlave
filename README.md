# STM32_ModbusSlave
Modbus slave implementation on STM32 with interruptions.\
Support multiple modbus channels without duplication of code : 2, 3 or more in parallel can work\
Minimal footprint in memory\
Modbus tables static (during compilation) to prevent memory leaks\
updates flag that is checked by the app.\



## Features


### Support following function codes
- 0x01 : read output coil
- 0x02 : read input coil
- 0x03 : read output reg
- 0x04 : read input reg
- 0x05 : write single output coil
- 0x06 : write single output reg
- 0x0F : write multiple output coils
- 0x10 : write multiple output regs

## Requirements

### Timer
Uses the timer for timeout of frame RX. Because can support multiple channels, timer continuously runs.\
This code uses TIM17 but change can be made without any issue

### Uart
Dedicated UART function to write with interrupts.
This is the only part that needs to be duplicated (future optimization ?)

### Interrupts
Uses the interrupts for RX char, TX char, Timer.



## How to use

### files 
- app_modbus.c : applicative file to define modbus registers and init all layer
- drv_modbus_crc.c : internal use for crc of modbus layer, to be reused for example with modbus master on another layer
- drv_modbus_slave.c : main module for modbus slave, RX and TX functions, managing modbus table...
- hal_uart.c : dirty biding of uart, needs to be fully replaced by stm hal_uart

### use
see app_modbus.c and main.c

