/*
 * I2Cmaster.h
 *
 *  Created on: 2018. 9. 28.
 *      Author: JuYeong
 */

#ifndef I2CMASTER_H_
#define I2CMASTER_H_

#include "stm32f1xx_hal.h"
#include "struct_type.h"


void i2c_read_bytes(uint8_t slave_address, uint8_t readstart_register,Raw_data* raw,uint8_t length);
void i2c_write_byte(uint8_t slave_address, uint8_t write_register, uint8_t data);
uint8_t i2c_read_byte(uint8_t slave_address, uint8_t read_register);


#define TWI_START_SRAM        0x40005400
#define TWI_END_SRAM          0x400057FF

#define TIME_OUT_TWI          1U

#define TWI_CONTROL1_OFFSET         0x00        //resetvalue = 0x0000
#define TWI_CONTROL2_OFFSET         0x04        //resetvalue = 0x0000
#define TWI_OWN1_OFFSET             0x08        //resetvalue = 0x0000
#define TWI_OWN2_OFFSET             0x0C        //resetvalue = 0x0000
#define TWI_DATA_OFFSET             0x10        //resetvalue = 0x0000
#define TWI_STATUS1_OFFSET          0x14        //resetvalue = 0x0000
#define TWI_STATUS2_OFFSET          0x18        //resetvalue = 0x0000
#define TWI_CLOCK_CONTROL_OFFSET    0x1C        //resetvalue = 0x0000
#define TWI_TRISE_OFFSET            0x20        //resetvalue = 0x0002

#define TWI_CR1               *(volatile uint32_t*)(TWI_START_SRAM | TWI_CONTROL1_OFFSET)
#define TWI_CR2               *(volatile uint32_t*)(TWI_START_SRAM | TWI_CONTROL2_OFFSET)
#define TWI_OR1               *(volatile uint32_t*)(TWI_START_SRAM | TWI_OWN1_OFFSET)
#define TWI_OR2               *(volatile uint32_t*)(TWI_START_SRAM | TWI_OWN2_OFFSET)
#define TWI_DR                *(volatile uint32_t*)(TWI_START_SRAM | TWI_DATA_OFFSET)
#define TWI_SR1               *(volatile uint32_t*)(TWI_START_SRAM | TWI_STATUS1_OFFSET)
#define TWI_SR2               *(volatile uint32_t*)(TWI_START_SRAM | TWI_STATUS2_OFFSET)
#define TWI_CLCR              *(volatile uint32_t*)(TWI_START_SRAM | TWI_CLOCK_CONTROL_OFFSET)
#define TWI_TR                *(volatile uint32_t*)(TWI_START_SRAM | TWI_TRISE_OFFSET)

#define BYTEMSK               0x0F
#define TWI_READ_BIT          0x01       //address<<1 + 1 : 읽기모드
#define TWI_WRITE_BIT         0x00       //address<<1 + 0 : 쓰기모드


#define TWI_PE        0
#define TWI_START     8
#define TWI_STOP      9
#define TWI_ACK       10
#define TWI_POS       11
#define TWI_SWRST     14

#define TWI_BTF       2
#define TWI_ADDR      1
#define TWI_RX        6
#define TWI_TX        7
#define TWI_SB        0




#endif /* I2CMASTER_H_ */
