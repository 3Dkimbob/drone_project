/*
 * I2Cmaster.c
 *
 *  Created on: 2018. 9. 28.
 *      Author: JuYeong
 */

#include "I2Cmaster.h"

extern I2C_HandleTypeDef hi2c1;


uint8_t i2c_read_byte(uint8_t slave_address, uint8_t read_register)
{
  uint8_t start=read_register;
  uint8_t address = (slave_address<<1) | TWI_READ_BIT;
  uint8_t data;
  HAL_I2C_Master_Transmit(&hi2c1, address, &start,1,TIME_OUT_TWI);
  HAL_I2C_Master_Receive(&hi2c1, address, &data, 1,TIME_OUT_TWI);
  return data;
}

void i2c_read_bytes(uint8_t slave_address, uint8_t readstart_register,Raw_data* raw,uint8_t length)
{
  uint8_t start=readstart_register;
  uint8_t address = (slave_address<<1);
  HAL_I2C_Master_Transmit(&hi2c1, address, &start,1,TIME_OUT_TWI);
  HAL_I2C_Master_Receive(&hi2c1, address, raw->data, length,TIME_OUT_TWI);
}

void i2c_write_byte(uint8_t slave_address, uint8_t write_register, uint8_t data)
{
  uint8_t address = (slave_address<<1);
  uint8_t array[2]={write_register,data};
  HAL_I2C_Master_Transmit(&hi2c1, address, array, sizeof(array),TIME_OUT_TWI);
}


























