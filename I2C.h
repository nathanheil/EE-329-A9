/**
******************************************************************************
* @file           : I2C.h
* @project        : EE329 Lab A9
* @author         : Nathan Heil
* @date           : 2025-05-18
* @brief          : Header for EEPROM I2C driver using direct register access
******************************************************************************
*/
#ifndef INC_I2C_H_
#define INC_I2C_H_
#include "stm32l4xx.h"
#include "delay.h"
// 7-bit address of EEPROM device (24LC256 or similar)
#define EEPROM_ADDR 0x57
// Function prototypes for EEPROM byte access
void EEPROM_Init(void);
uint8_t EEPROM_Read(uint8_t eeprom_addr, uint16_t mem_addr);
void EEPROM_Write(uint8_t eeprom_addr, uint16_t mem_addr, uint8_t data_byte);
#endif /* INC_I2C_H_ */
