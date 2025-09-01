/**
******************************************************************************
* @file           : I2C.c
* @project        : EE329 Lab A9 - EEPROM Interface
* @author         : Nathan Heil
* @date           : 2025-05-18
* @brief          : I2C read/write interface for 24LC256 EEPROM using register access
******************************************************************************
*/
#include "I2C.h"
#include "delay.h"
void EEPROM_Init(void) {
   // Enable GPIOB clock for SCL/SDA pins
   RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
   // Set PB8 and PB9 to alternate function mode (AF4 for I2C)
   GPIOB->MODER &= ~(GPIO_MODER_MODE8 | GPIO_MODER_MODE9);
   GPIOB->MODER |=  (GPIO_MODER_MODE8_1 | GPIO_MODER_MODE9_1);
   GPIOB->OTYPER |= (GPIO_OTYPER_OT8 | GPIO_OTYPER_OT9);         // Open-drain
   GPIOB->PUPDR  &= ~(GPIO_PUPDR_PUPD8 | GPIO_PUPDR_PUPD9);      // No pull-up/down
   GPIOB->OSPEEDR |= ((3 << GPIO_OSPEEDR_OSPEED8_Pos) |          // High speed
                      (3 << GPIO_OSPEEDR_OSPEED9_Pos));
   GPIOB->AFR[1] &= ~((0xF << GPIO_AFRH_AFSEL8_Pos) |
                      (0xF << GPIO_AFRH_AFSEL9_Pos));
   GPIOB->AFR[1] |=  ((0x4 << GPIO_AFRH_AFSEL8_Pos) |
                      (0x4 << GPIO_AFRH_AFSEL9_Pos));
   // Enable I2C1 clock
   RCC->APB1ENR1 |= RCC_APB1ENR1_I2C1EN;
   // Reset and configure I2C1
   I2C1->CR1 &= ~I2C_CR1_PE;
   I2C1->CR1 &= ~I2C_CR1_ANFOFF;  // analog filter enabled
   I2C1->CR1 &= ~I2C_CR1_DNF;     // digital filter disabled
   I2C1->TIMINGR = 0x00000508;    // Standard mode timing (100kHz @ 16 MHz)
   I2C1->CR2 |= I2C_CR2_AUTOEND;  // automatically send STOP after NBYTES
   I2C1->CR2 &= ~I2C_CR2_ADD10;   // 7-bit addressing
   I2C1->CR1 |= I2C_CR1_PE;       // Enable I2C
}
uint8_t EEPROM_Read(uint8_t eeprom_addr, uint16_t mem_addr) {
   // Set EEPROM internal address pointer (write mode)
   I2C1->CR1 |= I2C_CR1_PE;
   I2C1->CR2 &= ~I2C_CR2_SADD;
   I2C1->CR2 |= (eeprom_addr << (I2C_CR2_SADD_Pos + 1));
   I2C1->CR2 &= ~I2C_CR2_RD_WRN;
   I2C1->CR2 &= ~I2C_CR2_NBYTES;
   I2C1->CR2 |= (2 << I2C_CR2_NBYTES_Pos);
   I2C1->CR2 |= I2C_CR2_START;
   // Send 2-byte address (MSB first)
   while (!(I2C1->ISR & I2C_ISR_TXIS));
   I2C1->TXDR = (mem_addr >> 8);
   while (!(I2C1->ISR & I2C_ISR_TXE));
   I2C1->TXDR = (mem_addr & 0xFF);
   while (!(I2C1->ISR & I2C_ISR_STOPF));
   // Reset and re-enable I2C
   I2C1->CR1 &= ~I2C_CR1_PE;
   delay_us(5);  // EEPROM internal cycle wait
   I2C1->CR1 |= I2C_CR1_PE;
   // Read 1 byte from specified address
   I2C1->CR2 |= I2C_CR2_RD_WRN;
   I2C1->CR2 &= ~I2C_CR2_SADD;
   I2C1->CR2 |= (eeprom_addr << (I2C_CR2_SADD_Pos + 1));
   I2C1->CR2 &= ~I2C_CR2_NBYTES;
   I2C1->CR2 |= (1 << I2C_CR2_NBYTES_Pos);
   I2C1->CR2 |= I2C_CR2_START;
   // Receive data
   while (!(I2C1->ISR & I2C_ISR_RXNE));
   uint8_t received_byte = I2C1->RXDR;
   while (!(I2C1->ISR & I2C_ISR_STOPF));
   I2C1->CR1 &= ~I2C_CR1_PE;
   return received_byte;
}
void EEPROM_Write(uint8_t eeprom_addr, uint16_t mem_addr, uint8_t data_byte) {
   // Begin write: 2-byte address followed by 1 byte of data
   I2C1->CR1 |= I2C_CR1_PE;
   I2C1->CR2 &= ~I2C_CR2_SADD;
   I2C1->CR2 |= (eeprom_addr << (I2C_CR2_SADD_Pos + 1));
   I2C1->CR2 &= ~I2C_CR2_RD_WRN;
   I2C1->CR2 &= ~I2C_CR2_NBYTES;
   I2C1->CR2 |= (3 << I2C_CR2_NBYTES_Pos);
   I2C1->CR2 |= I2C_CR2_START;
   // Send 2-byte address and data
   while (!(I2C1->ISR & I2C_ISR_TXIS));
   I2C1->TXDR = (mem_addr >> 8);
   while (!(I2C1->ISR & I2C_ISR_TXE));
   I2C1->TXDR = (mem_addr & 0xFF);
   while (!(I2C1->ISR & I2C_ISR_TXE));
   I2C1->TXDR = data_byte;
   while (!(I2C1->ISR & I2C_ISR_TXE));
   while (!(I2C1->ISR & I2C_ISR_STOPF));
   I2C1->CR1 &= ~I2C_CR1_PE;
}
