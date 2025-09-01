/*file     : main.h
* project  : EE329 Lab A9 – EEPROM I2C Test
* author   : Nathan Heil
* date     : 2025-05-18
* brief    : Header file for main application logic and LED control
******************************************************************************
*/
#ifndef __MAIN_H
#define __MAIN_H
#ifdef __cplusplus
extern "C" {
#endif
/* --------------------------- Includes ------------------------------------- */
#include "stm32l4xx_hal.h"     // HAL definitions and device headers
/* --------------------------- Defines -------------------------------------- */
#define EEPROM_ADDRESS  0x57           // 7-bit I2C address of EEPROM
#define LED_PORT        GPIOB          // GPIO port for on-board LED
/* ---------------------- Function Prototypes ------------------------------- */
void Led_Config(void);                 // configure PB14 for LED output
void SystemClock_Config(void);        // system clock setup using MSI
void Error_Handler(void);             // trap function on failure
#ifdef __cplusplus
}
#endif
#endif /* __MAIN_H */
