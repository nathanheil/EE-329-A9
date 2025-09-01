/**
 ******************************************************************************
 * @file 	: main.c
 * @project  : EE329 Lab A9 – I2C EEPROM Interface
 * @author   : Nathan Heil
 * @date 	: 2025-05-18
 * @brief	: Writes and verifies EEPROM memory using direct I2C & LED status
 * target	: STM32L4A6ZG, PB8/PB9 (I2C), PB14 (LED)
 ******************************************************************************
 */
#include "main.h"
#include "delay.h"
#include "I2C.h"
/* ----------------------------------- main -----------------------------------
 * System entry point. Configures the clock, I2C, LED, and SysTick.
 * Writes a byte to EEPROM and checks if readback matches. LED is lit on match.
 * -------------------------------------------------------------------------- */
int main(void) {
   HAL_Init();         	// HAL system init (stack, Flash, SysTick)
   SystemClock_Config();   // Set SYSCLK = MSI (4 MHz)
   SysTick_Init();     	// SysTick delay_us setup
   EEPROM_Init();  	// Configure GPIOB and I2C1
   Led_Config();       	// Configure PB14 as output
   uint8_t Data;
   uint16_t Addr;
   while (1) {
  	Data = 0xDA;          	// Byte to write
  	Addr = 0x6084;        	// EEPROM internal address
  	LED_PORT->BRR = GPIO_PIN_14;  // Turn off LED before test
  	EEPROM_Write(EEPROM_ADDR, Addr, Data);  // Write EEPROM
  	delay_us(5000);                            	// Wait for write complete
  	if (EEPROM_Read(EEPROM_ADDR, Addr) == Data) {
         LED_PORT->BSRR = GPIO_PIN_14;  // Match → turn ON LED
  	}
  	delay_us(10000);  // Delay between test cycles
   }
}
/* ----------------------------- Led_Config -----------------------------------
 * Sets PB14 as output, push-pull, no pull-up/down, high speed.
 * Used to drive onboard LED for data match indication.
 * -------------------------------------------------------------------------- */
void Led_Config(void) {
   RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
   LED_PORT->MODER &= ~GPIO_MODER_MODE14;
   LED_PORT->MODER |= GPIO_MODER_MODE14_0;
   LED_PORT->OTYPER &= ~GPIO_OTYPER_OT14;
   LED_PORT->PUPDR &= ~GPIO_PUPDR_PUPD14;
   LED_PORT->OSPEEDR |= (3 << GPIO_OSPEEDR_OSPEED14_Pos);
   LED_PORT->BRR = GPIO_PIN_14;  // Ensure LED is off initially
}
/* ------------------------- SystemClock_Config -------------------------------
 * Configures the system clock to use MSI @ 4 MHz.
 * Uses HAL RCC structures and calls (convert to register config for HAL-free).
 * -------------------------------------------------------------------------- */
void SystemClock_Config(void) {
   RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
   RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
   // Configure regulator output voltage
   if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1)
         != HAL_OK) {
  	Error_Handler();
   }
   // Use MSI (4 MHz) as SYSCLK source
   RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
   RCC_OscInitStruct.MSIState = RCC_MSI_ON;
   RCC_OscInitStruct.MSICalibrationValue = 0;
   RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
   RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
   if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
  	Error_Handler();
   }
   // Set SYSCLK source = MSI, all bus dividers = /1
   RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK |
   RCC_CLOCKTYPE_SYSCLK |
   RCC_CLOCKTYPE_PCLK1 |
   RCC_CLOCKTYPE_PCLK2;
   RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
   RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
   RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
   RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
   if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
  	Error_Handler();
   }
}
/* ----------------------------- Error_Handler ---------------------------------
 * Called when a HAL error occurs. Halts CPU.
 * -------------------------------------------------------------------------- */
void Error_Handler(void) {
   __disable_irq();
   while (1) {
  	// trap forever
   }
}
#ifdef USE_FULL_ASSERT
/* --------------------------- assert_failed ----------------------------------
* Debug output for failed assertions (unused).
* -------------------------------------------------------------------------- */
void assert_failed(uint8_t *file, uint32_t line)
{
  // user can print error location here
}
#endif


