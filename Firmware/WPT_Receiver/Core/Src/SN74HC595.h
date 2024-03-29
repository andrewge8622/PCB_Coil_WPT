#ifndef SN74HC595_H__
#define SN74HC595_H__ 

#include "stm32g0xx_hal.h"
#include <stdint.h>

#define NUMBER_OF_REGS 5

#define SN74HC595_PORT GPIOA

#define SR_EN_PIN GPIO_PIN_3
#define RCLK_PIN GPIO_PIN_4
#define SR_CLK_PIN GPIO_PIN_5
#define SR_CLR_PIN GPIO_PIN_6
#define SER_O_PIN GPIO_PIN_7

// initialize shift registers and create array for internal representation
void SN74HC595_Init(void);

// enable shift register output
void SN74HC595_Enable(void);

// disable shift register output
void SN74HC595_Disable(void);

// PWM outputs via enable pin, duty cycle of 0 to 100
void SN74HC595_PWM(uint8_t DutyCycle);

// blank all outputs
void SN74HC595_ClearAll(void);

// set all outputs to high
void SN74HC595_SetAll(void);

// send a single byte of data
static void SN74HC595_SendByte(uint8_t Output);

// send multiple bytes of data
static void SN74HC595_SendBytes(uint8_t NumBytes, uint8_t *Values);

// send entire array out to shift registers
void SN74HC595_SendAllBytes(void);

// update a byte of data within internal array
void SN74HC595_UpdateByte(uint8_t ByteIndex, uint8_t Value);

// update multiple bytes of data within internal array
void SN74HC595_UpdateBytes(uint8_t ByteIndex, uint8_t NumBytes, uint8_t *Values);

// update a bit of data within internal array, where BitIndex starts with LSB within each byte
void SN74HC595_UpdateBit(uint8_t ByteIndex, uint8_t BitIndex, uint8_t Value);

// latch values into output register
void SN74HC595_LatchValues(void);

// demo all functions in this module
void SN74HC595_TestReel(void);

#endif
