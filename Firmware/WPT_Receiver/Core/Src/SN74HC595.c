#include "SN74HC595.h"

unsigned char ByteArray[NUMBER_OF_REGS] = {0};
extern SPI_HandleTypeDef hspi1; // why does this work?

void SN74HC595_Init(void) 
{
	HAL_GPIO_WritePin(SN74HC595_PORT, SR_EN_PIN, GPIO_PIN_SET); // immediately disable outputs, clear all data, then re-enable
	HAL_GPIO_WritePin(SN74HC595_PORT, SR_CLR_PIN, GPIO_PIN_RESET); 
	HAL_GPIO_WritePin(SN74HC595_PORT, SR_CLR_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SN74HC595_PORT, SR_EN_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SN74HC595_PORT, RCLK_PIN, GPIO_PIN_RESET); // set RCLK to low in preparation to latch out
}
void SN74HC595_Enable(void)
{
	HAL_GPIO_WritePin(SN74HC595_PORT, SR_EN_PIN, GPIO_PIN_RESET); // OE pin is active low
}

void SN74HC595_Disable(void)
{
	HAL_GPIO_WritePin(SN74HC595_PORT, SR_EN_PIN, GPIO_PIN_SET); // OE pin is active low
}

void SN74HC595_ClearAll(void)
{
	HAL_GPIO_WritePin(SN74HC595_PORT, SR_CLR_PIN, GPIO_PIN_RESET); // toggle SR_CLR pin (active low)
	HAL_GPIO_WritePin(SN74HC595_PORT, SR_CLR_PIN, GPIO_PIN_SET);
}

void SN74HC595_SetAll(void)
{
	
}

void SN74HC595_SendByte(uint8_t Output)
{
	HAL_SPI_Transmit(&hspi1, &Output, 1, 1);
}

void SN74HC595_SendBit(uint8_t ByteIndex, uint8_t BitIndex, uint8_t Value)
{
	
}

void SN74HC595_SendBytes(uint8_t NumBytes, uint8_t *Values)
{
	
}

void SN74HC595_LatchValues(void)
{
	HAL_GPIO_WritePin(SN74HC595_PORT, RCLK_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SN74HC595_PORT, RCLK_PIN, GPIO_PIN_RESET);
}
