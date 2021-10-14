#include "SN74HC595.h"

unsigned char ByteArray[NUMBER_OF_REGS] = {0};
extern SPI_HandleTypeDef hspi1; // why does this work?
extern TIM_HandleTypeDef htim2;
uint8_t DutyCycle;

void SN74HC595_Init(void) 
{
	DutyCycle = htim2.Instance->CCR4;
	SN74HC595_Disable(); // immediately disable outputs, clear all data, then re-enable
	SN74HC595_ClearAll();
	SN74HC595_Enable();
	HAL_GPIO_WritePin(SN74HC595_PORT, RCLK_PIN, GPIO_PIN_RESET); // set RCLK to low in preparation to latch out
}
void SN74HC595_Enable(void)
{
	htim2.Instance->CCR4 = DutyCycle; // Note: OE pin is active low
}

void SN74HC595_Disable(void)
{
	htim2.Instance->CCR4 = 0;
}

void SN74HC595_PWM(uint8_t NewDutyCycle) 
{
	DutyCycle = NewDutyCycle;
	htim2.Instance->CCR4 = DutyCycle;
}

void SN74HC595_ClearAll(void)
{
	HAL_GPIO_WritePin(SN74HC595_PORT, SR_CLR_PIN, GPIO_PIN_RESET);
	SN74HC595_LatchValues();
	HAL_GPIO_WritePin(SN74HC595_PORT, SR_CLR_PIN, GPIO_PIN_SET);
}

void SN74HC595_SetAll(void)
{
	
}

void SN74HC595_SendByte(uint8_t Output)
{
	
}

void SN74HC595_SendBytes(uint8_t NumBytes, uint8_t *Values)
{
	
}

void SN74HC595_SendAllBytes(void)
{
	
}

void SN74HC595_UpdateByte(uint8_t ByteIndex, uint8_t Value)
{
	
}

void SN74HC595_UpdateBytes(uint8_t ByteIndex, uint8_t NumBytes, uint8_t *Values)
{
	
}

void SN74HC595_UpdateBit(uint8_t ByteIndex, uint8_t BitIndex, uint8_t Value)
{
	
}

void SN74HC595_LatchValues(void)
{
	HAL_GPIO_WritePin(SN74HC595_PORT, RCLK_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SN74HC595_PORT, RCLK_PIN, GPIO_PIN_RESET);
}
