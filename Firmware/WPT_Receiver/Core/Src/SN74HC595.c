#include "SN74HC595.h"

unsigned char ByteArray[NUMBER_OF_REGS] = {0};
extern SPI_HandleTypeDef hspi1;
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
	for (int i = 0; i < NUMBER_OF_REGS; i++)
		ByteArray[i] = 0x00;
	HAL_GPIO_WritePin(SN74HC595_PORT, SR_CLR_PIN, GPIO_PIN_SET);
}

void SN74HC595_SetAll(void)
{
	for (int i = 0; i < NUMBER_OF_REGS; i++)
		ByteArray[i] = 0xFF;
	SN74HC595_SendAllBytes();
}

void SN74HC595_SendByte(uint8_t Output)
{
	HAL_SPI_Transmit(&hspi1, &Output, 1, 1);
}

void SN74HC595_SendBytes(uint8_t NumBytes, uint8_t *Values)
{
	HAL_SPI_Transmit(&hspi1, Values, NumBytes, 1);
}

void SN74HC595_SendAllBytes(void)
{
	SN74HC595_SendBytes(NUMBER_OF_REGS, ByteArray);
}

void SN74HC595_UpdateByte(uint8_t ByteIndex, uint8_t Value)
{
	ByteArray[ByteIndex] = Value;
}

void SN74HC595_UpdateBytes(uint8_t ByteIndex, uint8_t NumBytes, uint8_t *Values)
{
	for (int i = 0; i < NumBytes; i++)
		ByteArray[ByteIndex + i] = Values[i];
}

void SN74HC595_UpdateBit(uint8_t ByteIndex, uint8_t BitIndex, uint8_t Value)
{
	if (Value == 1)
		ByteArray[ByteIndex] |= 0x01 << BitIndex;
	else
		ByteArray[ByteIndex] &= ~(0x01 << BitIndex);
}

void SN74HC595_LatchValues(void)
{
	HAL_GPIO_WritePin(SN74HC595_PORT, RCLK_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SN74HC595_PORT, RCLK_PIN, GPIO_PIN_RESET);
}

void SN74HC595_TestReel(void)
{
	uint8_t TestBuffer[] = {0x03, 0xC0, 0x03, 0xF0, 0x0F};
	uint8_t ABuffer[] = {0x81, 0x81, 0xFF, 0x81, 0xFF};
	uint8_t GBuffer[] = {0xFF, 0x81, 0xFF, 0x80, 0xFF};
	uint8_t SingleByte = 0x01;
	
	
	SN74HC595_Init(); // turn on and off all LEDs
	SN74HC595_SetAll();
	SN74HC595_LatchValues();
	HAL_Delay(1000);
	SN74HC595_ClearAll();
	HAL_Delay(1000);
	
	SN74HC595_UpdateByte(0, 0xF0); // draw a pattern
	SN74HC595_SendAllBytes();
	SN74HC595_LatchValues();
	SN74HC595_UpdateByte(2, 0x0F);
	SN74HC595_UpdateByte(4, 0xF0);
	SN74HC595_SendAllBytes();
	SN74HC595_LatchValues();
	HAL_Delay(1000);
	
	SN74HC595_UpdateBit(1, 0, 1);
	SN74HC595_UpdateBit(2, 0, 0);
	SN74HC595_UpdateBit(3, 0, 1);
	SN74HC595_SendAllBytes();
	SN74HC595_LatchValues();
	HAL_Delay(1000);
	
	SN74HC595_UpdateBytes(1, 3, TestBuffer);
	SN74HC595_SendAllBytes();
	SN74HC595_LatchValues();
	HAL_Delay(1000);
	
	SN74HC595_ClearAll(); // move a pattern across the array
	for (uint8_t i = 0; i < 8; i++) {
		SN74HC595_SendByte(SingleByte);
		SN74HC595_LatchValues();
		SingleByte <<= 1;
		HAL_Delay(250);
	}
	HAL_Delay(1000);
	
	SN74HC595_UpdateBytes(0, 5, ABuffer); // flash initials
	SN74HC595_SendAllBytes();
	SN74HC595_LatchValues();
	HAL_Delay(1000);
	SN74HC595_PWM(50);
	HAL_Delay(1000);
	
	SN74HC595_PWM(10);
	SN74HC595_UpdateBytes(0, 5, GBuffer);
	SN74HC595_SendAllBytes();
	SN74HC595_LatchValues();
	HAL_Delay(1000);
	SN74HC595_PWM(50);
	HAL_Delay(1000);
	SN74HC595_PWM(10);
}
	
