/*
 * I2C_taller.c
 *
 *  Created on: Jul 10, 2026
 *      Author: marcop
 */

#include <stm32f4xx_hal.h>
#include <stdio.h>
#include <stdlib.h>



/*VARIABLES*/\
I2C_HandleTypeDef hi2c;
UART_HandleTypeDef huart2;


#define WHOAMI (0x68 << 1)
#define SMPLRT_DIV_REG 0x19
#define  ACCEL_CONFIG_REG 0x1C
#define PWR_MGMT_1_REG 0x6B
#define ACCEL_XOUT_H_REG 0x3B

/*headers*/
void i2c_init(void);
void UART_Init(void);
void MPU6050_Init(void);
void MPU6050_read (void);

int main (voi){
	HAL_Init();
	i2c_init();
	UART_Init();
	MPU6050_Init();
	while(1){

		MPU6050_read();

		int len = sprintf(msg, "Ax: %.3f g | Ay: %.3f g | Az: %.3f")

	}
	return 0;
}

void i2c_init(void){

	__HAL_RCC_I2C1_CLK_ENABLE();

	GPIO_InitTypeDef GPIO_Init = {0};

	//PB6 --> SCL
	//PB7 --> SDA

	GPIO_Init.Pin =  GPIO_PIN_6 | GPIO_PIN_7;
	GPIO_Init.Mode = GPIO_MODE_AF_OD;
	GPIO_Init.Pull = GPIO_NOPULL;
	GPIO_Init.Speed = GPIO_SPEED_VERY_HIGH;
	GPIO_Init.Alternate = GPIO_AF4_I2C1;

	HAL_GPIO_Init(GPIOB, &GPIO_Init);


	//configuracion del I2C

	hi2c.Instance = I2C1;
	//velocidad de transmicion estandar
	hi2c.Init.ClockSpeed = 100000;
	hi2c.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c.Init.OwnAddress1 = 0;
	hi2c.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c.Init.OwnAddress2 = 0;
	hi2c.Init.GeneralCallMode =I2C_GENERALCALL_DISABLED;
	hi2c.Init.NoStretchMode = I2C_NOSTRETCH_DISABLED;

}


void UART_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_USART2_CLK_ENABLE();

	GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	huart2.Instance = USART2;
	huart2.Init.BaudRate = 19200;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	HAL_UART_Init(&huart2);
	HAL_UART

}

void MPU6050_Init(void){
	uint8_t Data;

	Data = 0x00;

	HAL_I2C_Mem_Write(&hi2c1,WHOAMI, PWR_MGMT_1_REG, 1, &Data, 1, 100);

	Data = 0x07;

	HAL_I2C_Mem_Write(&hi2c1,WHOAMI, SMPLRT_DIV_REG, 1, &Data, 1, 100);

	Data  =  0x00;

	HAL_I2C_Mem_Write(&hi2c1,WHOAMI, ACCEL_XOUT_H_REG, 1, &Data, 1, 100);

}

void MPU6050_Read (void){

	uint8_t Rec_Data [6];
	HAL_StatusTypeDef status;

	status = HAL_I2C_Mem_Read(&hi2c, WHOAMI, ACCEL_XOUT_H_REG, 1, Rec_Daata, 6 ,100 );

	if (status == HAL_OK){

		init16_t x = (int16_t)(Rec_Data[0] <<8 | RecData [1]);
		init16_t y = (int16_t)(Rec_Data[2] <<8 | RecData [3]);
		init16_t z = (int16_t)(Rec_Data[4] <<8 | RecData [5]);

		if (x == 0 && y==0 && z==0){
			HAL_UART_Transmit_DMA(&huart2,(uint8_t *)"[DEBUG] sensor en sleep mode, Waking Up .. \r\n", pData, Size);
			MPU6050_Init();
		}
		else{
			Ax = Accel_X_RAW / 16384.0;
			Ay = Accel_Y_RAW / 16384.0;
			Az = Accel_z_RAW / 16384.0;
		}
	}

}



