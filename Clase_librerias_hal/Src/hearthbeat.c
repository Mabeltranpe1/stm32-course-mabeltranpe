/*
 * hearthbeat.c
 *
 *  Created on: Jul 16, 2026
 *      Author: marcop
 */


#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
/*VARIABLES*/
#define TableSize 360
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim3;
DMA_HandleTypeDef DMA_Config;
uint32_t duty = 0;
uint32_t pulse[TableSize] = {0};

/*HEADERS*/

static void CLK_Init(void);  //INCIO DEL RELOJ DEL SISTEMA
static void LEDOK_Init(void);
static void BreathTable(void);
static void DMA_Init(void);
static void PWM_Init(void);

/*MAIN*/
int main(void){
	BreathTable();
	HAL_Init();
	CLK_Init();
	LEDOK_Init();
	PWM_Init();
	DMA_Init();
	while(1){


	}
	return 0;
}
/*FUNTIONS*/
static void CLK_Init(void){
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_CLKInitStruct = {0};

	//configuracion del oscilador HSI
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	HAL_RCC_OscConfig(&RCC_OscInitStruct);

	RCC_CLKInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK |
			RCC_CLOCKTYPE_HCLK |
			RCC_CLOCKTYPE_PCLK1 |
			RCC_CLOCKTYPE_PCLK2;
	RCC_CLKInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_CLKInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;   //16 MHz
	RCC_CLKInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;    //16 MHz
	/* FLASH_LATENCY_0 = cero wait states, correcto para 16 MHz */
	HAL_RCC_ClockConfig(&RCC_CLKInitStruct, FLASH_LATENCY_0);

}

static void LEDOK_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	/* Habilitar reloj de GPIOH */
	__HAL_RCC_GPIOH_CLK_ENABLE();
	/* Configurar PH1 */
	GPIO_InitStruct.Pin = GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);
	/*EL TIMER USADO PARA CONFIGURAR EL LED OK ES EL TIMER 4. EL TIMER 4 ENTRE TODOS LOS TIMERS QUE HE ELEGIDO ES EL UNICO QUE NO TIENE LA FUNCION DE ADC
	 * CON TRIGGER, POR LO QUE ES MEJOR USARLO EN LED OK
	 */
	__HAL_RCC_TIM4_CLK_ENABLE();
	/* Configurar la base de TIM4 */
	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 15999;  //PRESCALER SE CONFIGURA A 1KHZ
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 249;  // EL LED PARPADEA CADA 250 ms
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	HAL_TIM_Base_Init(&htim4);
	/* Arrancar TIM4 en modo interrupción — habilita la interrupción de evento de actualización */
	HAL_TIM_Base_Start_IT(&htim4);
	/* Habilitar la línea de interrupción de TIM4 en el NVIC */
	HAL_NVIC_EnableIRQ(TIM4_IRQn);
}


//CONFIGURACION DE PWM
static void PWM_Init(void){

	GPIO_InitTypeDef led_struct = {0};
	//CONFIGURACION DEL PIN PA6 - TIMER3 _CH1
	__HAL_RCC_GPIOA_CLK_ENABLE();
	//CONFIGURACION DEL PIN CON FUNCION ESPECIAL
	led_struct.Pin = GPIO_PIN_6;
	led_struct.Mode = GPIO_MODE_AF_PP;
	led_struct.Pull = GPIO_NOPULL;
	led_struct.Speed = GPIO_SPEED_FREQ_HIGH;
	led_struct.Alternate = GPIO_AF2_TIM3;

	HAL_GPIO_Init(GPIOA,&led_struct);

	__HAL_RCC_TIM3_CLK_ENABLE();

	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 31; //prescaler de 500kHz
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	htim3.Init.Period = 999;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

	HAL_TIM_PWM_Init(&htim3);

	TIM_OC_InitTypeDef PWM_Config = {0};

	PWM_Config.OCMode = TIM_OCMODE_PWM1;
	PWM_Config.Pulse = duty;
	PWM_Config.OCPolarity = TIM_OCPOLARITY_HIGH;
	PWM_Config.OCFastMode = TIM_OCFAST_DISABLE;
	PWM_Config.OCIdleState = TIM_OCIDLESTATE_RESET;

	HAL_TIM_PWM_ConfigChannel(&htim3,&PWM_Config,TIM_CHANNEL_1);

}

static void DMA_Init(void){
	__HAL_RCC_DMA1_CLK_ENABLE();



	DMA_Config.Instance                 = DMA1_Stream2;
	DMA_Config.Init.Channel             = DMA_CHANNEL_5;
	DMA_Config.Init.Direction           = DMA_MEMORY_TO_PERIPH;
	DMA_Config.Init.PeriphInc           = DMA_PINC_DISABLE;   // CCR1 is a fixed address
	DMA_Config.Init.MemInc              = DMA_MINC_ENABLE;    // breathTable advances each transfer
	DMA_Config.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
	DMA_Config.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
	DMA_Config.Init.Mode                = DMA_CIRCULAR;
	DMA_Config.Init.Priority            = DMA_PRIORITY_MEDIUM;
	DMA_Config.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;

	HAL_DMA_Init(&DMA_Config);
	__HAL_LINKDMA(&htim3, hdma[TIM_DMA_ID_UPDATE], DMA_Config);

	HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);

	HAL_DMA_Start_IT(htim3.hdma[TIM_DMA_ID_UPDATE], (uint32_t)pulse,
			(uint32_t)&htim3.Instance->CCR1, TableSize);
	__HAL_TIM_ENABLE_DMA(&htim3, TIM_DMA_UPDATE);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

}


void BreathTable (void){
	for (uint32_t i=0; i < TableSize; i++){
		float angle = (float)i / (float)(TableSize - 1) * 3.14159f;
		pulse[i]=(uint32_t)(sinf(angle) * 999.0f);
	}
}


