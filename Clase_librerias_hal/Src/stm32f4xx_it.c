/*
 * stm32f4xx_it.c
 *
 *  Created on: Jun 11, 2026
 *      Author: marcop
 */
/*
 * stm32f4xx_it.c
 * Rutinas de servicio de interrupción
 * Autor: tu nombre
 */

#include "stm32f4xx_hal.h"

/* Declarar el handle de TIM3 — definido en main.c */
extern TIM_HandleTypeDef htim4;
//extern ADC_HandleTypeDef hadc1;
//extern UART_HandleTypeDef huart2;

/* Manejador de SysTick — requerido por HAL para HAL_Delay() y timeouts */
void SysTick_Handler(void)
{
    HAL_IncTick();
}

/* Manejador del evento de actualización de TIM3 */
void TIM4_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim4);
}

extern TIM_HandleTypeDef htim3;

void TIM3_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim4);
}

extern DMA_HandleTypeDef DMA_Config;

extern UART_HandleTypeDef uart_Config;

void UART_IRQHandler(void){
	HAL_UART_IRQHandler(&uart_Config);
}


/*manejo de la interrupcion por conversion ADC
 * las librerias HAL hacen el manejo
 */
//void ADC_IRQHandler(void)
//{
//	HAL_ADC_IRQHandler(&hadc1);
//}
///*INTERRUPCIONES POR RX UART*/
//void USART2_IRQHandler(void)
//{
//	HAL_UART_IRQHandler(&huart2);
//}
