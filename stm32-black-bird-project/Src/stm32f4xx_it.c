/*
 * stm32f4xx_it.c
 * Rutinas de servicio de interrupcion (ISR) del proyecto.
 *
 * Aqui van TODAS las ISR que use el proyecto. El nombre de cada funcion
 * debe coincidir exactamente con la entrada de la tabla de vectores en
 * Startup/startup_stm32f411retx.s
 */

#include "stm32f4xx_hal.h"


extern TIM_HandleTypeDef htim4;

extern TIM_HandleTypeDef htim3;

extern TIM_HandleTypeDef htim2;
/*
 * SysTick: es la base de tiempo del HAL.
 * HAL_Delay() y todos los timeouts internos del HAL dependen de que
 * este contador avance. Sin esta ISR, HAL_Delay() nunca retorna.
 */
void SysTick_Handler(void)
{
    HAL_IncTick();
}

void TIM4_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim4);
}

void TIM3_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim3);
}

void TIM2_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim2);
}
extern UART_HandleTypeDef huart1;

void USART1_IRQHandler(void)
{
	HAL_UART_IRQHandler(&huart1);
}
/*
 * A partir de aqui: tus ISR.
 *
 * Si usas un periferico con interrupcion via HAL, el patron es:
 *   - declarar el handle como extern (esta definido en tu main)
 *   - llamar al despachador del HAL, que se encarga de banderas y callbacks
 *
 * Ejemplo:
 *   extern TIM_HandleTypeDef htim3;
 *   void TIM3_IRQHandler(void) { HAL_TIM_IRQHandler(&htim3); }
 *
 * Si prefieres manejar el periferico a registro, escribe la ISR completa
 * aqui igual que lo venias haciendo (leer bandera -> limpiarla -> avisar a main).
 */

