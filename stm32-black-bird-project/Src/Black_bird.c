/*
 * Black_bird.c
 *
 *  Created on: Jul 28, 2026
 *      Author: Marco Antonio Beltrán Pérez
 *      mabeltranpe@unal.edu.co
 */

#include <stm32f4xx_hal.h>
#include <string.h>
#include <stdint.h>
#include <math.h>


//VARIABLES
uint16_t duty_servos = 1500;
uint16_t duty_esc = 1000;
uint8_t dato_recibido = 0;
int RXchange = 0;
/*----typedef------*/
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
UART_HandleTypeDef huart2;
//HEADERS
static void clk_Init(void);
void ledok_Init(void);
void pwm_Init(void);
void uart_Init(void);

//MAIN
int main(void){

	HAL_Init();
	clk_Init();
	ledok_Init();
	pwm_Init();
	uart_Init();
	while(1){
		if (dato_recibido == 1){
			dato_recibido = 0;
		}
	}
}
static void clk_Init(void){
	RCC_OscInitTypeDef rcc_OscInitStruct = {0};
	RCC_ClkInitTypeDef rcc_clkInitStruct = {0};

	/*VOY A TRABAJAR CON HSI A 16MHZ YA QUE ES SUFICIENTE PARA LA COMUNIACION SERIAL ENTRE MI AVION Y ESTACION EN TIERRA*/
	rcc_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	rcc_OscInitStruct.HSIState = RCC_HSI_ON;
	rcc_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	rcc_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;   //OPERACION A 16MHz

	HAL_RCC_OscConfig(&rcc_OscInitStruct);

	rcc_clkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK |
			RCC_CLOCKTYPE_HCLK |
			RCC_CLOCKTYPE_PCLK1 |
			RCC_CLOCKTYPE_PCLK2;
	rcc_clkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	rcc_clkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	rcc_clkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
	rcc_clkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	 /*segun la tabla de Table 5. Number of wait states according to CPU clock (HCLK) frequency en reference manual, el flash latency correcto es de 3ws (4 ciclos de cpu)*/
	HAL_RCC_ClockConfig(&rcc_clkInitStruct, FLASH_LATENCY_0);


}

void ledok_Init (void){

	//	ACTIVACON DEL LED PA5, YA QUE USARE UNICAMENTE LA PLACA, SIN LA SHELL.
	GPIO_InitTypeDef LEDOK_Init = {0};

	__HAL_RCC_GPIOA_CLK_ENABLE();

	LEDOK_Init.Pin = GPIO_PIN_5;
	LEDOK_Init.Mode = GPIO_MODE_OUTPUT_PP;
	LEDOK_Init.Pull = GPIO_NOPULL;
	LEDOK_Init.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &LEDOK_Init);

	__HAL_RCC_TIM4_CLK_ENABLE();
	/* Configurar la base de TIM4 */
	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 15999;  //PRESCALER SE CONFIGURA A 10KHZ
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


void pwm_Init(void){

	GPIO_InitTypeDef motores_InitStruct = {0};
	/*primero voy a encender el RCC de los pines A ya que son los que usaré para controlar loos servos y el motor
	 * EL PIN PC6 controla Aleron Izq
	 * EL PIN PC7 controla Aleron Der.
	 * EL PIN PC8 controla Elevador.
	 * EL PIN PC9 controla Timon de Cola
	 * EL PIN PA0 controla ESC
	 */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	/*INICIO DE CADA PIN*/
	motores_InitStruct.Pin = GPIO_PIN_6;
	motores_InitStruct.Mode = GPIO_MODE_AF_PP;         //INICIO PIN PA8
	motores_InitStruct.Pull = GPIO_NOPULL;
	motores_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	motores_InitStruct.Alternate = GPIO_AF2_TIM3;
	HAL_GPIO_Init(GPIOC, &motores_InitStruct);


	motores_InitStruct.Pin = GPIO_PIN_7;
	motores_InitStruct.Mode = GPIO_MODE_AF_PP;         //INICIO PIN PA8
	motores_InitStruct.Pull = GPIO_NOPULL;
	motores_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	motores_InitStruct.Alternate = GPIO_AF2_TIM3;
	HAL_GPIO_Init(GPIOC, &motores_InitStruct);

	motores_InitStruct.Pin = GPIO_PIN_8;
	motores_InitStruct.Mode = GPIO_MODE_AF_PP;         //INICIO PIN PA8
	motores_InitStruct.Pull = GPIO_NOPULL;
	motores_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	motores_InitStruct.Alternate = GPIO_AF2_TIM3;
	HAL_GPIO_Init(GPIOC, &motores_InitStruct);


	motores_InitStruct.Pin = GPIO_PIN_9;
	motores_InitStruct.Mode = GPIO_MODE_AF_PP;         //INICIO PIN PA8
	motores_InitStruct.Pull = GPIO_NOPULL;
	motores_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	motores_InitStruct.Alternate = GPIO_AF2_TIM3;
	HAL_GPIO_Init(GPIOC, &motores_InitStruct);


	motores_InitStruct.Pin = GPIO_PIN_0;
	motores_InitStruct.Mode = GPIO_MODE_AF_PP;         //INICIO PIN PA8
	motores_InitStruct.Pull = GPIO_NOPULL;
	motores_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	motores_InitStruct.Alternate = GPIO_AF1_TIM2;
	HAL_GPIO_Init(GPIOA, &motores_InitStruct);
	/*AHORA SIGUE INICIAR EL TIMER2 y TIMER3 QUE ES EL TIMER QUE CONTROLA ESTOS PINES,
	 * Y POR MEDIO DE LOS CANALES DE ESTE TIMER SE HARÁ EL CONTROL DE LOS SERVOS*/
/*

	 * ESTO SE PUEDE HACER DEBIDO A QUE CADA PWM COMPARTEN TODOS LA MISMA FRECUENCIA
	 * LO QUE CAMBIA ES EL DUTY CYCLE
	 * CH1 ASOCIADO A PC6
	 * CH2 ASOCIADO A PC7
	 * CH3 ASOCIADO A PC8
	 * CH3 ASOCIADO A PC9
	 * TIM3 CH1 ASOCIADO A PA0*/
	__HAL_RCC_TIM2_CLK_ENABLE();
	__HAL_RCC_TIM3_CLK_ENABLE();
	/* CONFIGURACION DE LA BASE DEL TIMER 1 A 5KHZ*/
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 15;  // PRESCALER QUEDA EN 500KHZ O CUENTAS DE 2 uS
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 19999;  //EL PERIODO SE CONFIGURA EN 2uS * 100 = 200 uS  1/200 uS = 5KHZ DE ESTA MANERA SE TIENE UNA RESOLUCION DE 100 PASOS
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	HAL_TIM_PWM_Init(&htim2);

	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 15;  // PRESCALER QUEDA EN 5KHZ
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 19999;  //EL PERIODO SE CONFIGURA EN 2uS * 100 = 200 uS  1/200 uS = 5KHZ DE ESTA MANERA SE TIENE UNA RESOLUCION DE 100 PASOS
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	HAL_TIM_PWM_Init(&htim3);


	TIM_OC_InitTypeDef servos_Config = {0};
	TIM_OC_InitTypeDef esc_Config = {0};
	servos_Config.OCMode = TIM_OCMODE_PWM1;
	servos_Config.Pulse = duty_servos;
	servos_Config.OCPolarity = TIM_OCPOLARITY_HIGH;
	servos_Config.OCFastMode = TIM_OCFAST_DISABLE;
	servos_Config.OCIdleState = TIM_OCIDLESTATE_RESET;

	esc_Config.OCMode = TIM_OCMODE_PWM1;
	esc_Config.Pulse = duty_esc;
	esc_Config.OCPolarity = TIM_OCPOLARITY_HIGH;
	esc_Config.OCFastMode = TIM_OCFAST_DISABLE;
	esc_Config.OCIdleState = TIM_OCIDLESTATE_RESET;

	HAL_TIM_PWM_ConfigChannel(&htim3,&servos_Config,TIM_CHANNEL_1);
	HAL_TIM_PWM_ConfigChannel(&htim3,&servos_Config,TIM_CHANNEL_2);
	HAL_TIM_PWM_ConfigChannel(&htim3,&servos_Config,TIM_CHANNEL_3);
	HAL_TIM_PWM_ConfigChannel(&htim3,&servos_Config,TIM_CHANNEL_4);
	HAL_TIM_PWM_ConfigChannel(&htim2,&esc_Config,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);

}

static void uart_Init(void){
	/*PARA EL UART SE VAN A CONFIGURAN LOS PINES PA2 Y PA3 YA QUE ESTOS ESTABLECEN UNA CONEXION DIRECTA CON EL PC MEDIANTE CABLE USB */
	GPIO_InitTypeDef GPIO_InitTX = {0}; //inicio de transmision de datos
	//USART2 esta conecato a APB1
	//acticvacion de PIN PA2 con RCC
	__HAL_RCC_GPIOA_CLK_ENABLE();
	//configuracion de PA2 para usarlo como UART

	GPIO_InitTX.Pin   = GPIO_PIN_2;   //PIN de TX
	GPIO_InitTX.Mode  = GPIO_MODE_AF_PP;  //se establece que se va a usar una funcion alternativa
	GPIO_InitTX.Pull  = GPIO_NOPULL;  //no es necesario tener un pull ya que eso esta controlado por el USART
	GPIO_InitTX.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitTX.Alternate = GPIO_AF7_USART1;  //se configura el uso de la funcion alternatica correspondeitne a AF07, o sea USART1
	//se carga la configuracion
	HAL_GPIO_Init(GPIOA, &GPIO_InitTX);

	//inicio del pin para RX, se va a usar Pa3 como se define en la tabla de funciones extra
	GPIO_InitTypeDef GPIO_InitRX = {0};
	GPIO_InitRX.Pin   = GPIO_PIN_3;       //PIN de RX
	GPIO_InitRX.Mode  = GPIO_MODE_AF_PP;
	GPIO_InitRX.Pull  = GPIO_NOPULL;
	GPIO_InitRX.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitRX.Alternate = GPIO_AF7_USART1;  //se configura el uso de la funcion alternatica correspondeitne a AF07, o sea USART1

	HAL_GPIO_Init(GPIOA, &GPIO_InitRX);

	__HAL_RCC_USART2_CLK_ENABLE();

	huart2.Instance = USART2;
	/*config 115200 8N1 - 8 bit data, TX y RX */
	huart2.Init.BaudRate = 115200;
	huart2.Init.Mode = UART_MODE_TX_RX;  //SE ACTIVA EL MODO DE ENVIO Y RECEPCION DE DATOS
	huart2.Init.Parity =  UART_PARITY_NONE;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	/*Cargar la configuracion del UART2 en los FSR del MCU */
	HAL_UART_Init(&huart2);
	/*SE CARGA LA CONFIGURACION DE RX DEL UART, ADEMÁS SE ESTABLECE LA VARIABLE DONDE SE VA A ALMACENAR LA LETRA QUE MODIFICA EL PWM,
	 * SE CONFIUGRA UN SIZE DE 1 YA QUE SOLO SE CONTROLA MEDIANTE UNA SOLA LETRA*/
	HAL_UART_Receive_IT(&huart2, &RXchange, 1);
	/*CONFIGURACION DE LA INTERUPCION EN EL NVIC*/
	HAL_NVIC_EnableIRQ(USART2_IRQn);

}

//INTERRUPCIONES
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM4)
	{
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);  //SE CAMBIA EL ESTADO DEL PIN ENTRE HIGH Y LOW

	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef * huart){
	/*CONFIGURACION DEL CONTROL MEDIANTE USART
	 * EN ESTA SECCION CONFIGURAMOS LOS 4 CONTROLES NECESARIOS PARA LA TAREA.
	 * SI DESDE EL TERMINAL EN EL PC SE OPRIME LA TECLA  " + " LA INTENSIDAD DE LA LUZ AUMENTA PROGRESIVAMENTE, POR CADA VEZ QUE SE OPRIME EL DUTY CYCLE  AUMENTA EN 1 UNIDAD DE RESOLUCION
	 * RECORDANDO QUE SE HA CONFIGURADO EN 100 DIVISIONES.
	 * sI SE OPRIME ÑA TECLA " - " LA INTENSIDAD DISMINUYE PROGRSIVAMENTE, DE IGUAL MANERA DISMINUYENDO EN UNA UNIDAD
	 * SI SE OPRIME LA LETRA U EL LED SE PONDRA AL MAXIMO DUTY CYCLE POSIBLE, LO QUE SERIA TENER EL LED CONECTADO A UN PIN EN HIGH
	 * SI SE OPIME LA LETRA D EL LED SE APAGA COMLETAMENTE YA QUE EL DUTY CYCLE SE SETEA EN 0*/
	if (huart->Instance == huart2)  //SE VERIFICA QUE LA INTERURUPCION PROVENGA DE USAR2, QUE ES EL QUE ESTA MANEJANDO LA COMUNICACION.
	{
		dato_recibido = 1;
	}
}

