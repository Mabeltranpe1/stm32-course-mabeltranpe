/*
 * parcial_final.c
 *
 *  Created on: Jul 21, 2026
 *      Author: marcop
 */
#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdio.h>

//VARIABLES
TIM_HandleTypeDef htim4;
UART_HandleTypeDef huart2;
RTC_HandleTypeDef hrtc;

volatile uint8_t RXchange = 0;  //VARIABLE QUE ALMACENA LA INFORMACION QUE LLEGA DEL COMPUTADOR EN LA COMUNICACION SERIAL, USADA PARA SUBIR O BAJAR PWM MEDIANTE USART
//HEADERS
static void rtc_Init(void);
static void ledok_Init(void);
static void uart_Init (void);

//MAIN
int main(void){
	HAL_Init();
	rtc_Init();
	ledok_Init();
	uart_Init();
	while(1){


	}
	return 0;
}

//FUNCIONES


static void rtc_Init(void){
	RCC_OscInitTypeDef rcc_OscInitStruct = {0};
	RCC_ClkInitTypeDef rcc_clkInitStruct = {0};

	/*EN ESTA FUNCION VOY A CONFIGURAR LOS OSCILADORES LSE Y HSI. EL LSE VA A SER CONFIGURADO PARA RTC, Y HSI VA A SER EL TIMER QUE ENLAZADO CON
	 * PLL ME VA A CONTROLAR LA FRECUENCIA DE OPERACION DEL NUCLEO PARA OPERAR A 100 MHZ
	 */
	rcc_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE | RCC_OSCILLATORTYPE_HSI;
	rcc_OscInitStruct.LSEState = RCC_LSE_ON ;
	rcc_OscInitStruct.HSIState = RCC_HSI_ON;
	rcc_OscInitStruct.PLL.PLLState = RCC_PLL_ON;   //ENCIENDO EL PHASE LOCKED LOOP
	rcc_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI; //ENLAZO PLL CON EL OSCILADOR HSI
	rcc_OscInitStruct.PLL.PLLM = 8;  //DIVISOR DE ENTRADA. LA FRECUENCIA DE ENTRADA ES DE 16MHZ ENTONCES 16 MHZ/ 8 = 2MHZ (LA DIVISION ENTRE LA FRECUENCIA DE ENTRADA Y PLLM DEBE ESTAR ENTR 1 Y 2)
	rcc_OscInitStruct.PLL.PLLN = 100; //MULTIPLICADOR EN VCO, 2 MHZ * 100 = 200 MHZ, SE ESOCJE ESTE VALOR PORQUE PLLP SOLO PUEDE SER UN VALOR ENTRE 2,4,6,8.
	rcc_OscInitStruct.PLL.PLLP = 2;   //DE ESTA MANERA AL PLLP TOMAR EL VALOR DE 2 OBTENGO LOS 100 MHZ4
	rcc_OscInitStruct.PLL.PLLQ = 2; //aunque no haga nada ya que es un registro para configurar el USB, LA CONFIGURACION ME EXIGE QUE ESTE ENTRE

	HAL_RCC_OscConfig(&rcc_OscInitStruct);

	rcc_clkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK |
			RCC_CLOCKTYPE_HCLK |
			RCC_CLOCKTYPE_PCLK1 |
			RCC_CLOCKTYPE_PCLK2;
	rcc_clkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK; //SE ACTIVA LA SEÑAL DE PLL QUE ESTA A 100MHZ PARA EL NUCELO
	rcc_clkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	rcc_clkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
	rcc_clkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2; //sSE DEBE DIVIDIR ENTRE 2 YA QUE APB1 TIENE UNA FRECUENCIA MAXIMA DE OPERACION DE 50 MHZ. COMO SE VE EN LA TABLA General operating conditions
	 /*segun la tabla de Table 5. Number of wait states according to CPU clock (HCLK) frequency en reference manual, el flash latency correcto es de 3ws (4 ciclos de cpu)*/
	HAL_RCC_ClockConfig(&rcc_clkInitStruct, FLASH_LATENCY_3);

	__HAL_RCC_PWR_CLK_ENABLE(); //SE ACTIVA QUE EL RELOJ PUEDA SEGUIR FUNCIONANDO CON LA ENERGIA DE VBAT INCLUSO DESPUES DE SER DESCONECTADO
	HAL_PWR_EnableBkUpAccess(); //SE ENCUIENDEN LOS REGISTROS DE BACKUP PARA NO PERDER LA INFORMACION DEL TIEMPO TRANSCURRIDO AL QUITAR LA ENERGIA
	__HAL_RCC_RTC_CONFIG(RCC_RTCCLKSOURCE_LSE); //SE ENLAZA EL RTC CON EL OSCILADOR LSE PREVIAMENTE CONFIGURADO
	__HAL_RCC_RTC_ENABLE(); //SE ENCIENDE EL RTC


	/*YA HE INICIADO LOS OSCILADORES Y LOS TIMERS A USAR, AHORA VOY A CONFIGURAR EL RTC PARA TENER LA HORA*/
	RTC_InitTypeDef rtc_Config = {0};

	rtc_Config.HourFormat = RTC_HOURFORMAT_24; //HORA MILITAR JEJE
	rtc_Config.AsynchPrediv = 127; //Actua como el PRESCALER en dos etapas, primero se divide una vez 32768 HZ/ (127 + 1) = 256 HZ
	rtc_Config.SynchPrediv = 255; //Luego se divide una vez mas 256 HZ/ (255 + 1) = 1Hz, de esta manera se cuenta una vez cada segundo, como los relojes normales.


	hrtc.Instance = RTC;
	hrtc.Init = rtc_Config;

	HAL_RTC_Init(&hrtc);

	/*TODOS LOS DATOS DE FECHA Y HORA SE INICIALIZAN EN 0, DE ESTA MANERA ACTUA COMO
	 *UNA ESPECIE DE CRONOMETRO QUE EMPIEZA A CONTAR JUSTO LA PRIMERA VEZ QUE SE ACTIVE EL MICRO*/

	RTC_TimeTypeDef time_format = {0};
	RTC_DateTypeDef date_format = {0};


	HAL_RTC_SetTime(&hrtc, &time_format, RTC_FORMAT_BIN);
	HAL_RTC_SetDate(&hrtc, &date_format, RTC_FORMAT_BIN);


}


static void ledok_Init(void)
{
	//configuracion del LED OK a 250 ms.
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
