/*
 * main_hañ.c
 *
 *  Created on: Jun 11, 2026
 *      Author: marcop
 */
/*
 * main.c
 * HAL Blinky — LED en PA5 conmutado cada 250 ms por TIM3
 * Autor: namontoy@unal.edu.co
 */

#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdio.h>

/* Handle de TIM3 — debe ser global para que stm32f4xx_it.c pueda accederlo */
TIM_HandleTypeDef htim3 = {0};

ADC_HandleTypeDef hadc1 = {0};
volatile uint16_t raw_adc = 0;
volatile uint8_t adc_done = 0;
float adc_mv = 0.0f;

UART_HandleTypeDef huart2;
volatile uint8_t showMsg = 0;
uint8_t msg_buffer [64] = {0};

/* Prototipos de funciones privadas */
static void SystemClock_Config(void);
static void gpio_Init(void);
static void tim3_Init(void);
static void uart2_init(void);
static void adc_Init(void);


int main(void)
{
    HAL_Init();           /* inicializa HAL: SysTick, caché, agrupación de prioridades */
    SystemClock_Config(); /* configura el árbol de relojes: HSI a 16 MHz               */
    gpio_Init();          /* configura PA5 como salida push-pull                        */
    tim3_Init();          /* configura TIM3: evento de actualización cada 250 ms        */
    uart2_init();
    adc_Init();

    HAL_ADC_Start_IT(&hadc1);

    while (1)
    {
    	if (showMsg == 1){
    		HAL_UART_Transmit (&huart2, (uint8_t *)"Hola\n\r",15,100);
    		showMsg = 0;
    		HAL_ADC_Start_IT(&hadc1);
    	}
    	if(adc_done == 1){
    		/*convirtiendo el dato raw en valores de mV*/
    		adc_mv = (float)((3300.0f)/(4));

			/*creado un memsaje, de forma dinamica con los valores del adc_mv*/
    		sprintf((char *)msg_buffer, "adc_value = %#.00f mVn \n\r", adc_mv);

    		/*ENVIANDO POR EL PUERTO SERIAL EL MENSAJE/*/
    		HAL_UART_Transmit(&huart2, msg_buffer, strlen((char *)msg_buffer),100);
    		/*DESSACTIVANDO PARA QUE SSOLO SE EJECUTE CUANDO EXISTA UN NUEVO DATO ADC*/
    		adc_done = 0;
    	}
        /* bucle de aplicación — la conmutación del LED ocurre en el callback */
    }
}

/*
 * SystemClock_Config
 * Usa el oscilador interno HSI a 16 MHz
 * Sin PLL — configuración de reloj más simple posible
 */
static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /* HSI ya está encendido al resetear — confirmar y usarlo */
    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_NONE;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    /* Seleccionar HSI como SYSCLK — todos los divisores de bus en 1 */
    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_SYSCLK |
                                       RCC_CLOCKTYPE_HCLK   |
                                       RCC_CLOCKTYPE_PCLK1  |
                                       RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;   /* HCLK  = 16 MHz */
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;     /* APB1  = 16 MHz */
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;     /* APB2  = 16 MHz */

    /* FLASH_LATENCY_0 = cero wait states, correcto para 16 MHz */
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0);
}

/*
 * gpio_Init
 * Configura PA5 como salida push-pull — LED de la tarjeta Nucleo
 */
static void gpio_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Habilitar reloj de GPIOA en el bus AHB1
       Equivalente bare-metal: RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN */
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* Configurar PA5 */
    GPIO_InitStruct.Pin   = GPIO_PIN_5;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    /*configurando la configuracion en los FSR del MCU*/
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

static void adc_Init (void){

	GPIO_InitTypeDef GPIO_adc_channel = {0};

	/* Habilitar reloj de GPIOA en el bus AHB1 Equivalente bare-metal: RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN */
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/* Configurar PA5 */
	GPIO_adc_channel.Pin   = GPIO_PIN_4;
	GPIO_adc_channel.Mode  = GPIO_MODE_ANALOG;
	GPIO_adc_channel.Pull  = GPIO_NOPULL;

	/*configurando la configuracion en los FSR del MCU*/
	HAL_GPIO_Init(GPIOA, &GPIO_adc_channel);
	__NOP();

	/*Encendemos la señal de reloj de ADC1, bus APB2*/
	__HAL_RCC_ADC1_CLK_ENABLE();

	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.ScanConvMode =  DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DMAContinuousRequests = DISABLE;

	/*Cargar los valores en los FSR del MCU*/
	if(HAL_ADC_Init(&hadc1) != HAL_OK){
		while(1){
			__NOP();     //si hay un error en la configuracion no continue.
		}
	}


	ADC_ChannelConfTypeDef potenciometro = {0};
	potenciometro.Channel = 4;
	potenciometro.Rank = 1;
	potenciometro.SamplingTime = ADC_SAMPLETIME_56CYCLES;
	potenciometro.Offset = 0;

	/*Cargamos los valores en los refistros FSR del MCU*/

	if (HAL_ADC_ConfigChannel(&hadc1, &potenciometro) != HAL_OK){
		__NOP();
	}
	/*Configurando Interrupcion por conversion ADC*/
	HAL_NVIC_EnableIRQ(ADC_IRQn);

}
/*
 * tim3_Init
 * Configura TIM3 para generar un evento de actualización cada 250 ms
 *
 * Cadena de reloj:
 *   HSI (16 MHz) → APB1 (16 MHz) → reloj TIM3 (16 MHz)
 *
 * PSC = 15999  →  tick = 16,000,000 / (15999 + 1) = 1,000 Hz  (1 ms por tick)
 * ARR = 249    →  período = (249 + 1) x 1 ms = 250 ms
 */
static void tim3_Init(void)
{
    /* Habilitar reloj de TIM3 en el bus APB1 */
    __HAL_RCC_TIM3_CLK_ENABLE();

    /* Configurar la base de TIM3 */
    htim3.Instance               = TIM3;
    htim3.Init.Prescaler         = 15999;
    htim3.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim3.Init.Period            = 249;
    htim3.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    /*cargamos la configuracion en los registros FSR del MCU */
    HAL_TIM_Base_Init(&htim3);

    /* Arrancar TIM3 en modo interrupción — habilita la interrupción de evento de actualización */
    HAL_TIM_Base_Start_IT(&htim3);

    /* Habilitar la línea de interrupción de TIM3 en el NVIC */
    HAL_NVIC_EnableIRQ(TIM3_IRQn);

    __NOP();
}

/*
 HAL_TIM_PeriodElapsedCallback
 * Llamado automáticamente por HAL_TIM_IRQHandler() cada vez que un evento
 * de actualización del timer se dispara. Es compartido por todos los timers
 * — siempre verifica htim->Instance.
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3)
    {
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
    }
}

static void uart2_init(void){


	GPIO_InitTypeDef GPIO_InitTX = {0};

	/* Habilitar reloj de GPIOA en el bus AHB1
	       Equivalente bare-metal: RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN */
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/* Configurar PA5 */
	GPIO_InitTX.Pin   = GPIO_PIN_2;
	GPIO_InitTX.Mode  = GPIO_MODE_AF_PP;
	GPIO_InitTX.Pull  = GPIO_NOPULL;
	GPIO_InitTX.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitTX.Alternate = GPIO_AF7_USART2;

	/*configurando la configuracion en los FSR del MCU*/
	HAL_GPIO_Init(GPIOA, &GPIO_InitTX);

	__HAL_RCC_USART2_CLK_ENABLE();

	huart2.Instance = USART2;
	/*config 19200 8N1 - 8 bit data, only TX */
	huart2.Init.BaudRate = 115200;
	huart2.Init.Mode = UART_MODE_TX;
	huart2.Init.Parity =  UART_PARITY_NONE;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;

	/*Cargar la configuracion del UART2 en los FSR del MCU */
	HAL_UART_Init(&huart2);

	HAL_UART_Transmit(&huart2,(uint8_t *) "Hola Mundo",10 , 100);

	__NOP();


}

/*Callback de la conversion ADC*/
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc){
	if (hadc->Instance == ADC1){
		raw_adc = hadc->Instance->DR;
	}
}
