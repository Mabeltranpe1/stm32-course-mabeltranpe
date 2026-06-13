/*
 * trabajo_2.c
 *
 *  Created on: Jun 10, 2026
 *      Author: marcop
 */


#include "stdio.h"
#include "stm32f4xx.h"

/*VARIABLES*/
uint16_t numero = 0;
uint16_t display = 0; //variable para controlar cual display del 7 segmentos se activa
uint16_t representacion = 0; //SE USA 16 BITS PARA PODER CONTAR HASTA 9999, es el numero completo a descomponer.
//uint8_t unidades = 0; //representa descomposicion de la unidad de representacion
//uint8_t decenas = 0; //representa descomposicion de las decenas de representacion
//uint8_t centenas = 0; //representa descomposicion de las centenas de representacion
//uint8_t umil = 0 ; //representa descomposicion de las unidades de mil de representacion

/*HEADERS*/

void init_GPIO(void);
void init_TIM(void);
uint16_t num_set(uint16_t numero); //configura los leds para representar el numero en el display
uint16_t display_select(uint16_t display);
uint16_t decimal(uint16_t representacion, uint16_t display);
/*MAIN*/
int main(void){
	init_GPIO();
	init_TIM();
	representacion = 6060;
	while(1){

		if (TIM2->SR & TIM_SR_UIF){


			GPIOB->ODR |= (0b1111 << 6) ;
			GPIOC->ODR |= (0b11111111 << 5) ;
			numero = decimal (representacion , display);
			GPIOB->ODR &= ~(display_select(display) << 6);
			GPIOC->ODR &= ~(num_set(numero) << 5);

			display++;
			if (display == 4){
				display = 0;
			}
			TIM2->SR &= ~(TIM_SR_UIF);
		}

		if (TIM3->SR & TIM_SR_UIF){
			GPIOH->ODR ^= GPIO_ODR_OD1;
			TIM3->SR &= ~(TIM_SR_UIF);
		}
	}
	return 0;
}
/*FUNCTIONS*/

void init_GPIO(void){

	/*inicio señales de reloj GPIOA*/
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

	/*inicio señales GPIOB*/
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

	/*inicio señales GPIOC*/
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;


	/*inicio GPIOB B6, B7, B8, B9 pines de seleccion de Display
	 B6 -- D1
	 B7 -- D3
	 B8 -- D4
	 B9 -- D2*/

	GPIOB->MODER &= ~(GPIO_MODER_MODE6 | GPIO_MODER_MODE7 | GPIO_MODER_MODE8 | GPIO_MODER_MODE9);
	GPIOB->MODER |= (GPIO_MODER_MODE6_0 | GPIO_MODER_MODE7_0 | GPIO_MODER_MODE8_0 | GPIO_MODER_MODE9_0);
	GPIOB->OTYPER &= ~(GPIO_OTYPER_OT6 | GPIO_OTYPER_OT7 | GPIO_OTYPER_OT8 | GPIO_OTYPER_OT9) ;
	GPIOB->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR6 | GPIO_OSPEEDER_OSPEEDR7 | GPIO_OSPEEDER_OSPEEDR8 | GPIO_OSPEEDER_OSPEEDR9);
	GPIOB->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR6_1 | GPIO_OSPEEDER_OSPEEDR7_1 | GPIO_OSPEEDER_OSPEEDR8_1 | GPIO_OSPEEDER_OSPEEDR9_1);
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD6 | GPIO_PUPDR_PUPD7 | GPIO_PUPDR_PUPD8 | GPIO_PUPDR_PUPD9);
	GPIOB->ODR |= (GPIO_ODR_OD6 | GPIO_ODR_OD7 | GPIO_ODR_OD8 | GPIO_ODR_OD9 );

	/*INCIO GPIOC C5, C6, C8, C9, C10, C11, C12. PINES QUE CONTROLAN CADA LED DEL 7 SEGMENTOS*/
	/*CONFIGURACION DE PIN CON CADA LED:
	 C12 -- A         --A--
	 C11 -- F        |     |
	 C10 -- B        F     B
	 C9  -- E         --G--
	 C8  -- D        E     C
	 C6  -- C        |     |
	 C5  -- G         --d--    */

	GPIOC->MODER &= ~(GPIO_MODER_MODE5 | GPIO_MODER_MODE6 | GPIO_MODER_MODE8 | GPIO_MODER_MODE9 | GPIO_MODER_MODE10 | GPIO_MODER_MODE11 | GPIO_MODER_MODE12);
	GPIOC->MODER |= (GPIO_MODER_MODE5_0 | GPIO_MODER_MODE6_0 | GPIO_MODER_MODE8_0 | GPIO_MODER_MODE9_0 | GPIO_MODER_MODE10_0 | GPIO_MODER_MODE11_0 | GPIO_MODER_MODE12_0);
	GPIOC->OTYPER &= ~(GPIO_OTYPER_OT5 | GPIO_OTYPER_OT6 | GPIO_OTYPER_OT8 |GPIO_OTYPER_OT9 | GPIO_OTYPER_OT10 | GPIO_OTYPER_OT11 | GPIO_OTYPER_OT12) ;
	GPIOC->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR5 | GPIO_OSPEEDER_OSPEEDR6 | GPIO_OSPEEDER_OSPEEDR8 | GPIO_OSPEEDER_OSPEEDR9 | GPIO_OSPEEDER_OSPEEDR10 | GPIO_OSPEEDER_OSPEEDR11 | GPIO_OSPEEDER_OSPEEDR12);
	GPIOC->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR5_1 | GPIO_OSPEEDER_OSPEEDR6_1 | GPIO_OSPEEDER_OSPEEDR8_1 | GPIO_OSPEEDER_OSPEEDR9_1 | GPIO_OSPEEDER_OSPEEDR10_1 | GPIO_OSPEEDER_OSPEEDR11_1 | GPIO_OSPEEDER_OSPEEDR12_1);
	GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPD5 | GPIO_PUPDR_PUPD6 | GPIO_PUPDR_PUPD8 | GPIO_PUPDR_PUPD9 | GPIO_PUPDR_PUPD10 | GPIO_PUPDR_PUPD11 | GPIO_PUPDR_PUPD12);
	GPIOC->ODR |= (GPIO_ODR_OD5 | GPIO_ODR_OD6 | GPIO_ODR_OD8 | GPIO_ODR_OD9 | GPIO_ODR_OD10 | GPIO_ODR_OD11 | GPIO_ODR_OD12);


	/*ACTIVACION LED OK!*/
	/*SEÑAL DE RELOJ*/
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOHEN;
	/*ACTIVACION GPIOH1*/
	GPIOH->MODER &= ~(GPIO_MODER_MODE1);
	GPIOH->MODER |= GPIO_MODER_MODE1_0;
	GPIOH->OTYPER &= ~(GPIO_OTYPER_OT1);
	GPIOH->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR1);
	GPIOH->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR1_1);
	GPIOH->PUPDR &= ~(GPIO_PUPDR_PUPD1);
	GPIOH->ODR |= GPIO_ODR_OD1;

}

void init_TIM(void){

	/*INCIANDO SEÑAL DE RELOJ PARA TIMER2*/
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

	/*INICIANDO SEÑAL DE RELOJ PARA TIMER3*/

	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

	/*INICIANDO TIMER LED OK*/
	TIM3->PSC = 15999;
	TIM3->ARR = 99;
	TIM3->CR1 |= TIM_CR1_CEN;

	/*INICIANDO TIMER TAZA DE REFRESCO 60 HZ*/
	TIM2->PSC = 15999;
	TIM2->ARR = 3;
	TIM2->CR1 |= TIM_CR1_CEN;

}

/*función que selecciona los leds que se iluminan para representar cada numero*/
uint16_t num_set(uint16_t numero){

	switch (numero){

		case 0: return 0b11111010;
		case 1: return 0b00100010;
		case 2: return 0b10111001;
		case 3: return 0b10101011;
		case 4: return 0b01100011;
		case 5: return 0b11001011;
		case 6: return 0b11011011;
		case 7: return 0b10100010;
		case 8: return 0b11111011;
		case 9: return 0b11111011;
		default: return 1;
	}
}
/*FUNCION QUE PERMITE ESCOGER CUAL DISPLAY SE VA A ILUMINAR PARA HACER EL MULTIPLEXEO*/
uint16_t display_select(uint16_t display){

	switch (display){
		case 0: return 0b0001;
		case 1: return 0b1000;
		case 2: return 0b0010;
		case 3: return 0b0100;
		default: return 0;
	}
}

uint16_t decimal(uint16_t representacion , uint16_t display){
	uint16_t num_posicion = 0;
//
//	num_posicion = representacion % (decima) ;
//	num_posicion /= (decima/10);
//	return num_posicion;
	switch(display){
	/* CALCULA QUE NUMERO ESTA EN LA UNIDADES*/
	case 0:
		num_posicion = representacion % 10000 ;
		num_posicion /= 1000 ;
		return num_posicion;
	/*CALCULA QUE NUMERO ESTA EN LAS DECENAS*/
	case 1:
		num_posicion = representacion % 1000;
		num_posicion /= 100;
		return num_posicion;
	/*CALCULA QUE NUMERO ESTA EN LAS CENTENAS*/
	case 2:
		num_posicion = representacion % 100;
		num_posicion /= 10;
		return num_posicion;
	/*CALCULA QUE NUMERO ESTA EN LAS UNIDADES DE MIL*/
	case 3:
		num_posicion = representacion % 10;
		return num_posicion;
	default: return 0;
	}
}


