/*
 * trabajo_2.c
 *
 *  Created on: Jun 10, 2026
 *      Author: marcop
 */


#include "stdio.h"
#include "stm32f4xx.h"

/*VARIABLES*/
volatile uint16_t numero = 0;  //esta variable se carga con el numero que se va a representar en el display correspondiente
volatile uint16_t display = 0; //variable para controlar cual display del 7 segmentos se activa
volatile uint16_t representacion = 0; //SE USA 16 BITS PARA PODER CONTAR HASTA 9999, es el numero completo a descomponer.
volatile uint8_t antirebote = 0; //variable que se usa para evitar el rebote a la hora de registrar una interrupcion en el fotointerruptor y que no se registren varios numeros de golpe
/*HEADERS*/

void init_GPIO(void);
void init_TIM(void);
void init_EXTI(void);
uint16_t num_set(uint16_t numero); //configura los leds para representar el numero en el display
uint16_t display_select(uint16_t display);
uint16_t decimal(uint16_t representacion, uint16_t display);

/*MAIN*/
int main(void){
	init_GPIO();
	init_TIM();
	init_EXTI();
	while(1){



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


	/*CONFIGURACION DE PINES A PARA EXTI*/
	GPIOA->MODER &=~(GPIO_MODER_MODE0 | GPIO_MODER_MODE1);
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD0 | GPIO_PUPDR_PUPD1);


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
	TIM3->CR1 |= TIM_CR1_CEN; //TIM3 configurado cada 100 ms

	/*INICIANDO TIMER TAZA DE REFRESCO*/
	TIM2->PSC = 15999;
	TIM2->ARR = 3;
	TIM2->DIER |= TIM_DIER_UIE;
	TIM2->CR1 |= TIM_CR1_CEN;    //frecuencia del TIM2 configurado cada 4ms, cada 4ms se activa un display y muestra el numero correspondiente
                                 //a esa posicion
	__NVIC_EnableIRQ(TIM2_IRQn);
}

void init_EXTI(void){

	/*INICIO SEÑAL DE RELOJ EXTI*/
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

	/*CONFIGURACION EXTI*/
	/*PIN A0*/
	SYSCFG->EXTICR[0] &=~(SYSCFG_EXTICR1_EXTI0);
	SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PA;

	EXTI->IMR |= EXTI_IMR_IM0 ; //se abre paso a interrupciones para el nvic
	EXTI->RTSR |= EXTI_RTSR_TR0 ;//se configura flancos de subida (cuando se tapa el foto interruptor)


	/*PIN A1*/
	SYSCFG->EXTICR[0] &=~(SYSCFG_EXTICR1_EXTI1);
	SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI1_PA;

	EXTI->IMR |= EXTI_IMR_IM1 ; //se abre paso a interrupciones para el nvic
	EXTI->RTSR |= EXTI_RTSR_TR1 ;//se configura flancos de subida (cuando se tapa el foto interruptor)

	EXTI->PR |= (EXTI_PR_PR0 | EXTI_PR_PR1);

	__NVIC_EnableIRQ(EXTI0_IRQn);
	__NVIC_EnableIRQ(EXTI1_IRQn);
}

/*función que selecciona los leds que se iluminan para representar cada numero*/
uint16_t num_set(uint16_t numero){

	switch (numero){

		case 0: return 0b11111010;    //cada case representa que registros se activan para representar su respectivo numero.
		case 1: return 0b00100010;    //esta funcion retorna un numero que posteriormente será shifteado a sus respectivas posiciones en el registro.
		case 2: return 0b10111001;    //siguiendo el equema de pines configurado en en la activacion de GPIO
		case 3: return 0b10101011;
		case 4: return 0b01100011;
		case 5: return 0b11001011;
		case 6: return 0b11011011;
		case 7: return 0b10100010;
		case 8: return 0b11111011;
		case 9: return 0b11100011;
		default: return 1;
	}
}
/*FUNCION QUE PERMITE ESCOGER CUAL DISPLAY SE VA A ILUMINAR PARA HACER EL MULTIPLEXEO*/
uint16_t display_select(uint16_t display){

	switch (display){
		case 0: return 0b0001;        //cada case de esta funcion retorna un numero binario el cual contiene un 1 que es shiteado en el registro
		case 1: return 0b1000;        //para activar el display correspondiente
		case 2: return 0b0010;
		case 3: return 0b0100;
		default: return 0;
	}
}

uint16_t decimal(uint16_t representacion , uint16_t display){
	uint16_t num_posicion = 0;
/* El algoritmo para poder descomponer el numero a representar, toma el numero y saca el residuo de la division de una potencia de 10
 * correspondiente a su posicion, de esta manera se descartan los numeros a la izquierda del que deseo separar, luego divido entre
 * otra potencia de 10 ^ (n-1) el cual elimina toda la parte izquierda del numero que quiero representar, aprovechando que la
 * division de enteros me da el numero entero.
 */
	switch(display){
	/* CALCULA QUE NUMERO ESTA EN LAS UNIDADES DE MIL*/
	case 0:
		num_posicion = representacion % 10000 ;
		num_posicion /= 1000 ;
		return num_posicion;
	/*CALCULA QUE NUMERO ESTA EN LAS CENTENAS*/
	case 1:
		num_posicion = representacion % 1000;
		num_posicion /= 100;
		return num_posicion;
	/*CALCULA QUE NUMERO ESTA EN LAS DECENAS*/
	case 2:
		num_posicion = representacion % 100;
		num_posicion /= 10;
		return num_posicion;
	/*CALCULA QUE NUMERO ESTA EN LAS UNIDADES*/
	case 3:
		num_posicion = representacion % 10;
		return num_posicion;
	default: return 0;
	}
}

void TIM2_IRQHandler(void){
	if (TIM2->SR & TIM_SR_UIF){


		GPIOB->ODR |= (0b1111 << 6) ;            //se limpian los registros que representan los numeros y que display se debe activar
		GPIOC->ODR |= (0b11111111 << 5) ;
		numero = decimal (representacion , display);  //se descompone el numero, el cual será representado en el display
		GPIOB->ODR &= ~(display_select(display) << 6); //se carga cual es el display que se va a representar
		GPIOC->ODR &= ~(num_set(numero) << 5);         //se configura cuales oines se activan para representar el numero

		display++;             //la posicion del display que se va a mostrar se actualiza
		if (display == 4){     // si esl display alcanza la cuarta posicion, vuelve al primer display
			display = 0;
		}
		if (antirebote > 0){        //el anti rebote se reduce en una unidad a la velocidad del TIM2,
			--antirebote;           //o sea, el antirebote bloquea la recepcion de interrupciones durante 25*4ms = 100 ms
		}
		TIM2->SR &= ~(TIM_SR_UIF);   //se baja la bandera
	}
}

void EXTI0_IRQHandler(void){
	if (EXTI->PR & EXTI_PR_PR0 ){   //se revisa si hay bandera levantada
		EXTI->PR |= EXTI_PR_PR0;    //se baja la bandera
		if (antirebote == 0){   //el antirebote debe estar en 0 para que se pueda volver a registrar una interrupcion.
			++representacion;
			antirebote = 25;    //cuando el contador aumenta, el antirebote se registra en 25.
		}
		if (representacion > 9999){   //se agrega limite superior para siempre asegurar leer hasta 9999.
			representacion = 0;      //aunque dudo mucho llegar a ese valor jejejjejejejjeje.
		}
	}
}

void EXTI1_IRQHandler(void){
	if (EXTI->PR & EXTI_PR_PR1){  //se revisa si hay bandera levantada
		EXTI->PR |= EXTI_PR_PR1;  //se baja la bandera
		if (antirebote == 0){     // de nuevo solo se atiende la interrupcion si ya paso el tiempo de antirebote
			if (!representacion == 0){      //se agrega proteccion para que no existan numeros negativos, no se pueda bajar de 0000
				--representacion;
				antirebote = 25;
			}
		}
	}
}
