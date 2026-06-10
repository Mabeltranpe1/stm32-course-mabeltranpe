#include "stm32f4xx.h"
#include "stdint.h"

//inicializacion de variables
volatile uint8_t num1       = 0;
volatile uint8_t num2       = 0;
volatile uint8_t operacion  = 0;   // 0=suma, 1=resta, 2=multiplica, 3=divide
volatile uint8_t resultado  = 0;

/*headers*/
void init_gpio(void);

void TIM_IRQHandler(void);

void led_OK(void);

/*Main*/
int main (void){

	init_gpio();

	while(1){
		switch (operacion){
		case 0: resultado = (num1 + num2) & 0x0F; break;
		case 1: resultado = (num1 - num2) & 0x0F; break;
		case 2: resultado = (num1 * num2) & 0x0F; break;
		case 3: resultado = (num2 != 0) ? (num1 / num2) & 0x0F : 0; break;
		}

	}
	return 0;
}

/*Funciones*/

void init_gpio(void){

	/*INCIANDO GPIO A0*/
	/*iniciando reloj*/
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	RCC->AHB1ENR |= RCC_AHC1ENR_GPIOBEN;

	/*inicializacion del GPIOA*/

	GPIOA->MODER &= ~(GPIO_MODER_MODE0 | GPIO_MODER_MODE1 | GPIO_MODER_MODE2 | GPIO_MODER_MODE3 | GPIO_MODER_MODE5);
	GPIOA->MODER |= (GPIO_MODER_MODE0_0 | GPIO_MODER_MODE1_0 | GPIO_MODER_MODE2_0 | GPIO_MODER_MODE3_0 | GPIO_MODER_MODE5_0);
	GPIOA->OTYPER &= ~(GPIO_OTYPER_OT0 | GPIO_OTYPER_OT1 | GPIO_OTYPER_OT2 | GPIO_OTYPER_OT3 | GPIO_OTYPER_OT5 );
	GPIOA->OSPEEDR &=~(GPIO_OSPEEDER_OSPEEDR0 | GPIO_OSPEEDER_OSPEEDR1 | GPIO_OSPEEDER_OSPEEDR2 | GPIO_OSPEEDER_OSPEEDR3 | GPIO_OSPEEDER_OSPEEDR5 );
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD0 | GPIO_PUPDR_PUPD1 | GPIO_PUPDR_PUPD2 | GPIO_PUPDR_PUPD5);
	GPIOA->ODR |= (GPIO_ODR_OD0 | GPIO_ODR_OD1 | GPIO_ODR_OD2 | GPIO_ODR_OD3 | GPIO_ODR_OD5 );

	/*inicializacion del GPIO*/

}

void init_botones(void){

	/*PUERTO A*/
	GPIOA->MODER &= ~(GPIO_MODER_MODE9 | GPIO_MODER_MODE15);
	GPIOA->PUPDR &= ~(GPIO_MODER_MODE9 | GPIO_MODER_MODE15);
	GPIOA->PUPDR |= (GPIO_MODER_MODE9_0 | GPIO_MODER_MODE15_0 );

	/*PUERTO B*/
	GPIOB->MODER &= ~(GPIO_MODER_MODE7| GPIO_MODER_MODE12);
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD7 | GPIO_PUPDR_PUPD12);
	GPIOB->PUPDR |= (GPIO_PUPDR_PUPD7_0 | GPIO_PUPDR_PUPD12_0);


}

void EXTI_IRQHanlder(void){

	/*EXTI*
	/*ENCENDER SEÑAL DE RELOJ*/
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

	SYSCFG->EXTICR[1] &= ~(SYSCFG_EXTICR2_EXTI7);
	SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI7_PB;

	SYSCFG->EXTICR[2] &= ~(SYSCFG_EXTICR3_EXTI9);
	SYSCFG->EXTICR[2] |= SYSCFG_EXTICR3_EXTI7_PA;

	SYSCFG->EXTICR[3] &= ~(SYSCFG_EXTICR4_EXTI12);
	SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI7_PA;

	SYSCFG->EXTICR[3] &= ~(SYSCFG_EXTICR4_EXTI15);
	SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI7_PA;

	/*IRM MASCARA*/
	EXTI->IMR |= (EXTI_IMR_IM7 | EXTI_IMR_IM9| EXTI_IMR_IM12 | EXTI_IMR_IM15);

	/**/
	EXTI->FTSR |= (EXTI_FTSR_TR7 |  EXTI_FTSR_TR9 | EXTI_FTSR_TR12 | EXTI_FTSR_TR15);

	EXTI->PR  |= (EXTI_PR_PR7 | EXTI_PR_PR9 | EXTI_PR_PR12 | EXTI_PR_PR15);


	__NVIC_EnableIRQ(EXTI15_10_IRQn);
	__NVIC_EnableIRQ(EXTI9_5_IRQn);

	/*TIMER*/
	RCC->APB1ENR |=  RCC_APB1ENR_TIM2EN;

	TIM2->PSC = 1600 -1;
	TIM2->ARR = 1000; //1s
	TIM2->CR1 |= TIM_CR1_ARPE;
	TIM2->CR1 |= TIM_CR1_DIR;
	TIM2->DIER



}

void led_OK(void){



}

void EXTI9_5_IRQHandler(void){

	if (EXTI->PR & EXTI_PR_PR7){
		EXTI->PR |= EXTI_PR_PR7;
		TIM2->ARR += 1000; //1 segundo

		if (TIM2->ARR > 5000){
			TIM2->ARR = 1000;
		}
	}
	if (EXTI->PR & EXTI_PR_PR9){
		EXTI->PR |= EXTI_PR_PR9;
		num2 = (num2 + 1) & 0x0F;

	}

}

void EXTI15_10_IRQHandler(void){
	if (EXTI->PR & EXTI_PR_PR12){
		EXTI->PR |= EXTI_PR_PR12;
		num1 = (num1+1) & 0x0F;
	}
	if (EXTI->PR & EXTI_PR_PR15){
		EXTI->PR |= EXTI_PR_PR15;
		operacion = (operacion + 1) % 4 ; //0->1->2->3->0
	}

}

void TIM2_IRQHandler(void){
	if(TIM2->SR & TIM_SR_UIF){
		TIM2->SR &= ~TIM_SR_UIF;
		GPIOC->ODR ^= GPIO_ODR_ODR13;
	}
}

void actualizar_leds (uint8_t valor){
	//Limpiar PA0-PA3
	GPIOA->ODR &= ~(GPIO_ODR_OD0 | GPIO_ODR_OD1 | GPIO_ODR_OD2 | GPIO_ODR_OD3);

	//Escribir bits
	GPIOA->ODR |= (valor & 0x0F);
}
