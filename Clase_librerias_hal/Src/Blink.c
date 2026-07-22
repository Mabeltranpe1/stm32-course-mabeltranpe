#include <stm32f411xe.h>


//cabecera funcion
void init_RCC(void);
void init_GPIOA(void);
void switch_GPIOA(void);

int main(void){

	init_RCC();
	init_GPIOA();

	while(1){

	}
	return 0;

}




void init_RCC(void){

	RCC->AHB1ENR|= RCC_AHB1ENR_GPIOAEN;

}

void init_GPIOA(void){

	//inicia modo I/O
	GPIOA->MODER &= ~(GPIO_MODER_MODE5_0);
	GPIOA->MODER |= (GPIO_MODER_MODE5_0);

	GPIOA->OTYPER &= ~(GPIO_OTYPER_OT5);
	GPIOA->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR5);
	GPIOA->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR5_0);

	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD5);

	GPIOA->ODR |= (GPIO_ODR_OD5);

}



