/*
 * taller_GPIO_week_04.c
 *
 *  Created on: May 22, 2026
 *      Author: marcop
 */

#include "stdint.h"
#include "stm32f4xx.h"
#include "stdio.h"

//Defincion de las varianles

uint8_t variable = 0;
uint8_t num1 = 10; //num1 debe ser mayor a num2
uint8_t num2 = 5;
char op = '/'; // '' si es una palabra y "" si son más de una
uint8_t resultado = 0;

//cabecera funcion

void int_RCC(void);
void init_PORTA(void);
void init_PORTB(void);
void init_PORTC(void);
int calcular (uint8_t num1,uint8_t num2, char  op)


int main(void){

	init_RCC();
	init_PORTA();
	init_PORTB();
	init_PORTC();
	resultado = calcular(num1, num2, op);
	return 0;
}

//Definiciion de funciones

void init_RCC(void){

	RCC->AHB1ENR |= (0b1 << 0); //encender se;al de relog de gpioA
	RCC->AHB1ENR |= (0b1 << 1); //encender se;al de reloj de GPIOB
	RCC->AHB1ENR |= (0b1 << 2); //encender se;al de reloj GPIOC

}

void init_PORTA(void){
	//papra pa5
	GPIOA->MODER &= ~(0b11 << 5*2);
	GPIOA->MODER |= (0b1 << 5*2);

	GPIOA->OTYPER &= ~(0b1 << 5*2);
	GPIOA->OSPEEDR &= ~(0b10 << 5*2);
	GPIOA->OSPEEDR |= (0b10 << 5*2);
	GPIOA->PUPDR &= ~(0b11 << 5*2);

	GPIOA->ODR |= (0b1 << 5);

	//para pa6
	GPIOA->MODER &= ~(0b11 << 6*2);
	GPIOA->MODER |= (0b1 << 6*2);

	GPIOA->OTYPER &= ~(0b1 << 6*2);

	GPIOA->OSPEEDR &= ~(0b10 << 6*2);
	GPIOA->OSPEEDR |= (0b10 << 6*2);

	GPIOA->PUPDR &= ~(0b11 << 6*2);

	GPIOA->ODR |= (0b1 << 6);

	//para pa7
	GPIOA->MODER &= ~(0b11 << 7*2);
	GPIOA->MODER |= (0b1 << 7*2);

	GPIOA->OTYPER &= ~(0b1 << 7*2);

	GPIOA->OSPEEDR &= ~(0b10 << 7*2);
	GPIOA->OSPEEDR |= (0b10 << 7*2);

	GPIOA->PUPDR &= ~(0b11 << 7*2);

	GPIOA->ODR |= (0b1 << 7);

	//para pa8
	GPIOA->MODER &= ~(0b11 << 8*2);
	GPIOA->MODER |= (0b1 << 8*2);

	GPIOA->OTYPER &= ~(0b1 << 8*2);

	GPIOA->OSPEEDR &= ~(0b10 << 8*2);
	GPIOA->OSPEEDR |= (0b10 << 8*2);

	GPIOA->PUPDR &= ~(0b11 << 8*2);

	GPIOA->ODR |= (0b1 << 8);

	//para pa9
	GPIOA->MODER &= ~(0b11 << 9*2);
	GPIOA->MODER |= (0b1 << 9*2);

	GPIOA->OTYPER &= ~(0b1 << 9*2);

	GPIOA->OSPEEDR &= ~(0b10 << 9*2);
	GPIOA->OSPEEDR |= (0b10 << 9*2);

	GPIOA->PUPDR &= ~(0b11 << 9*2);

	GPIOA->ODR |= (0b1 << 9);
}

void init_PORTB(void){
	//papra pa6
	GPIOB->MODER &= ~(0b11 << 6*2);
	GPIOB->MODER |= (0b1 << 6*2);

	GPIOB->OTYPER &= ~(0b1 << 6*2);
	GPIOB->OSPEEDR &= ~(0b10 << 6*2);
	GPIOB->OSPEEDR |= (0b10 << 6*2);
	GPIOB->PUPDR &= ~(0b11 << 6*2);

	GPIOB->ODR |= (0b1 << 6);

	//para pa8
	GPIOB->MODER &= ~(0b11 << 8*2);
	GPIOB->MODER |= (0b1 << 8*2);

	GPIOB->OTYPER &= ~(0b1 << 8*2);

	GPIOB->OSPEEDR &= ~(0b10 << 8*2);
	GPIOB->OSPEEDR |= (0b10 << 8*2);

	GPIOB->PUPDR &= ~(0b11 << 8*2);

	GPIOB->ODR |= (0b1 << 8);

	//para pa9
	GPIOB->MODER &= ~(0b11 << 9*2);
	GPIOB->MODER |= (0b1 << 9*2);

	GPIOB->OTYPER &= ~(0b1 << 9*2);

	GPIOB->OSPEEDR &= ~(0b10 << 9*2);
	GPIOB->OSPEEDR |= (0b10 << 9*2);

	GPIOB->PUPDR &= ~(0b11 << 9*2);

	GPIOB->ODR |= (0b1 << 9);
}

void init_PORTC(void){
	//papra pa5
	GPIOA->MODER &= ~(0b11 << 7*2);
	GPIOA->MODER |= (0b1 << 7*2);

	GPIOA->OTYPER &= ~(0b1 << 7*2);
	GPIOA->OSPEEDR &= ~(0b10 << 7*2);
	GPIOA->OSPEEDR |= (0b10 << 7*2);
	GPIOA->PUPDR &= ~(0b11 << 7*2);

	GPIOA->ODR |= (0b1 << 7);

}

/*Definición de funciones*/
int calcular (uint8_t num1, uint8_t num2, char op){
    if (op == '+'){
        return num1 + num2;
    }

    else if (op == '-'){
        return num1 - num2;
    }

    else if (op == '*'){
            return num1 * num2;
        }

    else if (op == '/'){
            return num1 / num2;
        }

    else{
        return 0;
    }
}

void mostrar(void){
	led0 = (resultado >> 0) & (0b1);
	led1 = (resultado >> 1) & (0b1);
}


