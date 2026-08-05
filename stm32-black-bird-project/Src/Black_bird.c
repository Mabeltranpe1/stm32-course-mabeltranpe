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
#include <stdio.h>


//VARIABLES
//DEFINICIONES PARA EL MPU
#define WHOAMI (0x68 << 1)
#define SMPLRT_DIV_REG 0x19
#define ACCEL_CONFIG_REG 0x1C
#define PWR_MGMT_1_REG 0x6B
#define ACCEL_XOUT_H_REG 0x3B
//DEFINICIONES PARA E BMP
#define BMP_ADDR (0x76 << 1)
#define BMP_ID 0xD0
#define PRESS_MSB_REG 0XF7
#define PRESS_LSB_REG 0xF8
#define PRESS_XLSB_REG 0xF9
/*LOS COEFICIENTES DE CALIBRACION VIVEN DESDE 0x88 Y SON 24 BYTES SEGUIDOS.
 * SON PROPIOS DE CADA UNIDAD Y SE LEEN UNA SOLA VEZ AL ARRANCAR*/
#define BMP_CALIB_REG 0x88
#define BMP_CTRL_MEAS_REG 0xF4
/*0x2F = TEMPERATURA x1, PRESION x4, MODO NORMAL (MIDE SOLO Y SIN PARAR)*/
#define BMP_CTRL_MEAS_VAL 0x2F

#define RAD_TO_GRAD 57.2957795f

uint16_t duty_aleron_der= 1500; //EL DUTY DE CADA PWM USADO EN CADA SUPERFICIE. SE INICIALIZA EN 1500 (1.5 MS) YA QUE ES EL VALOR CENTRAL DEL SERVO MOTOR)
uint16_t duty_aleron_izq = 1500;
uint16_t duty_timon = 1500;
uint16_t duty_elevador = 1500;
uint16_t duty_motor = 0;

uint8_t tipo_tecla_oprimida = 0; //variable para definir que ha llegado por serial y a que maquina de estados debe entrar
uint16_t delta_servo = 0;


uint16_t duty_esc = 1000;
volatile uint8_t dato_recibido = 0;
volatile uint8_t RXchange = 0;
volatile uint8_t BANDERA_MOVIMIENTO = 0;

int16_t Accel_X_raw, Accel_Y_raw, Accel_Z_raw;
float Ax, Ay, Az;
int ang_roll, ang_pitch;

/*COEFICIENTES DE CALIBRACION DEL BMP280. SON PROPIOS DE CADA SENSOR Y SE LEEN
 * UNA SOLA VEZ EN bmp280_Init(). SIN ELLOS LOS DATOS CRUDOS NO SIGNIFICAN NADA.
 * OJO CON LOS TIPOS: EL DATASHEET DICE CUALES SON CON SIGNO Y CUALES NO*/
uint16_t dig_T1, dig_P1;
int16_t  dig_T2, dig_T3;
int16_t  dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
/*t_fine LO CALCULA LA COMPENSACION DE TEMPERATURA Y LO NECESITA LA DE PRESION,
 * POR ESO LA TEMPERATURA SIEMPRE SE CALCULA PRIMERO*/
int32_t t_fine;

uint32_t presion_actual = 0;      //EN PASCALES
uint32_t presion_referencia = 0;  //PRESION AL ARRANCAR, ES EL CERO DEL ALTIMETRO
int altura_actual = 0;            //METROS SOBRE EL PUNTO DE ARRANQUE
/*----typedef------*/
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
UART_HandleTypeDef huart2;
I2C_HandleTypeDef hi2c;
//HEADERS
static void clk_Init(void);
void ledok_Init(void);
void pwm_Init(void);
static void uart_Init(void);
void i2c_init(void);
void mpu6050_Init(void);
void mpu6050_Read (void);
void bmp280_Init (void);
void bmp280_Read (void);
void modo_seteado (uint8_t modo_de_operacion);
void movimiento(void);
void avion_Init(void);
void cambio_fsm(uint8_t tecla_oprimida);
void transmitir_datos(void);
//ESTADOS
//
typedef enum {
	NO_ARMADO ,
	ARMADO ,
	TAKE_OFF ,
	CRUISE ,
	LANDING,
	FAIL_SAFE
} operacion;

typedef enum{
	ESPERANDO_CONTROL = 0,
	ALERON_IZQ = 'a',
	ALERON_DER = 'd',
	ELEVADOR_ARR = 's',
	ELEVADOR_ABJ = 'w',
	TIMON_IZQ = 'j',
	TIMON_DER = 'l',
}control;

typedef enum{
	POSITIVO,
	NEGATIVO,
	CENTRO
}posicion;


posicion pitch  = CENTRO;
posicion roll = CENTRO;
posicion yaw = CENTRO;

uint32_t timeout_pitch = 0;
uint32_t timeout_roll = 0;
uint32_t timeout_yaw = 0;
uint32_t timeout_failsafe = 0;
uint32_t tiempo_transmision_datos = 0;

uint16_t reposo_roll_izq = 1500;
uint16_t reposo_roll_der = 1500;
uint16_t reposo_pitch = 1500;
uint16_t reposo_yaw = 1500;


operacion modo_de_operacion = NO_ARMADO;
control superficie = ESPERANDO_CONTROL;

//ESTA TABLA LA USO PARA ENVIAR POR SERIAL EN QUE MODO ESTOY OPERANDO
const char *nombres_modo[] = {
     "DESARMADO", "ARMADO", "DESPEGUE", "CRUCERO", "ATERRIZAJE","FAILSAFE"
 };

const char *throttle[] = {
		"0%","0%","100%","55%","20%","0%"
};

int len_msg = 0;
//MAIN

int main(void){

	HAL_Init();
	clk_Init();
	ledok_Init();
	i2c_init();
	pwm_Init();
	uart_Init();
	bmp280_Init();
	mpu6050_Init();
	avion_Init();

	while(1){

		if (dato_recibido == 1){
			dato_recibido = 0;
			timeout_failsafe = HAL_GetTick();
			cambio_fsm(RXchange);
			modo_seteado(modo_de_operacion);
		}
		if (BANDERA_MOVIMIENTO == 1){
			BANDERA_MOVIMIENTO = 0;
			movimiento();

		}
		if (HAL_GetTick() - tiempo_transmision_datos > 200){
			transmitir_datos();
			tiempo_transmision_datos = HAL_GetTick();
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

void transmitir_datos(void){
	mpu6050_Read();
	bmp280_Read();
	uint8_t msg_buffer[100] = {0};
	len_msg = sprintf((char *)msg_buffer, "ROLL=%d PITCH=%d ALT=%d THR=%u MODO_OPERACION=%s \r\n", ang_roll, ang_pitch, altura_actual, duty_motor, nombres_modo[modo_de_operacion]);
	HAL_UART_Transmit(&huart2, msg_buffer, len_msg, 100);


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
	servos_Config.Pulse = 0;
	servos_Config.OCPolarity = TIM_OCPOLARITY_LOW;
	servos_Config.OCFastMode = TIM_OCFAST_DISABLE;
	servos_Config.OCIdleState = TIM_OCIDLESTATE_RESET;

	esc_Config.OCMode = TIM_OCMODE_PWM1;
	esc_Config.Pulse = duty_esc;
	esc_Config.OCPolarity = TIM_OCPOLARITY_LOW;
	esc_Config.OCFastMode = TIM_OCFAST_DISABLE;
	esc_Config.OCIdleState = TIM_OCIDLESTATE_RESET;

	HAL_TIM_PWM_ConfigChannel(&htim3,&servos_Config,TIM_CHANNEL_1);
	HAL_TIM_PWM_ConfigChannel(&htim3,&servos_Config,TIM_CHANNEL_2);
	HAL_TIM_PWM_ConfigChannel(&htim3,&servos_Config,TIM_CHANNEL_4);
	HAL_TIM_PWM_ConfigChannel(&htim3,&servos_Config,TIM_CHANNEL_3);
	HAL_TIM_PWM_ConfigChannel(&htim2,&esc_Config,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_NVIC_EnableIRQ(TIM3_IRQn);
	HAL_TIM_Base_Start_IT(&htim2);
	/* Habilitar la línea de interrupción de TIM4 en el NVIC */
	HAL_NVIC_EnableIRQ(TIM2_IRQn);

}

static void uart_Init(void){
	/*PARA EL UART SE VAN A CONFIGURAN LOS PINES PA2 Y PA3 YA QUE ESTOS ESTABLECEN UNA CONEXION DIRECTA CON EL PC MEDIANTE CABLE USB */
	GPIO_InitTypeDef GPIO_InitTX = {0}; //inicio de transmision de datos
	//USART1 esta conecato a APB1
	//acticvacion de PIN PA2 con RCC
	__HAL_RCC_GPIOA_CLK_ENABLE();
	//configuracion de PA2 para usarlo como UART

	GPIO_InitTX.Pin   = GPIO_PIN_2;   //PIN de TX
	GPIO_InitTX.Mode  = GPIO_MODE_AF_PP;  //se establece que se va a usar una funcion alternativa
	GPIO_InitTX.Pull  = GPIO_NOPULL;  //no es necesario tener un pull ya que eso esta controlado por el USART
	GPIO_InitTX.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitTX.Alternate = GPIO_AF7_USART2;  //se configura el uso de la funcion alternatica correspondeitne a AF07, o sea USART1
	//se carga la configuracion
	HAL_GPIO_Init(GPIOA, &GPIO_InitTX);

	//inicio del pin para RX, se va a usar Pa3 como se define en la tabla de funciones extra
	GPIO_InitTypeDef GPIO_InitRX = {0};
	GPIO_InitRX.Pin   = GPIO_PIN_3;       //PIN de RX
	GPIO_InitRX.Mode  = GPIO_MODE_AF_PP;
	GPIO_InitRX.Pull  = GPIO_NOPULL;
	GPIO_InitRX.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitRX.Alternate = GPIO_AF7_USART2;  //se configura el uso de la funcion alternatica correspondeitne a AF07, o sea USART1

	HAL_GPIO_Init(GPIOA, &GPIO_InitRX);

	__HAL_RCC_USART2_CLK_ENABLE();

	huart2.Instance = USART2;
	/*config 19200 8N1 - 8 bit data, TX y RX */
	huart2.Init.BaudRate = 19200;
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

void i2c_init(void){

	__HAL_RCC_I2C1_CLK_ENABLE();

	__HAL_RCC_GPIOB_CLK_ENABLE();

	GPIO_InitTypeDef GPIO_Init = {0};

	//PB8 --> SCL
	//PB9 --> SDA

	GPIO_Init.Pin =  GPIO_PIN_8 | GPIO_PIN_9;  //CONFIGURACION DE PINES DE SDA(PB7) Y SCL(PB6)
	GPIO_Init.Mode = GPIO_MODE_AF_OD;
	GPIO_Init.Pull = GPIO_NOPULL;
	GPIO_Init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_Init.Alternate = GPIO_AF4_I2C1;

	HAL_GPIO_Init(GPIOB, &GPIO_Init);


	//configuracion del I2C

	hi2c.Instance = I2C1;
	//velocidad de transmicion estandar
	hi2c.Init.ClockSpeed = 100000;
	hi2c.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c.Init.OwnAddress1 = 0;
	hi2c.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c.Init.OwnAddress2 = 0;
	hi2c.Init.GeneralCallMode =I2C_GENERALCALL_DISABLED;
	hi2c.Init.NoStretchMode = I2C_NOSTRETCH_DISABLED;

	HAL_I2C_Init(&hi2c);



}

void mpu6050_Init(void){
	uint8_t Data;

	Data = 0x00;

	HAL_I2C_Mem_Write(&hi2c,WHOAMI, PWR_MGMT_1_REG, 1, &Data, 1, 100);

	Data = 0x07;

	HAL_I2C_Mem_Write(&hi2c,WHOAMI, SMPLRT_DIV_REG, 1, &Data, 1, 100);

	Data  =  0x00;

	HAL_I2C_Mem_Write(&hi2c,WHOAMI, ACCEL_CONFIG_REG, 1, &Data, 1, 100);

}

void mpu6050_Read (void){

	uint8_t Rec_Data [6];
	HAL_StatusTypeDef status;

	//TIME OUT A 100MS PARA EVITAR QUE SE CONGELE POR MUCHO TIEMPO SI SE DESCONECTA
	status = HAL_I2C_Mem_Read(&hi2c, WHOAMI, ACCEL_XOUT_H_REG, 1, Rec_Data, 6 ,100 );

	if (status == HAL_OK){

		int16_t x = (int16_t)(Rec_Data[0] <<8 | Rec_Data [1]);
		int16_t y = (int16_t)(Rec_Data[2] <<8 | Rec_Data [3]);
		int16_t z = (int16_t)(Rec_Data[4] <<8 | Rec_Data [5]);

		if (x == 0 && y==0 && z==0){
			char str[] = "sensor en modo sueño, Despertando.... \r\n";
			HAL_UART_Transmit(&huart2,(uint8_t *)str, strlen(str), 100);
			mpu6050_Init();
		}
		else{
			Accel_X_raw = x;
			Accel_Y_raw = y;
			Accel_Z_raw = z;

			Ax = Accel_X_raw / 16384.0;
			Ay = Accel_Y_raw / 16384.0;
			Az = Accel_Z_raw / 16384.0;

			//DESEO ENTREGAR LOS RESULTADOS EN GRADOS, POR LO QUE DEBO HACER CONVERSION.
			ang_roll = (int)(atan2(Ay, Az) * RAD_TO_GRAD);
			ang_pitch = (int)(atan2(-Ax , sqrt(pow(Ay,2) + pow(Az,2))) * RAD_TO_GRAD);

		}
	}
	else {
		char err_msg[64];
		int len = sprintf(err_msg, "Error I2C (status=%d). Reintentando...\r\n", status);
		HAL_UART_Transmit(&huart2, (uint8_t *)err_msg, len, 100);

		// Si el bus I2C está bloqueado, se re-inicializa el periférico I2C1 de la STM32
		if (status == HAL_BUSY) {
			HAL_I2C_DeInit(&hi2c);
			HAL_I2C_Init(&hi2c);
		}

		mpu6050_Init();

	}
}

/*INICIALIZACION DEL BMP280. SE LLAMA UNA SOLA VEZ AL ARRANCAR:
 * DESPIERTA EL SENSOR Y LEE SUS COEFICIENTES DE CALIBRACION*/
void bmp280_Init(void){

	uint8_t Rec_Data[24];
	uint8_t Data;

	/*MODO NORMAL: EL SENSOR MIDE SOLO Y SIN PARAR*/
	Data = BMP_CTRL_MEAS_VAL;
	HAL_I2C_Mem_Write(&hi2c, BMP_ADDR, BMP_CTRL_MEAS_REG, 1, &Data, 1, 100);

	/*LOS 24 BYTES DE CALIBRACION DE UN SOLO TIRON, IGUAL QUE EN EL MPU*/
	HAL_I2C_Mem_Read(&hi2c, BMP_ADDR, BMP_CALIB_REG, 1, Rec_Data, 24, 100);

	/*OJO: LOS COEFICIENTES VIENEN CON EL BYTE BAJO PRIMERO, AL REVES QUE
	 * LOS REGISTROS DE DATOS. POR ESO AQUI EL INDICE IMPAR VA DESPLAZADO*/
	dig_T1 = (uint16_t)(Rec_Data[1] << 8 | Rec_Data[0]);
	dig_T2 =  (int16_t)(Rec_Data[3] << 8 | Rec_Data[2]);
	dig_T3 =  (int16_t)(Rec_Data[5] << 8 | Rec_Data[4]);
	dig_P1 = (uint16_t)(Rec_Data[7] << 8 | Rec_Data[6]);
	dig_P2 =  (int16_t)(Rec_Data[9] << 8 | Rec_Data[8]);
	dig_P3 =  (int16_t)(Rec_Data[11] << 8 | Rec_Data[10]);
	dig_P4 =  (int16_t)(Rec_Data[13] << 8 | Rec_Data[12]);
	dig_P5 =  (int16_t)(Rec_Data[15] << 8 | Rec_Data[14]);
	dig_P6 =  (int16_t)(Rec_Data[17] << 8 | Rec_Data[16]);
	dig_P7 =  (int16_t)(Rec_Data[19] << 8 | Rec_Data[18]);
	dig_P8 =  (int16_t)(Rec_Data[21] << 8 | Rec_Data[20]);
	dig_P9 =  (int16_t)(Rec_Data[23] << 8 | Rec_Data[22]);
}


/*LECTURA DEL BMP280. LEE LOS 6 BYTES DE DATOS, APLICA LA COMPENSACION DEL
 * DATASHEET Y CALCULA LA ALTURA SOBRE EL PUNTO DONDE SE ENCENDIO EL AVION*/
void bmp280_Read(void){

	uint8_t Rec_Data[6];
	HAL_StatusTypeDef status;
	int32_t adc_P, adc_T;
	int32_t var1, var2;
	uint32_t p;

	/*LOS 6 BYTES DE DATOS SEGUIDOS: PRESION (3) Y TEMPERATURA (3)*/
	status = HAL_I2C_Mem_Read(&hi2c, BMP_ADDR, PRESS_MSB_REG, 1, Rec_Data, 6, 100);

	if (status != HAL_OK){
		return;   //UNA LECTURA PERDIDA NO IMPORTA, SE REINTENTA EN LA SIGUIENTE
	}

	/*CADA MEDIDA SON 20 BITS REPARTIDOS EN 3 BYTES, CON EL MSB PRIMERO.
	 * EL XLSB TRAE SUS 4 BITS EN LA PARTE ALTA, POR ESO SE DESPLAZA A LA DERECHA*/
	adc_P = (int32_t)(((uint32_t)Rec_Data[0] << 12) | ((uint32_t)Rec_Data[1] << 4) | ((uint32_t)Rec_Data[2] >> 4));
	adc_T = (int32_t)(((uint32_t)Rec_Data[3] << 12) | ((uint32_t)Rec_Data[4] << 4) | ((uint32_t)Rec_Data[5] >> 4));

	/*COMPENSACION DE TEMPERATURA (FORMULA DEL DATASHEET).
	 * VA PRIMERO PORQUE DEJA t_fine, QUE NECESITA LA DE PRESION*/
	var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
	var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
	t_fine = var1 + var2;

	/*COMPENSACION DE PRESION (FORMULA DEL DATASHEET)*/
	var1 = (((int32_t)t_fine) >> 1) - (int32_t)64000;
	var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((int32_t)dig_P6);
	var2 = var2 + ((var1 * ((int32_t)dig_P5)) << 1);
	var2 = (var2 >> 2) + (((int32_t)dig_P4) << 16);
	var1 = (((dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) + ((((int32_t)dig_P2) * var1) >> 1)) >> 18;
	var1 = ((((32768 + var1)) * ((int32_t)dig_P1)) >> 15);

	if (var1 == 0){
		return;   //EVITA LA DIVISION POR CERO
	}

	p = (((uint32_t)(((int32_t)1048576) - adc_P) - (var2 >> 12))) * 3125;

	if (p < 0x80000000){
		p = (p << 1) / ((uint32_t)var1);
	}
	else {
		p = (p / (uint32_t)var1) * 2;
	}

	var1 = (((int32_t)dig_P9) * ((int32_t)(((p >> 3) * (p >> 3)) >> 13))) >> 12;
	var2 = (((int32_t)(p >> 2)) * ((int32_t)dig_P8)) >> 13;
	presion_actual = (uint32_t)((int32_t)p + ((var1 + var2 + dig_P7) >> 4));

	/*LA PRIMERA MEDIDA BUENA SE GUARDA COMO REFERENCIA: ES EL CERO DEL ALTIMETRO.
	 * ASI LA ALTURA QUE REPORTAMOS ES SOBRE EL PUNTO DE DESPEGUE, QUE ES LA UTIL*/
	if (presion_referencia == 0){
		presion_referencia = presion_actual;
	}

	/*FORMULA BAROMETRICA: ALTURA A PARTIR DE LA RELACION DE PRESIONES*/
	altura_actual = (int)(44330.0f * (1.0f - powf((float)presion_actual / (float)presion_referencia, 0.1903f)));
}

void avion_Init(void){

	modo_seteado(modo_de_operacion);

}

void modo_seteado(uint8_t modo_de_operacion){
	switch(modo_de_operacion){
	case NO_ARMADO:
		delta_servo = 50;
		duty_motor = 1000;
		tipo_tecla_oprimida = 0;
		break;
	case ARMADO:
		tipo_tecla_oprimida = 0;
		delta_servo = 50;
		duty_motor = 1000;
		break;
	case TAKE_OFF:
		delta_servo = 50;
		duty_motor = 2000;
		tipo_tecla_oprimida = 0;
		break;
	case CRUISE:
		delta_servo = 20;
		duty_motor = 1550;
		tipo_tecla_oprimida = 0;
		break;
	case LANDING:
		delta_servo = 25;
		duty_motor = 1200;
		tipo_tecla_oprimida = 0;
		break;
	case FAIL_SAFE:
		delta_servo = 20;
		duty_motor = 1000;
		tipo_tecla_oprimida = 0;
		break;
	default:
		break;
	}
}

void movimiento(void){

	if (HAL_GetTick() -timeout_pitch > 150){
		pitch = CENTRO;
	}
	if (HAL_GetTick() -timeout_roll > 150){
			roll = CENTRO;
	}
	if (HAL_GetTick() -timeout_yaw > 150){
			yaw = CENTRO;
	}
	if (HAL_GetTick() - timeout_failsafe > 8000){
		modo_de_operacion = FAIL_SAFE;
	}

	/*EN QUE CASO SE HABILITA O DESHABILITA ALGO?
	 * CUANDO SE SETEA UNO DE LOS EJES A CENTRO ES PORQUE
	 * QUEREMOS EVITAR QUE AL PASAR DE UN ESTADO QUE TIENE LA POSICION DE LOS CONTROLES FIJOS A UN ESTADO QUE PERMITE CAMBIARLOS
	 * PARTA DE UN ESTADO INICIAL CENTRAL. DE ESTA MANERA LAS TRANSICIONES NO SON BRUSCAS.
	 * LOS ESTADOS DE REPOSO TIENEN LIJERAS DEFLECTACIONES PARA PERMITIR QUE EL ALA SUSTENTE MAS A MAS BAJA VELOCIDAD*/
	reposo_roll_izq = 1500;
	reposo_roll_der = 1500;
	reposo_pitch    = 1500;
	reposo_yaw      = 1500;
	switch (modo_de_operacion){
	case NO_ARMADO:
		reposo_pitch = 1500;
		reposo_roll_der = 1500;
		reposo_roll_izq = 1500;
		reposo_yaw = 1500;
		break;
	case ARMADO:
		reposo_pitch = 1500;
		reposo_roll_der = 1500;
		reposo_roll_izq = 1500;
		reposo_yaw = 1500;
		break;
	case TAKE_OFF:
		roll = CENTRO;
		reposo_roll_izq = 1700;
		reposo_roll_der = 1300;
		break;
	case CRUISE:
		reposo_pitch = 1500;
		reposo_roll_der = 1500;
		reposo_roll_izq = 1500;
		reposo_yaw = 1500;
		break;
	case LANDING:
		roll = CENTRO;
		reposo_roll_izq = 1700;
		reposo_roll_der = 1300;
		break;

	case FAIL_SAFE:
		roll = CENTRO;
		pitch = CENTRO;
		yaw = CENTRO;
		reposo_pitch = 1500;
		reposo_roll_der = 1500;
		reposo_roll_izq = 1500;
		reposo_yaw = 1500;
		break;
	default:
		break;
	}
		//ALABEO
	switch (roll){
	case POSITIVO:
		if (duty_aleron_der < 2000){
			duty_aleron_der += delta_servo;
			duty_aleron_izq += delta_servo;
		}
		break;
	case NEGATIVO:
		if (duty_aleron_der > 1000){
			duty_aleron_izq -= delta_servo;
			duty_aleron_der -= delta_servo;
		}
		break;
	case CENTRO:
		if (duty_aleron_der > reposo_roll_der){
			duty_aleron_izq -= delta_servo;
			duty_aleron_der -= delta_servo;
		}
		else if (duty_aleron_der < reposo_roll_der){
			duty_aleron_der += delta_servo;
			duty_aleron_izq += delta_servo;

		}
		break;

	}

	//CABECEO
	switch(pitch){
	case POSITIVO:
		if (duty_elevador < 2000){
			duty_elevador += delta_servo;
		}
		break;
	case NEGATIVO:
		if (duty_elevador > 1000){
			duty_elevador -= delta_servo;
		}
		break;
	case CENTRO:
		if (duty_elevador > reposo_pitch){
			duty_elevador -= delta_servo;
		}
		else if (duty_elevador < reposo_pitch){
			duty_elevador += delta_servo;

		}
		break;
	}

	//GUIÑADA
	switch(yaw){
	case POSITIVO:
		if (duty_timon < 2000){
			duty_timon += delta_servo;
		}
		break;
	case NEGATIVO:
		if (duty_timon > 1000){
			duty_timon -= delta_servo;
		}
		break;
	case CENTRO:
		if (duty_timon > reposo_yaw){
			if (duty_timon - reposo_yaw <= delta_servo)
				duty_timon = reposo_yaw;
			else
				duty_timon -= delta_servo;
		}
		else if (duty_timon < reposo_yaw){
			if (reposo_yaw - duty_timon <= delta_servo)
				duty_timon = reposo_yaw;
			else
				duty_timon += delta_servo;
		}
		break;
	}

	//ACTUALIZACION DE TODA LA INFORMACION
	switch (modo_de_operacion){
		case NO_ARMADO:
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1, duty_aleron_izq);
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2, duty_aleron_der);
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4, duty_elevador);
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3, duty_timon);
			__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1, duty_motor);
			break;
		case ARMADO:
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1, duty_aleron_izq);
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2, duty_aleron_der);
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4, duty_elevador);
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3, duty_timon);
			__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1, duty_motor);
			break;
		case TAKE_OFF:
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1, reposo_roll_izq);   //LOS ALERONES PERMANECEN ESTATICOS Y UN POCO
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2, reposo_roll_der);   //DEFLECTADOS PARA DAR MAYOR SUSTENTACION EN EL DESPEGUE
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4, duty_elevador);
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3, duty_timon);
			__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1, duty_motor);
			break;
		case CRUISE:
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1, duty_aleron_izq);
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2, duty_aleron_der);
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4, duty_elevador);
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3, duty_timon);
			__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1, duty_motor);
			break;
		case LANDING:
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1, reposo_roll_izq);
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2, reposo_roll_der);
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4, duty_elevador);
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3, duty_timon);
			__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1, duty_motor);
			break;
		case FAIL_SAFE:
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1, reposo_roll_izq);
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2, reposo_roll_der);
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4, reposo_pitch);
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3, reposo_yaw);
			__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1, duty_motor);
			break;
		default:
			break;
	}

}

void cambio_fsm (uint8_t tecla_oprimida){
	switch (tecla_oprimida){
	case 'a':
		roll = POSITIVO;
		timeout_roll = HAL_GetTick();
		break;
	case 'd':
		roll = NEGATIVO;
		timeout_roll = HAL_GetTick();
		break;
	case'w':
		pitch = POSITIVO;
		timeout_pitch = HAL_GetTick();
		break;
	case's':
		pitch = NEGATIVO;
		timeout_pitch = HAL_GetTick();
		break;
	case'l' :
		yaw = POSITIVO;
		timeout_yaw = HAL_GetTick();
		break;
	case'j':
		yaw = NEGATIVO;
		timeout_yaw = HAL_GetTick();
		break;

		/*
		 * p armed
		 * o landing
		 * i cruise
		 * u take off
		 * ' ' disarmed*/

	case 'p':
		modo_de_operacion = ARMADO;
		break;
	case 'o':
		modo_de_operacion = LANDING;
		break;
	case 'i':
		modo_de_operacion = TAKE_OFF;
		break;
	case'u':
		modo_de_operacion = CRUISE;
		break;
	case' ':
		modo_de_operacion = NO_ARMADO;
		break;
	default:
		break;
	}
}


//INTERRUPCIONES
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM4)
	{
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);  //SE CAMBIA EL ESTADO DEL PIN ENTRE HIGH Y LOW

	}

	if (htim->Instance == TIM3)
	{
		BANDERA_MOVIMIENTO = 1;

	}
}




void HAL_UART_RxCpltCallback(UART_HandleTypeDef * huart){
	if (huart->Instance == USART2){
		dato_recibido = 1;  //SE ACTIVA UNA BANDERA PARA QUE LA LOGICA FUERTE SE EJECUTE DENTRO DEL MAIN Y NO EN LA INTERRUPCION
		HAL_UART_Receive_IT(huart, &RXchange, 1);  //SE BAJA LA BANDERA ESPERANDO QUE HAYA UNA NUEVA INTERRUPCION
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart){

	if (huart->Instance == USART2){          //

		if (huart->ErrorCode  & HAL_UART_ERROR_ORE){
			// subir contador de overrun
		}
		if (huart->ErrorCode  & HAL_UART_ERROR_NE){
			// subir contador de ruido
		}
		if (huart->ErrorCode  & HAL_UART_ERROR_FE){
			// subir contador de trama
		}
		__HAL_UART_CLEAR_PEFLAG(huart);
		HAL_UART_Receive_IT(huart, &RXchange, 1);// limpiar banderas      → macro de la familia __HAL_UART_CLEAR_
		// rearmar recepción     → la misma llamada que tienes en uart_Init()
	}

}

