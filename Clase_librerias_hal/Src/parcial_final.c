/*
 * parcial_final.c
 *
 *  Created on: Jul 21, 2026
 *      Author: marcop
 */

/* ===================== ESPECIFICACION DE LA MAQUINA DE ESTADOS =====================
      *
      * ESTADO PRINCIPAL:  'pantalla'  (tipo estado) -> cronometro | mpu | mco
      * SUB-ESTADO:        'tomar_tiempo' -> solo tiene sentido dentro de cronometro
      *                       0 = no estoy en cronometro
      *                       1 = estoy en cronometro, aun no he tomado vuelta
      *                       2 = estoy en cronometro y ya tome al menos una vuelta
      *                    Arranca en 1 porque el estado inicial YA es cronometro.
      *
      * ----------------------- TABLA DE TRANSICIONES (Mealy) -----------------------
      * Se evalua en actualizacion_caso(), una sola vez por cada caracter recibido.
      *
      *  ESTADO ORIGEN      COMANDO   ACCION DE ENTRADA                       DESTINO
      *  -----------------  --------  --------------------------------------  ----------
      *  cualquiera         'h'       mco1_Init('h') -> MCO1 = HSI / 4        mco
      *  cualquiera         'l'       mco1_Init('l') -> MCO1 = LSE / 1        mco
      *  cualquiera         'p'       mco1_Init('p') -> MCO1 = PLLCLK / 5     mco
      *  cualquiera         'n'       (ninguna)                               mpu
      *  mpu o mco          'r'       (ninguna: solo navega)                  cronometro
      *  cronometro         'r'       actualizacion_cronometro():             cronometro
      *                                 guarda la vuelta en get_time y
      *                                 reinicia el RTC a 00:00:00
      *  cualquiera         otro      (ninguna)                               sin cambio
      *
      * NOTA: 'r' es el unico comando cuyo efecto depende del estado de origen.
      *       Por eso la maquina es Mealy en esta transicion: la salida depende
      *       del estado actual Y de la entrada, no solo de la entrada.
      *
      * ------------------------- SALIDAS POR ESTADO (Moore) -------------------------
      * Se ejecutan en graficar_pantalla(), en cada vuelta del while(1).
      * Dependen UNICAMENTE del estado, no del comando que me trajo hasta aqui.
      *
      *  ESTADO       SALIDA
      *  -----------  ----------------------------------------------------------
      *  cronometro   Lee el RTC y dibuja "CRONOMETRO" + HH:MM:SS en curso.
      *               Si nuevo_tiempo == 1, dibuja tambien la ultima vuelta.
      *  mpu          actualizacion_mpu(): lee el MPU6050 por I2C y dibuja Ax/Ay/Az.
      *               (El sensor SOLO se lee estando en este estado.)
      *  mco          Dibuja que fuente esta saliendo por PA8, segun mco1_output.
      *
      * REPOSO: no hay estado IDLE. Al no llegar ningun comando, la maquina
      *         permanece en su estado re-ejecutando su salida, que es el
      *         comportamiento de reposo propio de una maquina de Moore.
      * ============================================================================== */

#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdio.h>
#include <stdint.h>

//----------VARIABLES------------
TIM_HandleTypeDef htim4;
UART_HandleTypeDef huart2;
RTC_HandleTypeDef hrtc;
I2C_HandleTypeDef hi2c;
//necesario para i2c
#define WHOAMI (0x68 << 1)
#define SMPLRT_DIV_REG 0x19
#define  ACCEL_CONFIG_REG 0x1C
#define PWR_MGMT_1_REG 0x6B
#define ACCEL_XOUT_H_REG 0x3B

int16_t Accel_X_raw, Accel_Y_raw, Accel_Z_raw;
float Ax, Ay, Az;

//variables para rtc
uint32_t status_rtc = 0;
//variable que recibe la información de la comunicacion serial
volatile uint8_t RXchange = 0;  //VARIABLE QUE ALMACENA LA INFORMACION QUE LLEGA DEL COMPUTADOR EN LA COMUNICACION SERIAL, USADA PARA SUBIR O BAJAR PWM MEDIANTE USART

/*VARIABLES BANDERA PARA LAS IRQ*/
uint8_t huart_flag = 0;

// ********* OLED **************
#define SSD1306_ADDR 0x78
static uint8_t SSD1306_Buffer[1024];


#define DS1307_ADDR 0xD0
// --- 5x7 ASCII FONT ---
const uint8_t Font5x7[][5] = {
    {0x00, 0x00, 0x00, 0x00, 0x00}, // 32 Space
    {0x00, 0x00, 0x5F, 0x00, 0x00}, // 33 !
    {0x00, 0x07, 0x00, 0x07, 0x00}, // 34 "
    {0x14, 0x7F, 0x14, 0x7F, 0x14}, // 35 #
    {0x24, 0x2A, 0x7F, 0x2A, 0x12}, // 36 $
    {0x23, 0x13, 0x08, 0x64, 0x62}, // 37 %
    {0x36, 0x49, 0x55, 0x22, 0x50}, // 38 &
    {0x00, 0x05, 0x03, 0x00, 0x00}, // 39 '
    {0x00, 0x1C, 0x22, 0x41, 0x00}, // 40 (
    {0x00, 0x41, 0x22, 0x1C, 0x00}, // 41 )
    {0x14, 0x08, 0x3E, 0x08, 0x14}, // 42 *
    {0x08, 0x08, 0x3E, 0x08, 0x08}, // 43 +
    {0x00, 0x50, 0x30, 0x00, 0x00}, // 44 ,
    {0x08, 0x08, 0x08, 0x08, 0x08}, // 45 -
    {0x00, 0x60, 0x60, 0x00, 0x00}, // 46 .
    {0x20, 0x10, 0x08, 0x04, 0x02}, // 47 /
    {0x3E, 0x51, 0x49, 0x45, 0x3E}, // 48 0
    {0x00, 0x42, 0x7F, 0x40, 0x00}, // 49 1
    {0x42, 0x61, 0x51, 0x49, 0x46}, // 50 2
    {0x21, 0x41, 0x45, 0x4B, 0x31}, // 51 3
    {0x18, 0x14, 0x12, 0x7F, 0x10}, // 52 4
    {0x27, 0x45, 0x45, 0x45, 0x39}, // 53 5
    {0x3C, 0x4A, 0x49, 0x49, 0x30}, // 54 6
    {0x01, 0x71, 0x09, 0x05, 0x03}, // 55 7
    {0x36, 0x49, 0x49, 0x49, 0x36}, // 56 8
    {0x06, 0x49, 0x49, 0x29, 0x1E}, // 57 9
    {0x00, 0x36, 0x36, 0x00, 0x00}, // 58 :
    {0x00, 0x56, 0x36, 0x00, 0x00}, // 59 ;
    {0x08, 0x14, 0x22, 0x41, 0x00}, // 60 <
    {0x14, 0x14, 0x14, 0x14, 0x14}, // 61 =
    {0x00, 0x41, 0x22, 0x14, 0x08}, // 62 >
    {0x02, 0x01, 0x51, 0x09, 0x06}, // 63 ?
    {0x32, 0x49, 0x79, 0x41, 0x3E}, // 64 @
    {0x7E, 0x11, 0x11, 0x11, 0x7E}, // 65 A
    {0x7F, 0x49, 0x49, 0x49, 0x36}, // 66 B
    {0x3E, 0x41, 0x41, 0x41, 0x22}, // 67 C
    {0x7F, 0x41, 0x41, 0x22, 0x1C}, // 68 D
    {0x7F, 0x49, 0x49, 0x49, 0x41}, // 69 E
    {0x7F, 0x09, 0x09, 0x09, 0x01}, // 70 F
    {0x3E, 0x41, 0x49, 0x49, 0x7A}, // 71 G
    {0x7F, 0x08, 0x08, 0x08, 0x7F}, // 72 H
    {0x00, 0x41, 0x7F, 0x41, 0x00}, // 73 I
    {0x20, 0x40, 0x41, 0x3F, 0x01}, // 74 J
    {0x7F, 0x08, 0x14, 0x22, 0x41}, // 75 K
    {0x7F, 0x40, 0x40, 0x40, 0x40}, // 76 L
    {0x7F, 0x02, 0x0C, 0x02, 0x7F}, // 77 M
    {0x7F, 0x04, 0x08, 0x10, 0x7F}, // 78 N
    {0x3E, 0x41, 0x41, 0x41, 0x3E}, // 79 O
    {0x7F, 0x09, 0x09, 0x09, 0x06}, // 80 P
    {0x3E, 0x41, 0x51, 0x21, 0x5E}, // 81 Q
    {0x7F, 0x09, 0x19, 0x29, 0x46}, // 82 R
    {0x46, 0x49, 0x49, 0x49, 0x31}, // 83 S
    {0x01, 0x01, 0x7F, 0x01, 0x01}, // 84 T
    {0x3F, 0x40, 0x40, 0x40, 0x3F}, // 85 U
    {0x1F, 0x20, 0x40, 0x20, 0x1F}, // 86 V
    {0x3F, 0x40, 0x38, 0x40, 0x3F}, // 87 W
    {0x63, 0x14, 0x08, 0x14, 0x63}, // 88 X
    {0x07, 0x08, 0x70, 0x08, 0x07}, // 89 Y
    {0x61, 0x51, 0x49, 0x45, 0x43}, // 90 Z
    {0x00, 0x7F, 0x41, 0x41, 0x00}, // 91 [
    {0x02, 0x04, 0x08, 0x10, 0x20}, // 92 backslash
    {0x00, 0x41, 0x41, 0x7F, 0x00}, // 93 ]
    {0x04, 0x02, 0x01, 0x02, 0x04}, // 94 ^
    {0x40, 0x40, 0x40, 0x40, 0x40}, // 95 _
    {0x00, 0x01, 0x02, 0x04, 0x00}, // 96 `
    {0x20, 0x54, 0x54, 0x54, 0x78}, // 97 a
    {0x7F, 0x48, 0x44, 0x44, 0x38}, // 98 b
    {0x38, 0x44, 0x44, 0x44, 0x20}, // 99 c
    {0x38, 0x44, 0x44, 0x48, 0x7F}, // 100 d
    {0x38, 0x54, 0x54, 0x54, 0x18}, // 101 e
    {0x08, 0x7E, 0x09, 0x01, 0x02}, // 102 f
    {0x0C, 0x52, 0x52, 0x52, 0x3E}, // 103 g
    {0x7F, 0x08, 0x04, 0x04, 0x78}, // 104 h
    {0x00, 0x44, 0x7D, 0x40, 0x00}, // 105 i
    {0x20, 0x40, 0x44, 0x3D, 0x00}, // 106 j
    {0x7F, 0x10, 0x28, 0x44, 0x00}, // 107 k
    {0x00, 0x41, 0x7F, 0x40, 0x00}, // 108 l
    {0x7C, 0x04, 0x18, 0x04, 0x78}, // 109 m
    {0x7C, 0x08, 0x04, 0x04, 0x78}, // 110 n
    {0x38, 0x44, 0x44, 0x44, 0x38}, // 111 o
    {0x7C, 0x14, 0x14, 0x14, 0x08}, // 112 p
    {0x08, 0x14, 0x14, 0x18, 0x7C}, // 113 q
    {0x7C, 0x08, 0x04, 0x04, 0x08}, // 114 r
    {0x48, 0x54, 0x54, 0x54, 0x20}, // 115 s
    {0x04, 0x3F, 0x44, 0x40, 0x20}, // 116 t
    {0x3C, 0x40, 0x40, 0x20, 0x7C}, // 117 u
    {0x1C, 0x20, 0x40, 0x20, 0x1C}, // 118 v
    {0x3C, 0x40, 0x30, 0x40, 0x3C}, // 119 w
    {0x44, 0x28, 0x10, 0x28, 0x44}, // 120 x
    {0x0C, 0x50, 0x50, 0x50, 0x3C}, // 121 y
    {0x44, 0x64, 0x54, 0x4C, 0x44}, // 122 z
    {0x00, 0x08, 0x36, 0x41, 0x00}, // 123 {
    {0x00, 0x00, 0x7F, 0x00, 0x00}, // 124 |
    {0x00, 0x41, 0x36, 0x08, 0x00}, // 125 }
    {0x0C, 0x02, 0x0C, 0x02, 0x0C}, // 126 ~
};

/* ------ OLED Drivers ------- */
void SSD1306_Init(void);
void SSD1306_Fill(uint8_t color);
void SSD1306_UpdateScreen(void);
void SSD1306_DrawPixel(uint8_t x, uint8_t y, uint8_t color);
void SSD1306_WriteString(uint8_t x, uint8_t y, char* str);




//VARIABLES Y TEXTOS QUE SE USAN PARA REPRESENTAR EN LAS PANTALLAS

uint8_t tiempo_actual[144] = {0};
char mco1_output = 'h';
char msg[128] = {0};
uint8_t get_time[144] = {0};
uint8_t nuevo_tiempo = 0;
 int tomar_tiempo = 1;


//ESTADOS DE MI MAQUINA DE ESTADOS
//ESTA VARIABLE REPRESENTA EEN QUE ESTADO ME ENCUENTRO, Y ESTO SE TRADUCE EN QU[É PANTALLA SE ESTÁ VISUALIZANDO
typedef enum {

	cronometro,
	mpu,
	mco,

} estado;

estado pantalla = cronometro;


//HEADERS
static void rtc_Init(void);
static void ledok_Init(void);
static void uart_Init (void);
void i2c_init(void);
void mpu6050_Init(void);
void mpu6050_Read (void);
void mco1_Init(char);
void graficar_pantalla(void);
void actualizacion_caso(uint8_t);
void actualizacion_cronometro(void);
void actualizacion_mpu(void);
//MAIN
int main(void){
	HAL_Init();
	ledok_Init();
	rtc_Init();
	uart_Init();
	i2c_init();
	SSD1306_Init();
	mpu6050_Init();


	while(1){
		/*MAQUINA DE ESTADOS. EN LA PRACTICA 3 COMETÍ EL ERROR DE RESOLVER MUCHAS COSAS EN LAS IRQ
		 * EN ESTE CODIGO EN CADA INTERRUPCION SIMPLEMENTE LEVANTARÉ UNA BANDERA QUE SE LEE EN ESTE WHILE Y SE EJECUTA LA ACCION CORRESPONDIENTE*/
		if (huart_flag == 1){
			huart_flag = 0;//Se baja bandera para el IRQ ya que ya se esta atendiendo la onterrupcion
			actualizacion_caso(RXchange);
		}
		graficar_pantalla();

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

	status_rtc = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR0);
	/*ESTOY USANDO LOS REGISTROS DE BACKUP PARA VERIFICAR SI EL RELOJ YA HA SIDO ATIVADO ANTES. SI HAY UN
	 * 1 EN EL REGISTRO, ES PORQUE YA HA SIDO ACTIVADO Y ESTA EJECUTANDOSE SIN QUE SE DESCONECTE VBAT, SIN EMBARGO
	 * SI ES 0 SE INCIA INICIALIZA EL RELOJ*/
	if (status_rtc == 0){

		//SE INICIALIZA LA FECHA Y LA HORA EN 0
		RTC_TimeTypeDef time_format = {0};
		RTC_DateTypeDef date_format = {0};
		HAL_RTC_SetTime(&hrtc, &time_format, RTC_FORMAT_BIN);
		HAL_RTC_SetDate(&hrtc, &date_format, RTC_FORMAT_BIN);

		HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR0, 1);
	}

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
	htim4.Init.Prescaler = 9999;  //PRESCALER SE CONFIGURA A 10KHZ
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 2499;  // EL LED PARPADEA CADA 250 ms
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
	HAL_UART_Receive_IT(&huart2,&RXchange, 1);
	/*CONFIGURACION DE LA INTERUPCION EN EL NVIC*/
	HAL_NVIC_EnableIRQ(USART2_IRQn);

}

void i2c_init(void){

	__HAL_RCC_I2C1_CLK_ENABLE();

	__HAL_RCC_GPIOB_CLK_ENABLE();

	GPIO_InitTypeDef GPIO_Init = {0};

	//PB6 --> SCL
	//PB7 --> SDA

	GPIO_Init.Pin =  GPIO_PIN_6 | GPIO_PIN_7;  //CONFIGURACION DE PINES DE SDA(PB7) Y SCL(PB6)
	GPIO_Init.Mode = GPIO_MODE_AF_OD;
	GPIO_Init.Pull = GPIO_NOPULL;
	GPIO_Init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_Init.Alternate = GPIO_AF4_I2C1;

	HAL_GPIO_Init(GPIOB, &GPIO_Init);


	//configuracion del I2C

	hi2c.Instance = I2C1;
	//velocidad de transmicion estandar
	hi2c.Init.ClockSpeed = 400000;
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

/*------FUNCIONES DE LA PANTALLA OLED-----*/
void WriteCmd(uint8_t c) {
	HAL_I2C_Mem_Write(&hi2c, SSD1306_ADDR, 0x00, 1, &c, 1, 10);
	// hi2c        -> I2C handle (PB6/PB7, configured in i2c1_Init)
	// SSD1306_ADDR -> 7-bit I2C slave address of the display (0x78, already shifted for R/W)
	// 0x00         -> "control register": D/C#=0 -> command mode
	// 1            -> control register address is 1 byte wide
	// &c, 1        -> the command byte itself, 1 byte to send
	// 10           -> I2C timeout in milliseconds
}


/*
 * SSD1306_Init
 *
 * SSD1306 controller startup sequence (Solomon Systech datasheet,
 * Rev 1.1, sections 9 "Command Table" and 10 "Command Descriptions").
 *
 * Leaves the controller configured with:
 *   - Page Addressing Mode (one "page" = 8 pixel rows, 8 pages -> 64 rows)
 *   - "Mirrored" orientation (Segment Remap + inverted COM Scan) so the
 *     module reads correctly given the typical physical mounting of these OLEDs
 *   - Multiplex 1:64, offset 0 -> 128x64 panel
 *   - Internal charge pump enabled (mandatory since the module is powered
 *     only from logic VDD, with no external panel VCC)
 *   - Display kept OFF until the very last command (0xAF), so no garbage
 *     is shown while it's being configured
 *
 * Each byte is sent with WriteCmd(), which sets D/C#=0 (command) by writing
 * to the SSD1306's 0x00 "control register" over I2C (see section 8.1.5).
 */
void SSD1306_Init(void) {
    HAL_Delay(100);                // Stabilization delay after power-on (sec. 8.9 Power ON sequence)

	WriteCmd(0xAE);                // Display OFF (sleep mode) while configuring (sec. 10.1.12)

	WriteCmd(0x20);                // Set Memory Addressing Mode (sec. 10.1.3)...
	WriteCmd(0x10);                // ...argument: A[1:0]=10b -> Page Addressing Mode

	WriteCmd(0xB0);                // Set Page Start Address = PAGE0, for Page Addressing Mode (sec. 10.1.13)

	WriteCmd(0xC8);                // Set COM Output Scan Direction remapped: COM[N-1] -> COM0 (sec. 10.1.14),
	                                // flips the image vertically to match the module's mounting

	WriteCmd(0x00);                // Set Lower Column Start Address = 0 (low nibble) (sec. 10.1.1)
	WriteCmd(0x10);                // Set Higher Column Start Address = 0 (high nibble) (sec. 10.1.2)

	WriteCmd(0x40);                // Set Display Start Line = 0 (RAM row 0 -> COM0) (sec. 10.1.6)

	WriteCmd(0x81);                // Set Contrast Control for BANK0 (sec. 10.1.7)...
	WriteCmd(0xFF);                // ...argument: contrast = 0xFF (maximum, out of 256 steps)

	WriteCmd(0xA1);                // Set Segment Re-map: column 127 -> SEG0 (sec. 10.1.8),
	                                // flips the image horizontally (together with 0xC8, corrects the mounting)

	WriteCmd(0xA6);                // Set Normal Display: a RAM bit of 1 = pixel ON (sec. 10.1.10)

	WriteCmd(0xA8);                // Set Multiplex Ratio (sec. 10.1.11)...
	WriteCmd(0x3F);                // ...argument: N-1=63 -> MUX 1:64 (64-row panel)

	WriteCmd(0xA4);                // Entire Display ON = follows GDDRAM content (does not force everything ON) (sec. 10.1.9)

	WriteCmd(0xD3);                // Set Display Offset (sec. 10.1.15)...
	WriteCmd(0x00);                // ...argument: no vertical shift (offset = 0)

	WriteCmd(0xD5);                // Set Display Clock Divide Ratio / Oscillator Frequency (sec. 10.1.16)...
	WriteCmd(0xF0);                // ...argument: A[7:4]=Fosc=1111b (high frequency), A[3:0]=divide ratio-1=0 (ratio=1)

	WriteCmd(0xD9);                // Set Pre-charge Period (sec. 10.1.17)...
	WriteCmd(0x22);                // ...argument: Phase1=2 DCLK, Phase2=2 DCLK (default values)

	WriteCmd(0xDA);                // Set COM Pins Hardware Configuration (sec. 10.1.18)...
	WriteCmd(0x12);                // ...argument: A[4]=1 alternative COM pin config, A[5]=0 no left/right remap

	WriteCmd(0xDB);                // Set VCOMH Deselect Level (sec. 10.1.19)...
	WriteCmd(0x20);                // ...argument: ~0.77 x VCC (default value)

	WriteCmd(0x8D);                // Charge Pump Setting (not detailed in this datasheet revision, but it's the
	                                // standard SSD1306 command for the internal charge-pump regulator)...
	WriteCmd(0x14);                // ...argument: 0x14 = enables the internal charge pump
	                                // (mandatory: without it, the OLED panel doesn't get enough voltage)

	WriteCmd(0xAF);                // Display ON: exits sleep mode and starts refreshing the panel (sec. 10.1.12)
}

/*
 * SSD1306_Fill
 *
 * Fills the whole local framebuffer (SSD1306_Buffer, 1024 bytes = 128 columns
 * x 8 pages, one byte per column per page) with a solid color. This only
 * touches the RAM copy in the MCU — nothing is sent to the display until
 * SSD1306_UpdateScreen() pushes the buffer out over I2C.
 */
void SSD1306_Fill(uint8_t color) {
	memset(SSD1306_Buffer, (color == 0)?0:0xFF, sizeof(SSD1306_Buffer));
	// color == 0 -> 0x00 per byte (all 8 pixels of the byte OFF)
	// color != 0 -> 0xFF per byte (all 8 pixels of the byte ON)
}

/*
 * SSD1306_UpdateScreen
 *
 * Flushes the local framebuffer (SSD1306_Buffer) to the SSD1306's GDDRAM
 * over I2C, one page (8-pixel-tall row) at a time, matching the Page
 * Addressing Mode configured in SSD1306_Init() (sec. 10.1.3).
 */
void SSD1306_UpdateScreen(void) {
    for(int i=0; i<8; i++) {
        // Point the controller at the start of page i, column 0, before
        // sending that page's data (sec. 10.1.13 Page Start Address,
        // sec. 10.1.1/10.1.2 Column Start Address low/high nibble)
        WriteCmd(0xB0+i);
        WriteCmd(0x00);
        WriteCmd(0x10);

        // Write the 128 bytes of page i to GDDRAM. Using register 0x40
        // (instead of 0x00) sets D/C#=1, i.e. "this is pixel data, not a
        // command" (sec. 8.1.5). The column pointer auto-increments after
        // each byte (Table 9-3), so the whole page is written in one shot.
        HAL_I2C_Mem_Write(&hi2c, SSD1306_ADDR, 0x40, 1, &SSD1306_Buffer[128*i], 128, 50);
    }
}

/*
 * SSD1306_DrawPixel
 *
 * Sets or clears a single pixel in the local framebuffer, using the same
 * byte layout the SSD1306 expects in GDDRAM: the buffer is organized as
 * 8 pages of 128 columns, and within a byte, bit 0 is the top row of the
 * page and bit 7 is the bottom row (LSB-first, vertically).
 */
void SSD1306_DrawPixel(uint8_t x, uint8_t y, uint8_t color) {
    if(x>=128 || y>=64) return;                                   // off-screen, ignore (also catches the
                                                                     // x-1/y-1 underflow used by SSD1306_DrawCross)
    if(color){
    	SSD1306_Buffer[x+(y/8)*128] |= (1<<(y%8));          // turn the pixel ON: set its bit
    }
    else {
    	SSD1306_Buffer[x+(y/8)*128] &= ~(1<<(y%8));         // turn the pixel OFF: clear its bit
    }
    // byte index  x + (y/8)*128 : column x, page (y/8)   -> selects the byte
    // bit index   y % 8                                  -> selects the row within that page
}

/*
 * SSD1306_WriteString
 *
 * Draws a null-terminated ASCII string into the framebuffer starting at
 * pixel (x,y), using the 5x7 bitmap font table (Font5x7[]). Each glyph is
 * 5 pixels wide plus 1 blank column of spacing, so the cursor advances by
 * 6 pixels per character.
 */
void SSD1306_WriteString(uint8_t x, uint8_t y, char* str) {
    while(*str) {
        char c = *str;
        uint8_t idx = 0;
        if (c >= 32 && c <= 126){
        	idx = c - 32;   // Font5x7[] starts at the space character (ASCII 32)
        }
        else {
        	idx = 0;                            // anything outside the printable range falls back to space
        }

        for(int i=0; i<5; i++) {                // walk the 5 columns of the glyph
            uint8_t b = Font5x7[idx][i];        // column i's pixel pattern, bit 0 = top row
            for(int j=0; j<8; j++){				// move cursor to the next character slot (5px glyph + 1px gap)
            	if((b>>j)&1){
            		SSD1306_DrawPixel(x+i, y+j, 1); // plot each set bit as an ON pixel at (x+i, y+j)
            	}
            }

        }
        x += 6;
        str++;
    }
}


/*
 * SSD1306_DrawEmptyRect
 *
 * Draws an unfilled rectangle outline: top-left corner at (x_zero, y_zero),
 * x_wide pixels wide and y_height pixels tall. x_wide/y_height are a SIZE,
 * not an end coordinate — the function converts them to the actual right/
 * bottom edge (x_end/y_end) internally before drawing.
 *
 * Bug history: the previous version used x_wide/y_height directly as the
 * end coordinates in the loop conditions. That happened to look right for
 * the big rectangle call (where the caller passed literal end coordinates
 * that were numerically larger than a real width/height), but broke for the
 * small square call (16, 20, 20, 20): with y_zero == y_height == 20 the
 * vertical loop "for(i=20; i<20; i++)" never ran, and the horizontal loop
 * drew both the "top" and "bottom" edge on the same row 20 — hence only one
 * line was visible instead of a square.
 */
void SSD1306_DrawEmptyRect(uint8_t x_zero, uint8_t y_zero, uint8_t x_wide, uint8_t y_height) {
	uint8_t x_end = x_zero + x_wide;    // right edge column = left edge + width
	uint8_t y_end = y_zero + y_height;  // bottom edge row   = top edge + height

	// Top border (row y_zero) and bottom border (row y_end), from column x_zero to x_end
    for(int i = x_zero; i <= x_end; i++) {
    	SSD1306_DrawPixel(i, y_zero, 1); // Top limit
    	SSD1306_DrawPixel(i, y_end, 1);  // Bottom limit
    }

    // Left border (column x_zero) and right border (column x_end), from row y_zero to y_end
    for(int i = y_zero; i <= y_end; i++) {
    	SSD1306_DrawPixel(x_zero, i, 1);
    	SSD1306_DrawPixel(x_end, i, 1);
    }

}

void mco1_Init(char port){

	GPIO_InitTypeDef gpio_mco = {0};
	/*LO PRIMERO QUE DEBO HACER ES CONFIGURAR EL GPIO8
	 * ESTA CONFIGURACION ES REDUNDANTE YA QUE EN EL ARCHIVO DRIVER DE RCC EN LA FUNCION DE HALL_RCC_MCOConfig
	 * YA ACTIVA POR SI MISMO EL PIN*/

	gpio_mco.Pin = GPIO_PIN_8;
	gpio_mco.Mode = GPIO_MODE_AF_OD ;
	gpio_mco.Pull = GPIO_NOPULL ;
	gpio_mco.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	gpio_mco.Alternate = GPIO_AF0_MCO;

	HAL_GPIO_Init(GPIOA,&gpio_mco);

	mco1_output = port;

	if (port == 'h'){
		HAL_RCC_MCOConfig (RCC_MCO1, RCC_MCO1SOURCE_HSI, RCC_MCODIV_4);
	}
	else if (port == 'l'){
		HAL_RCC_MCOConfig (RCC_MCO1, RCC_MCO1SOURCE_LSE, RCC_MCODIV_1);
	}
	else if (port == 'p'){
		HAL_RCC_MCOConfig (RCC_MCO1, RCC_MCO1SOURCE_PLLCLK, RCC_MCODIV_5);

	}

}

void graficar_pantalla (void){

	switch (pantalla){
	case cronometro:
		SSD1306_Fill(0);

		RTC_TimeTypeDef time_Now = {0};
		RTC_DateTypeDef Date_Now = {0};
		HAL_RTC_GetTime(&hrtc,&time_Now, RTC_FORMAT_BIN);
		HAL_RTC_GetDate(&hrtc,&Date_Now, RTC_FORMAT_BIN);
		sprintf((char *)tiempo_actual, "%02d:%02d:%02d", time_Now.Hours, time_Now.Minutes, time_Now.Seconds);
		SSD1306_WriteString( 34, 0, "CRONOMETRO");
		SSD1306_WriteString(34, 25, (char *)tiempo_actual);
		if (nuevo_tiempo == 1){
			SSD1306_WriteString(34, 35, (char *)get_time);
		}
		SSD1306_UpdateScreen();
		break;



	case mpu:

		actualizacion_mpu();
		break;

	case mco:
		SSD1306_Fill(0);
		SSD1306_WriteString(40, 0, "MCO1");

		if (mco1_output== 'h'){
			SSD1306_WriteString(0, 20, "HSI: 16MHz");
			SSD1306_WriteString(0, 30, "Div4: 4MHz");
		}
		else if (mco1_output == 'l'){
			SSD1306_WriteString(0, 20, "LSE: 32.768KHz");
			SSD1306_WriteString(0, 30, "Div1: 32.768KHz");
		}
		else if (mco1_output == 'p'){
			SSD1306_WriteString(0, 20, "PLL: 100MHz");
			SSD1306_WriteString(0, 30, "Div5: 20MHz");
		}

		SSD1306_UpdateScreen();
		break;

	default: break;
	}
}

/*COMANDOS DE MI MAQUINA DE ESTADOS
 * COMANDO     ACCION QUE EJECUTA    ESTADO DE DESTINO
 * h           mco1_init('h')        mco
 * l           mco1_init('l*)        mco
 * p           mco1_init('p')        mco
 * n           actualizacion_mpu()   mpu
 * r           actualizacion_cronometro();  cronometro*/


void actualizacion_caso (uint8_t caso){
	switch(caso){
		case 'h': case 'p': case 'l':
			pantalla = mco;
			mco1_Init(caso);
			tomar_tiempo = 0;
			break;
		case 'n':
			pantalla = mpu;
			tomar_tiempo = 0;
			break;
		case 'r':
			if (tomar_tiempo < 2){
				tomar_tiempo += 1;
			}
			pantalla = cronometro;
			if (tomar_tiempo == 2){
				actualizacion_cronometro();
			}
			break;
		default:
			break ;
	}
}

void actualizacion_cronometro(void){
	nuevo_tiempo = 1;   //ESTA BANDERA ME DICE QUE YA SE HA OPRIMIDO R AL MENOS UNA VES, NUNCA VUELVO A BAJAR LA BANDERA YA QUE SIEMPRE QUIERO QUE SE MUESTRE EL TIEMPO QUE YA HE TOMADO ANTES
	RTC_TimeTypeDef time_Now = {0};
	RTC_DateTypeDef Date_Now = {0};
	HAL_RTC_GetTime(&hrtc,&time_Now, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc,&Date_Now, RTC_FORMAT_BIN);
	sprintf((char *)get_time, "%02d:%02d:%02d", time_Now.Hours, time_Now.Minutes, time_Now.Seconds);
	RTC_TimeTypeDef time_format = {0};
	RTC_DateTypeDef date_format = {0};
	HAL_RTC_SetTime(&hrtc, &time_format, RTC_FORMAT_BIN);
	HAL_RTC_SetDate(&hrtc, &date_format, RTC_FORMAT_BIN);

}

void actualizacion_mpu(void){
	mpu6050_Read();

	SSD1306_Fill(0);

	SSD1306_WriteString(34, 0, "ACELEROMETRO");

	sprintf((char *)msg, "X:%.2f", Ax);
	SSD1306_WriteString(10, 20, (char *)msg);

	sprintf((char *)msg, "Y:%.2f", Ay);
	SSD1306_WriteString(10, 30, (char *)msg);

	sprintf((char *)msg, "Z:%.2f", Az);
	SSD1306_WriteString(10, 40, (char *)msg);

	SSD1306_UpdateScreen();
}
/* ----------- INTERRUPCIONES -----------*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef * huart){
	if (huart->Instance == USART2){
		huart_flag = 1;  //SE ACTIVA UNA BANDERA PARA QUE LA LOGICA FUERTE SE EJECUTE DENTRO DEL MAIN Y NO EN LA INTERRUPCION
		HAL_UART_Receive_IT(huart, &RXchange, 1);  //SE BAJA LA BANDERA ESPERANDO QUE HAYA UNA NUEVA INTERRUPCION
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM4)
	{
		HAL_GPIO_TogglePin(GPIOH, GPIO_PIN_1);  //SE CAMBIA EL ESTADO DEL PIN ENTRE HIGH Y LOW

	}
}

