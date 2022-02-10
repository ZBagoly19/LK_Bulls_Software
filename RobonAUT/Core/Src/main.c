/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
#include "../ECUAL/SERVO/SERVO.h"
#include "../ECUAL/SERVO/SERVO.c"
#include "../ECUAL/DC_MOTOR/DC_MOTOR.h"
#include "vl53l1_api.h"

#include <limits.h>
#include <stdio.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DC_MOTOR_PWM1 0
#define DC_MOTOR_PWM2 1
#define	SZERVO 0
#define CSUCS_SZAM 32 + 1		// +1 mert nekunk 1-tol kezdodnek a csucsok szamai, de a tombben lesz 0. elem is
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;
DMA_HandleTypeDef hdma_spi3_tx;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim12;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t btnEnable = 0;
uint8_t szervoEnable = 1;
uint8_t motvezEnable = 1;


//Minden uj minta utan ki kell kuldeni
uint8_t leszed[] = { 	0b00000000,		//32 led minta eleje
						0b00000000,		//
						0b00000000,		//
						0b00000000,		//32 led minta vege
						0b11111111,		//8 vezerlojel az U6-ba
						0b11111111		//8 vezerlojel az U5-be
};
uint8_t teszt_minta[] = { 	0b11111100,		//32 led minta eleje
							0b11111000,		//
							0b11110000,		//
							0b11100000,		//32 led minta vege
							0b00001111,		//8 vezerlojel az U6-ba
							0b00001111		//8 vezerlojel az U5-be
};

//hatso minták
uint8_t minta4_adc4h[] = { 	0b10001000,		//32 led minta eleje
							0b10001000,		//
							0b10001000,		//
							0b10001000,		//32 led minta vege
							0b01010111,		//8 vezerlojel az U6-ba
							0b01011111		//8 vezerlojel az U5-be
};
uint8_t minta4_adc3h[] = { 	0b10001000,		//32 led minta eleje
							0b10001000,		//
							0b10001000,		//
							0b10001000,		//32 led minta vege
							0b01011011,		//8 vezerlojel az U6-ba
							0b01011111		//8 vezerlojel az U5-be
};
uint8_t minta4_adc2h[] = { 	0b10001000,		//32 led minta eleje
							0b10001000,		//
							0b10001000,		//
							0b10001000,		//32 led minta vege
							0b01011101,		//8 vezerlojel az U6-ba
							0b01011111		//8 vezerlojel az U5-be
};
uint8_t minta4_adc1h[] = { 	0b10001000,		//32 led minta eleje
							0b10001000,		//
							0b10001000,		//
							0b10001000,		//32 led minta vege
							0b01011110,		//8 vezerlojel az U6-ba
							0b01011111		//8 vezerlojel az U5-be
};
uint8_t minta3_adc4h[] = { 	0b01000100,		//32 led minta eleje
							0b01000100,		//
							0b01000100,		//
							0b01000100,		//32 led minta vege
							0b01010111,		//8 vezerlojel az U6-ba
							0b01011111		//8 vezerlojel az U5-be
};
uint8_t minta3_adc3h[] = { 	0b01000100,		//32 led minta eleje
							0b01000100,		//
							0b01000100,		//
							0b01000100,		//32 led minta vege
							0b01011011,		//8 vezerlojel az U6-ba
							0b01011111		//8 vezerlojel az U5-be
};
uint8_t minta3_adc2h[] = { 	0b01000100,		//32 led minta eleje
							0b01000100,		//
							0b01000100,		//
							0b01000100,		//32 led minta vege
							0b01011101,		//8 vezerlojel az U6-ba
							0b01011111		//8 vezerlojel az U5-be
};
uint8_t minta3_adc1h[] = { 	0b01000100,		//32 led minta eleje
							0b01000100,		//
							0b01000100,		//
							0b01000100,		//32 led minta vege
							0b01011110,		//8 vezerlojel az U6-ba
							0b01011111		//8 vezerlojel az U5-be
};
uint8_t minta2_adc4h[] = { 	0b00100010,		//32 led minta eleje
							0b00100010,		//
							0b00100010,		//
							0b00100010,		//32 led minta vege
							0b01010111,		//8 vezerlojel az U6-ba
							0b01011111		//8 vezerlojel az U5-be
};
uint8_t minta2_adc3h[] = { 	0b00100010,		//32 led minta eleje
							0b00100010,		//
							0b00100010,		//
							0b00100010,		//32 led minta vege
							0b01011011,		//8 vezerlojel az U6-ba
							0b01011111		//8 vezerlojel az U5-be
};
uint8_t minta2_adc2h[] = { 	0b00100010,		//32 led minta eleje
							0b00100010,		//
							0b00100010,		//
							0b00100010,		//32 led minta vege
							0b01011101,		//8 vezerlojel az U6-ba
							0b01011111		//8 vezerlojel az U5-be
};
uint8_t minta2_adc1h[] = { 	0b00100010,		//32 led minta eleje
							0b00100010,		//
							0b00100010,		//
							0b00100010,		//32 led minta vege
							0b01011110,		//8 vezerlojel az U6-ba
							0b01011111		//8 vezerlojel az U5-be
};
uint8_t minta1_adc4h[] = { 	0b00010001,		//32 led minta eleje
							0b00010001,		//
							0b00010001,		//
							0b00010001,		//32 led minta vege
							0b01010111,		//8 vezerlojel az U6-ba
							0b01011111		//8 vezerlojel az U5-be
};
uint8_t minta1_adc3h[] = { 	0b00010001,		//32 led minta eleje
							0b00010001,		//
							0b00010001,		//
							0b00010001,		//32 led minta vege
							0b01011011,		//8 vezerlojel az U6-ba
							0b01011111		//8 vezerlojel az U5-be
};
uint8_t minta1_adc2h[] = { 	0b00010001,		//32 led minta eleje
							0b00010001,		//
							0b00010001,		//
							0b00010001,		//32 led minta vege
							0b01011101,		//8 vezerlojel az U6-ba
							0b01011111		//8 vezerlojel az U5-be
};
uint8_t minta1_adc1h[] = { 	0b00010001,		//32 led minta eleje
							0b00010001,		//
							0b00010001,		//
							0b00010001,		//32 led minta vege
							0b01011110,		//8 vezerlojel az U6-ba
							0b01011111		//8 vezerlojel az U5-be
};

//elso minták
uint8_t minta4_adc4e[] = { 	0b10001000,		//32 led minta eleje
							0b10001000,		//
							0b10001000,		//
							0b10001000,		//32 led minta vege
							0b01011111,		//8 vezerlojel az U6-ba
							0b01010111		//8 vezerlojel az U5-be
};
uint8_t minta4_adc3e[] = { 	0b10001000,		//32 led minta eleje
							0b10001000,		//
							0b10001000,		//
							0b10001000,		//32 led minta vege
							0b01011111,		//8 vezerlojel az U6-ba
							0b01011011		//8 vezerlojel az U5-be
};
uint8_t minta4_adc2e[] = { 	0b10001000,		//32 led minta eleje
							0b10001000,		//
							0b10001000,		//
							0b10001000,		//32 led minta vege
							0b01011111,		//8 vezerlojel az U6-ba
							0b01011101		//8 vezerlojel az U5-be
};
uint8_t minta4_adc1e[] = { 	0b10001000,		//32 led minta eleje
							0b10001000,		//
							0b10001000,		//
							0b10001000,		//32 led minta vege
							0b01011111,		//8 vezerlojel az U6-ba
							0b01011110		//8 vezerlojel az U5-be
};
uint8_t minta3_adc4e[] = { 	0b01000100,		//32 led minta eleje
							0b01000100,		//
							0b01000100,		//
							0b01000100,		//32 led minta vege
							0b01011111,		//8 vezerlojel az U6-ba
							0b01010111		//8 vezerlojel az U5-be
};
uint8_t minta3_adc3e[] = { 	0b01000100,		//32 led minta eleje
							0b01000100,		//
							0b01000100,		//
							0b01000100,		//32 led minta vege
							0b01011111,		//8 vezerlojel az U6-ba
							0b01011011		//8 vezerlojel az U5-be
};
uint8_t minta3_adc2e[] = { 	0b01000100,		//32 led minta eleje
							0b01000100,		//
							0b01000100,		//
							0b01000100,		//32 led minta vege
							0b01011111,		//8 vezerlojel az U6-ba
							0b01011101		//8 vezerlojel az U5-be
};
uint8_t minta3_adc1e[] = { 	0b01000100,		//32 led minta eleje
							0b01000100,		//
							0b01000100,		//
							0b01000100,		//32 led minta vege
							0b01011111,		//8 vezerlojel az U6-ba
							0b01011110		//8 vezerlojel az U5-be
};
uint8_t minta2_adc4e[] = { 	0b00100010,		//32 led minta eleje
							0b00100010,		//
							0b00100010,		//
							0b00100010,		//32 led minta vege
							0b01011111,		//8 vezerlojel az U6-ba
							0b01010111		//8 vezerlojel az U5-be
};
uint8_t minta2_adc3e[] = { 	0b00100010,		//32 led minta eleje
							0b00100010,		//
							0b00100010,		//
							0b00100010,		//32 led minta vege
							0b01011111,		//8 vezerlojel az U6-ba
							0b01011011		//8 vezerlojel az U5-be
};
uint8_t minta2_adc2e[] = { 	0b00100010,		//32 led minta eleje
							0b00100010,		//
							0b00100010,		//
							0b00100010,		//32 led minta vege
							0b01011111,		//8 vezerlojel az U6-ba
							0b01011101		//8 vezerlojel az U5-be
};
uint8_t minta2_adc1e[] = { 	0b00100010,		//32 led minta eleje
							0b00100010,		//
							0b00100010,		//
							0b00100010,		//32 led minta vege
							0b01011111,		//8 vezerlojel az U6-ba
							0b01011110		//8 vezerlojel az U5-be
};
uint8_t minta1_adc4e[] = { 	0b00010001,		//32 led minta eleje
							0b00010001,		//
							0b00010001,		//
							0b00010001,		//32 led minta vege
							0b01011111,		//8 vezerlojel az U6-ba
							0b01010111		//8 vezerlojel az U5-be
};
uint8_t minta1_adc3e[] = { 	0b00010001,		//32 led minta eleje
							0b00010001,		//
							0b00010001,		//
							0b00010001,		//32 led minta vege
							0b01011111,		//8 vezerlojel az U6-ba
							0b01011011		//8 vezerlojel az U5-be
};
uint8_t minta1_adc2e[] = { 	0b00010001,		//32 led minta eleje
							0b00010001,		//
							0b00010001,		//
							0b00010001,		//32 led minta vege
							0b01011111,		//8 vezerlojel az U6-ba
							0b01011101		//8 vezerlojel az U5-be
};
uint8_t minta1_adc1e[] = { 	0b00010001,		//32 led minta eleje
							0b00010001,		//
							0b00010001,		//
							0b00010001,		//32 led minta vege
							0b01011111,		//8 vezerlojel az U6-ba
							0b01011110		//8 vezerlojel az U5-be
};

//barmi csak nem 11, ADD2, ADD1, ADD0, barmi csak nem 001, tehat: 00 _ _ _ 000
uint8_t adc_chanel0 =	0b00000000;		//input chanel 0; minta1-nel kell
uint8_t adc_chanel1 =	0b00001000;		//input chanel 1; minta2-nel kell
uint8_t adc_chanel2 =	0b00010000;		//input chanel 2; minta3-nel kell
uint8_t adc_chanel3 =	0b00011000;		//input chanel 3; minta4-nel kell
uint8_t adc_chanel4 =	0b00100000;		//input chanel 4; minta1-nel kell
uint8_t adc_chanel5 =	0b00101000;		//input chanel 5; minta2-nel kell
uint8_t adc_chanel6 =	0b00110000;		//input chanel 6; minta3-nel kell
uint8_t adc_chanel7 =	0b00111000;		//input chanel 7; minta4-nel kell

double VONAL_THRESHOLD_H = 9.0;
double VONAL_THRESHOLD_E = 7.0;

uint8_t vonalak_h[5] = { '-' };
uint8_t vonalak_e[5] = { '-' };

uint8_t vonal_eredmeny_h[33] = { 0 };
uint8_t vonal_eredmeny_e[33] = { 0 };

double vonal_kovetni_h = 0;
double vonal_kovetni_e = 0;

uint8_t iranyok[20];
//uint8_t iranyok1[] = { 2, 0,2 };
uint8_t aktualis_irany = 1;
uint8_t keresztezodes_szam = 0;
bool keresztezodesben = false;		// 0: egyenesben; 1: keresztezodesben
uint8_t kereszt_cnt = 0;
uint8_t egyenes_cnt = 0;
bool tolatas = false;
uint8_t kovi_irany = 9;

double cel = 0;
float szervoSzog = 90;
//int szervoSzog_emlek = 90;
double kormanyzas_agresszivitas = 0.35;			//elvileg minel nagyobb, annal agresszivabb; ]0, vegtelen[    tolatashoz: kb 0.7


int motvez_d = 1023;
int motvez_k = 512;
int veretesi_cnt = 0;
int fekezes_cnt = 0;

int bluetooth_flag = 0;
int bluetooth_j = 0;
int bluetooth_a = 0;
//Bluetooth send
uint8_t bluetooth_rx;
char bluetooth_str1[60] = { 0 };
int kivant_sebesseg = -1;
char kanyarban_vagy_egyenes = '_';
char sc_vagy_gyorskor = '_';
int bluetooth_len = 0;
//Bluetooth receive
char bluetooth_buffer[100] = { 0 };
int bluetooth_i = 0;

uint8_t kapuk[6] = { 'E', 'd', 'c'};
uint8_t temp_radio = '?';
bool letsGo = false;
uint8_t radio_i = 0;
bool uj_kapu = false;

int road[20] = {29, -1, -1, -1, -1, -1, -1, -1, -1, -1,
				-1, -1, -1, -1, -1, -1, -1, -1, -1, -1};
int graf_csucs[CSUCS_SZAM][CSUCS_SZAM];
int graf_irany[CSUCS_SZAM][CSUCS_SZAM][8];
uint8_t iranyok_elem = 0;

int source = -1;
int target1 = -1;
int target2 = -1;


uint8_t timer_counter = 0;
bool olvasok = false;


//Tavolsagszenzor I2C addresses of GPIO expanders on the X-NUCLEO-53L1A1
//ezek nem jok
 #define EXPANDER_1_ADDR 0x84 // 0x42 << 1
 #define EXPANDER_2_ADDR 0x86 // 0x43 << 1
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_UART4_Init(void);
static void MX_TIM8_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C3_Init(void);
static void MX_TIM12_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
static void Vonalszenzor_Init(void);
static void Vonalszenzor_operal(uint8_t teljes_kiolvasott_h[], uint8_t teljes_kiolvasott_e[]);
void Vonalszenzor_minta_kuldes(uint8_t minta[]);
void Vonalas_tombok_torlese(void);
void Vonalszenzor_meres_kiolvasas(uint8_t chanel, uint8_t* eredmeny);	//aktualisan chip selectelt adc-bol parameterben adott chanelen olvas; ret: [0, 3]
void Vonalas_tombok_feltoltese(void);
void Szervo_szog_beallit(void);
void Irany_valaszto(void);
void Kovetendo_vonal_valaszto(double* elso, double* hatso, uint8_t irany);
int MinDistance(int dist[], bool sptSet[]);
void Dijkstra(int graph[CSUCS_SZAM][CSUCS_SZAM], int src, int target1, int target2);
void Graf_csucs_feltolt(void);
void Graf_irany_feltolt(void);
void Kapuk_letilt(void);
void Iranyok_torlo(void);
void Iranyok_osszeallito(void);
void Source_Target_allito(void);
void Kapukbol_iranyok(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) {
	olvasok = true;
	HAL_UART_Receive_IT(&huart1, &temp_radio, 1);
	if(temp_radio == 0x30)
		letsGo = true;
	if(temp_radio < 0x60 && 0x40 < temp_radio) {
		if(temp_radio != kapuk[0]) {
			uj_kapu = true;
			letsGo = true;
		} else {
			uj_kapu = false;
		}
		radio_i = 0;
		for(int j=0; j < 6; j++)
			kapuk[j] = '-';
	}
	kapuk[radio_i] = temp_radio;
	radio_i++;

	if(uj_kapu == true && temp_radio == '\n') {
		Kapukbol_iranyok();
	}
	olvasok = false;

	/*if (huart == &huart2) {
		//erosen kerdeses
		if (bluetooth_rx == 0x0A)
			bluetooth_flag = 1;
		bluetooth_str1[bluetooth_a] = bluetooth_rx;
		bluetooth_a++;
	}
	HAL_UART_Receive(&huart2, &bluetooth_rx, 1, 5000);*/
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

	if (olvasok == false) {
		if (htim == &htim2) {
			timer_counter += 1;
			if(9 < timer_counter) {
				Vonalas_tombok_torlese();
				Vonalszenzor_operal(vonal_eredmeny_h, vonal_eredmeny_e);
				Vonalas_tombok_feltoltese();
				Irany_valaszto();
				Kovetendo_vonal_valaszto(&vonal_kovetni_e, &vonal_kovetni_h, aktualis_irany);
				Szervo_szog_beallit();
				timer_counter = 0;
			}

			//itt kell kiirni amire kivancsiak vagyunk a stringben*/
			/*sprintf(bluetooth_buffer,
					"%i -edik uzenet \t kivant sebesseg: %i \t allapot: %c kanyar/egyenes: %c \r\n",
					bluetooth_i, kivant_sebesseg, sc_vagy_gyorskor,
					kanyarban_vagy_egyenes);
			bluetooth_i++;
			bluetooth_len = strlen(bluetooth_buffer);
			//HAL_UART_Transmit(&huart2, bluetooth_buffer, bluetooth_len, 100);*/
		}
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint8_t tavolsag1_buff[50];
	VL53L1_RangingMeasurementData_t RangingData;
	VL53L1_Dev_t vl53l1_c; // center module
	VL53L1_DEV Dev = &vl53l1_c;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_UART4_Init();
  MX_TIM8_Init();
  MX_USART1_UART_Init();
  MX_I2C3_Init();
  MX_TIM12_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
	SERVO_Init(SZERVO);
	SERVO_MoveTo(SZERVO, 90);
	DC_MOTOR_Init(DC_MOTOR_PWM1);
	DC_MOTOR_Init(DC_MOTOR_PWM2);
	DC_MOTOR_Start(DC_MOTOR_PWM1, 0);
	DC_MOTOR_Start(DC_MOTOR_PWM2, 0);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);		// motvez EN
	motvez_k = motvez_d / 2;   									//455

	Vonalszenzor_minta_kuldes(leszed);
	Vonalszenzor_minta_kuldes(teszt_minta);		//csak hogy lassuk, hogy bekapcsolt
	HAL_Delay(100);
	Vonalszenzor_minta_kuldes(leszed);
	//HAL_UART_Receive(&huart2, &bluetooth_rx, 1, 5000);
	HAL_TIM_Base_Start_IT(&htim2);

	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);   //PWM jel start

	//Vonalszenzor inicializacio
	Vonalszenzor_Init();

	Graf_irany_feltolt();
	HAL_UART_Receive_IT(&huart1, &temp_radio, 1);

	// initialize vl53l1x communication parameters
	Dev->I2cHandle = &hi2c1;
	Dev->I2cDevAddr = 0x52;

	/*** Initialize GPIO expanders ***/
	// Unused GPIO should be configured as outputs to minimize the power consumption
	tavolsag1_buff[0] = 0x14; // GPDR (GPIO set direction register)
	tavolsag1_buff[1] = 0xFF; // GPIO_0 - GPIO_7
	tavolsag1_buff[2] = 0xFF; // GPIO_8 - GPIO_15
	HAL_I2C_Master_Transmit( &hi2c1, EXPANDER_1_ADDR, tavolsag1_buff, 3, 0xFFFF );
	HAL_I2C_Master_Transmit( &hi2c1, EXPANDER_2_ADDR, tavolsag1_buff, 3, 0xFFFF );

	// clear XSHUT (disable center module) -> expander 1, GPIO_15
	tavolsag1_buff[0] = 0x13; // GPSR + 1 ( GPIO set pin state register)
	HAL_I2C_Master_Transmit( &hi2c1, EXPANDER_1_ADDR, tavolsag1_buff, 1, 0xFFFF );
	HAL_I2C_Master_Receive( &hi2c1, EXPANDER_1_ADDR, tavolsag1_buff, 1, 0xFFFF );
	tavolsag1_buff[1] = tavolsag1_buff[0] & ~( 1 << ( 15 - 8 ) ); // clear GPIO_15
	tavolsag1_buff[0] = 0x13; // GPSR + 1 ( GPIO set pin state register)
	HAL_I2C_Master_Transmit( &hi2c1, EXPANDER_1_ADDR, tavolsag1_buff, 2, 0xFFFF );

	HAL_Delay( 2 ); // 2ms reset time

	// set XSHUT (enable center module) -> expander 1, GPIO_15
	tavolsag1_buff[0] = 0x13; // GPSR + 1 ( GPIO set pin state)
	HAL_I2C_Master_Transmit( &hi2c1, EXPANDER_1_ADDR, tavolsag1_buff, 1, 0xFFFF );
	HAL_I2C_Master_Receive( &hi2c1, EXPANDER_1_ADDR, tavolsag1_buff, 1, 0xFFFF );
	tavolsag1_buff[1] = tavolsag1_buff[0] | ( 1 << ( 15 - 8 ) ); // set GPIO_15
	tavolsag1_buff[0] = 0x13; // GPSR + 1 ( GPIO set pin state register)
	HAL_I2C_Master_Transmit( &hi2c1, EXPANDER_1_ADDR, tavolsag1_buff, 2, 0xFFFF );

	HAL_Delay( 2 );

	/*** VL53L1X Initialization ***/
	VL53L1_WaitDeviceBooted( Dev );
	VL53L1_DataInit( Dev );
	VL53L1_StaticInit( Dev );
	VL53L1_SetDistanceMode( Dev, VL53L1_DISTANCEMODE_LONG );
	VL53L1_SetMeasurementTimingBudgetMicroSeconds( Dev, 50000 );
	VL53L1_SetInterMeasurementPeriodMilliSeconds( Dev, 500 );
	VL53L1_StartMeasurement( Dev );

	//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);	//LED felvilagitasa, kell a .ioc GPIO Output PA5-re
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	while (1) {
		/*if(uj_kapu == true && temp_radio == '\n') {
			Kapukbol_iranyok();
		}

		if(kesz == true) {
			kesz = false;
		}*/

		//radios modul
		/*for(int i=0; i < 6; i++) {
			HAL_UART_Receive(&huart1, &temp_radio, 1, 1000);	//ez nagyon nagyon hosszu, all miatta a while(1)
			if(temp_radio == 0x30)
				letsGo = true;
			if(temp_radio < 0x60 && 0x40 < temp_radio) {
				i = 0;
				for(int j=0; j < 6; j++)
					kapuk[j] = '-';
			}
			kapuk[i] = temp_radio;
		}*/

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		//uint8_t Test[] = "Hello World\r\n"; //Data to send
		/*int size = sizeof(minta1);
		HAL_UART_Transmit(&huart2, minta1, size, 100);// Sending in normal mode
		HAL_Delay(1000);*/

		VL53L1_WaitMeasurementDataReady( Dev );
		VL53L1_GetRangingMeasurementData( Dev, &RangingData );
		/*sprintf( (char*)buff, "%d, %d, %.2f, %.2f\n\r", RangingData.RangeStatus, RangingData.RangeMilliMeter,
				 ( RangingData.SignalRateRtnMegaCps / 65536.0 ), RangingData.AmbientRateRtnMegaCps / 65336.0 );
		HAL_UART_Transmit( &huart2, buff, strlen( (char*)buff ), 0xFFFF );*/
		VL53L1_ClearInterruptAndStartMeasurement( Dev );

		if (btnEnable == 1) {
			if (motvezEnable == 1) {
				//motvez_k = 562;
				/*if (10 < fekezes_cnt) {
					while (motvez_k < 440) {
						motvez_k += 5;
						HAL_Delay(1);
					}
					veretesi_cnt = 0;
				} else {
					if (30 < veretesi_cnt) {
						while (400 < motvez_k) {
							motvez_k -= 5;
							HAL_Delay(1);
						}
					} else {
						while (motvez_k < 450) {
							motvez_k += 5;	// 0 - 1023-ig 440 a minimum, az alatt karos a motornak
							HAL_Delay(1);
						}
					}
				}*/
				// ha sok helyen latunk sotetet, akkor megallunk
				/*uint8_t sotetek = 0;
				for(int i = 1; i < 33; i++) {
					if(VONAL_THRESHOLD_E < vonal_eredmeny_e[i]) {
						sotetek++;
					}
				}
				if(8 < sotetek) {
					motvez_k = motvez_d / 2;
					tolatas = true;
					kormanyzas_agresszivitas = 0.7;
				}*/
				if(letsGo == true){
					if(tolatas == true) {
						motvez_k = 560;
					} else {
						motvez_k = 455;
					}
				}
				//if (motvez_d /2 > motvez_k) {							// motvez_d / 2 -nel nagyobb a hatramenet, pl. 900: gyors tolatás
					DC_MOTOR_Set_Speed(DC_MOTOR_PWM1, motvez_k); 		// ha pwm1 nagyobb, hatramenet
					DC_MOTOR_Set_Speed(DC_MOTOR_PWM2, motvez_d - motvez_k);
				//}
			}
		} else {
			veretesi_cnt = 0;
			fekezes_cnt = 0;
			//SERVO_MoveTo(SZERVO, 90);
			DC_MOTOR_Set_Speed(DC_MOTOR_PWM1, motvez_d / 2);	// ez a ketto a megallas
			DC_MOTOR_Set_Speed(DC_MOTOR_PWM2, motvez_d - (motvez_d / 2));
		}
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV8;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 45-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED3;
  htim8.Init.Period = 65535;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 1;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

	//Itt kell megivni a DC_MOTOR_Init() -et
  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief TIM12 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM12_Init(void)
{

  /* USER CODE BEGIN TIM12_Init 0 */

  /* USER CODE END TIM12_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM12_Init 1 */

  /* USER CODE END TIM12_Init 1 */
  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 19;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 44999;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim12, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM12_Init 2 */
	HAL_TIM_Base_Start_IT(&htim12);
  /* USER CODE END TIM12_Init 2 */
  HAL_TIM_MspPostInit(&htim12);

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 9600;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_12|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC2 PC3 PC4
                           PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB2 PB12 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_12|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1, 2);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
static void Vonalszenzor_Init(void) {
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);	// PCB2: Von_OE1  0
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);	// PCB2: Von_OE2  0
}

static void Vonalszenzor_operal(uint8_t* teljes_kiolvasott_h, uint8_t* teljes_kiolvasott_e) {
	Vonalszenzor_minta_kuldes(leszed);
	uint8_t eredmeny_16bit_temp[2] = {0b1110000, 0b00000000};
	//hatso vonalszenzor
	Vonalszenzor_minta_kuldes(minta1_adc1h);
	Vonalszenzor_meres_kiolvasas(adc_chanel0, eredmeny_16bit_temp);
	teljes_kiolvasott_h[32] = (uint8_t) eredmeny_16bit_temp[0] - 5;
	Vonalszenzor_meres_kiolvasas(adc_chanel4, eredmeny_16bit_temp);
	teljes_kiolvasott_h[28] = (uint8_t) eredmeny_16bit_temp[0] - 2;
	Vonalszenzor_minta_kuldes(leszed);
	Vonalszenzor_minta_kuldes(minta1_adc2h);
	Vonalszenzor_meres_kiolvasas(adc_chanel0, eredmeny_16bit_temp);
	teljes_kiolvasott_h[24] = (uint8_t) eredmeny_16bit_temp[0];
	Vonalszenzor_meres_kiolvasas(adc_chanel4, eredmeny_16bit_temp);
	teljes_kiolvasott_h[20] = (uint8_t) eredmeny_16bit_temp[0];
	Vonalszenzor_minta_kuldes(leszed);
	Vonalszenzor_minta_kuldes(minta1_adc3h);
	Vonalszenzor_meres_kiolvasas(adc_chanel0, eredmeny_16bit_temp);
	teljes_kiolvasott_h[16] = (uint8_t) eredmeny_16bit_temp[0];
	Vonalszenzor_meres_kiolvasas(adc_chanel4, eredmeny_16bit_temp);
	teljes_kiolvasott_h[12] = (uint8_t) eredmeny_16bit_temp[0];
	Vonalszenzor_minta_kuldes(leszed);
	Vonalszenzor_minta_kuldes(minta1_adc4h);
	Vonalszenzor_meres_kiolvasas(adc_chanel0, eredmeny_16bit_temp);
	teljes_kiolvasott_h[8] = (uint8_t) eredmeny_16bit_temp[0];
	Vonalszenzor_meres_kiolvasas(adc_chanel4, eredmeny_16bit_temp);
	teljes_kiolvasott_h[4] = (uint8_t) eredmeny_16bit_temp[0];
	//HAL_Delay(200);
	Vonalszenzor_minta_kuldes(leszed);

	Vonalszenzor_minta_kuldes(minta2_adc1h);
	Vonalszenzor_meres_kiolvasas(adc_chanel1, eredmeny_16bit_temp);
	teljes_kiolvasott_h[31] = (uint8_t) eredmeny_16bit_temp[0];
	Vonalszenzor_meres_kiolvasas(adc_chanel5, eredmeny_16bit_temp);
	teljes_kiolvasott_h[27] = (uint8_t) eredmeny_16bit_temp[0];
	Vonalszenzor_minta_kuldes(leszed);
	Vonalszenzor_minta_kuldes(minta2_adc2h);
	Vonalszenzor_meres_kiolvasas(adc_chanel1, eredmeny_16bit_temp);
	teljes_kiolvasott_h[23] = (uint8_t) eredmeny_16bit_temp[0];
	Vonalszenzor_meres_kiolvasas(adc_chanel5, eredmeny_16bit_temp);
	teljes_kiolvasott_h[19] = (uint8_t) eredmeny_16bit_temp[0];
	Vonalszenzor_minta_kuldes(leszed);
	Vonalszenzor_minta_kuldes(minta2_adc3h);
	Vonalszenzor_meres_kiolvasas(adc_chanel1, eredmeny_16bit_temp);
	teljes_kiolvasott_h[15] = (uint8_t) eredmeny_16bit_temp[0];
	Vonalszenzor_meres_kiolvasas(adc_chanel5, eredmeny_16bit_temp);
	teljes_kiolvasott_h[11] = (uint8_t) eredmeny_16bit_temp[0];
	Vonalszenzor_minta_kuldes(leszed);
	Vonalszenzor_minta_kuldes(minta2_adc4h);
	Vonalszenzor_meres_kiolvasas(adc_chanel1, eredmeny_16bit_temp);
	teljes_kiolvasott_h[7] = (uint8_t) eredmeny_16bit_temp[0];
	Vonalszenzor_meres_kiolvasas(adc_chanel5, eredmeny_16bit_temp);
	teljes_kiolvasott_h[3] = (uint8_t) eredmeny_16bit_temp[0];
	//HAL_Delay(200);
	Vonalszenzor_minta_kuldes(leszed);

	Vonalszenzor_minta_kuldes(minta3_adc1h);
	Vonalszenzor_meres_kiolvasas(adc_chanel2, eredmeny_16bit_temp);
	teljes_kiolvasott_h[30] = (uint8_t) eredmeny_16bit_temp[0];
	Vonalszenzor_meres_kiolvasas(adc_chanel6, eredmeny_16bit_temp);
	teljes_kiolvasott_h[26] = (uint8_t) eredmeny_16bit_temp[0] + 1;
	Vonalszenzor_minta_kuldes(leszed);
	Vonalszenzor_minta_kuldes(minta3_adc2h);
	Vonalszenzor_meres_kiolvasas(adc_chanel2, eredmeny_16bit_temp);
	teljes_kiolvasott_h[22] = (uint8_t) eredmeny_16bit_temp[0];
	Vonalszenzor_meres_kiolvasas(adc_chanel6, eredmeny_16bit_temp);
	teljes_kiolvasott_h[18] = (uint8_t) eredmeny_16bit_temp[0];
	Vonalszenzor_minta_kuldes(leszed);
	Vonalszenzor_minta_kuldes(minta3_adc3h);
	Vonalszenzor_meres_kiolvasas(adc_chanel2, eredmeny_16bit_temp);
	teljes_kiolvasott_h[14] = (uint8_t) eredmeny_16bit_temp[0];
	Vonalszenzor_meres_kiolvasas(adc_chanel6, eredmeny_16bit_temp);
	teljes_kiolvasott_h[10] = (uint8_t) eredmeny_16bit_temp[0];
	Vonalszenzor_minta_kuldes(leszed);
	Vonalszenzor_minta_kuldes(minta3_adc4h);
	Vonalszenzor_meres_kiolvasas(adc_chanel2, eredmeny_16bit_temp);
	teljes_kiolvasott_h[6] = (uint8_t) eredmeny_16bit_temp[0];
	Vonalszenzor_meres_kiolvasas(adc_chanel6, eredmeny_16bit_temp);
	teljes_kiolvasott_h[2] = (uint8_t) eredmeny_16bit_temp[0];
	//HAL_Delay(200);
	Vonalszenzor_minta_kuldes(leszed);

	Vonalszenzor_minta_kuldes(minta4_adc1h);
	Vonalszenzor_meres_kiolvasas(adc_chanel3, eredmeny_16bit_temp);
	teljes_kiolvasott_h[29] = (uint8_t) eredmeny_16bit_temp[0];
	Vonalszenzor_meres_kiolvasas(adc_chanel7, eredmeny_16bit_temp);
	teljes_kiolvasott_h[25] = (uint8_t) eredmeny_16bit_temp[0];
	Vonalszenzor_minta_kuldes(leszed);
	Vonalszenzor_minta_kuldes(minta4_adc2h);
	Vonalszenzor_meres_kiolvasas(adc_chanel3, eredmeny_16bit_temp);
	teljes_kiolvasott_h[21] = (uint8_t) eredmeny_16bit_temp[0];
	Vonalszenzor_meres_kiolvasas(adc_chanel7, eredmeny_16bit_temp);
	teljes_kiolvasott_h[17] = (uint8_t) eredmeny_16bit_temp[0];
	Vonalszenzor_minta_kuldes(leszed);
	Vonalszenzor_minta_kuldes(minta4_adc3h);
	Vonalszenzor_meres_kiolvasas(adc_chanel3, eredmeny_16bit_temp);
	teljes_kiolvasott_h[13] = (uint8_t) eredmeny_16bit_temp[0];
	Vonalszenzor_meres_kiolvasas(adc_chanel7, eredmeny_16bit_temp);
	teljes_kiolvasott_h[9] = (uint8_t) eredmeny_16bit_temp[0];
	Vonalszenzor_minta_kuldes(leszed);
	Vonalszenzor_minta_kuldes(minta4_adc4h);
	Vonalszenzor_meres_kiolvasas(adc_chanel3, eredmeny_16bit_temp);
	teljes_kiolvasott_h[5] = (uint8_t) eredmeny_16bit_temp[0];
	Vonalszenzor_meres_kiolvasas(adc_chanel7, eredmeny_16bit_temp);
	teljes_kiolvasott_h[1] = (uint8_t) eredmeny_16bit_temp[0];
	//HAL_Delay(200);
	Vonalszenzor_minta_kuldes(leszed);

	//elso vonalszenzor
	Vonalszenzor_minta_kuldes(minta1_adc1e);
	Vonalszenzor_meres_kiolvasas(adc_chanel0, eredmeny_16bit_temp);
	teljes_kiolvasott_e[1] = (uint8_t) eredmeny_16bit_temp[0] - 4;
	Vonalszenzor_meres_kiolvasas(adc_chanel4, eredmeny_16bit_temp);
	teljes_kiolvasott_e[5] = (uint8_t) eredmeny_16bit_temp[0];
	Vonalszenzor_minta_kuldes(leszed);
	Vonalszenzor_minta_kuldes(minta1_adc2e);
	Vonalszenzor_meres_kiolvasas(adc_chanel0, eredmeny_16bit_temp);
	teljes_kiolvasott_e[9] = (uint8_t) eredmeny_16bit_temp[0];
	Vonalszenzor_meres_kiolvasas(adc_chanel4, eredmeny_16bit_temp);
	teljes_kiolvasott_e[13] = (uint8_t) eredmeny_16bit_temp[0];
	Vonalszenzor_minta_kuldes(leszed);
	Vonalszenzor_minta_kuldes(minta1_adc3e);
	Vonalszenzor_meres_kiolvasas(adc_chanel0, eredmeny_16bit_temp);
	teljes_kiolvasott_e[17] = (uint8_t) eredmeny_16bit_temp[0];
	Vonalszenzor_meres_kiolvasas(adc_chanel4, eredmeny_16bit_temp);
	teljes_kiolvasott_e[21] = (uint8_t) eredmeny_16bit_temp[0];
	Vonalszenzor_minta_kuldes(leszed);
	Vonalszenzor_minta_kuldes(minta1_adc4e);
	Vonalszenzor_meres_kiolvasas(adc_chanel0, eredmeny_16bit_temp);
	teljes_kiolvasott_e[25] = (uint8_t) eredmeny_16bit_temp[0];
	Vonalszenzor_meres_kiolvasas(adc_chanel4, eredmeny_16bit_temp);
	teljes_kiolvasott_e[29] = (uint8_t) eredmeny_16bit_temp[0];
	//HAL_Delay(200);
	Vonalszenzor_minta_kuldes(leszed);

	Vonalszenzor_minta_kuldes(minta2_adc1e);
	Vonalszenzor_meres_kiolvasas(adc_chanel1, eredmeny_16bit_temp);
	teljes_kiolvasott_e[2] = (uint8_t) eredmeny_16bit_temp[0];
	Vonalszenzor_meres_kiolvasas(adc_chanel5, eredmeny_16bit_temp);
	teljes_kiolvasott_e[6] = (uint8_t) eredmeny_16bit_temp[0] +2;
	Vonalszenzor_minta_kuldes(leszed);
	Vonalszenzor_minta_kuldes(minta2_adc2e);
	Vonalszenzor_meres_kiolvasas(adc_chanel1, eredmeny_16bit_temp);
	teljes_kiolvasott_e[10] = (uint8_t) eredmeny_16bit_temp[0];
	Vonalszenzor_meres_kiolvasas(adc_chanel5, eredmeny_16bit_temp);
	teljes_kiolvasott_e[14] = (uint8_t) eredmeny_16bit_temp[0] +1;
	Vonalszenzor_minta_kuldes(leszed);
	Vonalszenzor_minta_kuldes(minta2_adc3e);
	Vonalszenzor_meres_kiolvasas(adc_chanel1, eredmeny_16bit_temp);
	teljes_kiolvasott_e[18] = (uint8_t) eredmeny_16bit_temp[0];
	Vonalszenzor_meres_kiolvasas(adc_chanel5, eredmeny_16bit_temp);
	teljes_kiolvasott_e[22] = (uint8_t) eredmeny_16bit_temp[0];
	Vonalszenzor_minta_kuldes(leszed);
	Vonalszenzor_minta_kuldes(minta2_adc4e);
	Vonalszenzor_meres_kiolvasas(adc_chanel1, eredmeny_16bit_temp);
	teljes_kiolvasott_e[26] = (uint8_t) eredmeny_16bit_temp[0];
	Vonalszenzor_meres_kiolvasas(adc_chanel5, eredmeny_16bit_temp);
	teljes_kiolvasott_e[30] = (uint8_t) eredmeny_16bit_temp[0];
	//HAL_Delay(200);
	Vonalszenzor_minta_kuldes(leszed);

	Vonalszenzor_minta_kuldes(minta3_adc1e);
	Vonalszenzor_meres_kiolvasas(adc_chanel2, eredmeny_16bit_temp);
	teljes_kiolvasott_e[3] = (uint8_t) eredmeny_16bit_temp[0];
	Vonalszenzor_meres_kiolvasas(adc_chanel6, eredmeny_16bit_temp);
	teljes_kiolvasott_e[7] = (uint8_t) eredmeny_16bit_temp[0];
	Vonalszenzor_minta_kuldes(leszed);
	Vonalszenzor_minta_kuldes(minta3_adc2e);
	Vonalszenzor_meres_kiolvasas(adc_chanel2, eredmeny_16bit_temp);
	teljes_kiolvasott_e[11] = (uint8_t) eredmeny_16bit_temp[0];
	Vonalszenzor_meres_kiolvasas(adc_chanel6, eredmeny_16bit_temp);
	teljes_kiolvasott_e[15] = (uint8_t) eredmeny_16bit_temp[0] +2;
	Vonalszenzor_minta_kuldes(leszed);
	Vonalszenzor_minta_kuldes(minta3_adc3e);
	Vonalszenzor_meres_kiolvasas(adc_chanel2, eredmeny_16bit_temp);
	teljes_kiolvasott_e[19] = (uint8_t) eredmeny_16bit_temp[0];
	Vonalszenzor_meres_kiolvasas(adc_chanel6, eredmeny_16bit_temp);
	teljes_kiolvasott_e[23] = (uint8_t) eredmeny_16bit_temp[0];
	Vonalszenzor_minta_kuldes(leszed);
	Vonalszenzor_minta_kuldes(minta3_adc4e);
	Vonalszenzor_meres_kiolvasas(adc_chanel2, eredmeny_16bit_temp);
	teljes_kiolvasott_e[27] = (uint8_t) eredmeny_16bit_temp[0];
	Vonalszenzor_meres_kiolvasas(adc_chanel6, eredmeny_16bit_temp);
	teljes_kiolvasott_e[31] = (uint8_t) eredmeny_16bit_temp[0];
	//HAL_Delay(200);
	Vonalszenzor_minta_kuldes(leszed);

	Vonalszenzor_minta_kuldes(minta4_adc1e);
	Vonalszenzor_meres_kiolvasas(adc_chanel3, eredmeny_16bit_temp);
	teljes_kiolvasott_e[4] = (uint8_t) eredmeny_16bit_temp[0];
	Vonalszenzor_meres_kiolvasas(adc_chanel7, eredmeny_16bit_temp);
	teljes_kiolvasott_e[8] = (uint8_t) eredmeny_16bit_temp[0] +2;
	Vonalszenzor_minta_kuldes(leszed);
	Vonalszenzor_minta_kuldes(minta4_adc2e);
	Vonalszenzor_meres_kiolvasas(adc_chanel3, eredmeny_16bit_temp);
	teljes_kiolvasott_e[12] = (uint8_t) eredmeny_16bit_temp[0];
	Vonalszenzor_meres_kiolvasas(adc_chanel7, eredmeny_16bit_temp);
	teljes_kiolvasott_e[16] = (uint8_t) eredmeny_16bit_temp[0];
	Vonalszenzor_minta_kuldes(leszed);
	Vonalszenzor_minta_kuldes(minta4_adc3e);
	Vonalszenzor_meres_kiolvasas(adc_chanel3, eredmeny_16bit_temp);
	teljes_kiolvasott_e[20] = (uint8_t) eredmeny_16bit_temp[0];
	Vonalszenzor_meres_kiolvasas(adc_chanel7, eredmeny_16bit_temp);
	teljes_kiolvasott_e[24] = (uint8_t) eredmeny_16bit_temp[0];
	Vonalszenzor_minta_kuldes(leszed);
	Vonalszenzor_minta_kuldes(minta4_adc4e);
	Vonalszenzor_meres_kiolvasas(adc_chanel3, eredmeny_16bit_temp);
	teljes_kiolvasott_e[28] = (uint8_t) eredmeny_16bit_temp[0];
	Vonalszenzor_meres_kiolvasas(adc_chanel7, eredmeny_16bit_temp);
	teljes_kiolvasott_e[32] = (uint8_t) eredmeny_16bit_temp[0];
	//HAL_Delay(200);
	Vonalszenzor_minta_kuldes(leszed);
}

void Vonalszenzor_minta_kuldes(uint8_t* minta) {
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);	// PC4: Von_latch1  0
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);	// PB1: Von_latch2  0
	HAL_SPI_Transmit(&hspi2, minta, 6, 100);				// minta kikuldes
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);		// PC4: Von_latch1  1
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);		// PB1: Von_latch2  1
}

void Vonalas_tombok_torlese(void) {
	for(int i=0; i < 5; i++) {		/* 5: vonalak_elso[] es _hatso merete */
		vonalak_h[i] = '-';
		vonalak_e[i] = '-';
	}
	for(int i=1; i < 33; i++) {		/* 1-32: vonal_eredmeny_elso[] es _hatso merete; 0. elem az fix 0 erteku */
		vonal_eredmeny_h[i] = 0;
		vonal_eredmeny_e[i] = 0;
	}
}

void Vonalszenzor_meres_kiolvasas(uint8_t chanel, uint8_t* eredmeny) {
	uint8_t temp1[2]= {chanel,0};
	HAL_SPI_Transmit(&hspi1, temp1, 2, 100);
	HAL_SPI_Receive(&hspi1, eredmeny, 2, 100);
}

void Vonalas_tombok_feltoltese(void) {
	for(int poz=1; poz < 33-1; poz++) {
	// 33 -1: 31-ig megyunk, mert a 32. sosem lehet egy 2 szeles vonal jobb szele
		if(VONAL_THRESHOLD_E < vonal_eredmeny_e[poz]) {
			if(VONAL_THRESHOLD_E < vonal_eredmeny_e[poz+1]) {
				if(vonal_eredmeny_e[poz-1] <= VONAL_THRESHOLD_E) {
					int i = 0;
					while(vonalak_e[i] != '-') {
						i++;
					}
					vonalak_e[i] = poz;
				}
			} else {
				if(vonal_eredmeny_e[poz-1] <= VONAL_THRESHOLD_E) {
					if(VONAL_THRESHOLD_E + 1 < vonal_eredmeny_e[poz-1] + vonal_eredmeny_e[poz+1]) {
						int j = 0;
						while(vonalak_e[j] != '-') {
							j++;
						}
						vonalak_e[j] = poz;
					}
				}
			}
		}
		if(VONAL_THRESHOLD_H < vonal_eredmeny_h[poz]) {
			if(VONAL_THRESHOLD_H < vonal_eredmeny_h[poz+1]) {
				if(vonal_eredmeny_h[poz-1] <= VONAL_THRESHOLD_H) {
					int k = 0;
					while(vonalak_h[k] != '-') {
						k++;
					}
					vonalak_h[k] = poz;
				}
			} else {
				if(vonal_eredmeny_h[poz-1] <= VONAL_THRESHOLD_H) {
					if(VONAL_THRESHOLD_H + 1 < vonal_eredmeny_h[poz-1] + vonal_eredmeny_h[poz+1]) {
						int l = 0;
						while(vonalak_h[l] != '-') {
							l++;
						}
						vonalak_h[l] = poz;
					}
				}
			}
		}
	}
	/*for(int poz=1; poz < 33-1; poz++) {
	// 33 -1: 31-ig megyunk, mert a 32. sosem lehet egy 2 szeles vonal jobb szele

	}*/
}

void Irany_valaszto(void) {
	if(keresztezodesben == false) {
		if(vonalak_e[1] < 33) {
			bool ok = true;
			int i = 0;
			while(vonalak_e[i] < 33) {		//kulonben '-' van benne, ami 45
				if((-8 > vonal_kovetni_e - (vonalak_e[i] - 16))  ||  (vonal_kovetni_e - (vonalak_e[i] - 16) > 8)) {
					//if((-9.5 > vonal_kovetni_h - (vonalak_h[i] - 16))  ||  (vonal_kovetni_h - (vonalak_h[i] - 16) > 9.5)) {
				// ha barhol van olyan vonal, ami tul messze van az aktualisan kovetettol
						ok = false;
					//}
				}
				i++;
			}
			if(egyenes_cnt < 30)
				ok = false;
			if(ok == true) {
				kereszt_cnt++;
				if(13 < kereszt_cnt) {
					keresztezodesben = true;
					tolatas = false;
					aktualis_irany = iranyok[keresztezodes_szam];
					egyenes_cnt = 0;
					kovi_irany = iranyok[keresztezodes_szam + 1];
					if(aktualis_irany == 9) {
						motvez_k = motvez_d / 2;	// ez a megallas
						letsGo = false;
					} else {
						//motvez_k = motvez_d / 2;	// ez a megallas
						keresztezodes_szam++;
						letsGo = true;
						kormanyzas_agresszivitas = 0.35;
					}
				}
			}
		}
	} else if(33 < vonalak_e[1]) {
		keresztezodesben = false;
		aktualis_irany = 1;
		kereszt_cnt = 0;
	}
	if(egyenes_cnt < 200)
		egyenes_cnt++;
}

void Kovetendo_vonal_valaszto(double* elso, double* hatso, uint8_t irany) {
	double elso_sum = 0.0;
	double hatso_sum = 0.0;
	double e_db = 0.0001;
	double h_db = 0.0001;
	if(irany == 0) {							// jobbra at
		*elso = vonalak_e[0] - 16;
		for(int i=0; i < 5; i++) {				// 6: vonalak[] merete
			if((vonalak_h[i] < 33)  &&			// kulonben '-' van benne, ami 45
			   ((-6 < vonal_kovetni_h - (vonalak_h[i] - 16))  &&  (vonal_kovetni_h - (vonalak_h[i] - 16) < 6))) {
				hatso_sum += vonalak_h[i] - 16;
				h_db += 1.0;
			}
		}
		*hatso = (-1)* hatso_sum / h_db;
	} else if (irany == 2) {					// balra at
		int j = 4;								// 4: vonalak_e merete
		while(33 < vonalak_e[j]) {
			j--;
		}
		*elso = vonalak_e[j] - 16;
		for(int i=0; i < 5; i++) {				// 6: vonalak[] merete
			if((vonalak_h[i] < 33)  &&			// kulonben '-' van benne, ami 45
			   ((-6 < vonal_kovetni_h - (vonalak_h[i] - 16))  &&  (vonal_kovetni_h - (vonalak_h[i] - 16) < 6))) {
				hatso_sum += vonalak_h[i] - 16;
				h_db += 1.0;
			}
		}
		*hatso = (-1)* hatso_sum / h_db;
	} else {									// irany == 1: kozep es egyeb, rossz iranyokra is ezt csinaljuk
		for(int i=0; i < 5; i++) {				// 6: vonalak[] merete
			if((vonalak_e[i] < 33)  &&			// kulonben '-' van benne, ami 45
			   ((-5 < vonal_kovetni_e - (vonalak_e[i] - 16))  &&  (vonal_kovetni_e - (vonalak_e[i] - 16) < 5))) {
				elso_sum += vonalak_e[i] - 16;
				e_db += 1.0;
			}
			if(vonalak_h[i] < 33) {				// kulonben '-' van benne, ami 45
				hatso_sum += vonalak_h[i] - 16;
				h_db += 1.0;
			}
		}
		if(0.9 < e_db) {
			*elso = elso_sum / e_db;
		} else {
			*elso = *elso;
		}
		if(0.9 < h_db) {
			*hatso = hatso_sum / h_db;
		} else {
			*hatso = *hatso;
		}
	}

	if(1.9 < e_db) {
		fekezes_cnt += 1;
	} else {
		fekezes_cnt = 0;
	}
	if((-5 < *elso && *elso < 5)  &&  (-5 < *hatso && *hatso < 5)) {
		veretesi_cnt += 1;
	} else {
		veretesi_cnt = 0;
	}
}

void Szervo_szog_beallit(void) {
	if (btnEnable == 1 && szervoEnable == 1) {
		if (tolatas == true) {		// tolatas	// 10 - (10- -7)*0.5 =
			kormanyzas_agresszivitas = 0.7;
			cel = vonal_kovetni_h + (((vonal_kovetni_h) - (vonal_kovetni_e)) *kormanyzas_agresszivitas);
			motvez_k = 560;
		} else {				// elore menet es rossz input
			cel = vonal_kovetni_e + (((vonal_kovetni_e) - (vonal_kovetni_h)) *kormanyzas_agresszivitas);
			//motvez_k = 455;
		}
		if(cel < -15) {
			szervoSzog = 0;
		} else if(15 < cel) {
			szervoSzog = 180;
		} else {
			szervoSzog = 90 + cel *6;
		}

		SERVO_MoveTo(SZERVO, szervoSzog);
	}
}

// A utility function to find the vertex with minimum distance value, from
// the set of vertices not yet included in shortest path tree
int MinDistance(int dist[], bool sptSet[])
{
    // Initialize min value
    int min = INT_MAX, min_index;

    for (int v = 0; v < CSUCS_SZAM; v++)
        if (sptSet[v] == false && dist[v] <= min)
            min = dist[v], min_index = v;

    return min_index;
}


void Source_Target_allito(void) {
    source = road[0];

    if            (kapuk[0] == 'A') {
            target1 = 1;
            target2 = 2;
    } else if    (kapuk[0] == 'B') {
            target1 = 3;
            target2 = 4;
    } else if    (kapuk[0] == 'C') {
            target1 = 5;
            target2 = 6;
    } else if    (kapuk[0] == 'D') {
            target1 = 7;
            target2 = 8;
    } else if    (kapuk[0] == 'E') {
            target1 = 9;
            target2 = 10;
    } else if    (kapuk[0] == 'F') {
            target1 = 11;
            target2 = 12;
    } else if    (kapuk[0] == 'G') {
            target1 = 13;
            target2 = 14;
    } else if    (kapuk[0] == 'H') {
            target1 = 15;
            target2 = 16;
    } else if    (kapuk[0] == 'I') {
            target1 = 17;
            target2 = 18;
    } else if    (kapuk[0] == 'J') {
            target1 = 19;
            target2 = 20;
    } else if    (kapuk[0] == 'K') {
            target1 = 21;
            target2 = 22;
    } else if    (kapuk[0] == 'L') {
            target1 = 23;
            target2 = 24;
    } else if    (kapuk[0] == 'M') {
            target1 = 25;
            target2 = 26;
    } else if    (kapuk[0] == 'N') {
            target1 = 27;
            target2 = 28;
    } else if    (kapuk[0] == 'O') {
            target1 = 29;
            target2 = 29;
    } else if    (kapuk[0] == 'X') {
            target1 = 31;
            target2 = 32;
    }
}

void Kapukbol_iranyok(void) {
	Source_Target_allito();
	Graf_csucs_feltolt();
	Kapuk_letilt();
	Dijkstra(graf_csucs, source, target1, target2);
	Iranyok_torlo();
	Iranyok_osszeallito();
}

// Function that implements Dijkstra's single source shortest path algorithm
// for a graph represented using adjacency matrix representation
void Dijkstra(int graph[CSUCS_SZAM][CSUCS_SZAM], int src, int target1, int target2) {
	int dist[CSUCS_SZAM]; // The output array. dist[i] will hold the shortest
	// distance from src to i
  	int r[CSUCS_SZAM];
  	for(int i = 0; i < CSUCS_SZAM; i++) {
      	r[i] = -1;
    }

	bool sptSet[CSUCS_SZAM]; // sptSet[i] will be true if vertex i is included in shortest
	// path tree or shortest distance from src to i is finalized

	// Initialize all distances as INFINITE and stpSet[] as false
	for (int i = 0; i < CSUCS_SZAM; i++)
		dist[i] = INT_MAX, sptSet[i] = false;

	// Distance of source vertex from itself is always 0
	dist[src] = 0;

	// Find shortest path for all vertices
	for (int count = 0; count < CSUCS_SZAM - 1; count++) {
		// Pick the minimum distance vertex from the set of vertices not
		// yet processed. u is always equal to src in the first iteration.
		int u = MinDistance(dist, sptSet);

		// Mark the picked vertex as processed
		sptSet[u] = true;

		// Update dist value of the adjacent vertices of the picked vertex.
		for (int v = 0; v < CSUCS_SZAM; v++)

			// Update dist[v] only if is not in sptSet, there is an edge from
			// u to v, and total weight of path from src to v through u is
			// smaller than current value of dist[v]
			if (!sptSet[v] && graph[u][v] && dist[u] != INT_MAX
				&& dist[u] + graph[u][v] < dist[v]) {
				dist[v] = dist[u] + graph[u][v];
    			r[v] = u; }
	}

  	int ultimate_trg = target1;
  	if(dist[target2] < dist[target1])
      	ultimate_trg = target2;

  	for(int i = 0; i < 20; i++) {
      	road[i] = -1;
    }
  	road[0] = ultimate_trg;
  	int last_v = r[ultimate_trg];
  	int k = 1;
  	//cout <<last_v<< endl;
  	while(last_v != src){
  		road[k] = last_v;
  		k++;
        last_v = r[last_v];
      	//cout <<last_v<< endl;
     }
  	road[k] = last_v;
}

void Graf_csucs_feltolt(void) {
  	for(int u = 0; u < CSUCS_SZAM; u++) {
      	for(int v= 0; v < CSUCS_SZAM; v++) {
      		graf_csucs[u][v] = 5000000;
        }
    }
  	graf_csucs[1][3] = 4891;
    graf_csucs[1][5] = 6060;
    graf_csucs[1][7] = 7143;
    graf_csucs[2][3] = 5260;
    graf_csucs[2][5] = 6429;
    graf_csucs[2][7] = 7512;
    graf_csucs[3][9] = 4202;
    graf_csucs[3][11] = 5373;
    graf_csucs[4][1] = 5260;
    graf_csucs[4][2] = 4891;
    graf_csucs[5][11] = 3657;
    graf_csucs[6][1] = 6429;
    graf_csucs[6][2] = 6060; 	// C csucs kesz
    graf_csucs[7][11] = 2899;
    graf_csucs[8][1] = 7512;
    graf_csucs[8][2] = 7143;
    graf_csucs[9][17] = 6770;
    graf_csucs[9][19] = 8874;
    graf_csucs[10][4] = 4202;
    graf_csucs[11][14] = 1697;
    graf_csucs[11][15] = 2370;
    graf_csucs[11][21] = 8569;
    graf_csucs[11][23] = 13602;
    graf_csucs[11][25] = 14059;
    graf_csucs[11][27] = 15560;
    graf_csucs[12][4] = 5373;
    graf_csucs[12][6] = 3657;
    graf_csucs[12][8] = 2899; 	// F csucs kesz
    graf_csucs[13][12] = 1697;
    graf_csucs[14][17] = 4396;
    graf_csucs[14][19] = 6500;
    graf_csucs[15][21] = 6494;
    graf_csucs[15][23] = 11527;
    graf_csucs[15][25] = 11984;
    graf_csucs[15][27] = 13485;
    graf_csucs[16][12] = 2370;
    graf_csucs[17][21] = 2969;
    graf_csucs[17][23] = 8002;
    graf_csucs[17][25] = 8459;
    graf_csucs[17][27] = 9960;
    graf_csucs[18][13] = 4396;
    graf_csucs[18][10] = 6770; 	// I csucs kesz
    graf_csucs[19][23] = 5615;
    graf_csucs[19][25] = 6072;
    graf_csucs[19][27] = 7573;
    graf_csucs[20][10] = 8874;
    graf_csucs[20][13] = 6500;
    graf_csucs[21][23] = 4727;
    graf_csucs[21][25] = 5184;
    graf_csucs[21][27] = 6685;
    graf_csucs[22][12] = 8569;
    graf_csucs[22][16] = 6494;
    graf_csucs[22][18] = 2969;
    graf_csucs[23][29] = 10948;
    graf_csucs[23][32] = 13441;
    graf_csucs[24][12] = 13602;
    graf_csucs[24][16] = 11527;
    graf_csucs[24][18] = 8002;
    graf_csucs[24][20] = 5615;
    graf_csucs[24][22] = 4727; 	// L csucs kesz
    graf_csucs[25][29] = 10485;
    graf_csucs[25][32] = 12978;
    graf_csucs[26][12] = 14059;
    graf_csucs[26][16] = 11984;
    graf_csucs[26][18] = 8459;
    graf_csucs[26][20] = 6072;
    graf_csucs[26][22] = 5184;
    graf_csucs[27][31] = 3047;
    graf_csucs[28][12] = 15560;
    graf_csucs[28][16] = 13485;
    graf_csucs[28][18] = 9960;
    graf_csucs[28][20] = 7573;
    graf_csucs[28][22] = 6685;
    graf_csucs[29][32] = 9659;
    graf_csucs[30][29] = 6981;
    graf_csucs[30][32] = 9474;
    graf_csucs[31][24] = 13441;
    graf_csucs[31][26] = 12978;
    graf_csucs[32][28] = 3047;
}

void Graf_irany_feltolt(void) {
	for(int u = 0; u < CSUCS_SZAM; u++) {
		for(int v = 0; v < CSUCS_SZAM; v++) {
			for(int d = 0; d < 8; d++) {
				graf_irany[u][v][d] = -1;
			}
		}
	}
	graf_irany[1][3][0] = 2;
	graf_irany[1][5][0] = 0;
	graf_irany[1][5][1] = 2;
	graf_irany[1][7][0] = 0;
	graf_irany[1][7][1] = 0;
	graf_irany[2][3][0] = 2;
	graf_irany[2][5][0] = 0;
	graf_irany[2][5][1] = 2;
	graf_irany[2][7][0] = 0;
	graf_irany[2][7][1] = 0;
	graf_irany[3][9][0] = 2;
	graf_irany[3][11][0] = 0;
	graf_irany[3][11][1] = 1;
	graf_irany[3][11][2] = 2;
	graf_irany[4][1][0] = 0;
	graf_irany[4][2][0] = 2;
	graf_irany[5][11][0] = 0;
	graf_irany[5][11][1] = 2;
	graf_irany[6][1][0] = 0;
	graf_irany[6][1][1] = 0;
	graf_irany[6][2][0] = 0; 	// C csucs kesz
	graf_irany[6][2][1] = 2; 	// C csucs kesz
	graf_irany[7][11][0] = 1;
	graf_irany[8][1][0] = 1;
	graf_irany[8][1][1] = 0;
	graf_irany[8][2][0] = 1;
	graf_irany[8][2][1] = 2;
	graf_irany[9][17][0] = 2;
	graf_irany[9][17][1] = 0;
	graf_irany[9][19][0] = 2;
	graf_irany[9][19][1] = 2;
	graf_irany[10][4][0] = 0;
	graf_irany[11][14][0] = 2;
	graf_irany[11][15][0] = 0;
	graf_irany[11][21][0] = 1;
	graf_irany[11][21][1] = 2;
	graf_irany[11][21][2] = 1;
	graf_irany[11][23][0] = 1;
	graf_irany[11][23][1] = 2;
	graf_irany[11][23][2] = 0;
	graf_irany[11][23][3] = 0;
	graf_irany[11][25][0] = 1;
	graf_irany[11][25][1] = 2;
	graf_irany[11][25][2] = 0;
	graf_irany[11][25][3] = 1;
	graf_irany[11][27][0] = 1;
	graf_irany[11][27][1] = 2;
	graf_irany[11][27][2] = 0;
	graf_irany[11][27][3] = 2;
	graf_irany[12][4][0] = 0;
	graf_irany[12][4][1] = 0;
	graf_irany[12][4][2] = 1;
	graf_irany[12][6][0] = 0;
	graf_irany[12][6][1] = 2;
	graf_irany[12][8][0] = 2; 	// F csucs kesz
	graf_irany[13][12][0] = 1;
	graf_irany[14][17][0] = 1;
	graf_irany[14][17][1] = 0;
	graf_irany[14][19][0] = 1;
	graf_irany[14][19][1] = 2;
	graf_irany[15][21][0] = 1;
	graf_irany[15][21][1] = 1;
	graf_irany[15][23][0] = 1;
	graf_irany[15][23][1] = 0;
	graf_irany[15][23][2] = 0;
	graf_irany[15][25][0] = 1;
	graf_irany[15][25][1] = 0;
	graf_irany[15][25][2] = 1;
	graf_irany[15][27][0] = 1;
	graf_irany[15][27][1] = 0;
	graf_irany[15][27][2] = 2;
	graf_irany[16][12][0] = 1;
	graf_irany[17][21][0] = 2;		// vagy kozep, fura keresztezodes
	graf_irany[17][23][0] = 0;
	graf_irany[17][23][1] = 0;
	graf_irany[17][25][0] = 0;
	graf_irany[17][25][1] = 1;
	graf_irany[17][27][0] = 0;
	graf_irany[17][27][1] = 2;
	graf_irany[18][13][0] = 1;
	graf_irany[18][13][1] = 2;
	graf_irany[18][10][0] = 1; 	// I csucs kesz
	graf_irany[18][10][1] = 0; 	// I csucs kesz
	graf_irany[19][23][0] = 0;
	graf_irany[19][25][0] = 1;
	graf_irany[19][27][0] = 2;
	graf_irany[20][10][0] = 0;
	graf_irany[20][10][1] = 0;
	graf_irany[20][13][0] = 0;
	graf_irany[20][13][1] = 2;
	graf_irany[21][23][0] = 0;
	graf_irany[21][25][0] = 1;
	graf_irany[21][27][0] = 2;
	graf_irany[22][12][0] = 1;
	graf_irany[22][12][1] = 0;
	graf_irany[22][12][2] = 1;
	graf_irany[22][16][0] = 1;
	graf_irany[22][16][1] = 2;
	graf_irany[22][18][0] = 0;
	graf_irany[23][29][0] = 0;
	graf_irany[23][29][1] = 0;
	graf_irany[23][32][0] = 0;
	graf_irany[23][32][1] = 2;
	graf_irany[23][32][2] = 0;
	graf_irany[23][32][3] = 0;
	graf_irany[23][32][4] = 0;
	//graf_irany[23][32][5] = 0;
	//graf_irany[23][32][6] = 0;
	graf_irany[24][12][0] = 2;
	graf_irany[24][12][1] = 2;
	graf_irany[24][12][2] = 0;
	graf_irany[24][12][3] = 1;
	graf_irany[24][16][0] = 2;
	graf_irany[24][16][1] = 2;
	graf_irany[24][16][2] = 2;
	graf_irany[24][18][0] = 2;
	graf_irany[24][18][1] = 0;
	graf_irany[24][20][0] = 0;
	graf_irany[24][22][0] = 1; 	// L csucs kesz
	graf_irany[25][29][0] = 0;
	graf_irany[25][29][1] = 0;
	graf_irany[25][32][0] = 1;
	graf_irany[25][32][1] = 2;
	graf_irany[25][32][2] = 0;
	graf_irany[25][32][3] = 0;
	graf_irany[25][32][4] = 0;
	//graf_irany[25][32][5] = 0;
	//graf_irany[25][32][6] = 0;
	graf_irany[26][12][0] = 2;
	graf_irany[26][12][1] = 2;
	graf_irany[26][12][2] = 0;
	graf_irany[26][12][3] = 1;
	graf_irany[26][16][0] = 2;
	graf_irany[26][16][1] = 2;
	graf_irany[26][16][2] = 2;
	graf_irany[26][18][0] = 2;
	graf_irany[26][18][1] = 0;
	graf_irany[26][20][0] = 0;
	graf_irany[26][22][0] = 1;
	graf_irany[27][31][0] = 2;
	graf_irany[27][31][1] = 2;
	graf_irany[27][31][2] = 2;
	//graf_irany[27][31][3] = 2;
	//graf_irany[27][31][4] = 2;
	graf_irany[28][12][0] = 2;
	graf_irany[28][12][1] = 2;
	graf_irany[28][12][2] = 0;
	graf_irany[28][12][3] = 1;
	graf_irany[28][16][0] = 2;
	graf_irany[28][16][1] = 2;
	graf_irany[28][16][2] = 2;
	graf_irany[28][18][0] = 2;
	graf_irany[28][18][1] = 0;
	graf_irany[28][20][0] = 0;
	graf_irany[28][22][0] = 1;
	//graf_irany[29][32][0] = 2;
	graf_irany[29][32][0] = 2;
	graf_irany[29][32][1] = 0;
	//graf_irany[29][32][2] = 0;
	//graf_irany[29][32][3] = 0;
	//graf_irany[29][32][4] = 0;
	//graf_irany[29][32][5] = 0;
	graf_irany[30][29][0] = 0;
	graf_irany[30][32][0] = 2;
	graf_irany[30][32][1] = 0;
	graf_irany[30][32][2] = 0;
	graf_irany[30][32][3] = 0;
	//graf_irany[30][32][4] = 0;
	//graf_irany[30][32][5] = 0;
	graf_irany[31][24][0] = 0;
	graf_irany[31][24][1] = 2;
	graf_irany[31][26][0] = 0;
	graf_irany[31][26][1] = 0;
	// graf_irany[32][28][0] = -1;		egyenes ut vezet
}

void Kapuk_letilt(void) {
	for(int i = 0; i < 6; i++) {
		if			(kapuk[i] == 'a') {
			for(int j = 1; j < CSUCS_SZAM; j++) {
				graf_csucs[1][j] = 5000000;
				graf_csucs[2][j] = 5000000;
			}
		} else if	(kapuk[i] == 'b') {
			for(int j = 1; j < CSUCS_SZAM; j++) {
				graf_csucs[3][j] = 5000000;
				graf_csucs[4][j] = 5000000;
			}
		} else if	(kapuk[i] == 'c') {
			for(int j = 1; j < CSUCS_SZAM; j++) {
				graf_csucs[5][j] = 5000000;
				graf_csucs[6][j] = 5000000;
			}
		} else if	(kapuk[i] == 'd') {
			for(int j = 1; j < CSUCS_SZAM; j++) {
				graf_csucs[7][j] = 5000000;
				graf_csucs[8][j] = 5000000;
			}
		} else if	(kapuk[i] == 'e') {
			for(int j = 1; j < CSUCS_SZAM; j++) {
				graf_csucs[9][j] = 5000000;
				graf_csucs[10][j] = 5000000;
			}
		} else if	(kapuk[i] == 'f') {
			for(int j = 1; j < CSUCS_SZAM; j++) {
				graf_csucs[11][j] = 5000000;
				graf_csucs[12][j] = 5000000;
			}
		} else if	(kapuk[i] == 'g') {
			for(int j = 1; j < CSUCS_SZAM; j++) {
				graf_csucs[13][j] = 5000000;
				graf_csucs[14][j] = 5000000;
			}
		} else if	(kapuk[i] == 'h') {
			for(int j = 1; j < CSUCS_SZAM; j++) {
				graf_csucs[15][j] = 5000000;
				graf_csucs[16][j] = 5000000;
			}
		} else if	(kapuk[i] == 'i') {
			for(int j = 1; j < CSUCS_SZAM; j++) {
				graf_csucs[17][j] = 5000000;
				graf_csucs[18][j] = 5000000;
			}
		} else if	(kapuk[i] == 'j') {
			for(int j = 1; j < CSUCS_SZAM; j++) {
				graf_csucs[19][j] = 5000000;
				graf_csucs[20][j] = 5000000;
			}
		} else if	(kapuk[i] == 'k') {
			for(int j = 1; j < CSUCS_SZAM; j++) {
				graf_csucs[21][j] = 5000000;
				graf_csucs[22][j] = 5000000;
			}
		} else if	(kapuk[i] == 'l') {
			for(int j = 1; j < CSUCS_SZAM; j++) {
				graf_csucs[23][j] = 5000000;
				graf_csucs[24][j] = 5000000;
			}
		} else if	(kapuk[i] == 'm') {
			for(int j = 1; j < CSUCS_SZAM; j++) {
				graf_csucs[25][j] = 5000000;
				graf_csucs[26][j] = 5000000;
			}
		} else if	(kapuk[i] == 'n') {
			for(int j = 1; j < CSUCS_SZAM; j++) {
				graf_csucs[27][j] = 5000000;
				graf_csucs[28][j] = 5000000;
			}
		}
	}
}

void Iranyok_torlo(void) {
	for(int i = 0; i < 100; i++) {
		iranyok[i] = 9;				// 9: nem igazi iranyt jelol
	}
}

void Iranyok_osszeallito(void) {
	iranyok_elem = 0;
	for(int i = 19; 0 < i; i--) {
		if(road[i] != -1) {
			if(road[i] == 29) {
				tolatas = true;
			}
			for(int j = 0; j < 8; j++) {
				// road[i]-bol road[i-1]-be "0 2 0" beirni az iranyokba
				if(graf_irany[ road[i] ] [ road[i-1] ] [ j ]  != -1) {
					iranyok[iranyok_elem] = graf_irany[ road[i] ] [ road[i-1] ] [ j ];
					iranyok_elem++;
				}
			}
		}
	}
	keresztezodes_szam = 0;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == B1_Pin) {
		btnEnable = !btnEnable;
		//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);	//LED felvilagitasa
		/*szervoSzog += 90;
		if(szervoSzog > 200)
			szervoSzog = 0;
		SERVO_MoveTo(SZERVO, szervoSzog);*/
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

