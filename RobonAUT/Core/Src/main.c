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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DC_MOTOR_PWM1 0
#define DC_MOTOR_PWM2 1
#define	SZERVO 0
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
uint8_t szervoEnable = 0;
uint8_t motvezEnable = 0;

//Lehet, hogy az adc CS-ek alapbol 1-ben kell legyenek es 0-val vannak selectalva
//Minden uj minta utan ki kell kuldeni
uint8_t leszed[] = { 	0b00000000,		//32 led minta eleje
						0b00000000,		//
						0b00000000,		//
						0b00000000,		//32 led minta vege
						0b11111111,		//8 vezerlojel az U6-ba
						0b11111111		//8 vezerlojel az U5-be
};
uint8_t minta0[] = { 	0b11111100,		//32 led minta eleje
						0b11111000,		//
						0b11110000,		//
						0b11100010,		//32 led minta vege
						0b00001111,		//8 vezerlojel az U6-ba: elolso v.sz.
						0b00001111		//8 vezerlojel az U5-be: hatulso v.sz.
};
uint8_t minta1[] = { 	0b10001000,		//32 led minta eleje
						0b10001000,		//
						0b10001000,		//
						0b10001000,		//32 led minta vege
						0b00001111,		//8 vezerlojel az U6-ba
						0b00001111		//8 vezerlojel az U5-be
};
uint8_t minta2[] = { 	0b01000100,		//32 led minta eleje
						0b01000100,		//
						0b01000100,		//
						0b01000100,		//32 led minta vege
						0b00001111,		//8 vezerlojel az U6-ba
						0b00001111		//8 vezerlojel az U5-be
};
uint8_t minta3[] = { 	0b00100010,		//32 led minta eleje
						0b00100010,		//
						0b00100010,		//
						0b00100010,		//32 led minta vege
						0b00001111,		//8 vezerlojel az U6-ba
						0b00001111		//8 vezerlojel az U5-be
};
uint8_t minta4[] = { 	0b00010001,		//32 led minta eleje
						0b00010001,		//
						0b00010001,		//
						0b00010001,		//32 led minta vege
						0b00001111,		//8 vezerlojel az U6-ba
						0b00001111		//8 vezerlojel az U5-be
};
uint8_t minta4_adc4e[] = { 	0b10001000,		//32 led minta eleje
							0b10001000,		//
							0b10001000,		//
							0b10001000,		//32 led minta vege
							0b00000111,		//8 vezerlojel az U6-ba
							0b00001111		//8 vezerlojel az U5-be
};
uint8_t minta4_adc3e[] = { 	0b10001000,		//32 led minta eleje
							0b10001000,		//
							0b10001000,		//
							0b10001000,		//32 led minta vege
							0b00001011,		//8 vezerlojel az U6-ba
							0b00001111		//8 vezerlojel az U5-be
};
uint8_t minta4_adc2e[] = { 	0b10001000,		//32 led minta eleje
							0b10001000,		//
							0b10001000,		//
							0b10001000,		//32 led minta vege
							0b00001101,		//8 vezerlojel az U6-ba
							0b00001111		//8 vezerlojel az U5-be
};
uint8_t minta4_adc1e[] = { 	0b10001000,		//32 led minta eleje
							0b10001000,		//
							0b10001000,		//
							0b10001000,		//32 led minta vege
							0b00001110,		//8 vezerlojel az U6-ba
							0b00001111		//8 vezerlojel az U5-be
};
uint8_t minta3_adc4e[] = { 	0b01000100,		//32 led minta eleje
							0b01000100,		//
							0b01000100,		//
							0b01000100,		//32 led minta vege
							0b00000111,		//8 vezerlojel az U6-ba
							0b00001111		//8 vezerlojel az U5-be
};
uint8_t minta3_adc3e[] = { 	0b01000100,		//32 led minta eleje
							0b01000100,		//
							0b01000100,		//
							0b01000100,		//32 led minta vege
							0b00001011,		//8 vezerlojel az U6-ba
							0b00001111		//8 vezerlojel az U5-be
};
uint8_t minta3_adc2e[] = { 	0b01000100,		//32 led minta eleje
							0b01000100,		//
							0b01000100,		//
							0b01000100,		//32 led minta vege
							0b00001101,		//8 vezerlojel az U6-ba
							0b00001111		//8 vezerlojel az U5-be
};
uint8_t minta3_adc1e[] = { 	0b01000100,		//32 led minta eleje
							0b01000100,		//
							0b01000100,		//
							0b01000100,		//32 led minta vege
							0b00001110,		//8 vezerlojel az U6-ba
							0b00001111		//8 vezerlojel az U5-be
};
uint8_t minta2_adc4e[] = { 	0b00100010,		//32 led minta eleje
							0b00100010,		//
							0b00100010,		//
							0b00100010,		//32 led minta vege
							0b00000111,		//8 vezerlojel az U6-ba
							0b00001111		//8 vezerlojel az U5-be
};
uint8_t minta2_adc3e[] = { 	0b00100010,		//32 led minta eleje
							0b00100010,		//
							0b00100010,		//
							0b00100010,		//32 led minta vege
							0b00001011,		//8 vezerlojel az U6-ba
							0b00001111		//8 vezerlojel az U5-be
};
uint8_t minta2_adc2e[] = { 	0b00100010,		//32 led minta eleje
							0b00100010,		//
							0b00100010,		//
							0b00100010,		//32 led minta vege
							0b00001101,		//8 vezerlojel az U6-ba
							0b00001111		//8 vezerlojel az U5-be
};
uint8_t minta2_adc1e[] = { 	0b00100010,		//32 led minta eleje
							0b00100010,		//
							0b00100010,		//
							0b00100010,		//32 led minta vege
							0b00001110,		//8 vezerlojel az U6-ba
							0b00001111		//8 vezerlojel az U5-be
};
uint8_t minta1_adc4e[] = { 	0b00010001,		//32 led minta eleje
							0b00010001,		//
							0b00010001,		//
							0b00010001,		//32 led minta vege
							0b00000111,		//8 vezerlojel az U6-ba
							0b00001111		//8 vezerlojel az U5-be
};
uint8_t minta1_adc3e[] = { 	0b00010001,		//32 led minta eleje
							0b00010001,		//
							0b00010001,		//
							0b00010001,		//32 led minta vege
							0b00001011,		//8 vezerlojel az U6-ba
							0b00001111		//8 vezerlojel az U5-be
};
uint8_t minta1_adc2e[] = { 	0b00010001,		//32 led minta eleje
							0b00010001,		//
							0b00010001,		//
							0b00010001,		//32 led minta vege
							0b00001101,		//8 vezerlojel az U6-ba
							0b00001111		//8 vezerlojel az U5-be
};
uint8_t minta1_adc1e[] = { 	0b00010001,		//32 led minta eleje
							0b00010001,		//
							0b00010001,		//
							0b00010001,		//32 led minta vege
							0b00001110,		//8 vezerlojel az U6-ba
							0b00001111		//8 vezerlojel az U5-be
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

int VONAL_THRESHOLD = 6;

int vonal1 = 100;
int sotet0 = 0;
int sotet1 = 0;
int sotet2 = 0;
int sotet3 = 0;
int sotet4 = 0;
int sotet5 = 0;
int sotet6 = 0;
int sotet7 = 0;
int sotet8 = 0;
int sotet9 = 0;
int sotet10 = 0;
int sotet11 = 0;
int sotet12 = 0;
int sotet13 = 0;
int sotet14 = 0;
int sotet15 = 0;
int sotet16 = 0;
int sotet17 = 0;
int sotet18 = 0;
int sotet19 = 0;
int sotet20 = 0;
int sotet21 = 0;
int sotet22 = 0;
int sotet23 = 0;
int sotet24 = 0;
int sotet25 = 0;
int sotet26 = 0;
int sotet27 = 0;
int sotet28 = 0;
int sotet29 = 0;
int sotet30 = 0;
int sotet31 = 0;

int motvez_d = 1024;

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
static void Vonalszenzor_operal(uint8_t teljes_kiolvasott[]);
void Vonalszenzor_minta_kuldes(uint8_t minta[]);
void Vonalszenzor_meres_kiolvasas(uint8_t chanel, uint8_t* eredmeny);	//aktualisan chip selectelt adc-bol parameterben adott chanelen olvas; ret: [0, 3]
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart == &huart2) {
		//erosen kerdeses
		if (bluetooth_rx == 0x0A)
			bluetooth_flag = 1;
		bluetooth_str1[bluetooth_a] = bluetooth_rx;
		bluetooth_a++;
	}
	HAL_UART_Receive_IT(&huart2, &bluetooth_rx, 1);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

	if (htim == &htim2) {
		//itt kell kiirni amire kivancsiak vagyunk a stringben
		sprintf(bluetooth_buffer,
				"%i -edik uzenet \t kivant sebesseg: %i \t allapot: %c kanyar/egyenes: %c \r\n",
				bluetooth_i, kivant_sebesseg, sc_vagy_gyorskor,
				kanyarban_vagy_egyenes);
		bluetooth_i++;
		bluetooth_len = strlen(bluetooth_buffer);
		//HAL_UART_Transmit(&huart2, bluetooth_buffer, bluetooth_len, 100);
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
	uint8_t tavolsag_1_buff[50];
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
	DC_MOTOR_Init(DC_MOTOR_PWM1);
	DC_MOTOR_Init(DC_MOTOR_PWM2);
	DC_MOTOR_Start(DC_MOTOR_PWM1, 0);
	DC_MOTOR_Start(DC_MOTOR_PWM2, 0);

	HAL_UART_Receive_IT(&huart2, &bluetooth_rx, 1);
	HAL_TIM_Base_Start_IT(&htim2);

	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);   //PWM jel start

	//Vonalszenzor init
	Vonalszenzor_Init();

	// initialize vl53l1x communication parameters
	Dev->I2cHandle = &hi2c1;
	Dev->I2cDevAddr = 0x52;

  /*** Initialize GPIO expanders ***/
  // Unused GPIO should be configured as outputs to minimize the power consumption
  buff[0] = 0x14; // GPDR (GPIO set direction register)
  buff[1] = 0xFF; // GPIO_0 - GPIO_7
  buff[2] = 0xFF; // GPIO_8 - GPIO_15
  HAL_I2C_Master_Transmit( &hi2c1, EXPANDER_1_ADDR, buff, 3, 0xFFFF );
  HAL_I2C_Master_Transmit( &hi2c1, EXPANDER_2_ADDR, buff, 3, 0xFFFF );

  // clear XSHUT (disable center module) -> expander 1, GPIO_15
  buff[0] = 0x13; // GPSR + 1 ( GPIO set pin state register)
  HAL_I2C_Master_Transmit( &hi2c1, EXPANDER_1_ADDR, buff, 1, 0xFFFF );
  HAL_I2C_Master_Receive( &hi2c1, EXPANDER_1_ADDR, buff, 1, 0xFFFF );
  buff[1] = buff[0] & ~( 1 << ( 15 - 8 ) ); // clear GPIO_15
  buff[0] = 0x13; // GPSR + 1 ( GPIO set pin state register)
  HAL_I2C_Master_Transmit( &hi2c1, EXPANDER_1_ADDR, buff, 2, 0xFFFF );

  HAL_Delay( 2 ); // 2ms reset time

  // set XSHUT (enable center module) -> expander 1, GPIO_15
  buff[0] = 0x13; // GPSR + 1 ( GPIO set pin state)
  HAL_I2C_Master_Transmit( &hi2c1, EXPANDER_1_ADDR, buff, 1, 0xFFFF );
  HAL_I2C_Master_Receive( &hi2c1, EXPANDER_1_ADDR, buff, 1, 0xFFFF );
  buff[1] = buff[0] | ( 1 << ( 15 - 8 ) ); // set GPIO_15
  buff[0] = 0x13; // GPSR + 1 ( GPIO set pin state register)
  HAL_I2C_Master_Transmit( &hi2c1, EXPANDER_1_ADDR, buff, 2, 0xFFFF );

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
		uint8_t vonal_eredmeny[32] = { 0 };
		Vonalszenzor_operal(vonal_eredmeny);
		for(int i=0; i < 32-1; i++) {
			if(vonal_eredmeny[i] > 9) {
				if(vonal_eredmeny[i+1] > VONAL_THRESHOLD) {
					vonal1 = i;
				}
			}
		}
		sotet0 = vonal_eredmeny[0];
		sotet1 = vonal_eredmeny[1];
		sotet2 = vonal_eredmeny[2];
		sotet3 = vonal_eredmeny[3];
		sotet4 = vonal_eredmeny[4];
		sotet5 = vonal_eredmeny[5];
		sotet6 = vonal_eredmeny[6];
		sotet7 = vonal_eredmeny[7];
		sotet8 = vonal_eredmeny[8];
		sotet9 = vonal_eredmeny[9];
		sotet10 = vonal_eredmeny[10];
		sotet11 = vonal_eredmeny[11];
		sotet12 = vonal_eredmeny[12];
		sotet13 = vonal_eredmeny[13];
		sotet14 = vonal_eredmeny[14];
		sotet15 = vonal_eredmeny[15];
		sotet16 = vonal_eredmeny[16];
		sotet17 = vonal_eredmeny[17];
		sotet18 = vonal_eredmeny[18];
		sotet19 = vonal_eredmeny[19];
		sotet20 = vonal_eredmeny[20];
		sotet21 = vonal_eredmeny[21];
		sotet22 = vonal_eredmeny[22];
		sotet23 = vonal_eredmeny[23];
		sotet24 = vonal_eredmeny[24];
		sotet25 = vonal_eredmeny[25];
		sotet26 = vonal_eredmeny[26];
		sotet27 = vonal_eredmeny[27];
		sotet28 = vonal_eredmeny[28];
		sotet29 = vonal_eredmeny[29];
		sotet30 = vonal_eredmeny[30];
		sotet31 = vonal_eredmeny[31];


		//Bluetooth iras/olvasas logika
		//HC05 Module-ban van puska

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		//uint8_t Test[] = "Hello World\r\n"; //Data to send
		/*int size = sizeof(minta1);
		HAL_UART_Transmit(&huart2, minta1, size, 100);// Sending in normal mode
		HAL_Delay(1000);*/


		//Szervo
		if (btnEnable == 1) {
			if (szervoEnable == 1) {
				if 			(0 <= vonal1 && vonal1 < 6) {
					SERVO_MoveTo(SZERVO, 0);
				} else if 	(6 <= vonal1 && vonal1 < 13) {
					SERVO_MoveTo(SZERVO, 60);
				} else if 	(13 <= vonal1 && vonal1 < 19) {
					SERVO_MoveTo(SZERVO, 90);
				} else if 	(19 <= vonal1 && vonal1 < 26) {
					SERVO_MoveTo(SZERVO, 120);
				} else if 	(26 <= vonal1 && vonal1 < 32) {
					SERVO_MoveTo(SZERVO, 180);
				}
			}

			if (motvezEnable == 1) {
				int k = 200;
				if (k < motvez_d / 2) {
					DC_MOTOR_Set_Speed(DC_MOTOR_PWM1, k); //ha pwm1 nagyobb, előremenet
					DC_MOTOR_Set_Speed(DC_MOTOR_PWM2, motvez_d - k);
				}
			}
		} else {
			SERVO_MoveTo(SZERVO, 90);
			DC_MOTOR_Set_Speed(DC_MOTOR_PWM1, 0);//ez nem egeszen megallas, csak a jel kisimitasa. megallni k = d -vel kell
			DC_MOTOR_Set_Speed(DC_MOTOR_PWM2, 0);
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
  htim2.Init.Prescaler = 8000-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000;
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
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 1);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
static void Vonalszenzor_Init(void) {
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);	// PCB2: Von_OE1  0
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);	// PCB2: Von_OE2  0
}

static void Vonalszenzor_operal(uint8_t* teljes_kiolvasott) {
	Vonalszenzor_minta_kuldes(leszed);
	uint8_t eredmeny_16bit_temp[2] = {0b1110000, 0b00000000};

	Vonalszenzor_minta_kuldes(minta1_adc1e);
	Vonalszenzor_meres_kiolvasas(adc_chanel0, eredmeny_16bit_temp);
	teljes_kiolvasott[0] = (uint8_t) eredmeny_16bit_temp[0];
	Vonalszenzor_meres_kiolvasas(adc_chanel4, eredmeny_16bit_temp);
	teljes_kiolvasott[4] = (uint8_t) eredmeny_16bit_temp[0];
	Vonalszenzor_minta_kuldes(leszed);
	Vonalszenzor_minta_kuldes(minta1_adc2e);
	Vonalszenzor_meres_kiolvasas(adc_chanel0, eredmeny_16bit_temp);
	teljes_kiolvasott[8] = (uint8_t) eredmeny_16bit_temp[0];
	Vonalszenzor_meres_kiolvasas(adc_chanel4, eredmeny_16bit_temp);
	teljes_kiolvasott[12] = (uint8_t) eredmeny_16bit_temp[0];
	Vonalszenzor_minta_kuldes(leszed);
	Vonalszenzor_minta_kuldes(minta1_adc3e);
	Vonalszenzor_meres_kiolvasas(adc_chanel0, eredmeny_16bit_temp);
	teljes_kiolvasott[16] = (uint8_t) eredmeny_16bit_temp[0];
	Vonalszenzor_meres_kiolvasas(adc_chanel4, eredmeny_16bit_temp);
	teljes_kiolvasott[20] = (uint8_t) eredmeny_16bit_temp[0];
	Vonalszenzor_minta_kuldes(leszed);
	Vonalszenzor_minta_kuldes(minta1_adc4e);
	Vonalszenzor_meres_kiolvasas(adc_chanel0, eredmeny_16bit_temp);
	teljes_kiolvasott[24] = (uint8_t) eredmeny_16bit_temp[0];
	Vonalszenzor_meres_kiolvasas(adc_chanel4, eredmeny_16bit_temp);
	teljes_kiolvasott[28] = (uint8_t) eredmeny_16bit_temp[0];
	//HAL_Delay(200);
	Vonalszenzor_minta_kuldes(leszed);


	Vonalszenzor_minta_kuldes(minta2_adc1e);
	Vonalszenzor_meres_kiolvasas(adc_chanel1, eredmeny_16bit_temp);
	teljes_kiolvasott[1] = (uint8_t) eredmeny_16bit_temp[0];
	Vonalszenzor_meres_kiolvasas(adc_chanel5, eredmeny_16bit_temp);
	teljes_kiolvasott[5] = (uint8_t) eredmeny_16bit_temp[0];
	Vonalszenzor_minta_kuldes(leszed);
	Vonalszenzor_minta_kuldes(minta2_adc2e);
	Vonalszenzor_meres_kiolvasas(adc_chanel1, eredmeny_16bit_temp);
	teljes_kiolvasott[9] = (uint8_t) eredmeny_16bit_temp[0];
	Vonalszenzor_meres_kiolvasas(adc_chanel5, eredmeny_16bit_temp);
	teljes_kiolvasott[13] = (uint8_t) eredmeny_16bit_temp[0];
	Vonalszenzor_minta_kuldes(leszed);
	Vonalszenzor_minta_kuldes(minta2_adc3e);
	Vonalszenzor_meres_kiolvasas(adc_chanel1, eredmeny_16bit_temp);
	teljes_kiolvasott[17] = (uint8_t) eredmeny_16bit_temp[0];
	Vonalszenzor_meres_kiolvasas(adc_chanel5, eredmeny_16bit_temp);
	teljes_kiolvasott[21] = (uint8_t) eredmeny_16bit_temp[0];
	Vonalszenzor_minta_kuldes(leszed);
	Vonalszenzor_minta_kuldes(minta2_adc4e);
	Vonalszenzor_meres_kiolvasas(adc_chanel1, eredmeny_16bit_temp);
	teljes_kiolvasott[25] = (uint8_t) eredmeny_16bit_temp[0];
	Vonalszenzor_meres_kiolvasas(adc_chanel5, eredmeny_16bit_temp);
	teljes_kiolvasott[29] = (uint8_t) eredmeny_16bit_temp[0];
	//HAL_Delay(200);
	Vonalszenzor_minta_kuldes(leszed);

	Vonalszenzor_minta_kuldes(minta3_adc1e);
	Vonalszenzor_meres_kiolvasas(adc_chanel2, eredmeny_16bit_temp);
	teljes_kiolvasott[2] = (uint8_t) eredmeny_16bit_temp[0];
	Vonalszenzor_meres_kiolvasas(adc_chanel6, eredmeny_16bit_temp);
	teljes_kiolvasott[6] = (uint8_t) eredmeny_16bit_temp[0];
	Vonalszenzor_minta_kuldes(leszed);
	Vonalszenzor_minta_kuldes(minta3_adc2e);
	Vonalszenzor_meres_kiolvasas(adc_chanel2, eredmeny_16bit_temp);
	teljes_kiolvasott[10] = (uint8_t) eredmeny_16bit_temp[0];
	Vonalszenzor_meres_kiolvasas(adc_chanel6, eredmeny_16bit_temp);
	teljes_kiolvasott[14] = (uint8_t) eredmeny_16bit_temp[0];
	Vonalszenzor_minta_kuldes(leszed);
	Vonalszenzor_minta_kuldes(minta3_adc3e);
	Vonalszenzor_meres_kiolvasas(adc_chanel2, eredmeny_16bit_temp);
	teljes_kiolvasott[18] = (uint8_t) eredmeny_16bit_temp[0];
	Vonalszenzor_meres_kiolvasas(adc_chanel6, eredmeny_16bit_temp);
	teljes_kiolvasott[22] = (uint8_t) eredmeny_16bit_temp[0];
	Vonalszenzor_minta_kuldes(leszed);
	Vonalszenzor_minta_kuldes(minta3_adc4e);
	Vonalszenzor_meres_kiolvasas(adc_chanel2, eredmeny_16bit_temp);
	teljes_kiolvasott[26] = (uint8_t) eredmeny_16bit_temp[0];
	Vonalszenzor_meres_kiolvasas(adc_chanel6, eredmeny_16bit_temp);
	teljes_kiolvasott[30] = (uint8_t) eredmeny_16bit_temp[0];
	//HAL_Delay(200);
	Vonalszenzor_minta_kuldes(leszed);


	Vonalszenzor_minta_kuldes(minta4_adc1e);
	Vonalszenzor_meres_kiolvasas(adc_chanel3, eredmeny_16bit_temp);
	teljes_kiolvasott[3] = (uint8_t) eredmeny_16bit_temp[0];
	Vonalszenzor_meres_kiolvasas(adc_chanel7, eredmeny_16bit_temp);
	teljes_kiolvasott[7] = (uint8_t) eredmeny_16bit_temp[0];
	Vonalszenzor_minta_kuldes(leszed);
	Vonalszenzor_minta_kuldes(minta4_adc2e);
	Vonalszenzor_meres_kiolvasas(adc_chanel3, eredmeny_16bit_temp);
	teljes_kiolvasott[11] = (uint8_t) eredmeny_16bit_temp[0];
	Vonalszenzor_meres_kiolvasas(adc_chanel7, eredmeny_16bit_temp);
	teljes_kiolvasott[15] = (uint8_t) eredmeny_16bit_temp[0];
	Vonalszenzor_minta_kuldes(leszed);
	Vonalszenzor_minta_kuldes(minta4_adc3e);
	Vonalszenzor_meres_kiolvasas(adc_chanel3, eredmeny_16bit_temp);
	teljes_kiolvasott[19] = (uint8_t) eredmeny_16bit_temp[0];
	Vonalszenzor_meres_kiolvasas(adc_chanel7, eredmeny_16bit_temp);
	teljes_kiolvasott[23] = (uint8_t) eredmeny_16bit_temp[0];
	Vonalszenzor_minta_kuldes(leszed);
	Vonalszenzor_minta_kuldes(minta4_adc4e);
	Vonalszenzor_meres_kiolvasas(adc_chanel3, eredmeny_16bit_temp);
	teljes_kiolvasott[27] = (uint8_t) eredmeny_16bit_temp[0];
	Vonalszenzor_meres_kiolvasas(adc_chanel7, eredmeny_16bit_temp);
	teljes_kiolvasott[31] = (uint8_t) eredmeny_16bit_temp[0];
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

void Vonalszenzor_meres_kiolvasas(uint8_t chanel, uint8_t* eredmeny) {
	uint8_t temp1[2]= {chanel,0};
	HAL_SPI_Transmit(&hspi1, temp1, 2, 100);
	HAL_SPI_Receive(&hspi1, eredmeny, 2, 100);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == B1_Pin) {
		btnEnable = !btnEnable;
		//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);	//LED felvilagitasa
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

