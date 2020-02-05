/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bmp280.h"
#include "bmp280_defs.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "arm_math.h"
#include "math_helper.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SPI_BUFFER_LEN 28
#define BMP280_DATA_INDEX 1
#define BMP280_ADDRESS_INDEX 2

/* PID parameters */
#define PID_PARAM_KP        350            /* Proporcional */
#define PID_PARAM_KI        0        /* Integral */
#define PID_PARAM_KD        0            /* Derivative */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
char data[4];
float TEMP_CURRENT;           /* Temperature we actually have */
float TEMP_WANT;              /* Temperature we want to have */
/* Uchyb PID */
float pid_uchyb;
uint8_t want_uart;
/* ARM PID Instance, float_32 */
arm_pid_instance_f32 PID;
/* Duty cycle for PWM */
float duty = 0;
int flag = 0;
uint8_t msg[4]; // zmienna przechowywujaca znak
uint16_t Sizemsg = 4;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
int8_t spi_reg_read(uint8_t cs, uint8_t reg_addr , uint8_t *reg_data , uint16_t length)
{
  HAL_StatusTypeDef status = HAL_OK;
  int32_t iError = BMP280_OK;
  uint8_t txarray[SPI_BUFFER_LEN] = {0,};
  uint8_t rxarray[SPI_BUFFER_LEN] = {0,};
  uint8_t stringpos;
 txarray[0] = reg_addr;
 HAL_GPIO_WritePin( SPI4_CS_GPIO_Port , SPI4_CS_Pin , GPIO_PIN_RESET );
 status = HAL_SPI_TransmitReceive( &hspi4 , (uint8_t*)(&txarray), (uint8_t*)(&rxarray), length+1, 5);
 while( hspi4.State == HAL_SPI_STATE_BUSY ) {};
 HAL_GPIO_WritePin( SPI4_CS_GPIO_Port , SPI4_CS_Pin , GPIO_PIN_SET );
 for (stringpos = 0; stringpos < length; stringpos++)
 {
	 *(reg_data + stringpos) = rxarray[stringpos + BMP280_DATA_INDEX];
 }
 if (status != HAL_OK)
 {
	 iError = (-1);
 }
 return (int8_t)iError;
}
int8_t spi_reg_write(uint8_t cs, uint8_t reg_addr , uint8_t *reg_data , uint16_t length)
{
HAL_StatusTypeDef status = HAL_OK;
int32_t iError = BMP280_OK;
uint8_t txarray[SPI_BUFFER_LEN * BMP280_ADDRESS_INDEX];
uint8_t stringpos;
txarray[0] = reg_addr;
for (stringpos = 0; stringpos < length; stringpos++)
{
	txarray[stringpos+BMP280_DATA_INDEX] = reg_data[stringpos];
}
 HAL_GPIO_WritePin( SPI4_CS_GPIO_Port , SPI4_CS_Pin , GPIO_PIN_RESET );
 status = HAL_SPI_Transmit( &hspi4 , (uint8_t*)(&txarray), length*2, 100);
 while( hspi4.State == HAL_SPI_STATE_BUSY ) {};
 HAL_GPIO_WritePin( SPI4_CS_GPIO_Port , SPI4_CS_Pin , GPIO_PIN_SET );
if (status != HAL_OK)
{  iError = (-1);

}
return (int8_t)iError;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	PID.Kp = PID_PARAM_KP;        /* Proporcional */
	PID.Ki = PID_PARAM_KI;        /* Integral */
	PID.Kd = PID_PARAM_KD;        /* Derivative */

	int8_t rslt;
	struct bmp280_dev bmp;
	struct bmp280_config conf;
	struct bmp280_uncomp_data ucomp_data;

	double temp;
	char buffer[40];
	uint8_t size;




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
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_SPI4_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  bmp.delay_ms = HAL_Delay;
  bmp.dev_id = 0;
  bmp.intf = BMP280_SPI_INTF;
  bmp.read = spi_reg_read;
  bmp.write = spi_reg_write;
  rslt = bmp280_init(&bmp);
  rslt = bmp280_get_config(&conf, &bmp);
  conf.filter = BMP280_FILTER_COEFF_2;
  conf.os_temp = BMP280_OS_4X;
  conf.os_pres = BMP280_OS_4X;
  conf.odr = BMP280_ODR_1000_MS;
  rslt = bmp280_set_config(&conf, &bmp);
  rslt = bmp280_set_power_mode(BMP280_NORMAL_MODE, &bmp);
  /* Initialize PID system, float32_t format */
  arm_pid_init_f32(&PID, 1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);// grzalka PC7
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);// wiatrak PA6
  HAL_UART_Receive_IT(&huart3, msg, Sizemsg);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  rslt = bmp280_get_uncomp_data(&ucomp_data, &bmp);

   /* Getting the compensated temperature as floating point value */
  rslt = bmp280_get_comp_temp_double(&temp, ucomp_data.uncomp_temp, &bmp);
  TEMP_CURRENT = temp;
  if(flag == 0){
	  TEMP_WANT = TEMP_CURRENT;
  }


  size = sprintf(buffer, "Temp:  %f [C]\n\r", TEMP_CURRENT);
  HAL_UART_Transmit(&huart3, (uint8_t*)buffer, size, 200);

  /* Liczymy uchyb */
  pid_uchyb = TEMP_WANT - TEMP_CURRENT;
  /* Output data will be returned, we will use it as duty cycle parameter */
  duty = arm_pid_f32(&PID, TEMP_WANT - TEMP_CURRENT);



  /* Check overflow, duty cycle in percent */
  if (duty > 100) {
     duty = 100;
  } else if (duty < -100) {
     duty = -100;
  }
  /* Set PWM duty cycle for DC FAN to cool down sensor for "TEMP_CURRENT" */
  if(TEMP_WANT > TEMP_CURRENT){
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);//CH1 WIATRAK
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 10*duty);
  }
  else if(TEMP_WANT < TEMP_CURRENT){
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, -10*duty);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
 }

  if(abs(pid_uchyb) < 0.2){
	  HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, 1);
  }
  else if(abs(pid_uchyb) > 0.2){
	  HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, 0);
  }
  /* Sleep time between measurements = BMP280_ODR_1000_MS */
  bmp.delay_ms(1000);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure LSE Drive Capability 
  */
  HAL_PWR_EnableBkUpAccess();
  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)//odbieranie zadanej temp
{
int czy_dobra = 0;

HAL_UART_Receive_IT(&huart3, msg, Sizemsg);//odebranie znaku

if(msg[0]>47 && msg[0]<58){//kontrola odebranej wiadomosci
	int value1 = msg[0] - '0';
	if(msg[1]>47 && msg[1]<58){
		int value2 = msg[1] - '0';
		if(msg[3]>47 && msg[3]<58){
			int value_afd = msg[3] - '0';
			czy_dobra = 1;
			TEMP_WANT = 10.0 * value1 + value2 + value_afd / 10.0;
			flag = 1;
	}
  }
}
if(czy_dobra != 1){
	char buf[20];
	uint8_t wiad = sprintf(buf, "Zla wartosc\n\r");
	HAL_UART_Transmit(&huart3, (uint8_t*)buf, wiad, 100);
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
  HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
  while(1)
  {
      HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
      HAL_Delay(100);
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
