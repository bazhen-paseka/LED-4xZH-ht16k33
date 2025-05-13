/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
	#include <stdio.h>
 	#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
	#define DEBUG_UART 		huart1
	#define DISPLAY_I2C 	hi2c1
	#define HT16K33_ADDR 	(0x70<<1)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
	char debug_buffer[0xff];
	uint16_t cnt = 0x5000;
	const uint16_t font_14seg[] = {
		[0x0] = 0b0000000000111111,
		[0x1] = 0b0000010000000110,
		[0x2] = 0b0000000011011011,
		[0x3] = 0b0000000011001111,
		[0x4] = 0b0000000011100110,
		[0x5] = 0b0000000011101101,
		[0x6] = 0b0000000011111101,
		[0x7] = 0b0000000000000111,
		[0x8] = 0b0000000011111111,
		[0x9] = 0b0000000011101111,
		[0xa] = 0b0000110010000110,
		[0xb] = 0b0000000011111100,
		[0xc] = 0b0000000000111001,
		[0xd] = 0b0000000011011110,
		[0xe] = 0b0000000011111001,
		[0xf] = 0b0000000011110001,
		//[] = 0b1000000000000000, // point
	};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
	int 	__io_putchar		(int);
	void 	UartDebug			(char*);
	void 	Scan_I2C_to_UART	(void);
	void	Init_HT16K33		(void);
	void	Print_HT16K33		(uint16_t);

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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	#define 	DATE_as_str 	(__DATE__)
	#define 	TIME_as_str 	(__TIME__)
	printf("\r\n\r\n\tLED-4xZH-ht16k33\r\n");
  //printf("\t%s, %s. \r\n" , DATE_as_str , TIME_as_str);
	printf("\tLED-4xZH-ht16k33\r\n");
	printf("\t0.54"" 14-segment LED HT16K33 BackPack\r\n");
	printf("\tDisplay HT16K33, 6 digit, 14 segments, 0.54"" \r\n");
	printf("\t2541AS\r\n");
	printf("\tCubeMX  6.14\r\n");
	printf("\tCubeIDE 1.18\r\n");
	printf("\tSW-DIO: PA13\r\n");
	printf("\tSW-CLK: PA14\r\n");
	printf("\tI2C1-SCK: PB6\r\n");
	printf("\tI2C1-SDA: PB7\r\n");
	printf("\ttest_LED: PA1\r\n");
	printf("\t   debug: PA9, UART1, 9600\r\n");
  //printf("\tUSART1->BRR: 0x%lx\r\n", USART1->BRR);

	Scan_I2C_to_UART();
	Init_HT16K33();
	Print_HT16K33(0x1234); HAL_Delay(1000);
	Print_HT16K33(0x5678); HAL_Delay(1000);
	Print_HT16K33(0x90ab); HAL_Delay(1000);
	Print_HT16K33(0xcdef); HAL_Delay(1000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		Print_HT16K33(cnt++);
		sprintf(debug_buffer, "0x%x ", cnt); UartDebug(debug_buffer);
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		HAL_Delay(1000);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void Init_HT16K33 (void) {
	uint8_t cmd;
	cmd = 0x21;     // Увімкнути осцилятор
	HAL_I2C_Master_Transmit(&DISPLAY_I2C, HT16K33_ADDR, &cmd, 1, HAL_MAX_DELAY);

	cmd = 0x81;     // Вивід увімкнено, без миготіння
	HAL_I2C_Master_Transmit(&DISPLAY_I2C, HT16K33_ADDR, &cmd, 1, HAL_MAX_DELAY);

	cmd = 0xEF;     // Яскравість макс.
	HAL_I2C_Master_Transmit(&DISPLAY_I2C, HT16K33_ADDR, &cmd, 1, HAL_MAX_DELAY);
} //**************************************************************************

void Print_HT16K33(uint16_t cipher) {
	uint8_t data[8] = {0};
	uint16_t D = font_14seg[(cipher/   1)%16];
	uint16_t C = font_14seg[(cipher/  16)%16];
	uint16_t B = font_14seg[(cipher/ 256)%16];
	uint16_t A = font_14seg[(cipher/4096)%16];
	data[0] = A & 0xFF;
	data[1] = A >> 8;
	data[2] = B & 0xFF;
	data[3] = B >> 8;
	data[4] = C & 0xFF;
	data[5] = C >> 8;
	data[6] = D & 0xFF;
	data[7] = D >> 8;

	HAL_I2C_Mem_Write(&DISPLAY_I2C, HT16K33_ADDR, 0x00, I2C_MEMADD_SIZE_8BIT, data, 8, HAL_MAX_DELAY);
} //**************************************************************************

int __io_putchar(int ch) {
    HAL_UART_Transmit(&DEBUG_UART, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}  //**************************************************************************

void UartDebug(char* _text) {
	HAL_UART_Transmit(&DEBUG_UART, (uint8_t*)_text, strlen(_text), 100);
} //**************************************************************************

void Scan_I2C_to_UART(void) {
	char debugChar[100];
	uint8_t device_cnt = 0;
	sprintf(debugChar,"Start scan I2C:\r\n" ); UartDebug(debugChar);

	for ( uint8_t i2c_address = 0x07; i2c_address < 0x78; i2c_address++) {
		if (HAL_I2C_IsDeviceReady( &hi2c1 , i2c_address << 1, 10, 100) == HAL_OK) {
			device_cnt++;
			switch (i2c_address) {
				case 0x23: sprintf(debugChar,"%d) BH1750  ", device_cnt ); break;
				case 0x27: sprintf(debugChar,"%d) FC113   ", device_cnt ); break;
				case 0x20: sprintf(debugChar,"%d) PCF8574 ", device_cnt ); break;
				case 0x38: sprintf(debugChar,"%d) PCF8574 ", device_cnt ); break;
				case 0x3F: sprintf(debugChar,"%d) LCD1602 ", device_cnt ); break;
			  //case 0x57: sprintf(debugChar,"%d) AT24C32 ", device_cnt ); break;
				case 0x57: sprintf(debugChar,"%d) MAX30100", device_cnt ); break;
				case 0x68: sprintf(debugChar,"%d) DS3231  ", device_cnt ); break;
			  //case 0x68: sprintf(debugChar,"%d) MPU9250 ", device_cnt ); break;
				case 0x70: sprintf(debugChar,"%d) HT16K33 ", device_cnt ); break;
				case 0x76: sprintf(debugChar,"%d) BMP280  ", device_cnt ); break;
				case 0x77: sprintf(debugChar,"%d) BMP180  ", device_cnt ); break;
				default:   sprintf(debugChar,"%d) Unknown ", device_cnt ); break;
			}
			char debugChar2[150];
			sprintf(debugChar2,"%s\tAdr: 0x%x\r\n", debugChar, i2c_address); UartDebug(debugChar2);
		} // if HAL I2C1
	} // for()
	sprintf(debugChar,"End scan I2C.\r\n"); UartDebug(debugChar);
} //**************************************************************************
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
  while (1)
  {
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
