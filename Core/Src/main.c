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
#include "can.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
//#include "shell.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
int it_uart = 0;
int it_uart4 = 0;
char uart_rx_buffer[64];
char uart_tx_buffer[64];
char uart4_rx_buffer[64];
char uart4_tx_buffer[64];
char prompt[256]="Guerin/Perret >>> \r\n";
char start[256]="Starting... \r\n";
uint8_t i2c_t_buffer[1] = {0xD0} ;
uint8_t i2c_r_buffer[1];
uint8_t i2c_t_conf_buffer[2] = {0xF4, 0x57} ;
uint8_t i2c_t_calibration_buffer[1]={0x88}; /*creating buffer to get calibration values
with I2C HAL functions
 */

uint8_t i2c_r_conf_buffer[1];
uint8_t i2c_t_data_buffer[1]= {0xF7};
uint8_t i2c_r_data_buffer[6];
uint32_t pres;
uint32_t temp;
uint32_t p;
uint32_t t,t_comp;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
	MX_USART2_UART_Init();
	MX_CAN1_Init();
	MX_UART4_Init();
	MX_I2C1_Init();
	/* USER CODE BEGIN 2 */
	printf("Starting...\r\n");
	HAL_UART_Receive_IT(&huart2, uart_rx_buffer, 1); //activer l'UART STM<-->Ordi
	HAL_UART_Receive_IT(&huart4, uart_rx_buffer, 1); //activer l'UART STM<-->Raspberry

	//identification du BMP280
	HAL_I2C_Master_Transmit(&hi2c1, 0x77<<1, i2c_t_buffer, 1, HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(&hi2c1, 0x77<<1, i2c_r_buffer, 1, HAL_MAX_DELAY);
	printf("%x \r\n", i2c_r_buffer[0]);

	//configuration du BMP280
	HAL_I2C_Master_Transmit(&hi2c1, 0x77<<1, i2c_t_conf_buffer, 2, HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(&hi2c1, 0x77<<1, i2c_r_conf_buffer, 1, HAL_MAX_DELAY);
	printf("%x \r\n", i2c_r_conf_buffer[0]);
	//configuring CAN
	uint8_t message[2]={0x54,0x01};
	uint8_t zero[2]={0x54,0x00};
	uint32_t mail;



	HAL_CAN_Start (&hcan1);
	CAN_TxHeaderTypeDef TxHeader;
	TxHeader.StdId=0x61;
	TxHeader.IDE=CAN_ID_STD;
	TxHeader.RTR=CAN_RTR_DATA;
	TxHeader.DLC=2;
	TxHeader.TransmitGlobalTime=DISABLE;
	//Setting motor to zero
	HAL_CAN_AddTxMessage(&hcan1, &TxHeader, (uint8_t) {0x00,0x00} , &mail);
	HAL_Delay(100);


	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		HAL_CAN_AddTxMessage(&hcan1, &TxHeader, message, &mail);
		//Send_CAN_Message(message,&hcan1,pHeader,mail);
		//to code with tasks
		HAL_Delay(1000);
		HAL_CAN_AddTxMessage(&hcan1, &TxHeader, zero, &mail);
		HAL_Delay(1000);
		//Send_CAN_Message(zero,&hcan1,pHeader,mail);

		//echo - affichage sur la console
		if(it_uart==1){
			//uart_tx_buffer[1] = uart_rx_buffer[1];
			//printf("Lettre = %c \r\n", uart_tx_buffer[1]);
			it_uart = 0;

		}

		//interprétation des commandes envoyées par la raspberry et réponse du STM
		if(it_uart4==1){
			HAL_I2C_Master_Transmit(&hi2c1, 0x77<<1, i2c_t_data_buffer, 1, 50);
			HAL_I2C_Master_Receive(&hi2c1, 0x77<<1, i2c_r_data_buffer, 6, 50);
			pres = i2c_r_data_buffer[0]<<12 | i2c_r_data_buffer[1]<<4 | i2c_r_data_buffer[2]>>4;
			temp = i2c_r_data_buffer[3]<<12 | i2c_r_data_buffer[4]<<4 | i2c_r_data_buffer[5]>>4;
			//Rotation du moteur selon la température extérieure
			if(uart4_rx_buffer[0]=='G'){
				if(uart4_rx_buffer[4]=='T'){
					//on demande T donc on veut renvoyer 'T=val' donc on doit mettre ça dans le buffer à transmettre
					printf("T = %x H \r\n", temp);
					/*uart4_tx_buffer[0]='T';
					uart4_tx_buffer[1]='=';
					uart4_tx_buffer[2]=temp;
					HAL_UART_Transmit(&huart4, uart4_tx_buffer, 3, 50);*/
				}
				if(uart4_rx_buffer[4]=='P'){
					//on demande P donc on veut renvoyer 'P=val' donc on doit mettre ça dans le buffer à transmettre
					printf("P = %x H \r\n", pres);
					/*uart4_tx_buffer[0]='P';
					uart4_tx_buffer[1]='=';
					uart4_tx_buffer[2]=pres;
					HAL_UART_Transmit(&huart4, uart4_tx_buffer, 3, 50);*/
				}
			}
			it_uart4 =0;
		}

		/*Affichage de la temp et de la press non compensés*/
		HAL_I2C_Master_Transmit(&hi2c1, 0x77<<1, i2c_t_data_buffer, 1, 50);
		HAL_I2C_Master_Receive(&hi2c1, 0x77<<1, i2c_r_data_buffer, 6, 50);
		p = i2c_r_data_buffer[0]<<12 | i2c_r_data_buffer[1]<<4 | i2c_r_data_buffer[2]>>4;
		t = i2c_r_data_buffer[3]<<12 | i2c_r_data_buffer[4]<<4 | i2c_r_data_buffer[5]>>4;
		printf("Debut \r\n");
		printf("Pression : %d et Temperature %d \r\n", p, t);
		//compensation maison
		t_comp=i2c_r_data_buffer[3]<<12/100;
		printf("Temperature compensée: %d \r\n",t_comp);
		printf("Fin \r\n");
		HAL_Delay(5000);

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

	/** Configure the main internal regulator output voltage
	 */
	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
	{
		Error_Handler();
	}
	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 1;
	RCC_OscInitStruct.PLL.PLLN = 10;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
	{
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart==&huart4){
		it_uart4 = 1;
		HAL_UART_Receive_IT(&huart4, uart4_rx_buffer, 1);
	}
	if(huart==&huart2){
		it_uart = 1;
		HAL_UART_Receive_IT(&huart2, uart_rx_buffer, 5); //commande de la rasp sur 5 caractères (GET_X)
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
