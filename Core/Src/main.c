/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "adc.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  //activation de l'horloge pour GPIOA et GPIOB
  RCC->IOPENR |= RCC_IOPENR_GPIOAEN | RCC_IOPENR_GPIOBEN;
  /*Ici la fonction LL_mDelay sera utilisée pour faire des temps d'attente en us Pour cette raison on initialise le nombre de ticks nécessaire pour faire 1ms à 16000 au lieu de 16000000
  */
  LL_Init1msTick(16000);

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_ADC_Init();

  /* USER CODE BEGIN 2 */

  //Initialisation du LCD
  lcdinit4(); //call lcdinit4
  LL_ADC_StartCalibration(ADC1);
  while(LL_ADC_IsCalibrationOnGoing(ADC1) != 0);


  /* USER CODE END 2 */
  float voltage=0;
  int Lumens=0,Resistance=0;
  uint16_t adc_value=0;
  int resistor_value=220;

  int tmpV=0,m=0,c=0,d=0,u=0;
  int tmpL=0,M100=0,M10=0,M=0,C=0,D=0,U=0;

  char Text_LCD1[16] = "Voltage u.dcm V ";
  //char Text_LCD2[16] = "Lumens MMMCDU lm";
  char Text_LCD2[16] = "Res MMMCDU ohm  ";
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

	    LL_ADC_Enable(ADC1);
		LL_ADC_REG_StartConversion(ADC1);
		while(LL_ADC_REG_IsConversionOngoing(ADC1) != 0);

		adc_value=LL_ADC_REG_ReadConversionData12(ADC1);

		voltage=adc_value*5.0/4095.0;

		Resistance=resistor_value * ((5.0/voltage) - 1.0); //10k * ( (Vin/Vout) -1)

		Lumens=Resistance;


		tmpV=voltage*1000;
		u=tmpV/1000; //Unite
		tmpV=tmpV%1000;
		d=tmpV/100; //Dixieme
		tmpV=tmpV%100;
		c=tmpV/10; //Centieme
		m=tmpV%10; //Milieme


		Text_LCD1[8] = u+48;
		Text_LCD1[10] = d+48;
		Text_LCD1[11] = c+48;
		Text_LCD1[12] = m+48;

		tmpL=Lumens;
		M10=tmpL/100000; //10^5
		tmpL=tmpL%100000;
		M10=tmpL/10000; //10^4
		tmpL=tmpL%10000;
		M=tmpL/1000; //10^3
		tmpL=tmpL%1000;
		C=tmpL/100; //10^2
		tmpL=tmpL%100;
		D=tmpL/10; //10^1
		U=tmpL%10; //10^0


//		Text_LCD2[7] = M100+48;
//		Text_LCD2[8] = M10+48;
//		Text_LCD2[9] = M+48;
//		Text_LCD2[10] = C+48;
//		Text_LCD2[11] = D+48;
//		Text_LCD2[12] = U+48;

		Text_LCD2[4] = M100+48;
		Text_LCD2[5] = M10+48;
		Text_LCD2[6] = M+48;
		Text_LCD2[7] = C+48;
		Text_LCD2[8] = D+48;
		Text_LCD2[9] = U+48;

		//Affichage sur le LCD
		Affichage_LCD(Text_LCD1, Text_LCD2); //call Affichage_LCD

		LL_mDelay(330000);

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
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_0)
  {
  }
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {

  }
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI)
  {

  }

  LL_Init1msTick(16000000);

  LL_SetSystemCoreClock(16000000);
  LL_RCC_SetUSARTClockSource(LL_RCC_USART2_CLKSOURCE_PCLK1);
}

/* USER CODE BEGIN 4 */

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
