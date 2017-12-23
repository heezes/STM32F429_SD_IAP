/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "fatfs.h"

/* USER CODE BEGIN Includes */
#include "command.h"
#include "flash_if.h"
#include <string.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
SD_HandleTypeDef hsd;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
HAL_SD_CardInfoTypeDef SDCardInfo;
/* Private variables ---------------------------------------------------------*/
#define RX_TIMEOUT          ((uint32_t)0xFFFFFFFF)
uint32_t  volatile  FlashProtection = 0;
pFunction   JumpToApplication;
uint32_t    JumpAddress;
FRESULT res;
const char DownloadFile[] = "binary.bin";
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
//static void RunApplication(void);
static void Main_Menu(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
char tx[100];
uint8_t key;
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_SDIO_SD_Init();
  MX_FATFS_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */
  res = f_mount(&SDFatFS, SDPath,1);
  if(res==FR_OK)
  {HAL_UART_Transmit(&huart1,(uint8_t *)"SD CARD Mounted\r\n",17,20);}
  HAL_Delay(2000);
  if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)==GPIO_PIN_SET)
  {
	  HAL_GPIO_TogglePin(GPIOG,GPIO_PIN_14);
	  Main_Menu();
  }
  else
  {
    /* Test if user code is programmed starting from address "APPLICATION_ADDRESS" */
    if (((*(__IO uint32_t*)APPLICATION_ADDRESS) & 0x2FFE0000 ) == 0x20000000)
    {
      /* Jump to user application */
      JumpAddress = *(__IO uint32_t*) (APPLICATION_ADDRESS + 4);
      JumpToApplication = (pFunction) JumpAddress;
#if   (defined ( __GNUC__ ))
    /* Compensation as the Stack Pointer is placed at the very end of RAM */
    __set_MSP((*(__IO uint32_t*) APPLICATION_ADDRESS) - 64);
#else  /* (defined  (__GNUC__ )) */
    __set_MSP(*(__IO uint32_t*) APPLICATION_ADDRESS);
#endif /* (defined  (__GNUC__ )) */
      JumpToApplication();
    }
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Activate the Over-Drive mode 
    */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* SDIO init function */
static void MX_SDIO_SD_Init(void)
{

  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 0;

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PG13 PG14 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/**
  * @brief  Display the Main Menu on Display
  * @param  None
  * @retval None
  */
void Main_Menu(void)
{
  while (1)
  {
	sprintf(tx,"\r\n=================== Main Menu ============================\r\n\n");
    HAL_UART_Transmit(&huart1,(uint8_t *)&tx,strlen(tx),30);
    memset(tx,'\0',sizeof(tx));
    sprintf(tx,"  Download image to the internal Flash ----------------- 0\r\n\n");
    HAL_UART_Transmit(&huart1,(uint8_t *)&tx,strlen(tx),30);
    memset(tx,'\0',sizeof(tx));
    sprintf(tx,"  Execute the loaded application ----------------------- 1\r\n\n");
    HAL_UART_Transmit(&huart1,(uint8_t *)&tx,strlen(tx),30);
    memset(tx,'\0',sizeof(tx));
    /* Test if any sector of Flash memory where user application will be loaded is write protected */
    FlashProtection = FLASH_If_GetWriteProtectionStatus();

    if ((FlashProtection & FLASHIF_PROTECTION_WRPENABLED) != 0)
    {
        sprintf(tx,"  Disable the write protection ------------------------- 2\r\n\n");
        HAL_UART_Transmit(&huart1,(uint8_t *)&tx,strlen(tx),50);
        memset(tx,'\0',sizeof(tx));
    }
    else
    {
        sprintf(tx,"  Enable the write protection ------------------------- 2\r\n\n");
        HAL_UART_Transmit(&huart1,(uint8_t *)&tx,strlen(tx),50);
        memset(tx,'\0',sizeof(tx));
    }
    HAL_UART_Receive(&huart1, &key, 1, RX_TIMEOUT);
    switch (key)
    {
    case '0' :
      if (f_open(&MyFile, DownloadFile, FA_READ) != FR_OK)
      {
        sprintf(tx,"  Open file error \r\n\n");
        HAL_UART_Transmit(&huart1,(uint8_t *)&tx,strlen(tx),50);
        memset(tx,'\0',sizeof(tx));
      }
      else
      {
        sprintf(tx,"  Download ongoing\r\n\n");
        HAL_UART_Transmit(&huart1,(uint8_t *)&tx,strlen(tx),50);
        memset(tx,'\0',sizeof(tx));
        if (COMMAND_DOWNLOAD() != DOWNLOAD_OK)
        {
          sprintf(tx," Flash download error\r\n\n");
          HAL_UART_Transmit(&huart1,(uint8_t *)&tx,strlen(tx),50);
          memset(tx,'\0',sizeof(tx));
        }
        else
        {
					  HAL_GPIO_TogglePin(GPIOG,GPIO_PIN_13);
            sprintf(tx," Download done\r\n\n");
            HAL_UART_Transmit(&huart1,(uint8_t *)&tx,strlen(tx),50);
            memset(tx,'\0',sizeof(tx));
        }
     }
      break;
    case '1' :
      sprintf(tx," Executing Program\r\n\n");
      HAL_UART_Transmit(&huart1,(uint8_t *)&tx,strlen(tx),50);
      memset(tx,'\0',sizeof(tx));
      /* Jump to user application */
      JumpAddress = *(__IO uint32_t*) (APPLICATION_ADDRESS + 4);
      JumpToApplication = (pFunction) JumpAddress;
#if   (defined ( __GNUC__ ))
    /* Compensation as the Stack Pointer is placed at the very end of RAM */
    __set_MSP((*(__IO uint32_t*) APPLICATION_ADDRESS) - 64);
#else  /* (defined  (__GNUC__ )) */
    __set_MSP(*(__IO uint32_t*) APPLICATION_ADDRESS);
#endif /* (defined  (__GNUC__ )) */
      JumpToApplication();
      break;

    case '2' :
        sprintf(tx," Enable/Disable Flash Protection\r\n\n");
        HAL_UART_Transmit(&huart1,(uint8_t *)&tx,strlen(tx),50);
        memset(tx,'\0',sizeof(tx));
        if (FlashProtection != FLASHIF_PROTECTION_NONE)
        {
          /* Disable the write protection */
          if (FLASH_If_WriteProtectionConfig(OB_WRPSTATE_DISABLE) == HAL_OK)
          {
            sprintf(tx," Write Protection disabled...\r\n\n");
            HAL_UART_Transmit(&huart1,(uint8_t *)&tx,strlen(tx),50);
            memset(tx,'\0',sizeof(tx));
            /* Launch the option byte loading */
            HAL_FLASH_OB_Launch();
            /* Ulock the flash */
            HAL_FLASH_Unlock();
          }
          else
          {
            sprintf(tx," Error: Flash write un-protection failed...\r\n\n");
            HAL_UART_Transmit(&huart1,(uint8_t *)&tx,strlen(tx),50);
            memset(tx,'\0',sizeof(tx));
          }
        }
        else
        {
          if (FLASH_If_WriteProtectionConfig(OB_WRPSTATE_ENABLE) == HAL_OK)
          {
            sprintf(tx," Write Protection enabled...\r\n\n");
            HAL_UART_Transmit(&huart1,(uint8_t *)&tx,strlen(tx),50);
            memset(tx,'\0',sizeof(tx));
            /* Launch the option byte loading */
            HAL_FLASH_OB_Launch();
          }
          else
          {
              sprintf(tx," Error: Flash write protection failed...\r\n\n");
              HAL_UART_Transmit(&huart1,(uint8_t *)&tx,strlen(tx),50);
              memset(tx,'\0',sizeof(tx));
          }
        }
        break;
      }
	}
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
