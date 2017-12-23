/**
  ******************************************************************************
  * @file    Src/memory_card.c
  * @author  MCD Application Team // Ported to stm32f4xx by Altamash Abdul Rahim
  * @version V1.0.0
  * @date    4-April-2016, *21-December-2017.
  * @brief   This sample code shows how to use FatFs with uSD card drive.
  ******************************************************************************
  */
#include "main.h"

uint8_t retSD;    /* Return value for SD */
char SDPath[4];   /* SD logical drive path */
FATFS SDFatFS;    /* File system object for SD logical drive */
FIL SDFile;       /* File object for SD */

/* USER CODE BEGIN Variables */

/* USER CODE END Variables */    

void MX_FATFS_Init(void) 
{
  /*## FatFS: Link the SD driver ###########################*/
  retSD = FATFS_LinkDriver(&SD_Driver, SDPath);

  /* USER CODE BEGIN Init */
  /* additional user code for init */     
  /* USER CODE END Init */
}
/**
  * @brief  Initializes the Card for reading
  * @param  None
  * @retval FR_OK: Card correctly initialized
            FR_DISK_ERR: Problem during card init process
  */
fresult_t Card_Init(void)
{
  fresult_t res = FR_OK;  /* FatFs function common result code */

  /* Link the micro SD disk I/O driver */
  if (FATFS_LinkDriver(&SD_Driver, SDPath) == 0)
  {
    /* Register the file system object to the FatFs module */
    if (f_mount(&SDFatFs, (TCHAR const*)SDPath, 0) != FR_OK) //change the last param to 1 for immediate mount.
    {
      /* FatFs Initialization Error */
      res = FR_DISK_ERR;
    }
  }
  return res;
}

/**
  * @brief  Closes the open file and unlinkes the card driver
  * @param  None
  * @retval FR_OK: Card correctly unlinked
            FR_NO_FILE: Problem during card unlink process
  */
fresult_t  Card_Unlink(void)
{
  fresult_t res = FR_OK;

  /* Close the open text file */
  if (f_close(&MyFile) != FR_OK )
  {
    res = FR_NO_FILE;
  }

  /* Unlink the RAM disk I/O driver */
  FATFS_UnLinkDriver(SDPath);

  return res;
}


/**
  * @brief  Gets Time from RTC 
  * @param  None
  * @retval Time in DWORD
  */
DWORD get_fattime(void)
{
  /* USER CODE BEGIN get_fattime */
  return 0;
  /* USER CODE END get_fattime */  
}

/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
