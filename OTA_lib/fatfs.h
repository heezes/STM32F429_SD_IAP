/**
  ******************************************************************************
  * @file    Inc/MemoryCard.h
  * @author  MCD Application Team //Ported to stm32f4xx by Altamash Abdul Rahim
  * @version V1.0.0
  * @date    4-April-2016, *21-December-2017
  * @brief   This file provides all the headers of the flash_if functions.
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __fatfs_H
#define __fatfs_H

/* Includes ------------------------------------------------------------------*/
/* FatFs includes component */
#include "main.h"
#include "ff_gen_drv.h"
#include "sd_diskio.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void MX_FATFS_Init(void);
fresult_t Card_Init(void);
fresult_t Card_Unlink(void);
/* Exported varables ------------------------------------------------------- */
extern FIL MyFile;     /* File object */

#endif  /* __fatfs_H */

