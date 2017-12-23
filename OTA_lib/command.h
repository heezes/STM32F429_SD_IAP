/**
  ******************************************************************************
  * Vecmocon Technologies Pvt Ltd
  * @file    Inc/command.h
  * @author  MCD Application Team //Ported to stm32f4xx by Altamash Abdul Rahim
  * @version V1.0.0
  * @date    4-April-2016, *20-December-2017
  * @brief   Header file for command.c
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _COMMAND_H
#define _COMMAND_H

#ifdef __cplusplus
extern "C"
{
#endif

  /* Includes ------------------------------------------------------------------*/
#include "flash_if.h"


  /* Exported types ------------------------------------------------------------*/
  /* Exported constants --------------------------------------------------------*/
  enum  /* Error code */
  {
    DOWNLOAD_OK = 0,
    DOWNLOAD_FAIL,
    DOWNLOAD_FILE_FAIL,
    DOWNLOAD_ERASE_FAIL,
    DOWNLOAD_WRITE_FAIL
  };

  /* Exported macros -----------------------------------------------------------*/
  /* Exported functions ------------------------------------------------------- */
  void COMMAND_JUMP(void);
  uint32_t COMMAND_DOWNLOAD(void);
  uint32_t COMMAND_ProgramFlashMemory(void);

#ifdef __cplusplus
}
#endif

#endif  /* _COMMAND_H */

