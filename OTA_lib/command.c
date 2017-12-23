/*
Vecmocon Technologies Pvt Ltd
 * @file    Src/command.c
 * @author  MCD Application Team //Ported to stm32f4xx by Altamash Abdul Rahim
 * @version V1.0.0
 * @date    4-April-2016, *20-December-2017
 * @brief   This file provides all the IAP command functions.
*/
/* Includes ------------------------------------------------------------------*/
#include "command.h"
#include "main.h"
#include "flash_if.h"
#include "memory_card.h"

/** @addtogroup USBH_USER
 * @{
 */

/* External variables --------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
#define BUFFERSIZE     ((uint16_t)1*8192)  
/* Private macros ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

uint8_t RAMBuf[BUFFERSIZE] = { 0x00 };

static uint32_t LastPGAddress = APPLICATION_ADDRESS;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/


/**
 * @brief  IAP write memory
 * @param  None
 * @retval None
 */
uint32_t COMMAND_DOWNLOAD(void) {
	fileinfo_t finfno = { 0 };

	if (f_stat(DownloadFile, &finfno) != FR_OK) {
		/* 'STM32.TXT' file Open for write Error */
		return DOWNLOAD_FILE_FAIL;
	}

	if (finfno.fsize < (FLASH_SIZE - IAP_SIZE)) {
		/* Erase necessary page to download image */
		if (FLASH_If_Erase(APPLICATION_ADDRESS) != 0) {
			return DOWNLOAD_ERASE_FAIL;
		}

		/* Program flash memory */
		return COMMAND_ProgramFlashMemory();
	}
	return DOWNLOAD_OK;
}

/**
 * @brief  IAP jump to user program
 * @param  None
 * @retval None
 */
void COMMAND_JUMP(void) {
	/* Software reset */
	NVIC_SystemReset();
}

/**
 * @brief  COMMAND_ProgramFlashMemory
 * @param  None
 * @retval DOWNLOAD_OK: Download process completed
 * @retval DOWNLOAD_FILE_FAIL: not possible to read from opend file
 * @retval DOWNLOAD_WRITE_FAIL: not possible to write to FLASH
 */
uint32_t COMMAND_ProgramFlashMemory(void) {
	__IO uint32_t read_size = 0x00, tmp_read_size = 0x00;
	uint32_t read_flag = TRUE;

	/* Erase address init */
	LastPGAddress = APPLICATION_ADDRESS;

	/* While file still contain data */
	while (read_flag == TRUE) {

		/* Read maximum "BUFFERSIZE" Kbyte from the selected file  */
		if (f_read(&MyFile, RAMBuf, BUFFERSIZE, (uint_t*) &read_size)
				!= FR_OK) {
			return DOWNLOAD_FILE_FAIL;
		}

		/* Temp variable */
		tmp_read_size = read_size;

		/* The read data < "BUFFERSIZE" Kbyte */
		if (tmp_read_size < BUFFERSIZE) {
			read_flag = FALSE;
		}

		/* Program flash memory */
		if (FLASH_If_Write(LastPGAddress, (uint32_t*) RAMBuf, read_size)
				!= FLASHIF_OK) {
			return DOWNLOAD_WRITE_FAIL;
		}

		/* Update last programmed address value */
		LastPGAddress = LastPGAddress + tmp_read_size;
	}

	return DOWNLOAD_OK;
}

