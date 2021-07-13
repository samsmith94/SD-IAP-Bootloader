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
#include "fatfs.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "fatfs_sd.h"
#include "bootloader.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CONF_FILENAME "app.bin"


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/*
FATFS fs;
FIL fil;
FRESULT fresult;

char buffer[1024];
UINT br, bw;

FATFS *pfs;
DWORD fre_clust;
uint32_t total, free_space;
*/

char SDPath[4]; /* SD logical drive path */
FATFS SDFatFs;  /* File system object for SD logical drive */
FIL SDFile;     /* File object for SD */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Enter_Bootloader(void);
void SD_Eject(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	MX_SPI2_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_FATFS_Init();
	/* USER CODE BEGIN 2 */

	/*
	 fresult = f_mount(&fs, "", 0);
	 if (fresult != FR_OK) {
	 printf("Error when mounting SD card.\r\n");
	 } else {
	 printf("SD card mounted successfully.\r\n");
	 }
	 HAL_Delay(10);
	 // Check free space
	 f_getfree("", &fre_clust, &pfs);

	 total = (uint32_t) ((pfs->n_fatent - 2) * pfs->csize * 0.5);
	 printf("SD card total size: %.2f GB\r\n", total / 1024.0 / 1024.0);

	 free_space = (uint32_t) (fre_clust * pfs->csize * 0.5);
	 printf("SD card free space: %.2f GB\r\n", free_space / 1024.0 / 1024.0);

	 // Open file to write/ create a file if it doesn't exist
	 fresult = f_open(&fil, "sleep.txt", FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
	 HAL_Delay(10);
	 // Writing text
	 fresult = f_write(&fil, "Hello World!\r\n", strlen("Hello World!\r\n"),
	 &bw);
	 fresult = f_write(&fil, "2. line\r\n", strlen("2. line\r\n"), &bw);

	 // Close file
	 fresult = f_close(&fil);
	 printf("file1.txt created and the data is written.\r\n");
	 */

	/**************************************************************************/
	printf("\nPower up, Boot started.\r\n");
	HAL_Delay(500);

	// Check system reset flags
	if (__HAL_RCC_GET_FLAG(RCC_FLAG_OBLRST)) {
		printf("OBL flag is active.\r\n");
#if(CLEAR_RESET_FLAGS)
        // Clear system reset flags
        __HAL_RCC_CLEAR_RESET_FLAGS();
        printf("Reset flags cleared.\r\n");
#endif
	}

	printf("Entering Bootloader...\r\n");
	Enter_Bootloader();

	// Check if there is application in user flash area
	if (Bootloader_CheckForApplication() == BL_OK) {
#if(USE_CHECKSUM)
        // Verify application checksum
        if(Bootloader_VerifyChecksum() != BL_OK)
        {
            printf("Checksum Error.\r\n");
            Error_Handler();
        }
        else
        {
            printf("Checksum OK.\r\n");
        }
#endif

		printf("Launching Application.\r\n");

		// De-initialize bootloader hardware & peripherals
		//TODO: SD_DeInit();
		//TODO: GPIO_DeInit();

		// Launch application
		Bootloader_JumpToApplication();
	}

	// No application found
	printf("No application in flash.\r\n");

	/**************************************************************************/

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

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
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
	PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
	/** Configure the main internal regulator output voltage
	 */
	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1)
			!= HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */

int __io_putchar(int ch) {
	HAL_UART_Transmit(&huart2, (uint8_t*) &ch, 1, 0xFFFF);
	return ch;
}

void SD_Eject(void) {
	f_mount(NULL, (TCHAR const*) SDPath, 0);
}

void SD_DeInit(void) {
	//BSP_SD_DeInit();
	//FATFS_DeInit();
	//SDCARD_OFF();
}

/*** Bootloader ***************************************************************/
void Enter_Bootloader(void) {
	FRESULT fr;
	UINT num;
	uint8_t status;
	uint64_t data;
	uint32_t cntr;
	uint32_t addr;
	char msg[40] = { 0x00 };

	/* Check for flash write protection */
	if (Bootloader_GetProtectionStatus() & BL_PROTECTION_WRP) {
		printf("Application space in flash is write protected. Disabling write protection and generating system reset...\r\n");
		Bootloader_ConfigProtection(BL_PROTECTION_NONE);
	}

	// Initialize SD card
	/*
	 if (SD_Init()) {
	 // SD init failed
	 printf("SD card cannot be initialized.\r\n");
	 return;
	 }*/

	// Mount SD card
	fr = f_mount(&SDFatFs, (TCHAR const*) SDPath, 1);
	if (fr != FR_OK) {
		// f_mount failed
		printf("SD card cannot be mounted. FatFs error code: %u\r\n", fr);
		return;
	}
	printf("SD mounted.\r\n");

	// Open file for programming
	fr = f_open(&SDFile, CONF_FILENAME, FA_READ);
	if (fr != FR_OK) {
		// f_open failed
		printf("File cannot be opened. FatFs error code: %u\r\n", fr);
		SD_Eject();
		printf("SD ejected.\r\n");
		return;
	}
	printf("Software found on SD.\r\n");

	// Check size of application found on SD card
	if (Bootloader_CheckSize(f_size(&SDFile)) != BL_OK) {
		printf("Error: app on SD card is too large.\r\n");

		f_close(&SDFile);
		SD_Eject();
		printf("SD ejected.\r\n");
		return;
	}
	printf("App size OK.\r\n");

	// Step 1: Init Bootloader and Flash
	Bootloader_Init();

	// Step 2: Erase Flash
	printf("Erasing flash...\r\n");
	Bootloader_Erase();
	printf("Flash erase finished.\r\n");

	// Step 3: Programming
	printf("Starting programming...\r\n");
	cntr = 0;
	Bootloader_FlashBegin();
	do {
		data = 0xFFFFFFFFFFFFFFFF;
		fr = f_read(&SDFile, &data, 8, &num);
		if (num) {
			status = Bootloader_FlashNext(data);
			if (status == BL_OK) {
				cntr++;
			} else {
				printf(msg, "Programming error at: %lu byte\r\n", (cntr * 8));
				f_close(&SDFile);
				SD_Eject();
				printf("SD ejected.\r\n");
				return;
			}
		}
	} while ((fr == FR_OK) && (num > 0));

	// Step 4: Finalize Programming
	Bootloader_FlashEnd();
	f_close(&SDFile);
	printf("Programming finished. Flashed: %lu bytes.\r\n", (cntr * 8));

	// Open file for verification
	fr = f_open(&SDFile, CONF_FILENAME, FA_READ);
	if (fr != FR_OK) {
		// f_open failed
		printf("File cannot be opened. FatFs error code: %u\r\n", fr);

		SD_Eject();
		printf("SD ejected.\r\n");
		return;
	}

	// Step 5: Verify Flash Content
	addr = APP_ADDRESS;
	cntr = 0;
	do {
		data = 0xFFFFFFFFFFFFFFFF;
		fr = f_read(&SDFile, &data, 4, &num);
		if (num) {
			if (*(uint32_t*) addr == (uint32_t) data) {
				addr += 4;
				cntr++;
			} else {
				printf("Verification error at: %lu byte.\r\n", (cntr * 4));
				f_close(&SDFile);
				SD_Eject();
				printf("SD ejected.\r\n");
				return;
			}
		}
	} while ((fr == FR_OK) && (num > 0));
	printf("Verification passed.\r\n");

	// Eject SD card
	SD_Eject();
	printf("SD ejected.\r\n");

	// Enable flash write protection
#if(USE_WRITE_PROTECTION)
    printf("Enablig flash write protection and generating system reset...\r\n");
    if(Bootloader_ConfigProtection(BL_PROTECTION_WRP) != BL_OK)
    {
        printf("Failed to enable write protection. Exiting Bootloader.\r\n");
    }
#endif
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
