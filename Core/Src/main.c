/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include "dma.h"
#include "fatfs.h"
#include "sdmmc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "icm20948.h"
#include "cQueue.h"

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
//extern int16_t accel_data[3];
//extern int16_t gyro_data[3];

volatile uint8_t userKey1Flag = 0;
volatile uint8_t dataReadyFlag = 0;
volatile int dataReadyCount = 0;

axises my_gyro;
axises my_accel;
axises my_mag;
int16_t mag_data[3];
IMU imuData;

QueueType* cQ;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
#ifdef __GNUC__
//With GCC, small printf (option LD Linker->Libraries->Small printf set to 'Yes') calls __io_putchar()
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

void
HAL_GPIO_EXTI_Callback (uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_4)
  {
    userKey1Flag = 1;

  }

  if (GPIO_Pin == GPIO_PIN_15)
  {
    dataReadyCount++;
    dataReadyFlag = 0;
    icm20948_accel_gyro_read(&imuData);
    if(!isCQueueFull(cQ)) enCQueue(cQ, imuData);
  }
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
  char uart_buffer[100];
  cQ = createCQueue();
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
  MX_SDMMC1_SD_Init();
  MX_UART4_Init();
  MX_SPI2_Init();
  MX_DMA_Init();
  MX_TIM7_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */

  printf ("start main()\r\n");

  HAL_Delay (10);
  icm20948_init();
  ak09916_init();
  HAL_Delay (10);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  char fileName[12];

  HAL_GPIO_WritePin (LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET);
  int i = 0;
  uint32_t bw, br;

  // 1.
  //printf("%c%c%c%c\r\n", SDPath[0], SDPath[1], SDPath[2], SDPath[3]);
  if ((retSD = f_mount (&SDFatFS, SDPath, 0)) == FR_OK)
  {
    //sprintf(str, "f_mount OK %d", retSD);
    printf ("1. f_mount OK %d \r\n", retSD);
  }
  else
  {
    printf ("1. f_mount OK failed %d\r\n", retSD);
  }
  // 2.

  int fileIndex = 0;

  do
  {
    fileIndex++;
    sprintf (fileName, "data%04d.bin", fileIndex);

    retSD = f_open (&SDFile, fileName, FA_OPEN_EXISTING | FA_READ);
    HAL_UART_Transmit (&huart4, fileName, 12, HAL_MAX_DELAY);
    printf ("  %d\r\n", retSD);
    HAL_Delay (100);
    if (retSD == FR_OK)
      f_close (&SDFile);
    if ( retSD == FR_NOT_READY)
      while(1);
  }while (retSD != FR_NO_FILE);

  HAL_GPIO_WritePin (LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_SET);

  uint8_t startFlag = 0;
  uint8_t save_Count = 0;


  while (1)
  {
    if (startFlag)
    {
      //ICM_SelectBank(USER_BANK_0);
      //ICM_ReadAccelGyro();

      //ICM_ReadMag(mag_data);
      // Print raw, but joined, axis data values to screen


      ///ak09916_mag_read(&imuData);

//      sprintf (uart_buffer, "%10d,%5d,%5d,%5d,%5d,%5d,%5d,%5d,%5d,%5d\r\n",
//               HAL_GetTick(),accel_data[0], accel_data[1], accel_data[2], gyro_data[0],
//               gyro_data[1], gyro_data[2], mag_data[0], mag_data[1],
//               mag_data[2]);
//      sprintf (uart_buffer, "%10d,%5d,%5d,%5d,%5d,%5d,%5d,%5d,%5d,%5d\r\n",
//               HAL_GetTick(),my_accel.x,my_accel.y,my_accel.z, my_gyro.x,
//               my_gyro.y, my_gyro.z, my_mag.x, my_mag.y,
//               my_mag.z);

//      imuData.time = HAL_GetTick();
//      imuData.ax = my_accel.x;
//      imuData.ax = my_accel.y;
//      imuData.ax = my_accel.z;
//
//      imuData.gx = my_gyro.x;
//      imuData.gx = my_gyro.y;
//      imuData.gx = my_gyro.z;
//
//      imuData.ax = my_mag.x;
//      imuData.ax = my_mag.y;
//      imuData.ax = my_mag.z;

      //HAL_UART_Transmit(&huart4, (uint8_t*) uart_buffer, strlen(uart_buffer), 1000);
      //printf("%d\r\n", strlen(uart_buffer));


      f_write (&SDFile,  (uint8_t*)&imuData, 22 , &bw);
      if(save_Count++ == 5){
        save_Count = 0;
        f_sync (&SDFile);
      }
      if( dataReadyCount%500 == 0){
        HAL_GPIO_TogglePin (LED_B_GPIO_Port, LED_B_Pin);
      }

    }

    if (userKey1Flag)
    {
      if (startFlag == 1){
        f_close (&SDFile);
        fileIndex++;
        sprintf (fileName, "data%04d.bin", fileIndex);
      }
      else
      {
        retSD = f_open (&SDFile, fileName, FA_OPEN_APPEND | FA_WRITE);

        if (retSD == FR_OK)
        {
          printf ("OK\r\n");
        }
        else
        {
          printf ("2. Failed %d \r\n", retSD);
        }
      }

      printf ("asdf\r\n");
      if(startFlag){
        HAL_GPIO_WritePin (LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET);
      }else{
        HAL_GPIO_WritePin (LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_SET);
      }
      startFlag = !startFlag;
      HAL_Delay (500);

      userKey1Flag = 0;
    }

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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Macro to configure the PLL clock source
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 160;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART3 and Loop until the end of transmission */
  HAL_UART_Transmit (&huart4, (uint8_t*) &ch, 1, 0xFFFF);

  return ch;
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
  __disable_irq ();
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
