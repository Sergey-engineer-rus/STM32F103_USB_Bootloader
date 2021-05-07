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
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ferenc_nemeth/uart.h"
#include "ferenc_nemeth/flash.h"
#include "ferenc_nemeth/xmodem.h"
#include "usbd_cdc.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TERMINAL_CONNECT_TIMEOUT 5  // sec.
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
IWDG_HandleTypeDef hiwdg;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_IWDG_Init(void);

/* USER CODE BEGIN PFP */
extern USBD_HandleTypeDef hUsbDeviceFS;
extern PCD_HandleTypeDef hpcd_USB_FS;
extern volatile uint8_t usb_state; 
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
  int boot_jumper = (HAL_GPIO_ReadPin(BOOT1_GPIO_Port, BOOT1_Pin) == GPIO_PIN_SET);  
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  // if it is not power reset or brown out reset
  if ((RCC->CSR & RCC_CSR_LPWRRSTF_Msk) == 0 && (RCC->CSR & RCC_CSR_PORRSTF_Msk) == 0)
  {
    MX_GPIO_Init();  
    for (int i = 0; i < 50; i++)
    {
      HAL_Delay(10);
      HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    }
    //HAL_Delay(500);
    USB_off(); // For restart if device reset
  }
  __HAL_RCC_CLEAR_RESET_FLAGS ();
  
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_IWDG_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
  for (int i = 0; i < 1000 && USBD_STATE_CONFIGURED != hUsbDeviceFS.dev_state; i++) 
  {
      HAL_Delay(1);
  }
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
  
  if (USBD_STATE_CONFIGURED != hUsbDeviceFS.dev_state) // USB not connected
  {
    flash_jump_to_app();
  }

  // waiting and reset USB CDC config from PC OS.
  for (int i = 0; i < 1000 && 0xFF == usb_state && USBD_STATE_CONFIGURED == hUsbDeviceFS.dev_state; i++) 
  {
      HAL_Delay(1);
  }
  HAL_Delay(100);
  usb_state = 0xFF; 
    
  for (int i = 0; i < 10 * TERMINAL_CONNECT_TIMEOUT || boot_jumper; i++)
  {
    HAL_IWDG_Refresh(&hiwdg);
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    HAL_Delay(100);  
    
    if (USBD_STATE_CONFIGURED == hUsbDeviceFS.dev_state && CDC_GET_LINE_CODING == usb_state) 
    {
     /*   
      static uint8_t buf[200];
      volatile int len = sizeof(buf) - 1;
      volatile uint32_t addr = FLASH_APP_START_ADDRESS;
      while (len == sizeof(buf) - 1) 
      {
          memcpy((void*)buf, (void*)addr, len);
          addr += len;
          for (int i = 0; i < len; i++)
          {
              if (0xFF == buf[i]) {
                  len = i;
                  break;           
              }
          }
          buf[len] = '\0';
          uart_transmit_str(buf);
      }
      HAL_Delay(1000);
      HAL_NVIC_SystemReset();
      */
      uart_transmit_str((uint8_t*)"\n\r================================\n\r");
      uart_transmit_str((uint8_t*)"UART Bootloader\n\r");
      uart_transmit_str((uint8_t*)"https://github.com/ferenc-nemeth\n\r");
      uart_transmit_str((uint8_t*)"https://github.com/Sergey-engineer-rus/STM32F103_USB_Bootloader\n\r");
      uart_transmit_str((uint8_t*)"================================\n\r\n\r");
      usb_state = 0xFF;
      break;
    }
    else 
    {
      usb_state = 0xFF;
    }
    if (i + 1 == 10 * TERMINAL_CONNECT_TIMEOUT && !boot_jumper) // Terminal "ExtraPuTTY" not connected
    {
       flash_jump_to_app(); 
    }
  }
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* Turn on the green LED to indicate, that we are in bootloader mode.*/
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
    /* Ask for new data and start the Xmodem protocol. */
    uart_transmit_str((uint8_t*)"Please send a new binary file with Xmodem protocol to update the firmware.\n\r"
                                "(You have 10 seconds to do this...)\n\r> ");
    
    if (X_ERROR_TIMEOUT == xmodem_receive()) 
    {
        uart_transmit_str((uint8_t*)"\n\rFile upload start timeout! Please try again.\n\r");
    }
    else
    {
      /* We only exit the xmodem protocol, if there are any errors.
       * In that case, notify the user and start over. */
      uart_transmit_str((uint8_t*)"\n\rFailed... Please try again.\n\r");
    }
    HAL_Delay(1000);
    HAL_NVIC_SystemReset();
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_128;
  hiwdg.Init.Reload = 3125;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
