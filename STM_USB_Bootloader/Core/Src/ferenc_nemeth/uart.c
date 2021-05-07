/**
 * @file    uart.c
 * @author  Ferenc Nemeth, edited by Sergey Kitaev
 * @date    05 May 2021
 * @brief   This module is a layer between the HAL USB-CDC functions and my Xmodem protocol.
 *
 *          Copyright (c) 2018 Ferenc Nemeth - https://github.com/ferenc-nemeth
 */

#include "uart.h"
#include "usbd_cdc_if.h"


/** Circular buffer   */
volatile uint8_t uart_rx_buffer[2 * APP_RX_DATA_SIZE];
volatile int rx_wp = 0, rx_rp = 0, rx_len = 0;
 
/** Received data over USB are stored in this buffer      */
extern uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];

/** Data to send over USB CDC are stored in this buffer   */
extern uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];

extern USBD_HandleTypeDef hUsbDeviceFS;

/**
 * @brief   Receives data from UART.
 * @param   *data: Array to save the received data.
 * @param   length:  Size of the data.
 * @return  status: Report about the success of the receiving.
 */
uart_status uart_receive(uint8_t *data, uint16_t length)
{
  uint32_t timer;
  
  timer = HAL_GetTick();
  
  while (HAL_GetTick() - timer < UART_TIMEOUT && length > 0) 
  {
    __NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);
    while (rx_len && length) 
    {
      *data = uart_rx_buffer[rx_rp];
      data++;
      rx_rp++;
      if (rx_rp == sizeof(uart_rx_buffer))
      {
        rx_rp = 0;
      }
      rx_len--;
      length--; 
    }
    __NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
  }
  if (length) 
  {
      return UART_ERROR;
  }
  else
  {
      return UART_OK;
  }
}

/**
 * @brief   Transmits a string to UART.
 * @param   *data: Array of the data.
 * @return  status: Report about the success of the transmission.
 */
uart_status uart_transmit_str(uint8_t *data)
{
  uart_status status = UART_ERROR;
  uint16_t length = 0u;

  /* Calculate the length. */
  while ('\0' != data[length])
  {
    length++;
  }
  for (int i = 0; i < UART_TIMEOUT; i++) 
  {  
    if (USBD_OK == CDC_Transmit_FS(data, length))
    {
        status = UART_OK;
        break;
    }
  }

  return status;
}

/**
 * @brief   Transmits a single char to UART.
 * @param   *data: The char.
 * @return  status: Report about the success of the transmission.
 */
uart_status uart_transmit_ch(uint8_t data)
{
  for (int i = 0; i < UART_TIMEOUT; i++) 
  {
    if (USBD_OK == CDC_Transmit_FS(&data, 1))
    {
      return UART_OK;
    }
  }
  return UART_ERROR;
}

/**
 * @brief   Physically disconnects the USB pins.
 * @param   void
 * @return  void
 */
void USB_off(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if (hUsbDeviceFS.pClass) 
  {
    USBD_DeInit(&hUsbDeviceFS);
    HAL_Delay(100);
  }
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11 | GPIO_PIN_12, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  HAL_Delay(1000);
}

