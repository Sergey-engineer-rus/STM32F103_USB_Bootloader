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

volatile uint32_t uart_len_target = 0, uart_len_current = 0;
extern volatile uint8_t usb_state;    
     
/** Received data over USB are stored in this buffer      */
extern uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];

/** Data to send over USB CDC are stored in this buffer   */
extern uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];

extern volatile uint8_t uart_rx_buffer[2 * APP_RX_DATA_SIZE];

extern IWDG_HandleTypeDef hiwdg;



/**
 * @brief   Receives data from UART.
 * @param   *data: Array to save the received data.
 * @param   length:  Size of the data.
 * @return  status: Report about the success of the receiving.
 */
uart_status uart_receive(uint8_t *data, uint16_t length)
{
  uart_status status = UART_ERROR;
  
  uart_len_current = 0;
  uart_len_target = length;
  for (int i = 0; i < UART_TIMEOUT; i++) 
  {
    HAL_Delay(1);
    if (uart_len_current == uart_len_target) 
    {
      status = UART_OK;
      memcpy((void*)data, (void*)uart_rx_buffer, length);
      HAL_IWDG_Refresh(&hiwdg);
    }
  }
  return status;
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
  uart_status status = UART_ERROR;

  /* Make available the UART module. */
  for (int i = 0; i < UART_TIMEOUT; i++) 
  {
    if (CDC_SET_CONTROL_LINE_STATE == usb_state) 
    {
      status = UART_OK;
    }
    HAL_Delay(1);
  }
  
  for (int i = 0; i < UART_TIMEOUT && UART_OK == status; i++) 
  {
    if (USBD_OK == CDC_Transmit_FS(&data, 1))
    {
      status = UART_OK;
      break;
    }
  }
  
  return status;
}
