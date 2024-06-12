/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    ux_device_cdc_acm.c
  * @author  MCD Application Team
  * @brief   USBX Device applicative file
  ******************************************************************************
    * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "ux_device_cdc_acm.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "main.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define APP_RX_DATA_SIZE                          2048
#define APP_TX_DATA_SIZE                          2048

/* Rx/TX flag */
#define RX_NEW_RECEIVED_DATA                      0x01
#define TX_NEW_TRANSMITTED_DATA                   0x02

#define APP_CDC_ACM_READ_STATE_TX_START  (UX_STATE_APP_STEP + 0)
#define APP_CDC_ACM_READ_STATE_TX_WAIT   (UX_STATE_APP_STEP + 1)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
static UINT write_state = UX_STATE_RESET;
static UINT read_state = UX_STATE_RESET;

volatile ULONG EventFlag = 0;
#if defined ( __ICCARM__ ) /* IAR Compiler */
#pragma location = 0x24028000
#elif defined ( __CC_ARM ) /* MDK ARM Compiler */
__attribute__((section(".UsbxAppSection")))
#elif defined ( __GNUC__ ) /* GNU Compiler */
__attribute__((section(".UsbxAppSection")))
#endif
extern UART_HandleTypeDef huart4;

UX_SLAVE_CLASS_CDC_ACM  *cdc_acm;

/* Data received over uart are stored in this buffer */
uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];

/* Data to send over USB CDC are stored in this buffer   */
uint8_t* UserTxBufferFS;

/* Increment this pointer or roll it back to
start address when data are received over USART */
uint32_t UserTxBufPtrIn;

/* Increment this pointer or roll it back to
start address when data are sent over USB */
uint32_t UserTxBufPtrOut;

uint32_t TxSize;
/* Data to send over USB CDC are stored in this buffer   */
UX_SLAVE_CLASS_CDC_ACM_LINE_CODING_PARAMETER CDC_VCP_LineCoding =
{
  115200, /* baud rate */
  0x00,   /* stop bits-1 */
  0x00,   /* parity - none */
  0x08    /* nb. of bits 8 */
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  USBD_CDC_ACM_Activate
  *         This function is called when insertion of a CDC ACM device.
  * @param  cdc_acm_instance: Pointer to the cdc acm class instance.
  * @retval none
  */
VOID USBD_CDC_ACM_Activate(VOID *cdc_acm_instance)
{
  /* USER CODE BEGIN USBD_CDC_ACM_Activate */

  /* Save the CDC instance */
  cdc_acm = (UX_SLAVE_CLASS_CDC_ACM*) cdc_acm_instance;
  /* USER CODE END USBD_CDC_ACM_Activate */

  return;
}

/**
  * @brief  USBD_CDC_ACM_Deactivate
  *         This function is called when extraction of a CDC ACM device.
  * @param  cdc_acm_instance: Pointer to the cdc acm class instance.
  * @retval none
  */
VOID USBD_CDC_ACM_Deactivate(VOID *cdc_acm_instance)
{
  /* USER CODE BEGIN USBD_CDC_ACM_Deactivate */
  UX_PARAMETER_NOT_USED(cdc_acm_instance);

  /* Reset the cdc acm instance */
  cdc_acm = UX_NULL;
  /* USER CODE END USBD_CDC_ACM_Deactivate */

  return;
}

/**
  * @brief  USBD_CDC_ACM_ParameterChange
  *         This function is invoked to manage the CDC ACM class requests.
  * @param  cdc_acm_instance: Pointer to the cdc acm class instance.
  * @retval none
  */
VOID USBD_CDC_ACM_ParameterChange(VOID *cdc_acm_instance)
{
  /* USER CODE BEGIN USBD_CDC_ACM_ParameterChange */
  UX_PARAMETER_NOT_USED(cdc_acm_instance);

  ULONG request;
  UX_SLAVE_TRANSFER *transfer_request;
  UX_SLAVE_DEVICE *device;

  /* Get the pointer to the device.  */
  device = &_ux_system_slave -> ux_system_slave_device;

  /* Get the pointer to the transfer request associated with the control endpoint. */
  transfer_request = &device -> ux_slave_device_control_endpoint.ux_slave_endpoint_transfer_request;

  request = *(transfer_request -> ux_slave_transfer_request_setup + UX_SETUP_REQUEST);

  switch (request)
  {
    case UX_SLAVE_CLASS_CDC_ACM_SET_LINE_CODING :

      /* Get the Line Coding parameters */
      if (ux_device_class_cdc_acm_ioctl(cdc_acm, UX_SLAVE_CLASS_CDC_ACM_IOCTL_GET_LINE_CODING,
                                        &CDC_VCP_LineCoding) != UX_SUCCESS)
      {
        Error_Handler();
      }

      break;

    case UX_SLAVE_CLASS_CDC_ACM_GET_LINE_CODING :

      /* Set the Line Coding parameters */
      if (ux_device_class_cdc_acm_ioctl(cdc_acm, UX_SLAVE_CLASS_CDC_ACM_IOCTL_SET_LINE_CODING,
                                        &CDC_VCP_LineCoding) != UX_SUCCESS)
      {
        Error_Handler();
      }

      break;

    case UX_SLAVE_CLASS_CDC_ACM_SET_CONTROL_LINE_STATE :
    default :
      break;
  }

  /* USER CODE END USBD_CDC_ACM_ParameterChange */

  return;
}

/* USER CODE BEGIN 1 */


void message_transmit(UCHAR *msg, ULONG len)
{
  UserTxBufferFS = msg;
  TxSize = len;
}


VOID CDC_ACM_Read_Task(VOID)
{
  UX_SLAVE_DEVICE *device;
  UX_SLAVE_INTERFACE *data_interface;
  UX_SLAVE_CLASS_CDC_ACM *cdc_acm;
  UINT  status;
  ULONG read_length;
  static ULONG actual_length;

  /* Get device */
  device = &_ux_system_slave->ux_system_slave_device;

  /* Check if device is configured */
  if (device->ux_slave_device_state != UX_DEVICE_CONFIGURED)
  {
    read_state = UX_STATE_RESET;
    return;
  }

  /* Get Data interface (interface 1) */
  data_interface = device->ux_slave_device_first_interface->ux_slave_interface_next_interface;
  cdc_acm =  data_interface->ux_slave_interface_class_instance;
  read_length = (_ux_system_slave->ux_system_slave_speed == UX_HIGH_SPEED_DEVICE) ? 512 : 64;

  /* Run state machine.  */
  switch(read_state)
  {
    case UX_STATE_RESET:
      read_state = UX_STATE_WAIT;
      /* Fall through.  */
    case UX_STATE_WAIT:
      status = ux_device_class_cdc_acm_read_run(cdc_acm, (UCHAR *)UserRxBufferFS, read_length, &actual_length);
      /* Error.  */
      if (status <= UX_STATE_ERROR)
      {
        /* Reset state.  */
        read_state = UX_STATE_RESET;
        return;
      }
      if (status == UX_STATE_NEXT)
      {
        if (actual_length != 0)
        {
          read_state = APP_CDC_ACM_READ_STATE_TX_START;
        }
        else
        {
          read_state = UX_STATE_RESET;
        }
        return;
      }
      /* Wait.  */
      return;
    case APP_CDC_ACM_READ_STATE_TX_START:
    #if 0
    	/* Need to add customer code for handling UserRxBufferFS */
    #endif
      /* DMA started.  */
      read_state = APP_CDC_ACM_READ_STATE_TX_WAIT;
      /* Fall through.  */
    case APP_CDC_ACM_READ_STATE_TX_WAIT:
      if (EventFlag & TX_NEW_TRANSMITTED_DATA)
      {  
        EventFlag &= ~TX_NEW_TRANSMITTED_DATA;
      }
      read_state = UX_STATE_WAIT;
      return;
    default:
      return;
  }
}


VOID CDC_ACM_Write_Task(VOID)
{
  UX_SLAVE_DEVICE    *device;
  UX_SLAVE_INTERFACE *data_interface;
  UX_SLAVE_CLASS_CDC_ACM *cdc_acm;
  ULONG actual_length;

  ULONG buffsize;
  UINT ux_status = UX_SUCCESS;

  /* Get device */
  device = &_ux_system_slave->ux_system_slave_device;

  /* Check if device is configured */
  if (device->ux_slave_device_state != UX_DEVICE_CONFIGURED)
  {
    read_state = UX_STATE_RESET;
    return;
  }

  /* Get Data interface */
  data_interface = device->ux_slave_device_first_interface->ux_slave_interface_next_interface;
  cdc_acm =  data_interface->ux_slave_interface_class_instance;

  switch(write_state)
  {
  case UX_STATE_RESET:

    /* Send data over the class cdc_acm_write */
    if(UserTxBufferFS != UX_NULL){
      buffsize = TxSize;
      ux_status = ux_device_class_cdc_acm_write_run(cdc_acm, UserTxBufferFS, buffsize, &actual_length);
      /* Please uncommant below line to tranmit the buffer synchronizing with timer frequency */
      //UserTxBufferFS = UX_NULL; // 
    }
          
    if (ux_status != UX_STATE_WAIT)
    {
      /* Reset state.  */
      read_state = UX_STATE_RESET;
      return;
    }
    write_state = UX_STATE_WAIT;
    return;
      
  case UX_STATE_WAIT:    	
    /* Continue to run state machine.  */
    ux_status = ux_device_class_cdc_acm_write_run(cdc_acm, UX_NULL, 0, &actual_length);
    /* Check if there is  fatal error.  */
    if (ux_status < UX_STATE_IDLE)
    {
      /* Reset state.  */
      read_state = UX_STATE_RESET;
      return;
    }
    /* Check if dataset is transmitted */
    if (ux_status <= UX_STATE_NEXT)
    {
      /* Increment the UserTxBufPtrOut pointer */
      UserTxBufPtrOut += actual_length;

      /* Rollback UserTxBufPtrOut if it equal to APP_TX_DATA_SIZE */
      if (UserTxBufPtrOut == APP_TX_DATA_SIZE)
      {
        UserTxBufPtrOut = 0;
      }
      write_state = UX_STATE_RESET;
    }
    /* Keep waiting.  */
    return;
  default:
    return;
  }
}

/* USER CODE END 1 */
