/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usb_device.h
  * @version        : v2.0_Cube
  * @brief          : Header for usb_device.c file.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USB_DEVICE__H__
#define __USB_DEVICE__H__

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx.h"
#include "stm32f1xx_hal.h"
#include "usbd_def.h"

/* USER CODE BEGIN INCLUDE */
/* USER CODE END INCLUDE */

/** @addtogroup USBD_OTG_DRIVER
  * @{
  */

/** @defgroup USBD_DEVICE USBD_DEVICE
  * @brief Device file for Usb otg low level driver.
  * @{
  */

/** @defgroup USBD_DEVICE_Exported_Variables USBD_DEVICE_Exported_Variables
  * @brief Public variables.
  * @{
  */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
#define HID_COMPOSITE_EPIN_BASE_ADDR 0x81U
#define HID_COMPOSITE_EPIN_SIZE 8U

#define HID_KEYBOARD_REPORT_DESC_SIZE 0x3F
#define HID_WHEEL_MOUSE_REPORT_DESC_SIZE 68U

#define USB_HID_ZERO_INTERF_CONFIG_DESC_SIZ 9U
#define USB_HID_INTERFACE_DESC_SIZ 25U
#define USB_HID_TWO_INTERF_CONFIG_DESC_SIZ (USB_HID_ZERO_INTERF_CONFIG_DESC_SIZ+2*USB_HID_INTERFACE_DESC_SIZ)
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/*
 * -- Insert your variables declaration here --
 */
/* USER CODE BEGIN VARIABLES */
typedef enum
{
  HID_COMPOSITE_IDLE = 0,
  HID_COMPOSITE_BUSY,
}
HID_COMPOSITE_StateTypeDef;


/** @defgroup USBD_CORE_Exported_TypesDefinitions
  * @{
  */

typedef struct
{
  uint32_t                              Protocol;
  uint32_t                              IdleState;
  uint32_t                              AltSetting;
  volatile HID_COMPOSITE_StateTypeDef   state;
  uint8_t                               interface;
}
USBD_HID_COMPOSITE_HandleTypeDef;

/* USER CODE END VARIABLES */
/**
  * @}
  */

/** @defgroup USBD_DEVICE_Exported_FunctionsPrototype USBD_DEVICE_Exported_FunctionsPrototype
  * @brief Declaration of public functions for Usb device.
  * @{
  */

/** USB Device initialization function. */
void MX_USB_DEVICE_Init(void);

/*
 * -- Insert functions declaration here --
 */
/* USER CODE BEGIN FD */
/** @defgroup USB_CORE_Exported_Functions
  * @{
  */


/**
  * @brief  USBD_HID_SendReport
  *         Send HID Report
  * @param  pdev: device instance
  * @param  interface: interface number
  * @param  buff: pointer to report
  * @retval status
  */
uint8_t USBD_HID_COMPOSITE_SendReport(USBD_HandleTypeDef *pdev,
                            USBD_HID_COMPOSITE_HandleTypeDef *hhid,
                            uint8_t *report,
                            uint16_t len);


/* USER CODE END FD */
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __USB_DEVICE__H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
