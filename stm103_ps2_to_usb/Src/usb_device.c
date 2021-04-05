/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usb_device.c
  * @version        : v2.0_Cube
  * @brief          : This file implements the USB Device
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

#include "usb_device.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_hid.h"

/* USER CODE BEGIN Includes */
extern USBD_HandleTypeDef hUsbDeviceFS;

/* USER CODE END Includes */

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#undef HID_EPIN_ADDR
#undef HID_MOUSE_REPORT_DESC_SIZE
/* USB HID device FS Configuration Descriptor */
/* Override the original one found in usbd_hid.c */
__ALIGN_BEGIN static uint8_t USBD_HID_CfgFSDesc[USB_HID_TWO_INTERF_CONFIG_DESC_SIZ]  __ALIGN_END =
{
  0x09, /* bLength: Configuration Descriptor size */
  USB_DESC_TYPE_CONFIGURATION, /* bDescriptorType: Configuration */
  USB_HID_ZERO_INTERF_CONFIG_DESC_SIZ,
  /* wTotalLength: Bytes returned */
  0x00,
  0x00,         /*bNumInterfaces: 0 interface, will be dynamically updated when PS/2 device are found */
  0x01,         /*bConfigurationValue: Configuration value*/
  0x00,         /*iConfiguration: Index of string descriptor describing
  the configuration*/
  0xA0,         /*bmAttributes: bus powered and Support Remote Wake-up */
  150,         /*MaxPower 300 mA: this current is used for detecting Vbus*/

};

static const uint8_t USBD_HID_KeyboardItrfFSDesc[USB_HID_INTERFACE_DESC_SIZ] =
{
  /************** Descriptor of Keyboard interface ****************/
  /* 09 */
  0x09,         /*bLength: Interface Descriptor size*/
  USB_DESC_TYPE_INTERFACE,/*bDescriptorType: Interface descriptor type*/
  0x00,         /*bInterfaceNumber: Number of Interface*/
  0x00,         /*bAlternateSetting: Alternate setting*/
  0x01,         /*bNumEndpoints*/
  0x03,         /*bInterfaceClass: HID*/
  0x01,         /*bInterfaceSubClass : 1=BOOT, 0=no boot*/
  0x01,         /*nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse*/
  0,            /*iInterface: Index of string descriptor*/
  /******************** Descriptor of Keyboard HID ********************/
  /* 18 */
  0x09,         /*bLength: HID Descriptor size*/
  HID_DESCRIPTOR_TYPE, /*bDescriptorType: HID*/
  0x11,         /*bcdHID: HID Class Spec release number*/
  0x01,
  0x00,         /*bCountryCode: Hardware target country*/
  0x01,         /*bNumDescriptors: Number of HID class descriptors to follow*/
  0x22,         /*bDescriptorType*/
  HID_KEYBOARD_REPORT_DESC_SIZE,/*wItemLength: Total length of Report descriptor*/
  0x00,
  /******************** Descriptor of Keyboard endpoint ********************/
  /* 27 */
  0x07,          /*bLength: Endpoint Descriptor size*/
  USB_DESC_TYPE_ENDPOINT, /*bDescriptorType:*/
  HID_COMPOSITE_EPIN_BASE_ADDR,     /*bEndpointAddress: Endpoint Address (IN)*/
  0x03,          /*bmAttributes: Interrupt endpoint*/
  HID_COMPOSITE_EPIN_SIZE, /*wMaxPacketSize */
  0x00,
  HID_FS_BINTERVAL,          /*bInterval: Polling Interval */
  /* 34 */
};

static const uint8_t USBD_HID_MouseItrfFSDesc[USB_HID_INTERFACE_DESC_SIZ] =
{
  /************** Descriptor of Mouse interface ****************/
  /* 09 */
  0x09,         /*bLength: Interface Descriptor size*/
  USB_DESC_TYPE_INTERFACE,/*bDescriptorType: Interface descriptor type*/
  0x00,         /*bInterfaceNumber: Number of Interface*/
  0x00,         /*bAlternateSetting: Alternate setting*/
  0x01,         /*bNumEndpoints*/
  0x03,         /*bInterfaceClass: HID*/
  0x01,         /*bInterfaceSubClass : 1=BOOT, 0=no boot*/
  0x02,         /*nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse*/
  0,            /*iInterface: Index of string descriptor*/
  /******************** Descriptor of Mouse HID ********************/
  /* 18 */
  0x09,         /*bLength: HID Descriptor size*/
  HID_DESCRIPTOR_TYPE, /*bDescriptorType: HID*/
  0x11,         /*bcdHID: HID Class Spec release number*/
  0x01,
  0x00,         /*bCountryCode: Hardware target country*/
  0x01,         /*bNumDescriptors: Number of HID class descriptors to follow*/
  0x22,         /*bDescriptorType*/
  HID_WHEEL_MOUSE_REPORT_DESC_SIZE,/*wItemLength: Total length of Report descriptor*/
  0x00,
  /******************** Descriptor of Mouse endpoint ********************/
  /* 27 */
  0x07,          /*bLength: Endpoint Descriptor size*/
  USB_DESC_TYPE_ENDPOINT, /*bDescriptorType:*/
  HID_COMPOSITE_EPIN_BASE_ADDR,     /*bEndpointAddress: Endpoint Address (IN)*/
  0x03,          /*bmAttributes: Interrupt endpoint*/
  HID_COMPOSITE_EPIN_SIZE, /*wMaxPacketSize */
  0x00,
  HID_FS_BINTERVAL,          /*bInterval: Polling Interval */
  /* 34 */
};

/* USB HID Keyboard HID Descriptor */
__ALIGN_BEGIN static uint8_t USBD_HID_KEYBOARD_Desc[USB_HID_DESC_SIZ]  __ALIGN_END  =
{
  /* 18 */
  0x09,         /*bLength: HID Descriptor size*/
  HID_DESCRIPTOR_TYPE, /*bDescriptorType: HID*/
  0x11,         /*bcdHID: HID Class Spec release number*/
  0x01,
  0x00,         /*bCountryCode: Hardware target country*/
  0x01,         /*bNumDescriptors: Number of HID class descriptors to follow*/
  0x22,         /*bDescriptorType*/
  HID_KEYBOARD_REPORT_DESC_SIZE,/*wItemLength: Total length of Report descriptor*/
  0x00,
};


__ALIGN_BEGIN static uint8_t HID_KEYBOARD_ReportDesc[HID_KEYBOARD_REPORT_DESC_SIZE]  __ALIGN_END =
{
    0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
    0x09, 0x06,                    // USAGE (Keyboard)
    0xa1, 0x01,                    // COLLECTION (Application)
    0x05, 0x07,                    //   USAGE_PAGE (Keyboard)
    0x19, 0xe0,                    //   USAGE_MINIMUM (Keyboard LeftControl)
    0x29, 0xe7,                    //   USAGE_MAXIMUM (Keyboard Right GUI)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x25, 0x01,                    //   LOGICAL_MAXIMUM (1)
    0x75, 0x01,                    //   REPORT_SIZE (1)
    0x95, 0x08,                    //   REPORT_COUNT (8)
    0x81, 0x02,                    //   INPUT (Data,Var,Abs)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x81, 0x03,                    //   INPUT (Cnst,Var,Abs)
    0x95, 0x03,                    //   REPORT_COUNT (3)
    0x75, 0x01,                    //   REPORT_SIZE (1)
    0x05, 0x08,                    //   USAGE_PAGE (LEDs)
    0x19, 0x01,                    //   USAGE_MINIMUM (Num Lock)
    0x29, 0x03,                    //   USAGE_MAXIMUM (Scroll Lock)
    0x91, 0x02,                    //   OUTPUT (Data,Var,Abs)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0x75, 0x05,                    //   REPORT_SIZE (5)
    0x91, 0x03,                    //   OUTPUT (Cnst,Var,Abs)
    0x95, 0x06,                    //   REPORT_COUNT (6)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x25, 0x65,                    //   LOGICAL_MAXIMUM (101)
    0x05, 0x07,                    //   USAGE_PAGE (Keyboard)
    0x19, 0x00,                    //   USAGE_MINIMUM (Reserved (no event indicated))
    0x29, 0x65,                    //   USAGE_MAXIMUM (Keyboard Application)
    0x81, 0x00,                    //   INPUT (Data,Ary,Abs)
    0xc0                           // END_COLLECTION
};

/* USB HID Mouse HID Descriptor */
__ALIGN_BEGIN static uint8_t USBD_HID_WHEEL_MOUSE_Desc[USB_HID_DESC_SIZ]  __ALIGN_END  =
{
  /* 18 */
  0x09,         /*bLength: HID Descriptor size*/
  HID_DESCRIPTOR_TYPE, /*bDescriptorType: HID*/
  0x11,         /*bcdHID: HID Class Spec release number*/
  0x01,
  0x00,         /*bCountryCode: Hardware target country*/
  0x01,         /*bNumDescriptors: Number of HID class descriptors to follow*/
  0x22,         /*bDescriptorType*/
  HID_WHEEL_MOUSE_REPORT_DESC_SIZE,/*wItemLength: Total length of Report descriptor*/
  0x00,
};


__ALIGN_BEGIN static uint8_t HID_WHEEL_MOUSE_ReportDesc[HID_WHEEL_MOUSE_REPORT_DESC_SIZE]  __ALIGN_END =
{
    0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
    0x09, 0x02,                    // USAGE (Mouse)
    0xa1, 0x01,                    // COLLECTION (Application)
    0x09, 0x01,                    //   USAGE (Pointer)
    0xa1, 0x00,                    //   COLLECTION (Physical)
    0x05, 0x09,                    //     USAGE_PAGE (Button)
    0x19, 0x01,                    //     USAGE_MINIMUM (Button 1)
    0x29, 0x03,                    //     USAGE_MAXIMUM (Button 3)
    0x15, 0x00,                    //     LOGICAL_MINIMUM (0)
    0x25, 0x01,                    //     LOGICAL_MAXIMUM (1)
    0x95, 0x03,                    //     REPORT_COUNT (3)
    0x75, 0x01,                    //     REPORT_SIZE (1)
    0x81, 0x02,                    //     INPUT (Data,Var,Abs)
    0x95, 0x01,                    //     REPORT_COUNT (1)
    0x75, 0x05,                    //     REPORT_SIZE (5)
    0x81, 0x03,                    //     INPUT (Cnst,Var,Abs)
    0x05, 0x01,                    //     USAGE_PAGE (Generic Desktop)
    0x09, 0x30,                    //     USAGE (X)
    0x09, 0x31,                    //     USAGE (Y)
    0x09, 0x38,                    //     USAGE (Wheel)
    0x15, 0x81,                    //     LOGICAL_MINIMUM (-127)
    0x25, 0x7f,                    //     LOGICAL_MAXIMUM (127)
    0x75, 0x08,                    //     REPORT_SIZE (8)
    0x95, 0x03,                    //     REPORT_COUNT (3)
    0x81, 0x06,                    //     INPUT (Data,Var,Rel)
    0xc0,                          //   END_COLLECTION
    0x09, 0x3c,                    //   USAGE (Motion Wakeup)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x25, 0x01,                    //   LOGICAL_MAXIMUM (1)
    0x75, 0x01,                    //   REPORT_SIZE (1)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0xb1, 0x22,                    //   FEATURE (Data,Var,Abs,NPrf)
    0x95, 0x07,                    //   REPORT_COUNT (7)
    0xb1, 0x01,                    //   FEATURE (Cnst,Ary,Abs)
    0xc0                           // END_COLLECTION
};

/* USER CODE END PV */

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
/**
  * @brief  USBD_CUSTOM_HID_EP0_RxReady
  *         Handles control request data.
  * @param  pdev: device instance
  * @retval status
  */
static uint8_t USBD_HID_EP0_RxReady(USBD_HandleTypeDef *pdev)
{
  PS2_HandleTypeDef **PS2_HandleMap = hUsbDeviceFS.pUserData;
  int i;

  for (i=0; i<2; i++) {
      if (PS2_HandleMap[i]->state.keyboard.isReportAvailable == 1)
      {
        PS2_SetLeds(PS2_HandleMap[i]);
        PS2_HandleMap[i]->state.keyboard.isReportAvailable = 0;
      }
  }

  return USBD_OK;
}


/**
  * @brief  Build the configuration descriptor with the PS/2 device information
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static void  BuildfFSCfgDesc(uint8_t *descriptor, PS2_HandleTypeDef *dev)
{
    PS2_HandleTypeDef **PS2_HandleMap = hUsbDeviceFS.pUserData;
    if (PS2_HandleMap[0] != dev && PS2_HandleMap[1] != dev && dev->connectedDevice != PS2_NO_DEVICE) {

        uint8_t previousCfgDescLen = descriptor[2];

        if (dev->connectedDevice == PS2_KEYBOARD) {
            /* Add keyboard interface description to configuration descriptor */
            memcpy(descriptor+previousCfgDescLen, USBD_HID_KeyboardItrfFSDesc, USB_HID_INTERFACE_DESC_SIZ);
        } else if (dev->connectedDevice == PS2_MOUSE) {
            /* Add keyboard interface description to configuration descriptor */
            memcpy(descriptor+previousCfgDescLen, USBD_HID_MouseItrfFSDesc, USB_HID_INTERFACE_DESC_SIZ);
        }

        /* Adjust to new size */
        descriptor[2] += USB_HID_INTERFACE_DESC_SIZ;

        /* Associate PS/2 Handle to USB interface */
        PS2_HandleMap[descriptor[4]] = dev;
        ((USBD_HID_COMPOSITE_HandleTypeDef *)dev->pUserData)->interface = descriptor[4];

        /* Add one interface */
        /* Assign the interface number (0 for the first one) */
        descriptor[previousCfgDescLen+2] = descriptor[4];
        /* Change endpoint address */
        descriptor[previousCfgDescLen+20] = HID_COMPOSITE_EPIN_BASE_ADDR + descriptor[4];
        /* Change interface count */
        descriptor[4] += 1;
    }
}


/**
  * @brief  USBD_HID_GetCfgFSDesc
  *         return FS configuration descriptor
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t  *USBD_HID_GetFSCfgDesc(uint16_t *length)
{
    BuildfFSCfgDesc(USBD_HID_CfgFSDesc, &ps2_1);
    BuildfFSCfgDesc(USBD_HID_CfgFSDesc, &ps2_2);

  *length = USBD_HID_CfgFSDesc[2];
  return USBD_HID_CfgFSDesc;
}

/**
  * @brief  USBD_HID_Setup
  *         Handle the HID specific requests
  * @param  pdev: instance
  * @param  req: usb requests
  * @retval status
  */
static uint8_t  USBD_HID_Setup(USBD_HandleTypeDef *pdev,
                               USBD_SetupReqTypedef *req)
{
  USBD_HID_HandleTypeDef *hhid = (USBD_HID_HandleTypeDef *) pdev->pClassData;
  uint16_t len = 0U;
  uint8_t *pbuf = NULL;
  uint16_t status_info = 0U;
  USBD_StatusTypeDef ret = USBD_OK;
  PS2_HandleTypeDef **PS2_HandleMap = pdev->pUserData;
  PS2_HandleTypeDef *ps2_dev;

  switch (req->bmRequest & USB_REQ_TYPE_MASK)
  {
    case USB_REQ_TYPE_CLASS :
      switch (req->bRequest)
      {
        case HID_REQ_SET_PROTOCOL:
          ps2_dev = PS2_HandleMap[req->wIndex];
          if (ps2_dev) {
              USBD_HID_COMPOSITE_HandleTypeDef *hhid = ps2_dev->pUserData;
              hhid->Protocol = (uint8_t)(req->wValue);
          } else {
              USBD_CtlError(pdev, req);
              ret = USBD_FAIL;
          }
          break;

        case HID_REQ_GET_PROTOCOL:
          ps2_dev = PS2_HandleMap[req->wIndex];
          if (ps2_dev) {
              USBD_HID_COMPOSITE_HandleTypeDef *hhid = ps2_dev->pUserData;
              USBD_CtlSendData(pdev, (uint8_t *)(void *)&hhid->Protocol, 1U);
          } else {
              USBD_CtlError(pdev, req);
              ret = USBD_FAIL;
          }
          break;

        case HID_REQ_SET_IDLE:
            ps2_dev = PS2_HandleMap[req->wIndex];
          if (ps2_dev) {
              USBD_HID_COMPOSITE_HandleTypeDef *hhid = ps2_dev->pUserData;
              hhid->IdleState = (uint8_t)(req->wValue >> 8);
          } else {
              USBD_CtlError(pdev, req);
              ret = USBD_FAIL;
          }
          break;

        case HID_REQ_GET_IDLE:
          ps2_dev = PS2_HandleMap[req->wIndex];
          if (ps2_dev) {
              USBD_HID_COMPOSITE_HandleTypeDef *hhid = ps2_dev->pUserData;
              USBD_CtlSendData(pdev, (uint8_t *)(void *)&hhid->IdleState, 1U);
          } else {
              USBD_CtlError(pdev, req);
              ret = USBD_FAIL;
          }
          break;

        /* TODO (nounours#3#) A améliorer si un jour une souris doit recevoir un report*/
        case HID_REQ_SET_REPORT:
          ps2_dev = PS2_HandleMap[req->wIndex];
          if (ps2_dev && ps2_dev->connectedDevice == PS2_KEYBOARD && req->wLength == 1) {
              ps2_dev->state.keyboard.isReportAvailable = 1;
              USBD_CtlPrepareRx(pdev, &(ps2_dev->state.keyboard.LEDReport), req->wLength);
              break;
          } /* Let control flow to default if it is not a keyboard */

        default:
          USBD_CtlError(pdev, req);
          ret = USBD_FAIL;
          break;
      }
      break;
    case USB_REQ_TYPE_STANDARD:
      switch (req->bRequest)
      {
        case USB_REQ_GET_STATUS:
          if (pdev->dev_state == USBD_STATE_CONFIGURED)
          {
            USBD_CtlSendData(pdev, (uint8_t *)(void *)&status_info, 2U);
          }
          else
          {
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
          }
          break;

        case USB_REQ_GET_DESCRIPTOR:
          ps2_dev = PS2_HandleMap[req->wIndex];
          len = 0;

          if (req->wValue >> 8 == HID_REPORT_DESC)
          {
            if (ps2_dev && ps2_dev->connectedDevice == PS2_KEYBOARD) {
                len = MIN(HID_KEYBOARD_REPORT_DESC_SIZE, req->wLength);
                pbuf = HID_KEYBOARD_ReportDesc;
            } else if (ps2_dev && ps2_dev->connectedDevice == PS2_MOUSE) {
                len = MIN(HID_WHEEL_MOUSE_REPORT_DESC_SIZE, req->wLength);
                pbuf = HID_WHEEL_MOUSE_ReportDesc;
                USBD_HID_COMPOSITE_HandleTypeDef *hhid = ps2_dev->pUserData;
                hhid->Protocol = 1;
            }
          }
          else if (req->wValue >> 8 == HID_DESCRIPTOR_TYPE)
          {
            if (ps2_dev && ps2_dev->connectedDevice == PS2_KEYBOARD) {
                pbuf = USBD_HID_KEYBOARD_Desc;
                len = MIN(USB_HID_DESC_SIZ, req->wLength);
            } else if (ps2_dev && ps2_dev->connectedDevice == PS2_MOUSE) {
                pbuf = USBD_HID_WHEEL_MOUSE_Desc;
                len = MIN(USB_HID_DESC_SIZ, req->wLength);
            }
          }
          else
          {
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
            break;
          }
          USBD_CtlSendData(pdev, pbuf, len);
          break;

        case USB_REQ_GET_INTERFACE :
          if (pdev->dev_state == USBD_STATE_CONFIGURED)
          {
            USBD_CtlSendData(pdev, (uint8_t *)(void *)&hhid->AltSetting, 1U);
          }
          else
          {
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
          }
          break;

        case USB_REQ_SET_INTERFACE :
          if (pdev->dev_state == USBD_STATE_CONFIGURED)
          {
            hhid->AltSetting = (uint8_t)(req->wValue);
          }
          else
          {
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
          }
          break;

        default:
          USBD_CtlError(pdev, req);
          ret = USBD_FAIL;
          break;
      }
      break;

    default:
      USBD_CtlError(pdev, req);
      ret = USBD_FAIL;
      break;
  }

  return ret;
}

/**
  * @brief  USBD_HID_Init
  *         Initialize the HID interface
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t  USBD_HID_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  PS2_HandleTypeDef **PS2_HandleMap = pdev->pUserData;
  int i;

  /* Open EP IN */
  for (i=0; i<2; i++) {
      if (PS2_HandleMap[0]->connectedDevice != PS2_NO_DEVICE) {
          USBD_LL_OpenEP(pdev, HID_COMPOSITE_EPIN_BASE_ADDR+i, USBD_EP_TYPE_INTR, HID_COMPOSITE_EPIN_SIZE);
          pdev->ep_in[(HID_COMPOSITE_EPIN_BASE_ADDR+i) & 0xFU].is_used = 1U;
      }
  }

  return USBD_OK;
}

/**
  * @brief  USBD_HID_DataIn
  *         handle data IN Stage
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t  USBD_HID_DataIn(USBD_HandleTypeDef *pdev,
                                uint8_t epnum)
{
  PS2_HandleTypeDef **PS2_HandleMap = pdev->pUserData;
  PS2_HandleTypeDef *ps2_dev = PS2_HandleMap[epnum-1];

  /* Ensure that the FIFO is empty before a new transfer, this condition could
  be caused by  a new transfer before the end of the previous transfer */
  if (ps2_dev) {
      ((USBD_HID_COMPOSITE_HandleTypeDef *)ps2_dev->pUserData)->state = HID_COMPOSITE_IDLE;
      return USBD_OK;
  } else {
      return USBD_FAIL;
  }
}


/* USER CODE END PFP */

/* USB Device Core handle declaration. */
USBD_HandleTypeDef hUsbDeviceFS;

/*
 * -- Insert your variables declaration here --
 */
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*
 * -- Insert your external function declaration here --
 */
/* USER CODE BEGIN 1 */
uint8_t USBD_HID_COMPOSITE_SendReport(USBD_HandleTypeDef *pdev,
                            USBD_HID_COMPOSITE_HandleTypeDef *hhid,
                            uint8_t *report,
                            uint16_t len)
{
  if (pdev->dev_state == USBD_STATE_CONFIGURED)
  {
    if (hhid->state == HID_COMPOSITE_IDLE)
    {
      hhid->state = HID_COMPOSITE_BUSY;
      USBD_LL_Transmit(pdev,
                       HID_COMPOSITE_EPIN_BASE_ADDR + hhid->interface,
                       report,
                       len);
    }
  }
  return USBD_OK;
}
/* USER CODE END 1 */

/**
  * Init USB device Library, add supported class and start the library
  * @retval None
  */
void MX_USB_DEVICE_Init(void)
{
  /* USER CODE BEGIN USB_DEVICE_Init_PreTreatment */
  PS2_PINS_Output_OD_High();
  BSP_PS2_Init();

  USBD_HID.GetFSConfigDescriptor = USBD_HID_GetFSCfgDesc;
  USBD_HID.Setup = USBD_HID_Setup;
  USBD_HID.Init = USBD_HID_Init;
  USBD_HID.EP0_RxReady = USBD_HID_EP0_RxReady;
  USBD_HID.DataIn = USBD_HID_DataIn;
  /* USER CODE END USB_DEVICE_Init_PreTreatment */

  /* Init Device Library, add supported class and start the library. */
  if (USBD_Init(&hUsbDeviceFS, &FS_Desc, DEVICE_FS) != USBD_OK)
  {
    Error_Handler();
  }
  if (USBD_RegisterClass(&hUsbDeviceFS, &USBD_HID) != USBD_OK)
  {
    Error_Handler();
  }
  if (USBD_Start(&hUsbDeviceFS) != USBD_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN USB_DEVICE_Init_PostTreatment */

  /* USER CODE END USB_DEVICE_Init_PostTreatment */
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
