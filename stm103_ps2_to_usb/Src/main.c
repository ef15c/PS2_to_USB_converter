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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_hid.h"
extern USBD_HandleTypeDef hUsbDeviceFS;

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/**
  * @brief  PS2 Device scan codes to USB key codes mapping
  */
static const uint8_t regularScanCodesToUSBKeyCodes[] = {
    1, /* Overrun Error  */
    66, /* F9  */
    0, /* Padding */
    62, /* F5  */
    60, /* F3  */
    58, /* F1  */
    59, /* F2  */
    69, /* F12  */
    0, /* Padding */
    67, /* F10  */
    65, /* F8  */
    63, /* F6  */
    61, /* F4  */
    43, /* Tab  */
    53, /* ` ~  */
    0, /* Padding */
    0, /* Padding */
    0, /* Padding */
    0, /* Padding */
    0, /* Padding */
    0, /* Padding */
    20, /* q Q  */
    30, /* 1 !  */
    0, /* Padding */
    0, /* Padding */
    0, /* Padding */
    29, /* z Z  */
    22, /* s S  */
    4, /* a A  */
    26, /* w W  */
    31, /* 2 @  */
    0, /* Padding */
    0, /* Padding */
    6, /* c C  */
    27, /* x X  */
    7, /* d D  */
    8, /* e E  */
    33, /* 4 $  */
    32, /* 3 #  */
    0, /* Padding */
    0, /* Padding */
    44, /* Space  */
    25, /* v V  */
    9, /* f F  */
    23, /* t T  */
    21, /* r R  */
    34, /* 5 %  */
    0, /* Padding */
    0, /* Padding */
    17, /* n N  */
    5, /* b B  */
    11, /* h H  */
    10, /* g G  */
    28, /* y Y  */
    35, /* 6 ^  */
    0, /* Padding */
    0, /* Padding */
    0, /* Padding */
    16, /* m M  */
    13, /* j J  */
    24, /* u U  */
    36, /* 7 &  */
    37, /* 8 *  */
    0, /* Padding */
    0, /* Padding */
    54, /* , <  */
    14, /* k K  */
    12, /* i I  */
    18, /* o O  */
    39, /* 0 )  */
    38, /* 9 (  */
    0, /* Padding */
    0, /* Padding */
    55, /* . >  */
    56, /* / ?  */
    15, /* l L  */
    51, /* ; :  */
    19, /* p P  */
    45, /* - _  */
    0, /* Padding */
    0, /* Padding */
    0, /* Padding */
    52, /* ' "  */
    0, /* Padding */
    47, /* [ {  */
    46, /* '= + */
    0, /* Padding */
    0, /* Padding */
    57, /* Caps Lock  */
    0, /* Padding */
    40, /* Return  */
    48, /* ] }  */
    0, /* Padding */
    49, /* \ |  */
    0, /* Padding */
    0, /* Padding */
    0, /* Padding */
    100, /* Europe 2 (Note 2)  */
    0, /* Padding */
    0, /* Padding */
    0, /* Padding */
    0, /* Padding */
    42, /* Backspace  */
    0, /* Padding */
    0, /* Padding */
    89, /* Keypad 1 End  */
    0, /* Padding */
    92, /* Keypad 4 Left  */
    95, /* Keypad 7 Home  */
    0, /* Padding */
    0, /* Padding */
    0, /* Padding */
    98, /* Keypad 0 Insert  */
    99, /* Keypad . Delete  */
    90, /* Keypad 2 Down  */
    93, /* Keypad 5  */
    94, /* Keypad 6 Right  */
    96, /* Keypad 8 Up  */
    41, /* Escape  */
    83, /* Num Lock  */
    68, /* F11  */
    87, /* Keypad +  */
    91, /* Keypad 3 PageDn  */
    86, /* Keypad -  */
    85, /* Keypad *  */
    97, /* Keypad 9 PageUp  */
    71, /* Scroll Lock */
    0, /* Padding */
    0, /* Padding */
    0, /* Padding */
    0, /* Padding */
    64, /* F7  */
};

#define PS2_EXTENDED_KEY_ON 0x100
#define PS2_EXTENDED_E1_KEY_ON 0x200
#define PS2_EXTENDED_14_KEY_ON 0x400

#define PS2_EXTENDED_KEY_OFFSET 361
static const uint8_t extendedScanCodesToUSBKeyCodes[] = {
    77, /* End (Note 1)  */
    0, /* Padding */
    80, /* Left Arrow (Note 1)  */
    74, /* Home (Note 1)  */
    0, /* Padding */
    0, /* Padding */
    0, /* Padding */
    73, /* Insert (Note 1)  */
    76, /* Delete (Note 1)  */
    81, /* Down Arrow (Note 1)  */
    0, /* Padding */
    79, /* Right Arrow (Note 1)  */
    82, /* Up Arrow (Note 1)  */
    0, /* Padding */
    0, /* Padding */
    0, /* Padding */
    0, /* Padding */
    78, /* Page Down (Note 1)  */
    0, /* Padding */
    70, /* Print Screen (Note 1) */
    75, /* Page Up (Note 1)  */
};

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
PS2_HandleTypeDef ps2_1;
PS2_HandleTypeDef ps2_2;
USBD_HID_COMPOSITE_HandleTypeDef hhids[2];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
static void processDeviceEvent(PS2_HandleTypeDef *dev);
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
  uint32_t toggleStart;
//  uint8_t kbLeds1 = 4;
//  uint8_t kbLeds2 = 2;
    /* Initialize pUserData tobe sure that initialization is done in proper order */
    hUsbDeviceFS.pUserData = NULL;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
    /* Enable microseconds delay via DWT */
    /* Disable TRC */
    CoreDebug->DEMCR &= ~CoreDebug_DEMCR_TRCENA_Msk; // ~0x01000000;
    /* Enable TRC */
    CoreDebug->DEMCR |=  CoreDebug_DEMCR_TRCENA_Msk; // 0x01000000;
    /* Disable clock cycle counter */
    DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk; //~0x00000001;
    /* Enable  clock cycle counter */
    DWT->CTRL |=  DWT_CTRL_CYCCNTENA_Msk; //0x00000001;
    /* Reset the clock cycle counter value */
    DWT->CYCCNT = 0;
    /* 3 NO OPERATION instructions */
    __ASM volatile ("NOP");
    __ASM volatile ("NOP");
    __ASM volatile ("NOP");

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  toggleStart = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (HAL_GetTick() - toggleStart > 500) {
        HAL_GPIO_TogglePin(ONBOARD_LED_GPIO_Port, ONBOARD_LED_Pin);
        toggleStart += 500;
    }
//    animateKeyboard(&ps2_1, &kbLeds1);
//    animateKeyboard(&ps2_2, &kbLeds2);
    processDeviceEvent(&ps2_1);
    processDeviceEvent(&ps2_2);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ONBOARD_LED_GPIO_Port, ONBOARD_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, KB1_Pin|MOUSE1_Pin|KB2_Pin|MOUSE2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : ONBOARD_LED_Pin */
  GPIO_InitStruct.Pin = ONBOARD_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ONBOARD_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : KB1_Pin MOUSE1_Pin KB2_Pin MOUSE2_Pin */
  GPIO_InitStruct.Pin = KB1_Pin|MOUSE1_Pin|KB2_Pin|MOUSE2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : CLOCK1_Pin */
  GPIO_InitStruct.Pin = CLOCK1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(CLOCK1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DATA1_Pin DATA2_Pin */
  GPIO_InitStruct.Pin = DATA1_Pin|DATA2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : CLOCK2_Pin */
  GPIO_InitStruct.Pin = CLOCK2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(CLOCK2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
/**
  * @brief GPIO Open drain default high
  * @param None
  * @retval None
  */
void PS2_PINS_Output_OD_High(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CLOCK1_GPIO_Port, CLOCK1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(DATA1_GPIO_Port, DATA1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(CLOCK2_GPIO_Port, CLOCK2_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(DATA2_GPIO_Port, DATA2_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : CLOCK1_Pin */
  GPIO_InitStruct.Pin = CLOCK1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CLOCK1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DATA1_Pin, CLOCK2_Pin, DATA2_Pin */
  GPIO_InitStruct.Pin = DATA1_Pin | CLOCK2_Pin | DATA2_Pin;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}


/**
  * @brief Initialize PS/2 handlers for the board
  * @param None
  * @retval None
  */
void BSP_PS2_Init(void)
{
    /* Wait for completion of PS/2 devices self tests */
    HAL_Delay(500);
    PS2_InterfaceMap[0] = NULL;
    PS2_InterfaceMap[1] = NULL;
    hUsbDeviceFS.pUserData = PS2_InterfaceMap;

    /* Probe PS/2 devices */
    ps2_1.clockPort = CLOCK1_GPIO_Port;
    ps2_1.clockPin = CLOCK1_Pin;
    ps2_1.dataPort = DATA1_GPIO_Port;
    ps2_1.dataPin = DATA1_Pin;
    ps2_1.KBLedPort = KB1_GPIO_Port;
    ps2_1.KBLedPin = KB1_Pin;
    ps2_1.MouseLedPort = MOUSE1_GPIO_Port;
    ps2_1.MouseLedPin = MOUSE1_Pin;
    PS2_initHandle(&ps2_1);
    ps2_1.pUserData = hhids+0;
    hhids[0].state = HID_COMPOSITE_IDLE;

    ps2_2.clockPort = CLOCK2_GPIO_Port;
    ps2_2.clockPin = CLOCK2_Pin;
    ps2_2.dataPort = DATA2_GPIO_Port;
    ps2_2.dataPin = DATA2_Pin;
    ps2_2.KBLedPort = KB2_GPIO_Port;
    ps2_2.KBLedPin = KB2_Pin;
    ps2_2.MouseLedPort = MOUSE2_GPIO_Port;
    ps2_2.MouseLedPin = MOUSE2_Pin;
    PS2_initHandle(&ps2_2);
    ps2_2.pUserData = hhids+1;
    hhids[1].state = HID_COMPOSITE_IDLE;
}


/**
  * @brief  EXTI line detection callbacks.
  * @param  GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  switch(GPIO_Pin) {
  case CLOCK1_Pin:
    /* First PS/2 port */
    PS2_ProcessData(&ps2_1);
    break;
  case CLOCK2_Pin:
    /* Second PS/2 port */
    PS2_ProcessData(&ps2_2);
    break;
  }
}

static void updateBit(uint8_t *byte, uint8_t position, uint8_t value)
{
    if (value) {
        *byte |= (1<<position);
    } else {
        *byte &= ~(1<<position);
    }
}

uint8_t addKeyToReport(PS2_HandleTypeDef *dev, uint8_t keyCode)
{
    int i;
    if (2+dev->state.keyboard.nbKeyPressed < sizeof dev->state.keyboard.report) {
        /* Add keyCode to report */
        dev->state.keyboard.report[2+(dev->state.keyboard.nbKeyPressed++)] = keyCode;
        return 0; /* OK */
    } else {
        for (i=2; i<sizeof dev->state.keyboard.report; i++) {
            dev->state.keyboard.report[i] = 1;
        }
        return 1; /* Phantom state */
    }
}


static uint8_t processNextMouseMovementAxis(uint8_t movement, uint8_t sign)
{
    movement >>= 1;
    if (sign) {
        /* Reltaive movement is negative */
        movement |= 0x80;
    }

    return movement;
}


static void processDeviceEvent(PS2_HandleTypeDef *dev)
{
    int16_t data;
    uint8_t keyCode;
    uint8_t eventReceived = 0;
    int i,j;
    uint8_t phantomState;
    int16_t movement;

    USBD_HID_COMPOSITE_HandleTypeDef *hhid = dev->pUserData;
    uint32_t period = (hhid->IdleState & 0xF0) >> 6;
    uint32_t currentTick;


    switch (dev->connectedDevice) {
    case PS2_KEYBOARD:
        while ((data = PS2_DrawByteFormFIFOBuffer(dev)) != -1) {
            data += dev->state.keyboard.extendedE1Key;
            switch (data) {
            case 0xE0: /* Extended key */
                dev->state.keyboard.extendedKey = PS2_EXTENDED_KEY_ON;
                break;
            case 0xE1: /* Pause */
                dev->state.keyboard.extendedE1Key = PS2_EXTENDED_E1_KEY_ON;
                break;
            case PS2_EXTENDED_E1_KEY_ON + 0x14: /* second byte of Pause */
                break;
            case 0xF0: /* Break scan code */
            case PS2_EXTENDED_E1_KEY_ON + 0xF0: /* Pseudo break scan code for Pause */
                dev->state.keyboard.breakCode = 1;
                break;
            default:
                data += dev->state.keyboard.extendedKey;
                eventReceived = 1;
                switch (data) {
                case 0x14: /* Left control */
                    updateBit(dev->state.keyboard.report, 0, !dev->state.keyboard.breakCode);
                    break;
                case 0x12: /* Left shift */
                    updateBit(dev->state.keyboard.report, 1, !dev->state.keyboard.breakCode);
                    break;
                case 0x11: /* Left alt */
                    updateBit(dev->state.keyboard.report, 2, !dev->state.keyboard.breakCode);
                    break;
                case PS2_EXTENDED_KEY_ON + 0x1F: /* Left GUI */
                    updateBit(dev->state.keyboard.report, 3, !dev->state.keyboard.breakCode);
                    break;
                case PS2_EXTENDED_KEY_ON + 0x14: /* Right control */
                    updateBit(dev->state.keyboard.report, 4, !dev->state.keyboard.breakCode);
                    break;
                case 0x59: /* Right shift */
                    updateBit(dev->state.keyboard.report, 5, !dev->state.keyboard.breakCode);
                    break;
                case PS2_EXTENDED_KEY_ON + 0x11: /* Right alt */
                    updateBit(dev->state.keyboard.report, 6, !dev->state.keyboard.breakCode);
                    break;
                case PS2_EXTENDED_KEY_ON + 0x27: /* Right GUI */
                    updateBit(dev->state.keyboard.report, 7, !dev->state.keyboard.breakCode);
                    break;
                default:
                    /* Non-modifier keys */
                    if (data < sizeof regularScanCodesToUSBKeyCodes) {
                        /* Regular scan code not beginning with 0xE0 or 0xE1 */
                        keyCode = regularScanCodesToUSBKeyCodes[data];
                    } else {
                        switch (data) {
                        case PS2_EXTENDED_E1_KEY_ON + 0x77:
                            keyCode = 72; /* Pause */
                            break;
                        case 303:
                            keyCode = 101; /* App  */
                            break;
                        case 330:
                            keyCode = 84; /* Keypad / */
                            break;
                        case 346:
                            keyCode = 88; /* Keypad Enter  */
                            break;
                        default:
                            if (data >= PS2_EXTENDED_KEY_OFFSET &&
                                data < PS2_EXTENDED_KEY_OFFSET + sizeof extendedScanCodesToUSBKeyCodes) {
                                    /* Extended scan code beginning with 0xE0 */
                                    keyCode = extendedScanCodesToUSBKeyCodes[data-PS2_EXTENDED_KEY_OFFSET];
                            } else {
                                keyCode = 0;
                            }
                        }
                    }
                    updateBit(dev->state.keyboard.keyPressed + (keyCode >> 3), keyCode & 7, !dev->state.keyboard.breakCode);
                }

                dev->state.keyboard.extendedKey = 0;
                dev->state.keyboard.extendedE1Key = 0;
                dev->state.keyboard.breakCode = 0;
            }
        }
        if (eventReceived || (period && (HAL_GetTick()-dev->tickLastReport) >= period)) {
            // dev->state.keyboard.report[1] = 0 /* Reserved */
            dev->state.keyboard.nbKeyPressed = 0;
            for(i=0; i<sizeof dev->state.keyboard.keyPressed; i++) {
                data = dev->state.keyboard.keyPressed[i];
                if (data) {
                    keyCode = i << 3;
                    for (j=0;j<8;j++) {
                        if (data & 1) {
                            phantomState = addKeyToReport(dev, keyCode+j);
                            if (phantomState) {
                                break;
                            }
                        }
                        data >>= 1;
                    }
                    if (phantomState) {
                        break;
                    }
                }
            }

            /* Pad report with 0 */
            for (i=2+dev->state.keyboard.nbKeyPressed; i<sizeof dev->state.keyboard.report; i++) {
                dev->state.keyboard.report[i] = 0;
            }

            currentTick = HAL_GetTick();
            while(hhid->state != HID_COMPOSITE_IDLE) {
                /* Wait for end of previous report transmission */
                if ((HAL_GetTick() - currentTick) > 10) {
                    break;
                }
            }

            USBD_HID_COMPOSITE_SendReport(&hUsbDeviceFS, hhid, dev->state.keyboard.report, 8);
            dev->tickLastReport = HAL_GetTick();
        }
        break;
    case PS2_MOUSE:
        while ((data = PS2_DrawByteFormFIFOBuffer(dev)) != -1) {
            /* Read a block of data from the mouse */
            /* Sanity control */
            if (!(data & 0x08)) {
                /* Bit 3 should be set in the first byte of a mouse data packet */
                /* This is not the case, so abort flush data and abort process */
                PS2_FIFOBuffer_Flush(dev);
                break;
            }
            /* Process button states
               USB M R L
               PS2 M R L */
            dev->state.mouse.report[0] = (data & 0x07); /* Left button */

            /* Process X movement */
            movement = PS2_WaitForAnswer(dev, 10);
            if (movement == -1) {
                /* Data not received. Abort */
                break;
            }
            dev->state.mouse.report[1] = processNextMouseMovementAxis(movement, data & 0x10);

            /* Process Y movement */
            movement = PS2_WaitForAnswer(dev, 10);
            if (movement == -1) {
                /* Data not received. Abort */
                break;
            }
            dev->state.mouse.report[2] = -(int8_t)processNextMouseMovementAxis(movement, data & 0x20);

            if (dev->state.mouse.haveWheel) {
                /* Process Wheel movement */
                movement = PS2_WaitForAnswer(dev, 10);
                if (movement == -1 || ((movement & 0xF8) != 0x00 && (movement & 0xF8) != 0xF8 )) {
                    /* Data not received or out of range. Abort */
                    PS2_FIFOBuffer_Flush(dev);
                    break;
                }
                dev->state.mouse.report[3] = -(int8_t)movement;
            } else {
                dev->state.mouse.report[3] = 0;
            }
            /* All is fine, we can report the information to the USB host */
            eventReceived = 1;
        }

        if (eventReceived || (period && (HAL_GetTick()-dev->tickLastReport) >= period)) {
            currentTick = HAL_GetTick();
            while(hhid->state != HID_COMPOSITE_IDLE) {
                /* Wait for end of previous report transmission */
                if ((HAL_GetTick() - currentTick) > 10) {
                    break;
                }
            }

            USBD_HID_COMPOSITE_SendReport(&hUsbDeviceFS, hhid, dev->state.mouse.report,
                                          (hhid->Protocol==1)?4:3);
            dev->tickLastReport = HAL_GetTick();
        }

        break;
    default:
        break;
    }
}

void PS2_SetLeds(PS2_HandleTypeDef *dev)
{
    /* Ordre dans LEDRerort : Scroll Lock, Caps Lock, Num Lock */
    uint8_t kbLEDS;
    uint8_t LEDReport = dev->state.keyboard.LEDReport;

    kbLEDS = (LEDReport & 4) >> 2; /* Scroll Lock */
    kbLEDS |= (LEDReport & 2) << 1; /* Caps Lock */
    kbLEDS |= (LEDReport & 1) << 1; /* Num Lock */

    if (PS2_SendByte(dev, 0xed) != PS2_SUCCESS) {
        return;
    }

    PS2_SendByte(dev, kbLEDS);
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
