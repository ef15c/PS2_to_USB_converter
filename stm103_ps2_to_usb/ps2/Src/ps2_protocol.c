/**
  Copyrignt 2021 Christian Schoffit

  This file is part of ps2_to usb.

    ps2_to usb is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    ps2_to usb is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with ps2_to usb.  If not, see <https://www.gnu.org/licenses/>.
*/

#include "main.h"

static void PS2_ProbeDevice(PS2_HandleTypeDef *dev);

/**
  * @brief Tell the PS/2 device to stop sending data and prepare to receive a
           command from host

  * @param  dev PS2 handle.
  * @retval None
  */

static void request_to_send(PS2_HandleTypeDef *dev)
{
    if (dev->ActivityLedPort) {
        /* Turn activity LED on */
        HAL_GPIO_WritePin(dev->ActivityLedPort, dev->ActivityLedPin, GPIO_PIN_SET);
    }
    HAL_GPIO_WritePin(dev->clockPort, dev->clockPin, GPIO_PIN_RESET);
    DWT_Delay_us(150); /* Should wait for at least 100 microseconds */
    HAL_GPIO_WritePin(dev->dataPort, dev->dataPin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(dev->clockPort, dev->clockPin, GPIO_PIN_SET);
}

void PS2_SendByteAsync(PS2_HandleTypeDef *dev, uint8_t byte)
{
    int i, parity = 1;
    uint32_t currentTick = HAL_GetTick();

    while (dev->direction != PS2_DEVICE_TO_HOST) {
        /* Wait until the PS/2 device is ready for the command */
        if ((HAL_GetTick()-currentTick)>1) {
            return;
        }
    }

    dev->direction = PS2_HOST_TO_DEVICE;

    dev->data = (byte | 0x200) << 1;

    /* Parity computation */
    for (i=0; i<8; i++) {
        if (byte & 1) {
            parity++;
        }
        byte >>= 1;
    }

    dev->data |= (parity & 1) << 9;

    dev->status = PS2_IN_PROGRESS;
    request_to_send(dev);
}


PS2_ResultCodeTypedef PS2_SendByte(PS2_HandleTypeDef *dev, uint8_t byte)
{
    PS2_SendByteAsync(dev, byte);
    while (dev->status == PS2_IN_PROGRESS) {
    /* Wait answer */
    }

    return dev->status;
}

void PS2_initHandle(PS2_HandleTypeDef *dev)
{
    dev->pUserData = NULL;
    dev->connectedDevice = PS2_NO_DEVICE;
    dev->direction = PS2_NONE;
    dev->dataBuffer.head = 0;
    dev->dataBuffer.tail = 0;

    dev->ActivityLedPort = 0;
    dev->DeviceTypeLedPort = 0;
    /* Synchronize clock */
    HAL_GPIO_WritePin(dev->clockPort, dev->clockPin, GPIO_PIN_RESET);
    HAL_Delay(1); /* Should wait for 100 microseconds, so 1 ms is OK */
    HAL_GPIO_WritePin(dev->clockPort, dev->clockPin, GPIO_PIN_SET);

    dev->numberOfBitsRemainingToBeRead = 11;
    dev->status = PS2_SUCCESS;
    dev->parityEnabled = 1;

    PS2_ProbeDevice(dev);
}


/**
  * @brief  Process a data bit from/to a PS/2 port.
  * @param  dev: the PS/2 port handle
  * @retval None
  */
void PS2_ProcessData(PS2_HandleTypeDef *dev)
{ /* Warning : this function is call via a GPIO interrupt : keep it as fast as possible */
    GPIO_PinState dataBit;

    switch (dev->direction) {
    case PS2_NONE:
        if (HAL_GPIO_ReadPin(dev->clockPort, dev->clockPin) == GPIO_PIN_SET) {
            dev->direction = PS2_DEVICE_TO_HOST;
            if (dev->ActivityLedPort) {
                /* Turn activity LED off */
                HAL_GPIO_WritePin(dev->ActivityLedPort, dev->ActivityLedPin, GPIO_PIN_RESET);
            }
        }
        break;
    case PS2_DEVICE_TO_HOST:
        if (HAL_GPIO_ReadPin(dev->clockPort, dev->clockPin) == GPIO_PIN_SET) {
            /* Rising edge of clock */
            dataBit = HAL_GPIO_ReadPin(dev->dataPort, dev->dataPin);
            if (dev->numberOfBitsRemainingToBeRead == 11) {
                if (dev->ActivityLedPort) {
                    /* Turn activity LED on */
                    HAL_GPIO_WritePin(dev->ActivityLedPort, dev->ActivityLedPin, GPIO_PIN_SET);
                }
                dev->status = (dataBit == GPIO_PIN_RESET)?PS2_IN_PROGRESS:PS2_FRAME_ERROR;
                dev->data = 0;
                dev->parity = 1;
            } else if (dev->status == PS2_IN_PROGRESS) {
                if (dev->numberOfBitsRemainingToBeRead>2) {
                    dev->data >>= 1;
                    dev->data |= ((dataBit ==  GPIO_PIN_RESET)?0:1) << 7;
                    if (dataBit == GPIO_PIN_SET) {
                        dev->parity++;
                    }
                } else if (dev->numberOfBitsRemainingToBeRead == 2 && dev->parityEnabled) {
                    if (dataBit == GPIO_PIN_RESET) {
                        dev->status = (dev->parity & 1)?PS2_PARITY_ERROR:PS2_IN_PROGRESS;
                    } else {
                        dev->status = (dev->parity & 1)?PS2_IN_PROGRESS:PS2_PARITY_ERROR;
                    }
                    if (dev->status == PS2_PARITY_ERROR) {
                        /* In case of error, ask device to resend information */
                        PS2_SendByteAsync(dev, 0xFE);
                    }
                } else {
                    dev->status = (dataBit == GPIO_PIN_SET)?PS2_SUCCESS:PS2_FRAME_ERROR;
                }
            }

            dev->numberOfBitsRemainingToBeRead--;
            if (dev->numberOfBitsRemainingToBeRead == 0) {
                dev->numberOfBitsRemainingToBeRead = 11;
                if (dev->status == PS2_SUCCESS || !dev->parityEnabled) {
                    PS2_PutByteInFIFOBuffer(dev, dev->data);
                } else { /* In case of error, ask device to resend information */
                    PS2_SendByteAsync(dev, 0xFE);
                }
                if (dev->ActivityLedPort) {
                    /* Turn activity LED off */
                    HAL_GPIO_WritePin(dev->ActivityLedPort, dev->ActivityLedPin, GPIO_PIN_RESET);
                }
            }
        }

        break;
    case PS2_HOST_TO_DEVICE:
            if (HAL_GPIO_ReadPin(dev->clockPort, dev->clockPin) == GPIO_PIN_RESET) {
                if (dev->data) {
                    /* Send data to device */
                        HAL_GPIO_WritePin(dev->dataPort, dev->dataPin, (dev->data & 1)?GPIO_PIN_SET:GPIO_PIN_RESET);
                        dev->data >>= 1;
                } else {
                    /* Check ACK from device */
                        dev->status = (HAL_GPIO_ReadPin(dev->dataPort, dev->dataPin) == GPIO_PIN_RESET)?PS2_SUCCESS:PS2_NAK;
                        dev->direction = PS2_NONE;
                        dev->numberOfBitsRemainingToBeRead = 11;
                }
            }
        break;
    }
}


void PS2_PutByteInFIFOBuffer(PS2_HandleTypeDef *dev, uint8_t byte)
{
    uint16_t next = (dev->dataBuffer.head+1) & PS2_BUFFER_SIZE_BIT_MASK;
    if (next != dev->dataBuffer.tail) {
        /* The buffer is not full */
        dev->dataBuffer.buffer[dev->dataBuffer.head] = byte;
        dev->dataBuffer.head = next;
    }
}


int16_t PS2_DrawByteFormFIFOBuffer(PS2_HandleTypeDef *dev)
{
    if (dev->dataBuffer.tail != dev->dataBuffer.head) {
        /* The buffer is not empty */
        int16_t res = dev->dataBuffer.buffer[dev->dataBuffer.tail];
        dev->dataBuffer.tail = (dev->dataBuffer.tail+1) & PS2_BUFFER_SIZE_BIT_MASK;
        return res;
    } else {
        return -1;
    }
}


uint8_t PS2_FIFOBuffer_IsEmpty(PS2_HandleTypeDef *dev)
{
    return dev->dataBuffer.tail != dev->dataBuffer.head;
}


void PS2_FIFOBuffer_Flush(PS2_HandleTypeDef *dev)
{
    dev->dataBuffer.tail = dev->dataBuffer.head;
}


int16_t PS2_WaitForAnswer(PS2_HandleTypeDef *dev, uint16_t timemout)
{
    int16_t answer;

    answer = PS2_DrawByteFormFIFOBuffer(dev);
    while (answer == -1 && timemout--) { /* Wait until buffer is not empty */
        HAL_Delay(1);
        answer = PS2_DrawByteFormFIFOBuffer(dev);
    }

    return answer;
}

static void PS2_ParseReadIdAnswer(PS2_HandleTypeDef *dev)
{
    int i;

    uint8_t id1 = PS2_WaitForAnswer(dev, 20);

    if (id1 == 0xab &&
        PS2_WaitForAnswer(dev, 20) == 0x83) {

        dev->connectedDevice = PS2_KEYBOARD;

        /* Map the device type and activity LED for a keyboard device */
        dev->DeviceTypeLedPort = dev->KBLedPort;
        dev->DeviceTypeLedPin = dev->KBLedPin;
        dev->ActivityLedPort = dev->MouseLedPort;
        dev->ActivityLedPin = dev->MouseLedPin;

        /* Turn on the LED corresponding to the type of the detected device */
        HAL_GPIO_WritePin(dev->DeviceTypeLedPort, dev->DeviceTypeLedPin, GPIO_PIN_SET);

        /* Initialize keyboard state */
        dev->state.keyboard.extendedKey = 0;
        dev->state.keyboard.extendedE1Key = 0;
        dev->state.keyboard.breakCode = 0;
        dev->state.keyboard.nbKeyPressed = 0;

        for (i=0; i<sizeof dev->state.keyboard.report; i++) {
            dev->state.keyboard.report[i] = 0;
        }

        for (i=0; i<sizeof dev->state.keyboard.keyPressed; i++) {
            dev->state.keyboard.keyPressed[i] = 0;
        }

        dev->state.keyboard.isReportAvailable = 0;
        dev->tickLastReport = HAL_GetTick();
    } else if (id1 == 0x00 || id1 == 0x03) {
        dev->connectedDevice = PS2_MOUSE;
        /* Disable parity in case of Aten KVM switch */
        dev->parityEnabled = 0;


        /* Map the device type and activity LED for a keyboard device */
        dev->DeviceTypeLedPort = dev->MouseLedPort;
        dev->DeviceTypeLedPin = dev->MouseLedPin;
        dev->ActivityLedPort = dev->KBLedPort;
        dev->ActivityLedPin = dev->KBLedPin;

        /* Turn on the LED corresponding to the type of the detected device */
        HAL_GPIO_WritePin(dev->DeviceTypeLedPort, dev->DeviceTypeLedPin, GPIO_PIN_SET);

        /* Initialize mouse state */
        dev->state.mouse.haveWheel = 0;

        for (i=0; i<sizeof dev->state.mouse.report; i++) {
            dev->state.mouse.report[i] = 0;
        }

        if (id1 == 0x03) {
            /* This is a wheel mouse */
            dev->state.mouse.haveWheel = 1;
        } else {
            /* Try to activate wheel mode */
            PS2_SendByteAsync(dev, 0xF3);
            if (PS2_WaitForAnswer(dev, 20) != 0xfa) {
                return;
            }
            PS2_SendByteAsync(dev, 200);
            if (PS2_WaitForAnswer(dev, 20) != 0xfa) {
                return;
            }

            PS2_SendByteAsync(dev, 0xF3);
            if (PS2_WaitForAnswer(dev, 20) != 0xfa) {
                return;
            }
            PS2_SendByteAsync(dev, 100);
            if (PS2_WaitForAnswer(dev, 20) != 0xfa) {
                return;
            }

            PS2_SendByteAsync(dev, 0xF3);
            if (PS2_WaitForAnswer(dev, 20) != 0xfa) {
                return;
            }
            PS2_SendByteAsync(dev, 80);
            if (PS2_WaitForAnswer(dev, 20) != 0xfa) {
                return;
            }

            /* Reread device type */
            PS2_SendByteAsync(dev, 0xF2);
            if (PS2_WaitForAnswer(dev, 20) != 0xfa) {
                return;
            }
            id1 = PS2_WaitForAnswer(dev, 20);
            dev->state.mouse.haveWheel = (id1 == 0x03)?1:0;
        }


//        /* Try to activate intellimouse */
//        PS2_SendByteAsync(dev, 0xF3);
//        PS2_SendByteAsync(dev, 200);
//        if (PS2_WaitForAnswer(dev, 20) != 0xfa) {
//            return;
//        }
//        PS2_SendByteAsync(dev, 0xF3);
//        PS2_SendByteAsync(dev, 200);
//        if (PS2_WaitForAnswer(dev, 20) != 0xfa) {
//            return;
//        }
//        PS2_SendByteAsync(dev, 0xF3);
//        PS2_SendByteAsync(dev, 80);
//        if (PS2_WaitForAnswer(dev, 20) != 0xfa) {
//            return;
//        }
//        /* Reread device type */
//        PS2_SendByteAsync(dev, 0xF2);
//        if (PS2_WaitForAnswer(dev, 20) != 0xfa) {
//            return;
//        }
//        id1 = PS2_WaitForAnswer(dev, 20);

        /* Enable data reporting */
        PS2_SendByteAsync(dev, 0xF4);
        id1 = PS2_WaitForAnswer(dev, 20);
        dev->tickLastReport = HAL_GetTick();

    } else {
        /* No known device connected */
        dev->connectedDevice = PS2_NO_DEVICE;
    }
}


static void PS2_ReadId(PS2_HandleTypeDef *dev) {
    int nbTries = 3;
    /* Send read ID command */
    do {
        PS2_SendByteAsync(dev, 0xF2);
    } while (--nbTries && PS2_WaitForAnswer(dev, 20) != 0xfa);
}


/**
  * @brief  Check if a device is connected on a PS/2 port.
  *         If true, try to determine its type: keyboard or mouse
  * @param  dev: the PS/2 port handle
  * @retval none
  */
static void PS2_ProbeDevice(PS2_HandleTypeDef *dev) {
    /* Send reset command */
    PS2_ReadId(dev);
    PS2_ParseReadIdAnswer(dev);
}

