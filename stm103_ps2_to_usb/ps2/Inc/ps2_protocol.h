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

#ifndef PS2_PROTOCOL_H_INCLUDED
#define PS2_PROTOCOL_H_INCLUDED

typedef enum {
    PS2_NONE,
    PS2_DEVICE_TO_HOST,
    PS2_HOST_TO_DEVICE
} PS2_DirectionTypedef;

typedef enum {
    PS2_SUCCESS,
    PS2_IN_PROGRESS,
    PS2_FRAME_ERROR,
    PS2_PARITY_ERROR,
    PS2_NAK,
    PS2_NO_ANSWER
} PS2_ResultCodeTypedef;

typedef enum {
    PS2_NO_DEVICE,
    PS2_KEYBOARD,
    PS2_MOUSE
} PS2_ConnectedDeviceTypedef;

#define PS2_BUFFER_SIZE_BIT_LENGTH 4
#define PS2_BUFFER_SIZE (1<<PS2_BUFFER_SIZE_BIT_LENGTH)
#define PS2_BUFFER_SIZE_BIT_MASK (PS2_BUFFER_SIZE-1)

typedef struct {
    uint8_t buffer[PS2_BUFFER_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
} CircularBufferTypedef;

typedef struct {
    uint16_t extendedKey;
    uint16_t extendedE1Key;
    uint16_t breakCode;

    /* USB HID report mapping */
    uint8_t report[8];
    uint8_t nbKeyPressed;

    /* Bitmap recording key press */
    uint8_t keyPressed[256/8];

    /* LED report */
    uint8_t LEDReport;
    uint8_t isReportAvailable;
} PS2_KeyboardStateTypeDef;

typedef struct {
    uint8_t haveWheel;

    /* USB HID report mapping */
    uint8_t report[4];
} PS2_MouseStateTypeDef;

typedef union {
    PS2_KeyboardStateTypeDef keyboard;
    PS2_MouseStateTypeDef mouse;
} PS2_DeviceStateTypedef;

/**
  * @brief  PS2 Device Handle Structure definition
  */
typedef struct {
    /* PS/2 Device information */
    PS2_ConnectedDeviceTypedef connectedDevice;
    PS2_DeviceStateTypedef state;
    uint32_t tickLastReport;

    void *pUserData;

    /* PS/2 protocol */
    volatile PS2_DirectionTypedef direction;
    uint16_t data;
    uint16_t numberOfBitsRemainingToBeRead;
    volatile PS2_ResultCodeTypedef status;
    int parity;
    uint16_t malformed;
    uint8_t parityEnabled;

    /* PS/2 device answers */
    CircularBufferTypedef dataBuffer;

    /* Pins associated to the port */
    GPIO_TypeDef *clockPort;
    uint16_t clockPin;
    GPIO_TypeDef *dataPort;
    uint16_t dataPin;

    /* Pins associated to the LEDs */
    GPIO_TypeDef *KBLedPort;
    uint16_t KBLedPin;
    GPIO_TypeDef *MouseLedPort;
    uint16_t MouseLedPin;

    /* LEDs pins configured according the device type */
    GPIO_TypeDef *DeviceTypeLedPort;
    uint16_t DeviceTypeLedPin;
    GPIO_TypeDef *ActivityLedPort;
    uint16_t ActivityLedPin;
} PS2_HandleTypeDef;


/**
  * @brief  Mapping of USB interfaces to PS2 Device Handle
  */
PS2_HandleTypeDef *PS2_InterfaceMap[2];


/**
  * @brief  Initiate sending a byte to a PS2 Device
  */

void PS2_SendByteAsync(PS2_HandleTypeDef *dev, uint8_t byte);

/**
  * @brief  Send a byte to a PS2 Device and wait for device acknowledgment
  */

PS2_ResultCodeTypedef PS2_SendByte(PS2_HandleTypeDef *dev, uint8_t byte);

/**
  * @brief  Wait for device answer and return first byte of it.
  * If the device does not answer within "timeout" milliseconds, the value -1
  * is returned by this function
  */

int16_t PS2_WaitForAnswer(PS2_HandleTypeDef *dev, uint16_t timemout);

/**
  * @brief  Initialize PS/2 Handle
  */

void PS2_initHandle(PS2_HandleTypeDef *dev);

/**
  * @brief  Process a data bit from/to a PS/2 port.
  * @param  dev: the PS/2 port handle
  * @retval None
  */
void PS2_ProcessData(PS2_HandleTypeDef *dev);

/**
  * @brief  Add a byte in PS/2 device circular buffer
  * @param  dev: the PS/2 port handle
  * @param  byte: the value of the byte to insert in PS/2 circular buffer
  * @retval None
  */
void PS2_PutByteInFIFOBuffer(PS2_HandleTypeDef *dev, uint8_t byte);

/**
  * @brief  Extract a byte from PS/2 device circular buffer
  * @param  dev: the PS/2 port handle
  * @retval the value of the byte to drawn from PS/2 circular buffer
  */
int16_t PS2_DrawByteFormFIFOBuffer(PS2_HandleTypeDef *dev);

/**
  * @brief  Flush PS/2 device circular buffer is empty
  * @param  dev: the PS/2 port handle
  * @retval none
  */
void PS2_FIFOBuffer_Flush(PS2_HandleTypeDef *dev);

/**
  * @brief  Tell if  PS/2 device circular buffer is empty
  * @param  dev: the PS/2 port handle
  * @retval 1 if PS/2 circular buffer is empty, else return 0
  */
uint8_t PS2_FIFOBuffer_IsEmpty(PS2_HandleTypeDef *dev);

#endif /* PS2_PROTOCOL_H_INCLUDED */
