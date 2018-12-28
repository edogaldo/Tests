/*
  HardwareSerial.cpp - Hardware serial library for Wiring
  Copyright (c) 2006 Nicholas Zambetti.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

  Modified 23 November 2006 by David A. Mellis
  Modified 28 September 2010 by Mark Sproul
  Modified 14 August 2012 by Alarus
  Modified 3 December 2013 by Matthijs Kooijman
*/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <assert.h>
#include "Arduino.h"
#include "USBSerial.h"

#include <usbd_core.h>
#include "usbd_desc.h"
#include "usbd_cdc_interface.h"




// #if defined(HAVE_USBSERIAL)
  // USBSerial SerialUSB();
  // void serialEventUSB() __attribute__((weak));
// #endif

// void serialEventRun(void)
// {
// #if defined(HAVE_USBSERIAL)
  // if (serialEventUSB && SerialUSB.available()) serialEventUSB();
// #endif
// }

// Constructors ////////////////////////////////////////////////////////////////
USBSerial::USBSerial()
{
  // init();
}

// void USBSerial::init(void)
// {
  // _rx_head = 0;
  // _rx_tail = 0;
// }

/*
// Actual interrupt handlers //////////////////////////////////////////////////////////////

void USBSerial::_rx_complete_irq(serial_t* obj)
{
  // No Parity error, read byte and store it in the buffer if there is room
  unsigned char c;

  if (uart_getc(obj, &c) == 0) {

    rx_buffer_index_t i = (unsigned int)(obj->rx_head + 1) % SERIAL_RX_BUFFER_SIZE;

    // if we should be storing the received character into the location
    // just before the tail (meaning that the head would advance to the
    // current location of the tail), we're about to overflow the buffer
    // and so we don't write the character or advance the head.
    if (i != obj->rx_tail) {
      obj->rx_buff[obj->rx_head] = c;
      obj->rx_head = i;
    }
  }
}

// Actual interrupt handlers //////////////////////////////////////////////////////////////

int HardwareSerial::_tx_complete_irq(serial_t* obj)
{
  // If interrupts are enabled, there must be more data in the output
  // buffer. Send the next byte
  obj->tx_tail = (obj->tx_tail + 1) % SERIAL_TX_BUFFER_SIZE;

  if (obj->tx_head == obj->tx_tail) {
    return -1;
  }

  return 0;
}
*/

// Public Methods //////////////////////////////////////////////////////////////

void USBSerial::begin(unsigned long baud, byte config)
{
  pinMode(PA12, OUTPUT);
  digitalWrite(PA12,LOW);
  USBD_LL_Delay(1);
  pinMode(PA12, INPUT);

  /* Init Device Library */
  USBD_Init(&USBD_Device, &VCP_Desc, 0);

  /* Add Supported Class */
  USBD_RegisterClass(&USBD_Device, USBD_CDC_CLASS);

  /* Add CDC Interface Class */
  USBD_CDC_RegisterInterface(&USBD_Device, &USBD_CDC_fops);

  /* Start Device Process */
  USBD_Start(&USBD_Device);
  
  /* Set Application Buffers */
  USBD_CDC_SetRxBuffer(&USBD_Device, _rx_buff);

  USBD_LL_Delay(1000);
  
}

void USBSerial::end()
{
  PCD_HandleTypeDef *hpcd = (PCD_HandleTypeDef *)USBD_Device.pData;
  
  // wait for transmission of outgoing data
  flush();
  
  if (hpcd != NULL)
  {
    /* Stop Device Process */
    HAL_PCD_DevDisconnect(hpcd);
    USBD_Stop(&USBD_Device);
    USBD_DeInit(&USBD_Device);
  }
  
  // clear any received data
  _rx_head = _rx_tail;
}

int USBSerial::available(void)
{
  return ((unsigned int)(SERIAL_RX_BUFFER_SIZE + _rx_head - _rx_tail)) % SERIAL_RX_BUFFER_SIZE;
}

int USBSerial::peek(void)
{
  if (_rx_head == _rx_tail) {
    return -1;
  } else {
    return _rx_buff[_rx_tail];
  }
}

int USBSerial::read(void)
{
  // if the head isn't ahead of the tail, we don't have any characters
  if (_rx_head == _rx_tail) {
    return -1;
  } else {
    unsigned char c = _rx_buff[_rx_tail];
    _rx_tail = (rx_buffer_index_t)(_rx_tail + 1) % SERIAL_RX_BUFFER_SIZE;
    return c;
  }
}

int USBSerial::availableForWrite(void)
{
  /*
  tx_buffer_index_t head = _tx_head;
  tx_buffer_index_t tail = _tx_tail;

  if (head >= tail) return SERIAL_TX_BUFFER_SIZE - 1 - head + tail;
  return tail - head - 1;
  */
  return 64;
}

void USBSerial::flush()
{
  // If we have never written a byte, no need to flush. This special
  // case is needed since there is no way to force the TXC (transmit
  // complete) bit to 1 during initialization
  if (!_written)
    return;

  uint8_t tResult = USBD_OK;
  do {
    tResult = USBD_CDC_TransmitPacket(&USBD_Device);
  } while (tResult == USBD_BUSY);
}

size_t USBSerial::write(uint8_t c)
{
  return write(&c, 1);
}

size_t USBSerial::write(const uint8_t *buf, size_t len)
{
  if (!(bool) *this || !buf) {
    return 0;
  }

  _written = true;

  USBD_CDC_SetTxBuffer(&USBD_Device, (uint8_t *)buf, len);

  uint8_t tResult = USBD_OK;
  do {
    tResult = USBD_CDC_TransmitPacket(&USBD_Device);
  } while (tResult == USBD_BUSY);
  
  return (tResult == USBD_OK ? len : 0);
}

