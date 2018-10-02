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
  init();
}

void USBSerial::init(void)
{
  _rx_head = 0;
  _rx_tail = 0;
  _tx_head = 0;
  _tx_tail = 0;
}

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
  /* Init Device Library */
  USBD_Init(&USBD_Device, &VCP_Desc, 0);

  /* Add Supported Class */
  USBD_RegisterClass(&USBD_Device, USBD_CDC_CLASS);

  /* Add CDC Interface Class */
  USBD_CDC_RegisterInterface(&USBD_Device, &USBD_CDC_fops);

  /* Start Device Process */
  USBD_Start(&USBD_Device);
  
  /* Set Application Buffers */
  USBD_CDC_SetTxBuffer(&USBD_Device, _tx_buff, 0);
  USBD_CDC_SetRxBuffer(&USBD_Device, _rx_buff);
  
}

void USBSerial::end()
{
  // wait for transmission of outgoing data
  flush();

  /* Stop Device Process */
  USBD_Stop(&USBD_Device);
  
  USBD_DeInit(&USBD_Device);

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
  tx_buffer_index_t head = _tx_head;
  tx_buffer_index_t tail = _tx_tail;

  if (head >= tail) return SERIAL_TX_BUFFER_SIZE - 1 - head + tail;
  return tail - head - 1;
}

void USBSerial::flush()
{
  // If we have never written a byte, no need to flush. This special
  // case is needed since there is no way to force the TXC (transmit
  // complete) bit to 1 during initialization
  if (!_written)
    return;

  while((_tx_head != _tx_tail)) {
    // nop, the interrupt handler will free up space for us
  }
  // If we get here, nothing is queued anymore (DRIE is disabled) and
  // the hardware finished tranmission (TXC is set).
}

size_t USBSerial::write(uint8_t c)
{
  _written = true;

  tx_buffer_index_t i = (_tx_head + 1) % SERIAL_TX_BUFFER_SIZE;

  // If the output buffer is full, there's nothing for it other than to
  // wait for the interrupt handler to empty it a bit
  while (i == _tx_tail) {
    // nop, the interrupt handler will free up space for us
  }

  _tx_buff[_tx_head] = c;
  _tx_head = i;
  
  USBD_CDC_SetTxBuffer(&USBD_Device, (uint8_t *) &_tx_buff[_tx_tail], 1);
  
  if (USBD_CDC_TransmitPacket(&USBD_Device) == USBD_OK) {
    _tx_tail = (_tx_tail + 1) % SERIAL_TX_BUFFER_SIZE;
  }

  return 1;
}
