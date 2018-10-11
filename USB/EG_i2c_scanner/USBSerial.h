/*
  USBSerial.h - USB serial library for Wiring
  Copyright (c) 2018 Edoardo Galdi.  All right reserved.

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

  Created 02 October 2018 bt Edoardo Galdi
  Modified dd month yyyy by Firstname Lastname
  Modified 14 August 2012 by Alarus
  Modified 3 December 2013 by Matthijs Kooijman
*/

#ifndef USBSerial_h
#define USBSerial_h

#include <inttypes.h>
#include <usbd_def.h>

#include "Stream.h"

#if  (SERIAL_RX_BUFFER_SIZE>256)
typedef uint16_t rx_buffer_index_t;
#else
typedef uint8_t rx_buffer_index_t;
#endif

class USBSerial : public Stream
{
  protected:
    // Has any byte been written to the UART since begin()
    bool _written;
    unsigned char _rx_buff[SERIAL_RX_BUFFER_SIZE];
    rx_buffer_index_t _rx_head;
    rx_buffer_index_t _rx_tail;

  public:
    USBSerial();
    void begin(unsigned long baud) { begin(baud, SERIAL_8N1); }
    void begin(unsigned long, uint8_t);
    void end();
    virtual int available(void);
    virtual int peek(void);
    virtual int read(void);
    int availableForWrite(void);
    virtual void flush(void);
    size_t write(uint8_t);
    size_t write(const uint8_t *, size_t);
    inline size_t write(unsigned long n) { return write((uint8_t)n); }
    inline size_t write(long n) { return write((uint8_t)n); }
    inline size_t write(unsigned int n) { return write((uint8_t)n); }
    inline size_t write(int n) { return write((uint8_t)n); }
    using Print::write; // pull in write(str) and write(buf, size) from Print
    operator bool() { return true; }

    friend class STM32LowPower;

    // Interrupt handlers
    //static void _rx_complete_irq(serial_t* obj);
    //static int _tx_complete_irq(serial_t* obj);
  private:
	USBD_HandleTypeDef  USBD_Device;
    //uint8_t _config;
    //void init(void);
    //void configForLowPower(void);
};

extern USBSerial SerialUSB;

extern void serialEventRun(void) __attribute__((weak));

#endif
