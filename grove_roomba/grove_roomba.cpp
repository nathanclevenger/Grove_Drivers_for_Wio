/*
 * grove_roomba.cpp
 *
 * Copyright (c) 2016 Nathan Clevenger
 * Website    : www.nathanclevenger.com
 * Author     : Nathan Clevenger
 *
 * The MIT License (MIT)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */


#include "suli2.h"
#include "grove_roomba.h"

#define BOUND_CONST(val,min,max) (val<min?min:(val>max?max:val))

GroveRoomba::GroveRoomba(int pintx, int pinrx)
{
    this->uart = (UART_T *)malloc(sizeof(UART_T));

    suli_uart_init(uart, pintx, pinrx, 19200);

    _select_storage();

    mode = MODE_OFF;
}

bool GroveRoomba::write_mode(int mode) {
  bool ret;
  switch (mode) {
    case MODE_OFF:
      ret = serial->sendOpcode(OC_POWER);
      break;
    case MODE_PASSIVE:
      ret = serial->sendOpcode(OC_START);
      break;
    case MODE_SAFE:
      ret = serial->sendOpcode(OC_SAFE);
      break;
    case MODE_FULL:
      ret = serial->sendOpcode(OC_FULL);
      break;
    default:
      //CERR("[create::Create] ", "cannot set robot to mode '" << mode << "'");
      ret = false;
  }
  if (ret) {
    this->mode = mode;
  }
  return ret;
}

bool GroveRoomba::write_dock() const {
  return sendOpcode(OC_DOCK);
}


bool GroveRoomba::write_drive_radius(float vel, float radius) {
  // Bound velocity
  float boundedVel = BOUND_CONST(vel, -0.5, 0.5);

  // Expects each parameter as two bytes each and in millimeters
  int16_t vel_mm = (int16_t)(boundedVel * 1000);
  int16_t radius_mm = (int16_t)(radius * 1000);

  // Bound radius if not a special case
  if (radius_mm != 32768 && radius_mm != 32767 &&
      radius_mm != -1 && radius_mm != 1) {
    BOUND(radius_mm, -MAX_RADIUS * 1000, MAX_RADIUS * 1000);
  }

  uint8_t cmd[5] = { OC_DRIVE,
                     vel_mm >> 8,
                     vel_mm & 0xff,
                     radius_mm >> 8,
                     radius_mm & 0xff
                   };


  suli_uart_write_bytes(uart, cmd, 5);

  return true
}


void GroveRoomba::_select_storage()
{
    uint8_t buff[8] = { 0x7e, 0xff, 0x06, 0x09, 0x00, 0x00, 0x02, 0xef };

    suli_uart_write_bytes(uart, buff, 8);
}

void GroveRoomba::_drain_uart()
{
    while (suli_uart_readable(uart))
    {
        suli_uart_read(uart);
    }
}

bool GroveRoomba::sendOpcode(const Opcode& code) {
  uint8_t buff[1] = { (uint8_t)code };
  suli_uart_write_bytes(uart, buff, 1);
  return true;
}
