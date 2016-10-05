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
#define BOUND(val,min,max) (val = BOUND_CONST(val,min,max))

GroveRoomba::GroveRoomba(int pintx, int pinrx)
{
    this->uart = (UART_T *)malloc(sizeof(UART_T));
    this->timer = (TIMER_T *)malloc(sizeof(TIMER_T));

    last_error="no error";

    suli_uart_init(uart, pintx, pinrx, 19200);

    sendOpcode(OC_START);
}

bool GroveRoomba::write_mode(int mode) {
  bool ret;
  switch (mode) {
    case MODE_OFF:
      ret = sendOpcode(OC_POWER);
      break;
    case MODE_PASSIVE:
      ret = sendOpcode(OC_START);
      break;
    case MODE_SAFE:
      ret = sendOpcode(OC_SAFE);
      break;
    case MODE_FULL:
      ret = sendOpcode(OC_FULL);
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

bool GroveRoomba::write_mode_off() {
  return write_mode(MODE_OFF);
}

bool GroveRoomba::write_mode_passive() {
  return write_mode(MODE_PASSIVE);
}

bool GroveRoomba::write_mode_safe() {
  return write_mode(MODE_SAFE);
}

bool GroveRoomba::write_mode_full() {
  return write_mode(MODE_FULL);
}

bool GroveRoomba::write_dock() {
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

  return true;
}

bool GroveRoomba::write_drive_wheels(float leftVel, float rightVel) {
  const float boundedLeftVel = BOUND_CONST(leftVel, -0.5, 0.5);
  const float boundedRightVel = BOUND_CONST(rightVel, -0.5, 0.5);
  int16_t leftCmd = roundf(boundedLeftVel * 1000);
  int16_t rightCmd = roundf(boundedRightVel * 1000);

  uint8_t cmd[5] = { OC_DRIVE_DIRECT,
                     rightCmd >> 8,
                     rightCmd & 0xff,
                     leftCmd >> 8,
                     leftCmd & 0xff
                   };
   suli_uart_write_bytes(uart, cmd, 5);

   return true;
}

bool GroveRoomba::write_drive_straight(float vel) {
  return write_drive_wheels(vel, vel);
}

bool GroveRoomba::write_drive_straight_duration(float vel, float duration) {
  suli_timer_install(timer, duration * 1000000, grove_roomba_timer_interrupt_handler, this, false);
  return write_drive_straight(vel);
}

bool GroveRoomba::write_drive_straight_distance(float vel, float distance) {
  return write_drive_straight_duration(vel, distance / vel);
}

bool GroveRoomba::write_turn_in_place_counter_clockwise(float vel) {
  return write_drive_wheels(-vel, vel);
}

bool GroveRoomba::write_turn_in_place_clockwise(float vel) {
  return write_drive_wheels(vel, -vel);
}

bool GroveRoomba::write_turn_in_place_counter_clockwise_duration(float vel, float duration) {
  suli_timer_install(timer, duration * 1000000, grove_roomba_timer_interrupt_handler, this, false);
  return write_turn_in_place_counter_clockwise(vel);
}

bool GroveRoomba::write_turn_in_place_clockwise_duration(float vel, float duration) {
  suli_timer_install(timer, duration * 1000000, grove_roomba_timer_interrupt_handler, this, false);
  return write_turn_in_place_clockwise(vel);
}

bool GroveRoomba::write_turn_in_place_counter_clockwise_degrees(float vel, float degrees) {
  return write_turn_in_place_counter_clockwise_duration(vel, (degrees / 360) * (PI * AXLE_LENGTH) / vel);
}

bool GroveRoomba::write_turn_in_place_clockwise_degrees(float vel, float degrees) {
  return write_turn_in_place_clockwise_duration(vel, (degrees / 360) * (PI * AXLE_LENGTH) / vel);
}

bool GroveRoomba::write_turn_radius_degrees(float vel, float radius, float degrees) {
  float duration = (degrees / 360) * (TWO_PI * radius) / vel;
  suli_timer_install(timer, duration * 1000000, grove_roomba_timer_interrupt_handler, this, false);
  return write_drive_radius(vel, radius);
}

bool GroveRoomba::read_sensor(uint16_t *value, uint8_t sensor) {
  uint8_t cmd[2] = { OC_SENSORS, sensor };
  _drain_uart();
  suli_uart_write_bytes(uart, cmd, 2);
  suli_delay_ms(1);

  int dataSize = 1;

  if (sensor == ID_VOLTAGE ||
      sensor == ID_CURRENT ||
      sensor == ID_CHARGE ||
      sensor == ID_CAPACITY ||
      sensor == ID_LEFT_ENC ||
      sensor == ID_RIGHT_ENC ||
      sensor == ID_LIGHT_LEFT ||
      sensor == ID_LIGHT_FRONT_LEFT ||
      sensor == ID_LIGHT_CENTER_LEFT ||
      sensor == ID_LIGHT_CENTER_RIGHT ||
      sensor == ID_LIGHT_FRONT_RIGHT ||
      sensor == ID_LIGHT_RIGHT) {
    dataSize = 2;
  }

  int expectedBytes = 4 + dataSize;

  byte data[expectedBytes];
  int n = suli_uart_read_bytes_timeout(uart,data,expectedBytes,100);

  if (n != expectedBytes)
  {
    last_error="too few bytes read";
    return false;
  }

  // checksum :
  int checksum = 0;
  for (size_t i = 0; i < expectedBytes; i++) {
    checksum = checksum + (byte)data[i];
  }

  if (1 + (0xFF ^ (byte)checksum) != data[expectedBytes - 1])
  {
    last_error="checksum error";
    return false;
  }

  switch (dataSize) {
    case 1:
      value = (int)data[4];
      return true;
      break;
    case 2:
      value = (int)data[4] * 256 + (int)data[5];
      return true;
      break;
    default:
      last_error="invalid packet length";
      return false;
  }
}

bool GroveRoomba::read_distance(float *value) {
  uint8_t cmd[2] = { OC_SENSORS, ID_DISTANCE };
  _drain_uart();
  suli_uart_write_bytes(uart, cmd, 2);
  suli_delay_ms(5);
  byte data[6];
  int n = suli_uart_read_bytes_timeout(uart,data,6,100);
  int mm = (data[4] << 8) | data[5];
  value = mm / 1000;
  return true;
}

bool GroveRoomba::read_angle(float *value) {
  uint8_t cmd[2] = { OC_SENSORS, ID_ANGLE };
  _drain_uart();
  suli_uart_write_bytes(uart, cmd, 2);
  suli_delay_ms(5);
  byte data[6];
  int n = suli_uart_read_bytes_timeout(uart,data,6,100);
  int rad = (data[4] << 8) | data[5];
  float degrees = (float)(rad / 0.324056);
  value = &degrees;
  return true;
}

bool GroveRoomba::read_left_encoder_count(int16_t *value) {
  uint8_t cmd[2] = { OC_SENSORS, ID_LEFT_ENC };
  _drain_uart();
  suli_uart_write_bytes(uart, cmd, 2);
  suli_delay_ms(5);
  byte data[6];
  int n = suli_uart_read_bytes_timeout(uart,data,6,100);
  value = (data[4] << 8) | data[5];
  return true;
}

bool GroveRoomba::read_right_encoder_count(int16_t *value) {
  uint8_t cmd[2] = { OC_SENSORS, ID_RIGHT_ENC };
  _drain_uart();
  suli_uart_write_bytes(uart, cmd, 2);
  suli_delay_ms(5);
  byte data[6];
  int n = suli_uart_read_bytes_timeout(uart,data,6,100);
  value = (data[4] << 8) | data[5];
  return true;
}

bool GroveRoomba::sendOpcode(const Opcode& code) {
  uint8_t cmd[1] = { code };
  suli_uart_write_bytes(uart, cmd, 1);
  return true;
}

const char* GroveRoomba::get_last_error(void)
{
   return last_error;
}

void GroveRoomba::_drain_uart()
{
    while (suli_uart_readable(uart))
    {
        suli_uart_read(uart);
    }
}

static void grove_roomba_timer_interrupt_handler(void *para)
{
  GroveRoomba *g = (GroveRoomba *)para;
  g->write_drive_straight(0);
}
