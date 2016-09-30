/*
 * grove_roomba.h
 *
 * Copyright (c) Nathan Clevenger
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


#ifndef __GROVE_ROOMBA_H__
#define __GROVE_ROOMBA_H__

#include "suli2.h"

//GROVE_NAME        "Grove - Roomba"
//SKU               iRobot Create 2
//IF_TYPE           UART
//IMAGE_URL         http://www.irobotweb.com/~/media/MainSite/Images/About/STEM/Create/create-overview.png
//DESCRIPTION       "iRobot Create is a hobbyist robot manufactured by iRobot that was introduced in 2007 and based on their Roomba vacuum cleaning platform. The iRobot Create is explicitly designed for robotics development and improves the experience beyond simply hacking the Roomba."
//WIKI_URL          https://en.wikipedia.org/wiki/IRobot_Create
//ADDED_AT          "2016-9-29"
//AUTHOR            "NATHAN CLEVENGER"

class GroveRoomba
{
public:

    GroveRoomba(int pintx, int pinrx);

    /** Change Create mode.
     * @param mode to put Create in.
     * @return bool - true if successful, false otherwise
     */
    bool write_mode(int mode);


    /** Starts the docking behaviour.
     * Changes mode to MODE_PASSIVE.
     * @return bool - true if successful, false otherwise
     */
    bool write_dock() const;


    /** Set the average wheel velocity and turning radius of Create.
     * @param velocity is in m/s bounded between [-0.5, 0.5]
     * @param radius in meters.
     *        Special cases: drive straight = CREATE_2_STRAIGHT_RADIUS,
     *                       turn in place counter-clockwise = CREATE_2_IN_PLACE_RADIUS,
     *                       turn in place clockwise = -CREATE_2_IN_PLACE_RADIUS
     * @return bool - true if successful, false otherwise
     */
    bool write_drive_radius(float velocity, float radius);


    enum SensorPacketID {
      ID_GROUP_0 = 0,
      ID_GROUP_1 = 1,
      ID_GROUP_2 = 2,
      ID_GROUP_3 = 3,
      ID_GROUP_4 = 4,
      ID_GROUP_5 = 5,
      ID_GROUP_6 = 6,
      ID_GROUP_100 = 100,
      ID_GROUP_101 = 101,
      ID_GROUP_106 = 106,
      ID_GROUP_107 = 107,
      ID_BUMP_WHEELDROP = 7,
      ID_WALL = 8,
      ID_CLIFF_LEFT = 9,
      ID_CLIFF_FRONT_LEFT = 10,
      ID_CLIFF_FRONT_RIGHT = 11,
      ID_CLIFF_RIGHT = 12,
      ID_VIRTUAL_WALL = 13,
      ID_OVERCURRENTS = 14,
      ID_DIRT_DETECT_LEFT = 15,
      ID_DIRT_DETECT_RIGHT = 16,
      ID_IR_OMNI = 17,
      ID_IR_LEFT = 52,
      ID_IR_RIGHT = 53,
      ID_BUTTONS = 18,
      ID_DISTANCE = 19,
      ID_ANGLE = 20,
      ID_CHARGE_STATE = 21,
      ID_VOLTAGE = 22,
      ID_CURRENT = 23,
      ID_TEMP = 24,
      ID_CHARGE = 25,
      ID_CAPACITY = 26,
      ID_WALL_SIGNAL = 27,
      ID_CLIFF_LEFT_SIGNAL = 28,
      ID_CLIFF_FRONT_LEFT_SIGNAL = 29,
      ID_CLIFF_FRONT_RIGHT_SIGNAL = 30,
      ID_CLIFF_RIGHT_SIGNAL = 31,
      ID_CARGO_BAY_DIGITAL_INPUTS = 32,
      ID_CARGO_BAY_ANALOG_SIGNAL = 33,
      ID_CHARGE_SOURCE = 34,
      ID_OI_MODE = 35,
      ID_SONG_NUM = 36,
      ID_PLAYING = 37,
      ID_NUM_STREAM_PACKETS = 38,
      ID_VEL = 39,
      ID_RADIUS = 40,
      ID_RIGHT_VEL = 41,
      ID_LEFT_VEL = 42,
      ID_LEFT_ENC = 43,
      ID_RIGHT_ENC = 44,
      ID_LIGHT = 45,
      ID_LIGHT_LEFT = 46,
      ID_LIGHT_FRONT_LEFT = 47,
      ID_LIGHT_CENTER_LEFT = 48,
      ID_LIGHT_CENTER_RIGHT = 49,
      ID_LIGHT_FRONT_RIGHT = 50,
      ID_LIGHT_RIGHT = 51,
      ID_LEFT_MOTOR_CURRENT = 54,
      ID_RIGHT_MOTOR_CURRENT = 55,
      ID_MAIN_BRUSH_CURRENT = 56,
      ID_SIDE_BRUSH_CURRENT = 57,
      ID_STASIS = 58,
      ID_NUM = 52
    };

    enum Opcode {
      OC_START = 128,
      OC_RESET = 7,
      OC_STOP = 173,
      OC_BAUD = 129,
      OC_CONTROL = 130,
      OC_SAFE = 131,
      OC_FULL = 132,
      OC_CLEAN = 135,
      OC_MAX = 136,
      OC_SPOT = 134,
      OC_DOCK = 143,
      OC_POWER = 133,
      OC_SCHEDULE = 167,
      OC_DATE = 168,
      OC_DRIVE = 137,
      OC_DRIVE_DIRECT = 145,
      OC_DRIVE_PWM = 146,
      OC_MOTORS = 138,
      OC_MOTORS_PWM = 144,
      OC_LEDS = 139,
      OC_SCHEDULING_LEDS = 162,
      OC_DIGIT_LEDS_RAW = 163,
      OC_BUTTONS = 165,
      OC_DIGIT_LEDS_ASCII = 164,
      OC_SONG = 140,
      OC_PLAY = 141,
      OC_SENSORS= 142,
      OC_QUERY_LIST=149,
      OC_STREAM = 148,
      OC_TOGGLE_STREAM = 150,
    };

    enum RoombaMode {
      MODE_OFF = 0,
      MODE_PASSIVE = 1,
      MODE_SAFE = 2,
      MODE_FULL = 3,
      MODE_UNAVAILABLE = -1
    };

    enum CleanMode {
      CLEAN_DEFAULT = OC_CLEAN,
      CLEAN_MAX = OC_MAX,
      CLEAN_SPOT = OC_SPOT
    };

    enum ChargingState {
      CHARGE_NONE = 0,
      CHARGE_RECONDITION = 1,
      CHARGE_FULL = 2,
      CHARGE_TRICKLE = 3,
      CHARGE_WAITING = 4,
      CHARGE_FAULT = 5
    };

    enum DayOfWeek {
      SUN = 0,
      MON = 1,
      TUE = 2,
      WED = 3,
      THU = 4,
      FRI = 5,
      SAT = 6
    };

    enum IRChars {
      IR_CHAR_NONE = 0,
      IR_CHAR_LEFT = 129,
      IR_CHAR_FORWARD = 130,
      IR_CHAR_RIGHT = 131,
      IR_CHAR_SPOT = 132,
      IR_CHAR_MAX = 133,
      IR_CHAR_SMALL = 134,
      IR_CHAR_MEDIUM = 135,
      IR_CHAR_LARGE = 136,
      IR_CHAR_CLEAN = 136,
      IR_CHAR_PAUSE = 137,
      IR_CHAR_POWER = 138,
      IR_CHAR_ARC_LEFT = 139,
      IR_CHAR_ARC_RIGHT = 140,
      IR_CHAR_STOP = 141,
      IR_CHAR_DOWNLOAD = 142,
      IR_CHAR_SEEK_DOCK = 143,
      IR_CHAR_DOCK_RESERVED = 240,
      IR_CHAR_RED_BUOY = 248,
      IR_CHAR_GREEN_BUOY = 244,
      IR_CHAR_FORCE_FIELD = 242,
      IR_CHAR_RED_GREEN_BUOY = 252,
      IR_CHAR_RED_FORCE_FIELD = 250,
      IR_CHAR_GREEN_FORCE_FIELD = 246,
      IR_CHAR_RED_GREEN_FORCE_FIELD = 254,
      IR_CHAR_600_DOCK_RESERVED = 160,
      IR_CHAR_600_FORCE_FIELD = 161,
      IR_CHAR_600_GREEN_BUOY = 164,
      IR_CHAR_600_GREEN_FORCE_FIELD = 165,
      IR_CHAR_600_RED_BUOY = 168,
      IR_CHAR_600_RED_FORCE_FIELD = 169,
      IR_CHAR_600_RED_GREEN_BUOY = 172,
      IR_CHAR_600_RED_GREEN_FORCE_FIELD = 173,
      IR_CHAR_VIRTUAL_WALL = 162
    };


private:

    UART_T *uart;

    void _select_storage();

    void _drain_uart();

    bool sendOpcode(const Opcode& code);

    RoombaMode mode;

    static const float MAX_RADIUS = 2.0;
    static const float STRAIGHT_RADIUS = 32.768;
    static const float IN_PLACE_RADIUS = 0.001;
    static const float PI = 3.14159;
    static const float TWO_PI = 6.28318;
    static const float EPS = 0.0001;

    inline float normalizeAngle(const float& angle) {
      float a = angle;
      while (a < -PI) a += TWO_PI;
      while (a > PI) a -= TWO_PI;
      return a;
    };

};

#endif
