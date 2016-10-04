/*
 * grove_comp_hmc5883l.h
 *
 * Copyright (c) 2012 seeed technology inc.
 * Website    : www.seeed.cc
 * Author     : Jacky Zhang
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



#ifndef __GROVE_IMU_9DOF_V2_CLASS_H__
#define __GROVE_IMU_9DOF_V2_CLASS_H__

#include "suli2.h"

//GROVE_NAME        "Grove - IMU 9DOF v2.0"
//SKU               101020080
//IF_TYPE           I2C
//IMAGE_URL         https://statics3.seeedstudio.com/images/product/101020080%202.jpg
//DESCRIPTION       "Grove - IMU 9DOF v2.0 is an upgraded version of Grove - IMU 9DOF v1.0 and it is a high performance 9-axis motion tracking module, which is based on MPU-9250. The MPU-9250 is an integrated 9-axis motion tracking device designed for the low power, low cost, and high performance requirements of consumer electronics equipment including smartphones, tablets and wearable sensors. MPU-9150 features three 16-bit ADC for digitizing the gyroscope outputs and three 16-bit ADCs for digitizing the accelerometer outputs and three 16-bit ADCs for digitizing the magnetometer outputs."
//WIKI_URL          http://wiki.seeed.cc/Grove-IMU_9DOF_v2.0/
//ADDED_AT          "2016-10-04"
//AUTHOR            "Nathan Clevenger"

//Magnetometer Registers
#define MPU9150_RA_MAG_ADDRESS	0x0C
#define MPU9150_RA_MAG_XOUT_L		0x03
#define MPU9150_RA_MAG_XOUT_H		0x04
#define MPU9150_RA_MAG_YOUT_L		0x05
#define MPU9150_RA_MAG_YOUT_H		0x06
#define MPU9150_RA_MAG_ZOUT_L		0x07
#define MPU9150_RA_MAG_ZOUT_H		0x08

//#define CONFIGURATION_REGISTERA 0x00
//#define CONFIGURATION_REGISTERB 0x01
//#define MODE_REGISTER 0x02
//#define DATA_REGISTER_BEGIN 0x03

//#define MEASUREMENT_CONTINUOUS 0x00
//#define MEASUREMENT_SINGLE_SHOT 0x01
//#define MEASUREMENT_IDLE 0x03

#ifndef PI
#define PI ((float)3.1415926)
#endif

class GroveIMU9DOFV2
{
public:
    GroveIMU9DOFV2(int pinsda, int pinscl);

    /**
     *
     *
     * @param heading_deg - the angle of heading relative to the north, unit: degree
     *
     * @return bool
     */
    bool read_compass_heading(float *heading_deg);
private:
    I2C_T *i2c;
    uint8_t databuf[6];
    uint8_t cmdbuf[2];
    uint8_t mode;
};

#endif
