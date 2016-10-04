/*
 * grove_comp_hmc5883l.cpp
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


#include "grove_imu_9dof_v2.h"

GroveIMU9DOFV2::GroveIMU9DOFV2(int pinsda, int pinscl)
{
    this->i2c = (I2C_T *)malloc(sizeof(I2C_T));
    suli_i2c_init(i2c, pinsda, pinscl);

    suli_delay_ms(5);

    //uint8_t config;
	//config = (0x01 << 5);
    //suli_i2c_write_reg(i2c, HMC5883L_ADDRESS, CONFIGURATION_REGISTERB, &config, 1);

    mode = MEASUREMENT_CONTINUOUS;
//    suli_i2c_write_reg(i2c, HMC5883L_ADDRESS, MODE_REGISTER, &mode, 1);

//    cmdbuf[0] = DATA_REGISTER_BEGIN;
//	suli_i2c_write(i2c, HMC5883L_ADDRESS, cmdbuf, 1);
}

bool GroveCompass::read_compass_heading(float *heading_deg)
{
  int16_t x,y,z;
  float cx, cy, cz;

  suli_i2c_write(i2c, MPU9150_RA_MAG_ADDRESS, 0x0A, 1);
  suli_delay_ms(10);
  suli_i2c_read(i2c, MPU9150_RA_MAG_ADDRESS, databuf, 6);

    //I2C_M.writeByte(MPU9150_RA_MAG_ADDRESS, 0x0A, 0x01); //enable the magnetometer
  	//delay(10);
  	//I2C_M.readBytes(MPU9150_RA_MAG_ADDRESS, MPU9150_RA_MAG_XOUT_L, 6, databuf);

	x = (databuf[0] << 8) | databuf[1];
	z = (databuf[2] << 8) | databuf[3];
	y = (databuf[4] << 8) | databuf[5];

	//cx = static_cast<float>(x * 4800 / 8192);
	//cy = static_cast<float>(y * 4800 / 8192);
	//cz = static_cast<float>(z * 4800 / 8192);

  cx = static_cast<float>(x);
	cy = static_cast<float>(y);
	cz = static_cast<float>(z);

	float head = atan2f(cy, cx) - 0.0457;

	// Correct for when signs are reversed.
	if(head < 0)
	head += 2*PI;

	// Check for wrap due to addition of declination.
	if(head > 2*PI)
	head -= 2*PI;

	// Convert radians to degrees for readability.
	*heading_deg = head * 180 / PI;

//    cmdbuf[0] = DATA_REGISTER_BEGIN;
//	suli_i2c_write(i2c, HMC5883L_ADDRESS, cmdbuf, 1);

	return true;
}
