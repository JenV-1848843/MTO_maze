////////////////////////////////////////////////////////////////////////////////
// License:  This  program  is  free software; you can redistribute it and/or //
// modify  it  under the terms of the GNU General Public License as published //
// by  the  Free Software Foundation; either version 3 of the License, or (at //
// your  option)  any  later version. This program is distributed in the hope //
// that it will be useful, but WITHOUT ANY WARRANTY; without even the implied //
// warranty  of  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the //
// GNU General Public License for more details.                               //
////////////////////////////////////////////////////////////////////////////////
// pi2c.cpp:
//////////////////////////////////////////////////////////////////////////////// 
#include "pi2c.h"
#include <cstdint>
#include <array>
#include <math.h>

Pi2c::Pi2c(int address, bool rev0){
	//Set up the filename of the I2C Bus. Choose appropriate bus for Raspberry Pi Rev.
	char filename[11] = "/dev/i2c-";
	if (rev0 == true){
		filename[9] = '0';
	}
	else {
		filename[9] = '1';
	}
	filename[10] = 0; //Add the null character onto the end of the array to make it a string
	
	i2cHandle_ = open(filename, O_RDWR); //Open the i2c file descriptor in read/write mode
	if (i2cHandle_ < 0) {
		std::cout << "Can't open I2C BUS" << std::endl; //If there's an error opening this, then display it.
	}
	if (ioctl(i2cHandle_, I2C_SLAVE, address) < 0) { //Using ioctl set the i2c device to talk to address in the "addr" variable.
		std::cout << "Can't set the I2C address for the slave device" << std::endl; //Display error setting the address for the slave.
	}
}

Pi2c::~Pi2c(){
	if (i2cHandle_){ //If the I2C File handle is still open...
		close(i2cHandle_); //...Close it.
	}
}

int Pi2c::i2cRead(char *data,int length){
	int er = read(i2cHandle_,data,length); //Read "length" number of bytes into the "data" buffer from the I2C bus.
	return er;
}
int Pi2c::i2cWrite(char *data,int length){
	int er = write(i2cHandle_,data,length);//Write "length" number of bytes from the "data" buffer to the I2C bus.
	return er;
}

int Pi2c::i2cReadArduinoInt(){
	const int arr_size = 2;
	char tmp[arr_size]; //We know an Arduino Int is 2 Bytes.
	int retval=-1;
	
	if (i2cRead(tmp,arr_size) > 0){
		retval = tmp[1] << 8 | tmp[0]; //Using bit shifting, turn the 2 byte array into an Int.
	}
	return retval;
}

int Pi2c::init(){
	// Wake up from sleep (register 0x6B = PWR_MGMT_1)
	writeReg(0x6B, 0x00);
	usleep(1000);
	// Set gyro range to ±250°/s (register 0x1B = GYRO_CONFIG)
	writeReg(0x1B, 0x00);

	usleep(100000); // 100ms settle time
	return 0;
}

int Pi2c::i2cWriteArduinoInt(int input){
	const int arr_size = 2;
	char tmp[arr_size]; //We know an Arduino Int is 2 Bytes.
	int retval=0;
	
	tmp[0] = input; //get lowest 8 bits into the first part of the array;
	tmp[1] = input >> 8; //get the highest 8 bits into the second part of the array;
	retval = (i2cWrite(tmp,arr_size) > 0);
	return retval;
}

void Pi2c::writeReg(char reg, char value){
	char buf[2] = {reg, value};
	i2cWrite(buf, 2);
}

std::array<float, 3> Pi2c::readGyro(){
	char gyro_reg = 0x43;
	int gyro_length = 6;

	char* gyro;

	i2cWrite(&gyro_reg, 1);
	i2cRead(gyro, gyro_length);

	// Combine high and low bytes into 16-bit signed integers
    int16_t gyro_x = (int16_t)((uint8_t)gyro[0] << 8 | (uint8_t)gyro[1]);
    int16_t gyro_y = (int16_t)((uint8_t)gyro[2] << 8 | (uint8_t)gyro[3]);
    int16_t gyro_z = (int16_t)((uint8_t)gyro[4] << 8 | (uint8_t)gyro[5]);
    // Convert raw values to degrees/sec (±250°/s range → 131 LSB per °/s)
    float gx = gyro_x / 131.0f;
    float gy = gyro_y / 131.0f;
    float gz = gyro_z / 131.0f;

	return {gx, gy, gz};
}

std::array<float, 5> Pi2c::readAccel(){
    char accel_reg = 0x3B;
    int accel_length = 6;

    char* accel;

    i2cWrite(&accel_reg, 1);
    i2cRead(accel, accel_length);

    // Combine high and low bytes into 16-bit signed integers
    int16_t accel_x = (int16_t)((uint8_t)accel[0] << 8 | (uint8_t)accel[1]);
    int16_t accel_y = (int16_t)((uint8_t)accel[2] << 8 | (uint8_t)accel[3]);
    int16_t accel_z = (int16_t)((uint8_t)accel[4] << 8 | (uint8_t)accel[5]);

    // Convert to g-force (±2g range → 16384 LSB per g)
    float fax = accel_x / 16384.0f;
    float fay = accel_y / 16384.0f;
    float faz = accel_z / 16384.0f;

    // Calculate roll and pitch in degrees
    float roll  = atan2(fay, faz) * 180.0f / M_PI;
    float pitch = atan2(-fax, sqrt(fay * fay + faz * faz)) * 180.0f / M_PI;
    return {roll, pitch, fax, fay, faz};  // yaw cannot be determined from accelerometer alone
}