/*
  ArduCAM.cpp - Arduino library support for CMOS Image Sensor
  Library modified by Tom Hightower
  Original Library Copyright (C)2011-2015 ArduCAM.com. All right reserved

  This is a version of the ArduCAM Library found at ArduCAM.com, modified
  by removing unused functions to encourage lightweight functionality

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.
*/

#include "ArduCAM.h"
#include <Wire.h>
#include <SPI.h>
#include "HardwareSerial.h"

//defaults to OV5642
ArduCAM::ArduCAM(int CS) {
	sensor_model = OV5642;
	sensor_addr = 0x78;
	P_CS  = portOutputRegister(digitalPinToPort(CS));
	B_CS  = digitalPinToBitMask(CS); 
	pinMode(CS, OUTPUT);
	sbi(P_CS, B_CS);
}

void ArduCAM::InitCAM() {
	if (sensor_model == OV5642) {
#if ( defined(OV5642_CAM) || defined(OV5642_MINI_5MP) || defined(OV5642_MINI_5MP_BIT_ROTATION_FIXED) || defined(OV5642_MINI_5MP_PLUS) )
		wrSensorReg16_8(0x3008, 0x80);
		wrSensorRegs16_8(OV5642_QVGA_Preview);
		delay(100);
		if (m_fmt == JPEG) {
			delay(100);
			wrSensorRegs16_8(OV5642_JPEG_Capture_QSXGA);
			wrSensorRegs16_8(ov5642_320x240);
			delay(100);
			wrSensorReg16_8(0x3818, 0xa8);
			wrSensorReg16_8(0x3621, 0x10);
			wrSensorReg16_8(0x3801, 0xb0);
#if (defined(OV5642_MINI_5MP_PLUS) || (defined ARDUCAM_SHIELD_V2))
			wrSensorReg16_8(0x4407, 0x04);
#else
			wrSensorReg16_8(0x4407, 0x0C);
#endif
		} else {							
			byte reg_val;
			wrSensorReg16_8(0x4740, 0x21);
			wrSensorReg16_8(0x501e, 0x2a);
			wrSensorReg16_8(0x5002, 0xf8);
			wrSensorReg16_8(0x501f, 0x01);
			wrSensorReg16_8(0x4300, 0x61);
			rdSensorReg16_8(0x3818, &reg_val);
			wrSensorReg16_8(0x3818, (reg_val | 0x60) & 0xff);
			rdSensorReg16_8(0x3621, &reg_val);
			wrSensorReg16_8(0x3621, reg_val & 0xdf);
		}		
#endif
	}
}

void ArduCAM::flush_fifo(void) {
	write_reg(ARDUCHIP_FIFO, FIFO_CLEAR_MASK);
}

void ArduCAM::start_capture(void) {
	write_reg(ARDUCHIP_FIFO, FIFO_START_MASK);
}

void ArduCAM::clear_fifo_flag(void ) {
	write_reg(ARDUCHIP_FIFO, FIFO_CLEAR_MASK);
}

uint32_t ArduCAM::read_fifo_length(void) {
	uint32_t len1,len2,len3,length=0;
	len1 = read_reg(FIFO_SIZE1);
	len2 = read_reg(FIFO_SIZE2);
	len3 = read_reg(FIFO_SIZE3) & 0x7f;
	length = ((len3 << 16) | (len2 << 8) | len1) & 0x07fffff;
	return length;	
}

void ArduCAM::set_fifo_burst() {
	SPI.transfer(BURST_FIFO_READ);		
}

void ArduCAM::CS_HIGH(void) {
	sbi(P_CS, B_CS);	
}
void ArduCAM::CS_LOW(void) {
	cbi(P_CS, B_CS);	
}

uint8_t ArduCAM::read_fifo(void) {
	uint8_t data;
	data = bus_read(SINGLE_FIFO_READ);
	return data;
}

uint8_t ArduCAM::read_reg(uint8_t addr) {
	uint8_t data;
	data = bus_read(addr & 0x7F);
	return data;
}

void ArduCAM::write_reg(uint8_t addr, uint8_t data) {
	bus_write(addr | 0x80, data);
}

//Set corresponding bit  
void ArduCAM::set_bit(uint8_t addr, uint8_t bit) {
	uint8_t temp;
	temp = read_reg(addr);
	write_reg(addr, temp | bit);
}
//Clear corresponding bit 
void ArduCAM::clear_bit(uint8_t addr, uint8_t bit) {
	uint8_t temp;
	temp = read_reg(addr);
	write_reg(addr, temp & (~bit));
}

//Get corresponding bit status
uint8_t ArduCAM::get_bit(uint8_t addr, uint8_t bit) {
	uint8_t temp;
	temp = read_reg(addr);
	temp = temp & bit;
	return temp;
}

//Set ArduCAM working mode
//MCU2LCD_MODE: MCU writes the LCD screen GRAM
//CAM2LCD_MODE: Camera takes control of the LCD screen
//LCD2MCU_MODE: MCU read the LCD screen GRAM
void ArduCAM::set_mode(uint8_t mode) {
	switch (mode)
		{
		case MCU2LCD_MODE:
			write_reg(ARDUCHIP_MODE, MCU2LCD_MODE);
			break;
		case CAM2LCD_MODE:
			write_reg(ARDUCHIP_MODE, CAM2LCD_MODE);
			break;
		case LCD2MCU_MODE:
			write_reg(ARDUCHIP_MODE, LCD2MCU_MODE);
			break;
		default:
			write_reg(ARDUCHIP_MODE, MCU2LCD_MODE);
			break;
		}
}

uint8_t ArduCAM::bus_write(int address,int value) {	
	cbi(P_CS, B_CS);
	SPI.transfer(address);
	SPI.transfer(value);
	sbi(P_CS, B_CS);
	return 1;
}

uint8_t ArduCAM:: bus_read(int address) {
	uint8_t value;
	cbi(P_CS, B_CS);
#if (defined(ESP8266) || defined(__arm__))
#if defined(OV5642_MINI_5MP)
	SPI.transfer(address);
	value = SPI.transfer(0x00);
	// correction for bit rotation from readback
	value = (byte)(value >> 1) | (value << 7);
	// take the SS pin high to de-select the chip:
	sbi(P_CS, B_CS);
	return value;
#else
	SPI.transfer(address);
	value = SPI.transfer(0x00);
	// take the SS pin high to de-select the chip:
	sbi(P_CS, B_CS);
	return value;
#endif
#else
	SPI.transfer(address);
	value = SPI.transfer(0x00);
	// take the SS pin high to de-select the chip:
	sbi(P_CS, B_CS);
	return value;
#endif
}

void ArduCAM::OV5642_set_JPEG_size(uint8_t size) {
#if defined(OV5642_CAM) || defined(OV5642_CAM_BIT_ROTATION_FIXED)|| defined(OV5642_MINI_5MP) || defined (OV5642_MINI_5MP_PLUS)
	uint8_t reg_val;
	switch (size) {
		case OV5642_320x240:
			wrSensorRegs16_8(ov5642_320x240);
			break;
		case OV5642_640x480:
			wrSensorRegs16_8(ov5642_640x480);
			break;
		case OV5642_1024x768:
			wrSensorRegs16_8(ov5642_1024x768);
			break;
		case OV5642_1280x960:
			wrSensorRegs16_8(ov5642_1280x960);
			break;
		case OV5642_1600x1200:
			wrSensorRegs16_8(ov5642_1600x1200);
			break;
		case OV5642_2048x1536:
			wrSensorRegs16_8(ov5642_2048x1536);
			break;
		case OV5642_2592x1944:
			wrSensorRegs16_8(ov5642_2592x1944);
			break;
		default:
			wrSensorRegs16_8(ov5642_320x240);
			break;
		}
#endif
}

void ArduCAM::set_format(byte fmt) {
	if (fmt == BMP)
		m_fmt = BMP;
	else
		m_fmt = JPEG;
}			
	
void ArduCAM::OV5642_set_Contrast(uint8_t Contrast) {
#if defined(OV5642_CAM) || defined(OV5642_CAM_BIT_ROTATION_FIXED)|| defined(OV5642_MINI_5MP) || defined (OV5642_MINI_5MP_PLUS)	
	switch(Contrast) {
		case Contrast4:
			wrSensorReg16_8(0x5001 ,0xff);
			wrSensorReg16_8(0x5580 ,0x04);
			wrSensorReg16_8(0x5587 ,0x30);
			wrSensorReg16_8(0x5588 ,0x30);
			wrSensorReg16_8(0x558a ,0x00);
			break;
		case Contrast3:
			wrSensorReg16_8(0x5001 ,0xff);
			wrSensorReg16_8(0x5580 ,0x04);
			wrSensorReg16_8(0x5587 ,0x2c);
			wrSensorReg16_8(0x5588 ,0x2c);
			wrSensorReg16_8(0x558a ,0x00);
			break;
		case Contrast2:
			wrSensorReg16_8(0x5001 ,0xff);
			wrSensorReg16_8(0x5580 ,0x04);
			wrSensorReg16_8(0x5587 ,0x28);
			wrSensorReg16_8(0x5588 ,0x28);
			wrSensorReg16_8(0x558a ,0x00);
			break;
		case Contrast1:
			wrSensorReg16_8(0x5001 ,0xff);
			wrSensorReg16_8(0x5580 ,0x04);
			wrSensorReg16_8(0x5587 ,0x24);
			wrSensorReg16_8(0x5588 ,0x24);
			wrSensorReg16_8(0x558a ,0x00);
			break;
		case Contrast0:
			wrSensorReg16_8(0x5001 ,0xff);
			wrSensorReg16_8(0x5580 ,0x04);
			wrSensorReg16_8(0x5587 ,0x20);
			wrSensorReg16_8(0x5588 ,0x20);
			wrSensorReg16_8(0x558a ,0x00);
			break;
		case Contrast_1:
			wrSensorReg16_8(0x5001 ,0xff);
			wrSensorReg16_8(0x5580 ,0x04);
			wrSensorReg16_8(0x5587 ,0x1C);
			wrSensorReg16_8(0x5588 ,0x1C);
			wrSensorReg16_8(0x558a ,0x1C);
			break;
		case Contrast_2:
			wrSensorReg16_8(0x5001 ,0xff);
			wrSensorReg16_8(0x5580 ,0x04);
			wrSensorReg16_8(0x5587 ,0x18);
			wrSensorReg16_8(0x5588 ,0x18);
			wrSensorReg16_8(0x558a ,0x00);
			break;
		case Contrast_3:
			wrSensorReg16_8(0x5001 ,0xff);
			wrSensorReg16_8(0x5580 ,0x04);
			wrSensorReg16_8(0x5587 ,0x14);
			wrSensorReg16_8(0x5588 ,0x14);
			wrSensorReg16_8(0x558a ,0x00);
			break;
		case Contrast_4:
			wrSensorReg16_8(0x5001 ,0xff);
			wrSensorReg16_8(0x5580 ,0x04);
			wrSensorReg16_8(0x5587 ,0x10);
			wrSensorReg16_8(0x5588 ,0x10);
			wrSensorReg16_8(0x558a ,0x00);
			break;
		}
#endif		
}
	
void ArduCAM::OV5642_set_Exposure_level(uint8_t level) {
#if defined(OV5642_CAM) || defined(OV5642_CAM_BIT_ROTATION_FIXED)|| defined(OV5642_MINI_5MP) || defined (OV5642_MINI_5MP_PLUS)	
	switch(level) {
		case Exposure_17_EV:
			wrSensorReg16_8(0x3a0f ,0x10);
			wrSensorReg16_8(0x3a10 ,0x08);
			wrSensorReg16_8(0x3a1b ,0x10);
			wrSensorReg16_8(0x3a1e ,0x08);
			wrSensorReg16_8(0x3a11 ,0x20);
			wrSensorReg16_8(0x3a1f ,0x10);
			break;
		case Exposure_13_EV:
			wrSensorReg16_8(0x3a0f ,0x18);
			wrSensorReg16_8(0x3a10 ,0x10);
			wrSensorReg16_8(0x3a1b ,0x18);
			wrSensorReg16_8(0x3a1e ,0x10);
			wrSensorReg16_8(0x3a11 ,0x30);
			wrSensorReg16_8(0x3a1f ,0x10);
			break;
		case Exposure_10_EV:
			wrSensorReg16_8(0x3a0f ,0x20);
			wrSensorReg16_8(0x3a10 ,0x18);
			wrSensorReg16_8(0x3a11 ,0x41);
			wrSensorReg16_8(0x3a1b ,0x20);
			wrSensorReg16_8(0x3a1e ,0x18);
			wrSensorReg16_8(0x3a1f ,0x10);
			break;
		case Exposure_07_EV:
			wrSensorReg16_8(0x3a0f ,0x28);
			wrSensorReg16_8(0x3a10 ,0x20);
			wrSensorReg16_8(0x3a11 ,0x51);
			wrSensorReg16_8(0x3a1b ,0x28);
			wrSensorReg16_8(0x3a1e ,0x20);
			wrSensorReg16_8(0x3a1f ,0x10);
			break;
		case Exposure_03_EV:
			wrSensorReg16_8(0x3a0f ,0x30);
			wrSensorReg16_8(0x3a10 ,0x28);
			wrSensorReg16_8(0x3a11 ,0x61);
			wrSensorReg16_8(0x3a1b ,0x30);
			wrSensorReg16_8(0x3a1e ,0x28);
			wrSensorReg16_8(0x3a1f ,0x10);
			break;
		case Exposure_default:
			wrSensorReg16_8(0x3a0f ,0x38);
			wrSensorReg16_8(0x3a10 ,0x30);
			wrSensorReg16_8(0x3a11 ,0x61);
			wrSensorReg16_8(0x3a1b ,0x38);
			wrSensorReg16_8(0x3a1e ,0x30);
			wrSensorReg16_8(0x3a1f ,0x10);
			wrSensorReg16_8(0x3a0f ,0x40);
			wrSensorReg16_8(0x3a10 ,0x38);
			wrSensorReg16_8(0x3a11 ,0x71);
			wrSensorReg16_8(0x3a1b ,0x40);
			wrSensorReg16_8(0x3a1e ,0x38);
			wrSensorReg16_8(0x3a1f ,0x10);
			break;
		case Exposure07_EV:
			wrSensorReg16_8(0x3a0f ,0x48);
			wrSensorReg16_8(0x3a10 ,0x40);
			wrSensorReg16_8(0x3a11 ,0x80);
			wrSensorReg16_8(0x3a1b ,0x48);
			wrSensorReg16_8(0x3a1e ,0x40);
			wrSensorReg16_8(0x3a1f ,0x20);
			break;
		case Exposure10_EV:
			wrSensorReg16_8(0x3a0f ,0x50);
			wrSensorReg16_8(0x3a10 ,0x48);
			wrSensorReg16_8(0x3a11 ,0x90);
			wrSensorReg16_8(0x3a1b ,0x50);
			wrSensorReg16_8(0x3a1e ,0x48);
			wrSensorReg16_8(0x3a1f ,0x20);
			break;
		case Exposure13_EV:
			wrSensorReg16_8(0x3a0f ,0x58);
			wrSensorReg16_8(0x3a10 ,0x50);
			wrSensorReg16_8(0x3a11 ,0x91);
			wrSensorReg16_8(0x3a1b ,0x58);
			wrSensorReg16_8(0x3a1e ,0x50);
			wrSensorReg16_8(0x3a1f ,0x20);
			break;
		case Exposure17_EV:
			wrSensorReg16_8(0x3a0f ,0x60);
			wrSensorReg16_8(0x3a10 ,0x58);
			wrSensorReg16_8(0x3a11 ,0xa0);
			wrSensorReg16_8(0x3a1b ,0x60);
			wrSensorReg16_8(0x3a1e ,0x58);
			wrSensorReg16_8(0x3a1f ,0x20);
			break;
		}
#endif	
}
			
void ArduCAM::OV5642_set_Compress_quality(uint8_t quality) {
#if defined(OV5642_CAM) || defined(OV5642_CAM_BIT_ROTATION_FIXED)|| defined(OV5642_MINI_5MP) || defined (OV5642_MINI_5MP_PLUS)	
	switch(quality) {
	case high_quality:
		wrSensorReg16_8(0x4407, 0x02);
		break;
	case default_quality:
		wrSensorReg16_8(0x4407, 0x04);
		break;
	case low_quality:
		wrSensorReg16_8(0x4407, 0x08);
		break;
	}
#endif
}	

// Write 8 bit values to 8 bit register address
int ArduCAM::wrSensorRegs8_8(const struct sensor_reg reglist[]) {
	int err = 0;
	uint16_t reg_addr = 0;
	uint16_t reg_val = 0;
	const struct sensor_reg *next = reglist;
	while ((reg_addr != 0xff) | (reg_val != 0xff)) {
			reg_addr = pgm_read_word(&next->reg);
			reg_val = pgm_read_word(&next->val);
			err = wrSensorReg8_8(reg_addr, reg_val);
			next++;
#if (defined(ESP8266)||defined(ESP32))
			yield();
#endif
		}
	return 1;
}

// Write 16 bit values to 8 bit register address
int ArduCAM::wrSensorRegs8_16(const struct sensor_reg reglist[]) {
	int err = 0;
	unsigned int reg_addr, reg_val;
	const struct sensor_reg *next = reglist;
	while ((reg_addr != 0xff) | (reg_val != 0xffff)) {
			reg_addr = pgm_read_word(&next->reg);
			reg_val = pgm_read_word(&next->val);
			err = wrSensorReg8_16(reg_addr, reg_val);
			next++;
#if defined(ESP8266)||defined(ESP32)
			yield();
#endif
		}
	return 1;
}

// Write 8 bit values to 16 bit register address
int ArduCAM::wrSensorRegs16_8(const struct sensor_reg reglist[]) {
	int err = 0;
	unsigned int reg_addr;
	unsigned char reg_val;
	const struct sensor_reg *next = reglist;
	while ((reg_addr != 0xffff) | (reg_val != 0xff)) {
			reg_addr = pgm_read_word(&next->reg);
			reg_val = pgm_read_word(&next->val);
			err = wrSensorReg16_8(reg_addr, reg_val);
			next++;
#if defined(ESP8266)||defined(ESP32)
			yield();
#endif
		}
	return 1;
}

//I2C Array Write 16bit address, 16bit data
int ArduCAM::wrSensorRegs16_16(const struct sensor_reg reglist[]) {
	int err = 0;
	unsigned int reg_addr, reg_val;
	const struct sensor_reg *next = reglist;
	reg_addr = pgm_read_word(&next->reg);
	reg_val = pgm_read_word(&next->val);
	while ((reg_addr != 0xffff) | (reg_val != 0xffff)) {
			err = wrSensorReg16_16(reg_addr, reg_val);
			//if (!err)
			//   return err;
			next++;
			reg_addr = pgm_read_word(&next->reg);
			reg_val = pgm_read_word(&next->val);
#if defined(ESP8266)||defined(ESP32)
			yield();
#endif
		}
	return 1;
}


// Read/write 8 bit value to/from 8 bit register address	
byte ArduCAM::wrSensorReg8_8(int regID, int regDat) {
	Wire.beginTransmission(sensor_addr >> 1);
	Wire.write(regID & 0x00FF);
	Wire.write(regDat & 0x00FF);
	if (Wire.endTransmission()) {
			return 0;
		}
	delay(1);
	return 1;
}

byte ArduCAM::rdSensorReg8_8(uint8_t regID, uint8_t* regDat) {
	Wire.beginTransmission(sensor_addr >> 1);
	Wire.write(regID & 0x00FF);
	Wire.endTransmission();
	Wire.requestFrom((sensor_addr >> 1), 1);
	if (Wire.available())
		*regDat = Wire.read();
	delay(1);
	return 1;
}

// Read/write 16 bit value to/from 8 bit register address
byte ArduCAM::wrSensorReg8_16(int regID, int regDat) {
	Wire.beginTransmission(sensor_addr >> 1);
	Wire.write(regID & 0x00FF);
	Wire.write(regDat >> 8);            // sends data byte, MSB first
	Wire.write(regDat & 0x00FF);
	if (Wire.endTransmission()) {
			return 0;
		}	
	delay(1);
	return 1;
}

byte ArduCAM::rdSensorReg8_16(uint8_t regID, uint16_t* regDat) {
	uint8_t temp;
	Wire.beginTransmission(sensor_addr >> 1);
	Wire.write(regID);
	Wire.endTransmission();
	Wire.requestFrom((sensor_addr >> 1), 2);
	if (Wire.available()) {
			temp = Wire.read();
			*regDat = (temp << 8) | Wire.read();
		}
	delay(1);
	return 1;
}

// Read/write 8 bit value to/from 16 bit register address
byte ArduCAM::wrSensorReg16_8(int regID, int regDat) {
	Wire.beginTransmission(sensor_addr >> 1);
	Wire.write(regID >> 8);            // sends instruction byte, MSB first
	Wire.write(regID & 0x00FF);
	Wire.write(regDat & 0x00FF);
	if (Wire.endTransmission()) {
			return 0;
		}
	delay(1);
	return 1;
}

byte ArduCAM::rdSensorReg16_8(uint16_t regID, uint8_t* regDat) {
	Wire.beginTransmission(sensor_addr >> 1);
	Wire.write(regID >> 8);
	Wire.write(regID & 0x00FF);
	Wire.endTransmission();
	Wire.requestFrom((sensor_addr >> 1), 1);
	if (Wire.available()) {
			*regDat = Wire.read();
		}
	delay(1);
	return 1;
}

//I2C Write 16bit address, 16bit data
byte ArduCAM::wrSensorReg16_16(int regID, int regDat) {
	Wire.beginTransmission(sensor_addr >> 1);
	Wire.write(regID >> 8);            // sends instruction byte, MSB first
	Wire.write(regID & 0x00FF);
	Wire.write(regDat >> 8);            // sends data byte, MSB first
	Wire.write(regDat & 0x00FF);
	if (Wire.endTransmission()) {
			return 0;
		}
	delay(1);
	return (1);
}

//I2C Read 16bit address, 16bit data
byte ArduCAM::rdSensorReg16_16(uint16_t regID, uint16_t* regDat) {
	uint16_t temp;
	Wire.beginTransmission(sensor_addr >> 1);
	Wire.write(regID >> 8);
	Wire.write(regID & 0x00FF);
	Wire.endTransmission();
	Wire.requestFrom((sensor_addr >> 1), 2);
	if (Wire.available()) {
			temp = Wire.read();
			*regDat = (temp << 8) | Wire.read();
		}
	delay(1);
	return (1);
}

#if defined(ESP8266)
inline void ArduCAM::setDataBits(uint16_t bits) {
	const uint32_t mask = ~((SPIMMOSI << SPILMOSI) | (SPIMMISO << SPILMISO));
	bits--;
	SPI1U1 = ((SPI1U1 & mask) | ((bits << SPILMOSI) | (bits << SPILMISO)));
}

void ArduCAM::transferBytes_(uint8_t * out, uint8_t * in, uint8_t size) {
	while (SPI1CMD & SPIBUSY) {}
	// Set in/out Bits to transfer
	setDataBits(size * 8);
	volatile uint32_t * fifoPtr = &SPI1W0;
	uint8_t dataSize = ((size + 3) / 4);
	if (out) {
		uint32_t * dataPtr = (uint32_t*) out;
		while (dataSize--) {
			*fifoPtr = *dataPtr;
			dataPtr++;
			fifoPtr++;
		}
	} else {
		// no out data only read fill with dummy data!
		while (dataSize--) {
			*fifoPtr = 0xFFFFFFFF;
			fifoPtr++;
		}
	}
	SPI1CMD |= SPIBUSY;
	while (SPI1CMD & SPIBUSY) {}
	if (in) {
		volatile uint8_t * fifoPtr8 = (volatile uint8_t *) &SPI1W0;
		dataSize = size;
		while (dataSize--) {
#if defined(OV5642_MINI_5MP)
			*in = *fifoPtr8;
			*in = (byte)(*in >> 1) | (*in << 7);
#else
			*in = *fifoPtr8;
#endif
			in++;
			fifoPtr8++;
		}
	}
}

void ArduCAM::transferBytes(uint8_t * out, uint8_t * in, uint32_t size) {
	while (size) {
		if (size > 64) {
			transferBytes_(out, in, 64);
			size -= 64;
			if (out) out += 64;
			if (in) in += 64;
		} else {
			transferBytes_(out, in, size);
			size = 0;
		}
	}
}
#endif
