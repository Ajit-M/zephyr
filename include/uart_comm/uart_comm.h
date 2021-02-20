/*
	SBUS.h
	Brian R Taylor
	brian.taylor@bolderflight.com

	Copyright (c) 2016 Bolder Flight Systems

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#ifndef SBUS_h
#define SBUS_h
#endif

#include <stdbool.h>
// #include <stdint.h>
#include <stdlib.h>
#include <zephyr.h>
#include <drivers/uart.h>

// #include "Arduino.h"
// #include "elapsedMillis.h" // 

/*
* Hardware Serial Supported:
* Teensy 3.0 || Teensy 3.1/3.2 || Teensy 3.5 || Teensy 3.6 || Teensy LC  || STM32L4 || Maple Mini || Arduino Mega 2560 || ESP32
*/

bool read(uint16_t* channels, bool* failsafe, bool* lostFrame, int uart_confg_ret, const struct device *uart_dev);
bool readCal(float* calChannels, bool* failsafe, bool* lostFrame);
void write(uint16_t* channels);
void writeCal(float *channels);
void setEndPoints(uint8_t channel,uint16_t min,uint16_t max);
void getEndPoints(uint8_t channel,uint16_t *min,uint16_t *max);
void setReadCal(uint8_t channel,float *coeff,uint8_t len);
void getReadCal(uint8_t channel,float *coeff,uint8_t len);
void setWriteCal(uint8_t channel,float *coeff,uint8_t len);
void getWriteCal(uint8_t channel,float *coeff,uint8_t len);
int begin();

const uint32_t _sbusBaud = 100000;
#define _numChannels 16
const uint8_t _sbusHeader = 0x0F;
const uint8_t _sbusFooter = 0x00;
const uint8_t _sbus2Footer = 0x04;
const uint8_t _sbus2Mask = 0x0F;
const uint32_t SBUS_TIMEOUT_US = 7000;
uint8_t _parserState, _prevByte = _sbusFooter, data_read;
#define _payloadSize 24
uint8_t _payload[_payloadSize];
const uint8_t _sbusLostFrame = 0x04;
const uint8_t _sbusFailSafe = 0x08;
const uint16_t _defaultMin = 172;
const uint16_t _defaultMax = 1811;
uint16_t _sbusMin[_numChannels];
uint16_t _sbusMax[_numChannels];
float _sbusScale[_numChannels];
float _sbusBias[_numChannels];
float **_readCoeff, **_writeCoeff;
uint8_t _readLen[_numChannels],_writeLen[_numChannels];
bool _useReadCoeff[_numChannels], _useWriteCoeff[_numChannels];
bool parse(int uart_confg_ret, const struct device *uart_dev);
void scaleBias(uint8_t channel);
float PolyVal(size_t PolySize, float *Coefficients, float X);
