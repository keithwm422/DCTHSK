/*
********************************************************************
  Name    : mcp3021.h
  Author  : Matthew Uniac
  Date    : 1 April, 2017
  Version : 1.0
  Notes   : A library that reads and writes to a mcp3021 10bit ADC
  Release : 
********************************************************************
*/

#ifndef mcp3021_h
#define mcp3021_h


#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "Wire.h"


	class MCP3021 {
		public:
		    void begin( TwoWire& w_,uint8_t deviceId);
  			void begin( TwoWire& w_);
			uint16_t readADC(); // read and return the ADC value
			float convertADC(); // read the ADC value and convert to voltage
			
		private:		
  			static const int mcp3021Address = 0b01001000; //base address for mcp3021
  			uint8_t _deviceId; // device id.  From actual IC.  ex: mcp3021A5 = 0b00000101
  			int _deviceAddress; // calculated device address using deviceID and constant mcp3021Adress
                        TwoWire* Wire_;

    };


#endif
