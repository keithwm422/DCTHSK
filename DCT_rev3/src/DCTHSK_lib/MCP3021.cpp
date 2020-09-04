/*
********************************************************************
  Name    : mcp3021.cpp
  Author  : Matthew Uniac
  Date    : 1 April, 2017
  Version : 1.0
  Notes   : A library that reads and writes to a mcp3021 10bit ADC
  Release : public
********************************************************************
*/

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "Wire.h"
#include "MCP3021.h"

//initiate instance of non default deviceId
void MCP3021::begin(TwoWire& w_, uint8_t deviceId) {
  Wire_=&w_;
  _deviceId = deviceId;
  _deviceAddress = (mcp3021Address|_deviceId); //base address for mcp3021 + device id
}

//default initiate to deviceID of 5
void MCP3021::begin(TwoWire& w_) {
  Wire_=&w_;
  _deviceId = 0b00000101;
  _deviceAddress = (mcp3021Address|_deviceId); //base address for mcp3021 + device id}
}

uint16_t MCP3021::readADC(){
    // store dataRead from ADC in an array
    uint8_t dataRead[2];
    dataRead[0] = 0b00000000; // set array to 0 at creation
    dataRead[1] = 0b00000000; // set array to 0 at creation
    
    uint16_t value; // declare the variable to store and then return the reading
    if(Wire_->available()) Wire_->flush();
    Wire_->requestFrom(_deviceAddress, 2);// request 2 bytes from mcp3021
    for(int i =0; Wire_->available() > 0;i++)//while device is available (should count to 2)
    {
        dataRead[i] = Wire_->read();
    }
    value = (value | dataRead[0]) << 6; // we take the first byte and shift left 6 places since Value is 2 bytes
    dataRead[1] = dataRead[1] >> 2; //we take the second byte and shift it to the right 2 places,  this drops the 00 off the end
    value = value | dataRead[1]; // we OR both bytes to add them togeather

    return value;
}
