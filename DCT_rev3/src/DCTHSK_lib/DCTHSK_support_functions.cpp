/*
 * DCTHSK_support_functions.cpp
 * 
 * Defines a set of functions to act as responses to received local commands for the DCT HSK board
 *
 */

/*******************************************************************************
* Defines
*******************************************************************************/
#include "DCTHSK_support_functions.h"
//#include "Wire_1.h"
#include "MCP4728_cathode.h"
#include "MCP4728_potential.h"
#include "MCP3021.h"
MCP3021 pressure_adc;
MCP4728_POT pot_dac;
MCP4728_CAT cat_dac;

uint8_t default_address=96;
/* Buffer for outgoing data */
uint8_t outgoingData [255] = {0};

/* Variables for performing a temperature reading + storing it in an array of uint8_t */
uint32_t TempRead;
uint32_t * tmp;

void whatToDoIfISR(uint8_t * data)
{		
	TempRead = analogRead(TEMPSENSOR);
	
	tmp = &TempRead;
	
	/* Fills outgoing data buffer */
	for(int i=0; i < 4; i++)
    {
        *(data+i) = *tmp;
        *tmp = *tmp>>8;
    }
}

/* Function flow:
 * --Defines two variables, val & channel, to store a channel's pot value
 * --Fills outgoing packet header w/protocol standard
 * --Writes to the heater control board (?) commands it can understand
 * --Waits for a reply byte to verify the message was received
 * --Sends that reply byte back to the SFC for verification
 * 
 * Function params:
 * hdr_in:		Incoming header pointer, can find the data pts from the location
 * hdr_out:		Pointer to the outgoing packet header
 * stream:		Serial port where the heater control is connected
 * 
 * Function variables:
 * val:			Potentiometer value to set
 * channel:		Which potentiometer
 * 
 *  */
int whatToDoIfHeaterControl(uint8_t * data, uint8_t len)
{	
	/* If 1 byte, change all channels  to data byte*/ 
	// will write the serial.write to the linduino here
//	serial.write(254); // always write this first byte
	//needs to be about respData packets..
	int retval = 0;
	(0x01 & len) ? retval=1 : retval=2;
	return retval;
}

int whatToDoIfTestHeaterControl(uint8_t* data, uint8_t len, uint8_t * respData)
{
	/* If 1 byte, change all channels  to data byte*/

	int retval = 0;
	(0x01 & len) ? retval=3 : retval=4;
	*respData = 254;
	if (len == 1) {
		*(respData + 1) = 170;
		*(respData + 2) = *data;
	}
	else if (len == 2) {
		*(respData + 1) = 171;
		*(respData + 2) = *data;
		*(respData + 3) = *(data + 1);
	}
	else retval = EBADLEN;
//	(0x01 & len) ? serial.write(170) : serial.write(171);
	return retval;
	// now just write the next value, and if len >1 write the second byte in data also. 
//	stream.write(*data);
//	if ((int)len == 2) serial.write(*data + 1);
   	/* Fill outgoing data with that byte */
    // where do we put the data?
}
int whatToDoIfThermistors(uint8_t * respData){

  int retval=1;
  //match a read somehow from the thermistors, right now make this up?
  *respData = 0x05;
  return retval;
}

// returns 0 if one of the DAC's can't be set to 0 (and doesn't read 0).
bool PressureSetup(TwoWire& wire){
  pressure_adc.begin(wire,0); // set the address (second arg) to 0 since Mike ordered a specific addr device
  return 0;
}

uint16_t PressureRead(){
  return pressure_adc.readADC();
}


// returns 0 if one of the DAC's can't be set to 0 (and doesn't read 0).
bool CATSetup(TwoWire& wire, uint8_t LDAC){
  cat_dac.attach(wire, LDAC); // set the LDAC pin of launchpad and the i2c comms
  // Setup all DAC channels to have VDD as reference.
  //There are 8 chips each with their own addresses and 4 channels on each chip
  cat_dac.setID(default_address);
  cat_dac.selectVref(MCP4728_CAT::VREF::VDD, MCP4728_CAT::VREF::VDD, MCP4728_CAT::VREF::VDD, MCP4728_CAT::VREF::VDD);
  cat_dac.analogWrite(0,0,0,0);
//  cat_dac.readRegisters();
  uint16_t dac_val=0;
//  for(int j=0;j<4;j++) dac_val=dac_val | cat_dac.getDACData(j);
  if(dac_val==0) return 1;
  return 0;
}

bool CATProgram_all(uint16_t * data){
  cat_dac.setID(default_address);
  delayMicroseconds(100);
  cat_dac.analogWrite(*data,*data,*data,*data);
  delayMicroseconds(100);
  return 1;
}

bool CATChannelProgram(uint16_t data, uint8_t channel){
  cat_dac.setID(default_address);
  cat_dac.analogWrite(channel, data);
  return 1;
}


// returns 0 if one of the DAC's can't be set to 0 (and doesn't read 0).
bool POTSetup(TwoWire& wire, uint8_t LDAC){
  pot_dac.attach(wire, LDAC); // set the LDAC pin of launchpad and the i2c comms
  // Setup all DAC channels to have VDD as reference.
  //There are 8 chips each with their own addresses and 4 channels on each chip
  pot_dac.setID(default_address);
  pot_dac.selectVref(MCP4728_POT::VREF::VDD, MCP4728_POT::VREF::VDD, MCP4728_POT::VREF::VDD, MCP4728_POT::VREF::VDD);
  pot_dac.analogWrite(0,0,0,0);
//  pot_dac.readRegisters();
  uint16_t dac_val=0;
//  for(int j=0;j<4;j++) dac_val=dac_val | pot_dac.getDACData(j);
  if(dac_val==0) return 1;
  return 0;
}

bool POTProgram_all(uint16_t * data){
  pot_dac.setID(default_address);
  delayMicroseconds(100);
  pot_dac.analogWrite(*data,*data,*data,*data);
  delayMicroseconds(100);
  return 1;
}

bool POTChannelProgram(uint16_t data, uint8_t channel){
  pot_dac.setID(default_address);
  pot_dac.analogWrite(channel, data);
  return 1;
}

