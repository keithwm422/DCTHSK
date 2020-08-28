/*
 * CommandResponse.h
 * 
 * Declares a set of functions to act as responses to received commands and 
 * error protocols
 *
 */

#pragma once

#include <PacketSerial.h>
#include <Arduino.h>
#include "Wire.h"
#include "Wire_1.h"
#include "Wire_2.h"


/* If these commands are received */
void whatToDoIfISR(uint8_t * data);
							
int whatToDoIfHeaterControl(uint8_t * data, uint8_t len);

int whatToDoIfTestHeaterControl(uint8_t* data, uint8_t len, uint8_t * respData);

int whatToDoIfThermistors(uint8_t* respData);

// for Pressure ADC
bool PressureSetup(TwoWire& wire);

uint16_t PressureRead();

// for the HV programming pins.
bool CATSetup(TwoWire_1& wire, uint8_t LDAC);

bool CATProgram_all(uint16_t * data);

bool CATChannelProgram(uint16_t data, uint8_t channel);

bool POTSetup(TwoWire_2& wire, uint8_t LDAC);

bool POTProgram_all(uint16_t * data);

bool POTChannelProgram(uint16_t data, uint8_t channel);
