/*
 * DCTHSK_protocol.cpp
 *
 * Declares the interface protocol for cross device communication.
 *
 * Housekeeping packet consists of header
 * 0-255 payload bytes
 * 1 byte CRCS (or checksum)
 */

/*****************************************************************************
 * Defines
 ****************************************************************************/
#pragma once
#include <stdint.h>

#define MIN_PERIOD      100

/*******************************************************************************
 * Typedef enums
 *******************************************************************************/

/* Command definitions */
typedef enum DCTHSK_cmd {
  // 2-248 are board-specific: these are test commands
	eDCTIntSensorRead = 0x02,
	eHeaterControlAll = 0x03,
	eHeaterControlChannel = 0x04,
        eThermistors = 0x05,
        eHVmon = 0x06,  // to send most recent voltages and currents from both supplies
        ePacketCount = 0x07,
        eThermistorsTest = 0x08,
        ePressure = 0x09,
        eVPGMPotential = 0x0A, //10
        eIPGMPotential = 0x0B, //11
        eVPGMCathode = 0x0C, //12
        eIPGMCathode = 0x0D, //13
        eVMONPotential = 0x0E, //14
        eIMONPotential = 0x0F, //15
        eVMONCathode = 0x10, //16
        eIMONCathode = 0x11, //17
        eHVMonConverted = 0x12, //18
        eHVMonOTEN = 0x13, //19
        eHVCatEN = 0x14, //20
        eHVPotEN = 0x15, //21
        eRampHVCat = 0x16, //22
        eRampHVPot = 0x17, //23
        eCancelRamping = 0x18, //24
        ePauseRamping = 0x19, //25
        eResumeRamping = 0x1A, //26
        eReadHeaterResponse = 0x1B, //27
        eHeaterStartupValue = 0x1C, //28
        eSetVREF = 0x1D, //29
        eDCTISR=0xA0,
} DCTHSK_cmd;

/*****DCT Hsk Structs*****/
/* DCT launchpad */
// subhsk_id=0x01, command associated with this struct: eISR
struct sDCTInternalTemp {
  float Temperature; // internal temperature sensor to uC
} __attribute__((packed));

/* DCT Thermistors */
// subhsk_id=0x03, command associated with this struct: eThermistors
struct sDCTThermistors {
  float Therms[25]; // float, Thermistors temperature
} __attribute__((packed));

/* HV supplies monitoring*/
// subhsk_id=0x03, command associated with this struct: eHVMon
struct sDCTHV {
  uint16_t CatVmon; // 12 bits ADC, Cathode HV supply Vmon
  uint16_t CatImon; // 12 bits ADC, Cathode HV supply Imon
  uint16_t PotVmon; // 12 bits ADC, Potential HV supply Vmon
  uint16_t PotImon; // 12 bits ADC, Potential HV supply Imon
} __attribute__((packed));

/* DCT Pressure Transducer */
// subhsk_id=0x03, command associated with this struct: ePressure
struct sDCTPressure {
  uint16_t Pressure_vessel; // 10 bits ADC, Pressure Transducer vessel reading from IC
  uint16_t Pressure_vacuumref; // 12 bits ADC, Pressure Transducer vacuum ref. reading on launchpad
} __attribute__((packed));

/* HV supplies monitoring*/
// subhsk_id=0x03, command associated with this struct: eHVMonConverted
struct sDCTHVConverted {
  float CatV; // 12 bits ADC converted to kV-Cathode HV supply
  float CatI; // 12 bits ADC, converted to mA-Cathode HV supply
  float PotV; // 12 bits ADC, converted to kV-Potential HV supply
  float PotI; // 12 bits ADC, converted to mA-Potential HV supply
} __attribute__((packed));

/*DCTHSK priority*/

struct sDCTLow {
  sDCTInternalTemp DCT_internal_temp;
} __attribute__((packed));

struct sDCTMed {
  sDCTHV DCT_hv;
} __attribute__((packed));

struct sDCTHi {
  sDCTThermistors DCT_thermistors;
  sDCTPressure DCT_pressure;
} __attribute__((packed));
