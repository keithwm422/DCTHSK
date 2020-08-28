#include <SPI.h>  // include the SPI library
#include "driverlib/uart.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_nvic.h"
#include "inc/hw_flash.h"
#include "driverlib/sysctl.h"
#include "configuration_constants.h"
#include "DCT_SPI_support_functions.h"
#include "config_functions.h"
///////////////


// *****************
// Configuration functions copied directly from the evalprom c-generated code
// *****************

void configure_channels(uint8_t cs)
{
  uint8_t channel_number;
  uint32_t channel_assignment_data;

  // ----- Channel 2: Assign Sense Resistor -----
  channel_assignment_data = 
    SENSOR_TYPE__SENSE_RESISTOR |
    (uint32_t) 0x9C4000 << SENSE_RESISTOR_VALUE_LSB;		// sense resistor - value: 10000.
  assign_channel(cs, 2, channel_assignment_data);
  // ----- Channel 4: Assign Thermistor 44006 10K@25C -----
  channel_assignment_data = 
    SENSOR_TYPE__THERMISTOR_44006_10K_25C |
    THERMISTOR_RSENSE_CHANNEL__2 |
    THERMISTOR_DIFFERENTIAL |
    THERMISTOR_EXCITATION_MODE__NO_SHARING_NO_ROTATION |
    THERMISTOR_EXCITATION_CURRENT__AUTORANGE;
  assign_channel(cs, 4, channel_assignment_data);
  // ----- Channel 6: Assign Sense Resistor -----
  channel_assignment_data = 
    SENSOR_TYPE__SENSE_RESISTOR |
    (uint32_t) 0x9C4000 << SENSE_RESISTOR_VALUE_LSB;		// sense resistor - value: 10000.
  assign_channel(cs, 6, channel_assignment_data);
  // ----- Channel 8: Assign Thermistor 44006 10K@25C -----
  channel_assignment_data = 
    SENSOR_TYPE__THERMISTOR_44006_10K_25C |
    THERMISTOR_RSENSE_CHANNEL__6 |
    THERMISTOR_DIFFERENTIAL |
    THERMISTOR_EXCITATION_MODE__NO_SHARING_NO_ROTATION |
    THERMISTOR_EXCITATION_CURRENT__AUTORANGE;
  assign_channel(cs, 8, channel_assignment_data);
  // ----- Channel 10: Assign Sense Resistor -----
  channel_assignment_data = 
    SENSOR_TYPE__SENSE_RESISTOR |
    (uint32_t) 0x9C4000 << SENSE_RESISTOR_VALUE_LSB;		// sense resistor - value: 10000.
  assign_channel(cs, 10, channel_assignment_data);
  // ----- Channel 12: Assign Thermistor 44006 10K@25C -----
  channel_assignment_data = 
    SENSOR_TYPE__THERMISTOR_44006_10K_25C |
    THERMISTOR_RSENSE_CHANNEL__10 |
    THERMISTOR_DIFFERENTIAL |
    THERMISTOR_EXCITATION_MODE__NO_SHARING_NO_ROTATION |
    THERMISTOR_EXCITATION_CURRENT__AUTORANGE;
  assign_channel(cs, 12, channel_assignment_data);
  // ----- Channel 14: Assign Sense Resistor -----
  channel_assignment_data = 
    SENSOR_TYPE__SENSE_RESISTOR |
    (uint32_t) 0x9C4000 << SENSE_RESISTOR_VALUE_LSB;		// sense resistor - value: 10000.
  assign_channel(cs, 14, channel_assignment_data);
  // ----- Channel 16: Assign Thermistor 44006 10K@25C -----
  channel_assignment_data = 
    SENSOR_TYPE__THERMISTOR_44006_10K_25C |
    THERMISTOR_RSENSE_CHANNEL__14 |
    THERMISTOR_DIFFERENTIAL |
    THERMISTOR_EXCITATION_MODE__NO_SHARING_NO_ROTATION |
    THERMISTOR_EXCITATION_CURRENT__AUTORANGE;
  assign_channel(cs, 16, channel_assignment_data);
  // ----- Channel 18: Assign Sense Resistor -----
  channel_assignment_data = 
    SENSOR_TYPE__SENSE_RESISTOR |
    (uint32_t) 0x9C4000 << SENSE_RESISTOR_VALUE_LSB;		// sense resistor - value: 10000.
  assign_channel(cs, 18, channel_assignment_data);
  // ----- Channel 20: Assign Thermistor 44006 10K@25C -----
  channel_assignment_data = 
    SENSOR_TYPE__THERMISTOR_44006_10K_25C |
    THERMISTOR_RSENSE_CHANNEL__18 |
    THERMISTOR_DIFFERENTIAL |
    THERMISTOR_EXCITATION_MODE__NO_SHARING_NO_ROTATION |
    THERMISTOR_EXCITATION_CURRENT__AUTORANGE;
  assign_channel(cs, 20, channel_assignment_data);

}

void configure_global_parameters(uint8_t cs) 
{
  // -- Set global parameters
  transfer_byte(cs, WRITE_TO_RAM, 0xF0, TEMP_UNIT__C |
    REJECTION__50_60_HZ);
  // -- Set any extra delay between conversions (in this case, 0*100us)
  transfer_byte(cs, WRITE_TO_RAM, 0xFF, 0);
}

//  measure_channel(CHIP_SELECT, 4, TEMPERATURE);      // Ch 4: Thermistor 44006 10K@25C
//  measure_channel(CHIP_SELECT, 8, TEMPERATURE);      // Ch 8: Thermistor 44006 10K@25C
//  measure_channel(CHIP_SELECT, 12, TEMPERATURE);     // Ch 12: Thermistor 44006 10K@25C
//  measure_channel(CHIP_SELECT, 16, TEMPERATURE);     // Ch 16: Thermistor 44006 10K@25C
//  measure_channel(CHIP_SELECT, 20, TEMPERATURE);     // Ch 20: Thermistor 44006 10K@25C
