/*!



LTC2983.ino:
Generated Linduino code from the LTC2983 Demo Software.
This code (plus the other code in this folder) is designed to be used by a Linduino,
but can also be used to understand how to program the LTC2983.




http://www.analog.com/en/products/analog-to-digital-converters/integrated-special-purpose-converters/digital-temperature-sensors/LTC2983.html

http://www.analog.com/en/products/analog-to-digital-converters/integrated-special-purpose-converters/digital-temperature-sensors/LTC2983#product-evaluationkit

$Revision: 1.7.9 $
$Date: September 7, 2018 $
Copyright (c) 2018, Analog Devices, Inc. (ADI)
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies,
either expressed or implied, of Analog Devices, Inc.

The Analog Devices Linduino is not affiliated with the official Arduino team.
However, the Linduino is only possible because of the Arduino team's commitment
to the open-source community.  Please, visit http://www.arduino.cc and
http://store.arduino.cc , and consider a purchase that will help fund their
ongoing work.
*/





#include <Arduino.h>
#include <stdint.h>
#include <stdbool.h>
//#include "Wire.h"
#include "stdio.h"
#include "math.h"

#include "LTC2983_configuration_constants.h"
#include "LTC2983_support_functions.h"
#include "LTC2983_table_coeffs.h"

const unsigned long CHIP_SELECT_E = 12; //usually denoted cs
const unsigned long CHIP_SELECT_B = 30;
const unsigned long CHIP_SELECT_C = 31;
const unsigned long CHIP_SELECT_D = 36;
const unsigned long CHIP_SELECT_A = 7;
//const unsigned long CHIP_SELECT=CHIP_SELECT_B;  // which one to test.
// Function prototypes
void configure_channels();
void configure_global_parameters();

// for Launchpad LED
#define LED GREEN_LED
#define LED_UPDATE_PERIOD 1000
unsigned long LEDUpdateTime=0; // keeping LED to visualize no hanging
bool is_high=true;
// -------------- Configure the LTC2983 -------------------------------
void setup() 
{

  delay(200);
  pinMode(CHIP_SELECT_A, OUTPUT); // Configure chip select pin on Linduino

 // pinMode(CHIP_SELECT, OUTPUT);
  pinMode(CHIP_SELECT_B, OUTPUT);
  pinMode(CHIP_SELECT_C, OUTPUT);
  pinMode(CHIP_SELECT_D, OUTPUT);
  pinMode(CHIP_SELECT_E, OUTPUT);
  // set all to high!
  set_CS_all_high();
  Serial.begin(115200);         // Initialize the serial port to the PC
    Serial.println("hello");
  delay(1000);
  initialize_TM4C(CHIP_SELECT_A);
  delay(1000);
  print_title();
  configure_channels(CHIP_SELECT_A);
  configure_global_parameters(CHIP_SELECT_A);
      Serial.println(CHIP_SELECT_A);
  configure_channels(CHIP_SELECT_B);  
  configure_global_parameters(CHIP_SELECT_B);
      Serial.println(CHIP_SELECT_B);
  configure_channels(CHIP_SELECT_C);  
  configure_global_parameters(CHIP_SELECT_C);
      Serial.println(CHIP_SELECT_C);
  configure_channels(CHIP_SELECT_D);  
  configure_global_parameters(CHIP_SELECT_D);
      Serial.println(CHIP_SELECT_D);
  configure_channels(CHIP_SELECT_E);  
  configure_global_parameters(CHIP_SELECT_E);
      Serial.println(CHIP_SELECT_E);		

      Serial.println("hello3");
  // LED on launchpad
  pinMode(LED, OUTPUT);
  digitalWrite(LED,HIGH);
      LEDUpdateTime= millis()+ LED_UPDATE_PERIOD;

}


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

// -------------- Run the LTC2983 -------------------------------------

void loop()
{
  if((long) (millis() - LEDUpdateTime) > 0){
    LEDUpdateTime+= LED_UPDATE_PERIOD;
    switch_LED();
  }
     //measure_channel(CHIP_SELECT, 4, TEMPERATURE);      // Ch 4: Thermistor 44006 10K@25C
  if(Serial.available()){
    char one_byte=Serial.read();
    if (one_byte==65){
      Serial.print(measure_channel(CHIP_SELECT_A, 4, TEMPERATURE),2);
    }
    else if(one_byte==66) {
      measure_channel(CHIP_SELECT_B, 8, TEMPERATURE);
    }
    else if(one_byte==67) {
      measure_channel(CHIP_SELECT_C, 8, TEMPERATURE);
    }
    else if(one_byte==68) {
      measure_channel(CHIP_SELECT_D, 8, TEMPERATURE);
    }
    else if(one_byte==69)measure_channel(CHIP_SELECT_E, 8, TEMPERATURE);
  }
 // measure_channel(CHIP_SELECT, 4, TEMPERATURE);      // Ch 4: Thermistor 44006 10K@25C
 // measure_channel(CHIP_SELECT, 8, TEMPERATURE);      // Ch 8: Thermistor 44006 10K@25C
 // measure_channel(CHIP_SELECT, 12, TEMPERATURE);     // Ch 12: Thermistor 44006 10K@25C
 // measure_channel(CHIP_SELECT, 16, TEMPERATURE);     // Ch 16: Thermistor 44006 10K@25C
 // measure_channel(CHIP_SELECT, 20, TEMPERATURE);     // Ch 20: Thermistor 44006 10K@25C
}

void switch_LED(){
  if(is_high){
    is_high=false;
    digitalWrite(LED,LOW);
    //digitalWrite(CHIP_SELECT,LOW);
  }
  else{    
    is_high=true;
    digitalWrite(LED,HIGH);
    //digitalWrite(CHIP_SELECT,HIGH);
  }
}
void set_CS_all_high(){
  digitalWrite(CHIP_SELECT_A, HIGH);
  digitalWrite(CHIP_SELECT_B, HIGH);
  digitalWrite(CHIP_SELECT_C, HIGH);
  digitalWrite(CHIP_SELECT_D, HIGH);
  digitalWrite(CHIP_SELECT_E, HIGH);

}
