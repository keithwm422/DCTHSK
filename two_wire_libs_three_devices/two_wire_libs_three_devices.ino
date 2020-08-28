// try out multiple I2C twowire libs
#include "driverlib/uart.h"
#include "inc/hw_nvic.h"
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <inc/hw_i2c.h>
#include <inc/hw_memmap.h>
#include <inc/hw_types.h>
#include <inc/hw_gpio.h>
#include <inc/hw_ints.h>
#include <inc/hw_pwm.h>
#include <driverlib/i2c.h>
#include <driverlib/sysctl.h>
#include <driverlib/gpio.h>
#include <driverlib/pin_map.h>
#include <driverlib/timer.h>
#include <driverlib/interrupt.h>
#include <driverlib/rom.h>
#include <driverlib/rom_map.h>
#include <driverlib/udma.h>
#include <driverlib/pwm.h>
#include <driverlib/ssi.h>
#include <driverlib/systick.h>
#include <driverlib/adc.h>
#include <string.h>
#include "Wire.h"
#include "MCP3021.h"
#include "MCP4728_cathode.h"
#include "MCP4728_potential.h"

#define  WIRE_INTERFACES_COUNT 4
unsigned long  i2c_1=1;
unsigned long  i2c_2=2;
unsigned long i2c_3=3;
// declare 3 two wire objs
//TwoWire *wire_1= new TwoWire(i2c_1); // i2C object for the i2c port on the launchpad
//TwoWire *wire_2= new TwoWire(i2c_2); // i2C object for the i2c port on the launchpad
//TwoWire *wire_3= new TwoWire(i2c_3); // i2C object for the i2c port on the launchpad
MCP3021 pressure_adc;
MCP4728_POT pot_dac;
MCP4728_CAT cat_dac;
char one_byte;
uint8_t default_address=96;
// for Launchpad LED
#define LED GREEN_LED
#define LED_UPDATE_PERIOD 1000
unsigned long LEDUpdateTime=0; // keeping LED to visualize no hanging
bool is_high=true;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.print("Hello, starting...");
  Serial.print("\n");
  pinMode(14,OUTPUT);
  pinMode(17,OUTPUT);
// begin both twowire libs (default off LEDs!)
//  wire_2->setModule(i2c_3);
//  wire_1->setModule(i2c_1);
  Wire2.begin();
//  wire_2->begin();
//  wire_3->begin();
//  wire_1->begin();
  Wire3.begin();
//  CATSetup(*wire_1, 14);
//  PressureSetup(Wire2);
  Serial.print(PressureRead());
  POTSetup(Wire3,17);
  Serial.println("entering loop");
  pinMode(LED,OUTPUT);
  digitalWrite(LED,HIGH);
  LEDUpdateTime= millis()+ LED_UPDATE_PERIOD;
}

void loop() {
  if((long) (millis() - LEDUpdateTime) > 0){
    LEDUpdateTime+= LED_UPDATE_PERIOD;
    switch_LED();
  }
  while(Serial.available()>0){
    one_byte=Serial.read();
    Serial.print("\n");
//    Serial.print("try writing i to test_prio_array? \n");
//    memcpy(test_prio_array,&i, sizeof(i));
    if(one_byte==49) Serial.print("1\n");
    else if(one_byte==50){ //2
      CATProgram_all(0);
      Serial.print("2\n");
    }
    else if(one_byte==51){  //3
      POTProgram_all(0);
      Serial.print("3\n");
    }
    else if(one_byte == 52){  //4
      Serial.print(PressureRead());
      Serial.print("4\n");
    }
    else if(one_byte==53){ // 5
      Serial.print("5\n");
    }
    else if(one_byte==54){  //6
// read pcf1
      Serial.print(" 6\n");
    }
    else if(one_byte==55){  //7
//read pcf2
      Serial.print(" 7\n");
    }
    else {
      Serial.print("AH\n");
    }
  }
  delay(3000);
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
  cat_dac.readRegisters();
  uint16_t dac_val=0;
  for(int j=0;j<4;j++) dac_val=dac_val | cat_dac.getDACData(j);
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
  pot_dac.readRegisters();
  uint16_t dac_val=0;
  for(int j=0;j<4;j++) dac_val=dac_val | pot_dac.getDACData(j);
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
