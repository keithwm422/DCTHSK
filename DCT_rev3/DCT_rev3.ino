/*
 * DCTHSK_prototype.ino
 * 
 * Initiates serial ports & follows HSK protocol for command responses and error
 * reporting. This program can be used on other devices by changing the device
 * address (myID) and the upStream serial connection (direct line to the SFC)
 *
 * CONSTANTS:
 * --PACKETMARKER is defined in Cobs_encoding.h
 * --MAX_PACKET_LENGTH is defined in PacketSerial
 * --NUM_LOCAL_CONTROLS is defined here
 * --FIRST_LOCAL_COMMAND is defined here
 * --TEST_MODE_PERIOD is defined here
 * --BAUD rates are defined here
 *
 */

#include <Core_protocol.h>
#include <PacketSerial.h>
#include <driverlib/sysctl.h>
#include <Hsk_all_data_types.h>
/* These are device specific */
#include "src/DCTHSK_lib/DCTHSK_protocol.h"
// from magnet hsk for LTC2983

#include "driverlib/uart.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_nvic.h"
#include "inc/hw_flash.h"
#include "driverlib/sysctl.h"
#include "src/DCTHSK_lib/configuration_constants.h"
#include "src/DCTHSK_lib/DCT_SPI_support_functions.h"
#include "src/DCTHSK_lib/Wire.h"
#include "src/DCTHSK_lib/DCTHSK_support_functions.h"
#include "src/DCTHSK_lib/config_functions.h"
// these files above need to be changed based on the thermistor or rtd, etc. So if we borrow the examples from magnet hsk then we can change the channels and types in just these files but keep the function the same. 
//////////////////////////////////////
#define DOWNBAUD 115200 // Baudrate to the SFC
#define UPBAUD 19200    // Baudrate to upsteam devices
#define TEST_MODE_PERIOD 100 // period in milliseconds between testmode packets being sent
#define FIRST_LOCAL_COMMAND 2 // value of hdr->cmd that is the first command local to the board
#define NUM_LOCAL_CONTROLS 8 // how many commands total are local to the board
#define MAX519 32 // testing DAC
#define MCP_4728_CATHODE 96  // 7-bit slave address for DACs
#define MCP_4728_POTENTIAL 96  // 7-bit slave address for DACs
#define SENSOR_UPDATE_PERIOD 1000// how often to check/write sensors
#define  WIRE_INTERFACES_COUNT 4   // needed for multiple instances of TwoWire
/* Declare instances of PacketSerial to set up the serial lines */
PacketSerial downStream1; // mainHSK

/* Name of this device */
housekeeping_id myID = eDCTHsk;

/* Outgoing buffer, for up or downstream. Only gets used once a complete packet
 * is received -- a command or forward is executed before anything else happens,
 * so there shouldn't be any over-writing here. */
uint8_t outgoingPacket [MAX_PACKET_LENGTH] ={0}; 

/* Use pointers for all device's housekeeping headers and the autopriorityperiods*/
housekeeping_hdr_t * hdr_in;     housekeeping_hdr_t * hdr_out;
housekeeping_err_t * hdr_err;   housekeeping_prio_t * hdr_prio;
/* Memory buffers for housekeeping system functions */
uint8_t numDevices = 0;           // Keep track of how many devices are upstream
uint8_t commandPriority[NUM_LOCAL_CONTROLS] = {1,0,0,3,2,0,0,3};     // Each command's priority takes up one byte
PacketSerial *serialDevices = &downStream1;
uint8_t addressList = 0; // List of all downstream devices

/* Utility variables for internal use */
size_t hdr_size = sizeof(housekeeping_hdr_t)/sizeof(hdr_out->src); // size of the header
uint8_t numSends = 0; // Used to keep track of number of priority commands executed
uint16_t currentPacketCount=0;
static_assert(sizeof(float) == 4);
unsigned long timelastpacket; //for TestMode
/*Structs for storing most current reads*/
sDCTThermistors thermistors;

uint32_t ADCValues[1];

/* SENSOR STATES AND VARS */
// I2C port map
const int port_cathode=1;
const int port_potential=3;
const int port_pressure=2;
TwoWire * wire_cathode= new TwoWire(port_cathode);
TwoWire * wire_pressure= new TwoWire(port_pressure);
TwoWire * wire_potential= new TwoWire(port_potential);
// I2C wire class (nonblocking writes and reads)
// 10 bit ADC read from i2c.
sDCTPressure dct_pressure;
#define PRESSURE_UPDATE_PERIOD 2000
unsigned long pressureUpdateTime=0; // keeping time for the sensor check/write

// FOR HV (channel D(3) of MCP4728 is VPGM (for both) and channel A(0) is IPGM (for both).
uint8_t cat_LDAC=14; // pin 2 of launchpad is the LDAC pin we use here. 
uint8_t pot_LDAC=17; // pin 2 of launchpad is the LDAC pin we use here. 
unsigned long hvmonUpdateTime=0; // keeping time for the sensor check/write
#define HVMON_UPDATE_PERIOD 1000
uint16_t voltage_potential=0;
uint16_t voltage_cathode=0;
uint16_t current_potential=0;
uint16_t current_cathode=0;
int OT_POT = 19;
int EN_POT=18;
int OT_CAT=38; // DCTHSK board needs jumper from pos 16 pad to pos 38 pad and remove the pin (D'OH- pin 16 is RESET pin and is always high).
int EN_CAT=15;
uint32_t hv_read;
uint8_t which_adc = 0;
sDCTHV hvmon;
sDCTHVConverted hvmonconverted;
bool is_cathode_disabled=true;
bool is_potential_disabled=true;
int ot_cathode_read=0;
int ot_potential_read=0;
// convert the ADC reads to a voltage (and to a mA or kV).
float pres_val=0;
float cat_v=0;
float conversion_factor=3.3*1406.0/(1000.0*4096.0); // each ADC has a voltage divider and is 12-bits (3.3V FSR) so thats these numbers. 
float conv_factor_HV=conversion_factor*(1000.0/4.64);
float conv_factor_current=conversion_factor*(1.5/4.64);
// for ramping and pausing or resuming the ramping
bool step_HV_C=false; // for storing the status of ramping or not ramping
bool step_HV_P=false;
unsigned long step_HV_C_time=0;
unsigned long step_HV_P_time=0;
#define CATHODE_VOLTAGE_CUTOFF 4000
#define CATHODE_VOLTAGE_STEP_PERIOD 5000 // five seconds
#define CATHODE_VOLTAGE_STEP_VALUE 130 // in DAC counts (12-bit DAC)
#define POTENTIAL_VOLTAGE_CUTOFF 4000
#define POTENTIAL_VOLTAGE_STEP_PERIOD 5000 // five seconds
#define POTENTIAL_VOLTAGE_STEP_VALUE 130 // in DAC counts
// For thermistors
#define NUM_THERMS 25
//float thermistor_temps[NUM_THERMS]={0};
// there are five thermistors per chip_select, so need to cycle five times on the chip then go on to the next chip (or mod 5 it?)
#define THERMS_UPDATE_PERIOD 1000
unsigned long thermsUpdateTime=0; // keeping time for the sensor check/write
int counter=0; // counter 0,4 are CSA, 5,9 are CSB, 10,14 C, 15,19 D 20,24 E. 
int chip_to_read=0;
uint8_t temp_channels[5]={4,8,12,16,20};
int counter_all=0;
// So switch chip select when counter+1 %5==0?? (and then if counter==24 set counter back to 0).

// for heater sheets
uint8_t change_all[3]={254,171,0}; // default to zero
uint8_t change_one[4]={254,170,0,0};
uint8_t potentiometer1_says[100];
uint8_t potentiometer2_says[100];
int pot_1_iter=0;
int pot_2_iter=0;

// for Launchpad LED
#define LED GREEN_LED
#define LED_UPDATE_PERIOD 1350
unsigned long LEDUpdateTime=0; // keeping LED to visualize no hanging
bool is_high=true;
/*******************************************************************************
* Main program
*******************************************************************************/
void setup()
{
  // to DEBUG this device when connecting directly to computer, use this serial port instead of Serial.1
  //Serial.begin(DOWNBAUD);
  //downStream1.setStream(&Serial);
  //downStream1.setPacketHandler(&checkHdr);
  // Serial port for downstream to Main HSK
  Serial1.begin(DOWNBAUD);
  downStream1.setStream(&Serial1);
  downStream1.setPacketHandler(&checkHdr);
  // LED on launchpad
  pinMode(LED, OUTPUT);
  digitalWrite(LED,HIGH);
  Serial3.begin(UPBAUD); //Pot A serial port
  Serial2.begin(UPBAUD); //Pot B serial port
  // Point to data in a way that it can be read as a header
  hdr_out = (housekeeping_hdr_t *) outgoingPacket;
  hdr_err = (housekeeping_err_t *) (outgoingPacket + hdr_size);
  currentPacketCount=0;
  // functions begin SPI connections, should only need one SPI thingy
//
//  InitADC();
  analogReadResolution(12);
// 4 ADCs used for the HV monitoring: they are pins 26 (Potential field V Mon),27 (Potential field I Mon),28 (Cathode field V Mon),29 (Cathode field I Mon)
// These are, in order, A4,A2,A1,A0 (ADC labels from TM4C launchpad). 
// HV System has Binary pins also: OverTemp Fault, and Enable Input: 19 (OT Potential Field), 18 (EN Potential Field), 16 (OT Cathode Field), 15 (EN Cathode Field)
// like this: pinMode(15,INPUT);
  pinMode(OT_POT,INPUT);
  pinMode(EN_POT,OUTPUT);
  pinMode(OT_CAT,INPUT);
  pinMode(EN_CAT,OUTPUT);
  digitalWrite(EN_CAT,LOW);
  digitalWrite(EN_POT,LOW);
// The DACs on I2C on ports 1 and 3 have an LDAC pin that goes to 17 (LDAC Potential DAC) and 14 (LDAC Cathode DAC) 
// which can be used if programming the DACs address and updating output registers.
  pinMode(pot_LDAC,OUTPUT);
  pinMode(cat_LDAC,OUTPUT);
  wire_pressure->begin();
  wire_potential->begin(); // pass the I2C ports number 0=not used, 1=cathode pgm, 2=pressure, 3=potential pgm
  wire_cathode->begin();
  // run setup of I2C stuff from support_functions
  POTSetup(*wire_potential,pot_LDAC);
  CATSetup(*wire_cathode, cat_LDAC);
  PressureSetup(*wire_pressure);
  delay(100);
  // Setup thermistors reads
  // chip selects setup here and set to all high
  pinMode(CHIP_SELECT_A, OUTPUT);
  pinMode(CHIP_SELECT_B, OUTPUT);
  pinMode(CHIP_SELECT_C, OUTPUT);
  pinMode(CHIP_SELECT_D, OUTPUT);
  pinMode(CHIP_SELECT_E, OUTPUT);
  // set all CS to high
  set_CS_all_high();
  delay (1000);
  Initialize_TM4C123();
  //delay(200);
  // Setup Thermistors after Initiliazing the SPI and chip selects
  
  configure_channels((uint8_t)CHIP_SELECT_A);
  configure_global_parameters((uint8_t)CHIP_SELECT_A);
  configure_channels((uint8_t)CHIP_SELECT_B);
  configure_global_parameters((uint8_t)CHIP_SELECT_B);
  configure_channels((uint8_t)CHIP_SELECT_C);
  configure_global_parameters((uint8_t)CHIP_SELECT_C);
  configure_channels((uint8_t)CHIP_SELECT_D);
  configure_global_parameters((uint8_t)CHIP_SELECT_D);
  configure_channels((uint8_t)CHIP_SELECT_E);
  configure_global_parameters((uint8_t)CHIP_SELECT_E);
  
  digitalWrite(LED,LOW);

}

/*******************************************************************************
 * Main program
 ******************************************************************************/
void loop()
{
  // see if potentiometers have said anything recently and just store the most recent byte
  if(Serial3.available()){
        potentiometer1_says[pot_1_iter]=Serial3.read();
        pot_1_iter++;
  }
  if(Serial2.available()){
        potentiometer2_says[pot_2_iter]=Serial2.read();
        pot_2_iter++;
  }
  if(pot_1_iter>99) pot_1_iter=0;
  if(pot_2_iter>99) pot_2_iter=0;

  // can try this code first if desired, but blocking should be fine for the DCT HSK
  

  // secondary code is simply blocking version with wire.
  // read in thermistors
  // do a read based on timer and for the counter and chip_to_read
  // channel number is the pin number from the ltc2983 reading method. 4,8,12,16,20.
  // if counter is 0,5,10,15,20 then read 4, if 1,6,11,16,21 then 8, etc.
  if((long) (millis() - thermsUpdateTime) > 0){
    thermsUpdateTime+= THERMS_UPDATE_PERIOD;
   /* switch(chip_to_read){
      case 0:
        thermistors.Therms[counter_all] = measure_channel((uint8_t)CHIP_SELECT_A, temp_channels[counter],TEMPERATURE);
        break;
      case 1:
        thermistors.Therms[counter_all] = measure_channel((uint8_t)CHIP_SELECT_B, temp_channels[counter],TEMPERATURE);
        break;
      case 2:
        thermistors.Therms[counter_all] = measure_channel((uint8_t)CHIP_SELECT_C, temp_channels[counter],TEMPERATURE);
        break;
      case 3:
        thermistors.Therms[counter_all] = measure_channel((uint8_t)CHIP_SELECT_D, temp_channels[counter],TEMPERATURE);
        break;
      case 4:
        thermistors.Therms[counter_all] = measure_channel((uint8_t)CHIP_SELECT_E, temp_channels[counter],TEMPERATURE);
        break;
    }
    */
    counter++;
    counter_all++;
    if(counter>=5){
      chip_to_read++;
      counter=0;
    }
    if(chip_to_read>=5) chip_to_read=0;
    if(counter_all>=25) counter_all=0;    
    thermistors.Therms[0] = measure_channel((uint8_t)CHIP_SELECT_A, temp_channels[0],TEMPERATURE);
  }
  
  // read in HV monitoring
  if((long) (millis() - hvmonUpdateTime) > 0){
    hvmonUpdateTime+= HVMON_UPDATE_PERIOD;
    switch(which_adc){
      case 0: hvmon.PotVmon=(uint16_t) analogRead(A4); // VMON potential
      case 1: hvmon.PotImon=(uint16_t) analogRead(A2); // IMON potential
      case 2: hvmon.CatVmon=(uint16_t) analogRead(A1); // VMON cathode
      case 3: hvmon.CatImon=(uint16_t) analogRead(A0); // IMON cathode
    }
    which_adc++;
    if(which_adc>=4) which_adc=0;
    ot_potential_read=digitalRead(OT_POT);
    ot_cathode_read=digitalRead(OT_CAT);
  }
  // read in pressure
  if((long) (millis() - pressureUpdateTime) > 0){
    pressureUpdateTime+= PRESSURE_UPDATE_PERIOD;
    dct_pressure.Pressure=PressureRead();
  }
  if((long) (millis() - LEDUpdateTime) > 0){
    LEDUpdateTime+= LED_UPDATE_PERIOD;
    switch_LED();
  }

  // RAMPING STUFF:
  if(step_HV_C){
    if((int) voltage_cathode >= CATHODE_VOLTAGE_CUTOFF){
      step_HV_C=false;
      //Serial.println("Ramping of Cathode wires complete, cutoff reached");
      // should just stay at this value and then can be overwritten by the Cxxx command
     // do i need some more stuff?
    }
    else{
      if((long) (millis() - step_HV_C_time) > 0){
        step_HV_C_time+= CATHODE_VOLTAGE_STEP_PERIOD;
        voltage_cathode+=CATHODE_VOLTAGE_STEP_VALUE;
        if((int) voltage_cathode <= CATHODE_VOLTAGE_CUTOFF) CATChannelProgram(voltage_cathode, 3);
        else{
          step_HV_C=false;
          //Serial.println("Ramping of Cathode wires complete, cutoff reached");
        }
      }
    }
  }
  if(step_HV_P){
    if((int) voltage_potential >= POTENTIAL_VOLTAGE_CUTOFF){
      step_HV_P=false;
      //Serial.println("Ramping of Potential wires complete, cutoff reached");
      // should just stay at this value and then can be overwritten by the Cxxx command
     // do i need some more stuff?
    }
    else{
      if((long) (millis() - step_HV_P_time) > 0){
        step_HV_P_time+= POTENTIAL_VOLTAGE_STEP_PERIOD;
        voltage_potential+=POTENTIAL_VOLTAGE_STEP_VALUE;
        if((int) voltage_potential <= POTENTIAL_VOLTAGE_CUTOFF) POTChannelProgram(voltage_potential, 3);
        else {
          step_HV_P=false;
          //Serial.println("Ramping of Potential wires complete, cutoff reached");
        }
      }
    }
  }
  // for debugging just one channel at a time
//  if((long) (millis() - thermsUpdateTime) > 0){
 //   thermsUpdateTime+= THERMS_UPDATE_PERIOD;
  //  thermistors.Therms[counter_all] = return_temperature_2((uint8_t)CHIP_SELECT_E, temp_channels[counter]);
 // }
  /* Continuously read in one byte at a time until a packet is received */
  if (downStream1.update() != 0) badPacketReceived(&downStream1);
}
/*******************************************************************************
 * Packet handling functions
 *******************************************************************************/
void checkHdr(const void *sender, const uint8_t *buffer, size_t len) {
  // Default header & error data values
  hdr_out->src = myID;          // Source of data packet
  hdr_in = (housekeeping_hdr_t *)buffer;
  hdr_prio = (housekeeping_prio_t *) (buffer + hdr_size);
    // If an error occurs at this device from a message
  if (hdr_in->dst == eBroadcast || hdr_in->dst==myID) hdr_err->dst = myID;
  else hdr_err->dst = hdr_in->dst;
  // If the checksum didn't match, throw a bad args error
  // Check for data corruption
  if (!(verifyChecksum((uint8_t *) buffer))) {
      //error_badArgs(hdr_in, hdr_out, hdr_err);  
      buildError(hdr_err, hdr_out, hdr_in, EBADARGS);
      fillChecksum((uint8_t *) outgoingPacket);
      downStream1.send(outgoingPacket, hdr_size + hdr_out->len + 1);
      currentPacketCount++;
  }
  else {
  // Check if the message is a broadcast or local command and only then execute it. 
    if (hdr_in->dst == eBroadcast || hdr_in->dst==myID) {
      if(hdr_in->cmd==eTestMode) handleTestMode(hdr_in, (uint8_t *) hdr_in + hdr_size, (uint8_t *) outgoingPacket);
      else if ((int)(hdr_in->cmd < 254) && (int)(hdr_in->cmd > 249)) handlePriority(hdr_in->cmd, (uint8_t *) outgoingPacket); // for doing a send of priority type.
      else handleLocalCommand(hdr_in, (uint8_t *) hdr_in + hdr_size, (uint8_t *) outgoingPacket); // this constructs the outgoingpacket when its a localcommand and sends the packet.
    } 
  // If the message wasn't meant for this device pass it along (up is away from SFC and down and is to SFC
    else forwardDown(buffer, len, sender);
  }
}
// forward downstream to the SFC
void forwardDown(const uint8_t * buffer, size_t len, const void * sender) {
  downStream1.send(buffer, len);
  checkDownBoundDst(sender);
  currentPacketCount++;
}

/* checkDownBoundDst Function flow:
 * --Checks to see if the downstream device that sent the message is known
 *    --If not, add it to the list of known devices
 *    --If yes, just carry on
 * --Executed every time a packet is received from downStream
 * 
 * Function params:
 * sender:    PacketSerial instance (serial line) where the message was received
 * 
 */
void checkDownBoundDst(const void * sender) {
  if (serialDevices == (PacketSerial *) sender){
    if (addressList == 0) {
      addressList = (uint8_t) hdr_in->src;
      numDevices++;
      return;
    }
  }
}
/* Function flow:
 * --Find the device address that produced the error
 * --Execute the bad length function & send the error to the SFC
 * Function params:
 * sender:    PacketSerial instance which triggered the error protocol
 * Send an error if a packet is unreadable in some way */
void badPacketReceived(PacketSerial * sender){
  if (sender == serialDevices){
    hdr_in->src = addressList;
  }
  hdr_out->src = myID;
  buildError(hdr_err, hdr_out, hdr_in, EBADLEN);
  fillChecksum((uint8_t *) outgoingPacket);
  downStream1.send(outgoingPacket, hdr_size + hdr_out->len + 1);
  currentPacketCount++;
}

// Function for building the error packets to send back when an error is found (see the Core_Protocol.h for the defs of the errors and the error typdefs).
void buildError(housekeeping_err_t *err, housekeeping_hdr_t *respHdr, housekeeping_hdr_t * hdr, int error){
  respHdr->cmd = eError;
  respHdr->len = 4;
  err->src = hdr->src;
  err->dst = hdr->dst;
  err->cmd = hdr->cmd;
  err->error = error;
}
/***********************
/*******************************************************************************
 * END OF Packet handling functions
 *******************************************************************************/

// sending priority command function
// probably can be cleaned up
// Note: SendAll is 253 and SendLow is 250 so we made SendLow-> int priority=1 for checking the device's list of command's priorities.
// got a priority request from destination dst
void handlePriority(uint8_t prio_in, uint8_t * responsePacketBuffer){
  housekeeping_hdr_t *respHdr = (housekeeping_hdr_t *) responsePacketBuffer;
  uint8_t *respData = responsePacketBuffer + hdr_size;
  int priority=0;
  int retval = 0;
  uint8_t sum = 0; // hdr length of data atatched from all those commands data
//  respHdr->cmd = hdr_in->cmd;
  // priority == 4 when this function is called is code for "eSendAll"
  // otherwise priority=1,2,3 and that maps to eSendLowPriority+priority
  if(prio_in==eSendAll) priority=4;
  else priority = prio_in - 249;
//  int retval;
  respHdr->src = myID;
  respHdr->dst = eSFC;
  respHdr->cmd =  prio_in;
  // go through every priority
  for (int i=0;i<NUM_LOCAL_CONTROLS;i++) {
    if (commandPriority[i] == (uint8_t)priority || priority==4) {
      retval=handleLocalRead((uint8_t) i + FIRST_LOCAL_COMMAND, respData+sum);
      // if that read overflowed the data???? fix later?
      sum+= (uint8_t) retval;
    }
    else sum+=0;
  }
  respHdr->len=sum;
  fillChecksum(responsePacketBuffer);
  downStream1.send(responsePacketBuffer, respHdr->len + hdr_size + 1);
  currentPacketCount++;
}

// function for when a "SetPriority" command is received by this device, adding that commands priority value to the array/list
void setCommandPriority(housekeeping_prio_t * prio, uint8_t * respData, uint8_t len) {
//  housekeeping_prio_t * set_prio = (housekeeping_prio_t *) prio;
  commandPriority[prio->command-FIRST_LOCAL_COMMAND] = (uint8_t) prio->prio_type;
  memcpy(respData, (uint8_t*)prio, len);
}
// Fn to handle a local command write.
// This gets called when a local command is received
// with data (len != 0).
int handleLocalWrite(uint8_t localCommand, uint8_t * data, uint8_t len, uint8_t * respData) {
  int retval = 0;
  switch(localCommand) {
  case eSetPriority:
    setCommandPriority((housekeeping_prio_t *)data,respData,len);
    retval=len;
    break;
  case eHeaterControlAll:{
    //uint8_t pot_array[3]={254,171,*data}; // real potentiometer code
    //Serial3.write(pot_array,3);
    uint8_t pot_array[4]={254,171,*data,13}; // emulator code (add carriage return to the end)
    Serial3.write(pot_array,4);
    Serial2.write(pot_array,4);
    retval = 0;
    break;
  }
  case eHeaterControlChannel:{
    //uint8_t pot_array[4]={254,170,*data,*(data+1)}; // real potentiometer
    uint8_t pot_array[5]={254,170,*data,*(data+1),13}; // emulator code (add carriage return to the end)
    if(pot_array[2] >=0 && pot_array[2]<24){
      //Serial3.write(pot_array,4);
      //Serial2.write(pot_array,4);
      Serial3.write(pot_array,5);
      Serial2.write(pot_array,5);
    }
    retval = 0;
    break;
  }
  case eVPGMPotential:{
    // write to I2C? should be 12-bit DAC, MCP4728, aka two bytes (not more than 4096 in total value).
    memcpy((uint8_t * ) &voltage_potential, data, len);
    if(((int) voltage_potential <= 4095) && ((int) voltage_potential >= 0)){
      POTChannelProgram(voltage_potential, 3);
      memcpy(respData,(uint8_t *) &voltage_potential, len);
    }
    else {
      voltage_potential=65535;  // set to all ones if the value was too high for the 12bit DAC
      memcpy(respData,(uint8_t *) &voltage_potential, len);
    }
    retval=len;
    break;
  }
  case eIPGMPotential:{
    // write to I2C? should be 12-bit DAC, MCP4728, aka two bytes (not more than 4096 in total value).
    memcpy((uint8_t * ) &current_potential, data, len);
    if(((int) current_potential <= 4095) && ((int) current_potential >= 0)){
      POTChannelProgram(current_potential, 0);
      memcpy(respData,(uint8_t *) &current_potential, len);
    }
    else {
      current_potential=65535;  // set to all ones if the value was too high for the 12bit DAC
      memcpy(respData,(uint8_t *) &current_potential, len);
    }
    retval=len;
    break;
  }
  case eVPGMCathode:{
    // write to I2C? should be 12-bit DAC, MCP4728, aka two bytes (not more than 4096 in total value).
    memcpy((uint8_t * ) &voltage_cathode, data, len);
    if(((int) voltage_cathode <= 4095) && ((int) voltage_cathode >= 0)){
      CATChannelProgram(voltage_cathode, 3);
      memcpy(respData,(uint8_t *) &voltage_cathode, len);
    }
    else {
      voltage_cathode=65535;  // set to all ones if the value was too high for the 12bit DAC
      memcpy(respData,(uint8_t *) &voltage_cathode, len);
    }
    retval=len;
    break;
  }
  case eIPGMCathode:{
    // write to I2C? should be 12-bit DAC, MCP4728, aka two bytes (not more than 4096 in total value).
    memcpy((uint8_t * ) &current_cathode, data, len);
    if(((int) current_cathode <= 4095) && ((int) current_cathode >= 0)){
      CATChannelProgram(current_cathode, 0);
      memcpy(respData,(uint8_t *) &current_cathode, len);
    }
    else {
      current_cathode=65535;  // set to all ones if the value was too high for the 12bit DAC
      memcpy(respData,(uint8_t *) &current_cathode, len);
    }
    retval=len;
    break;
  }
  case eHVCatEN:{
    uint8_t to_enable;
    memcpy((uint8_t * ) &to_enable, data, len);
    if(to_enable){
      is_cathode_disabled=false;
      digitalWrite(EN_CAT,HIGH);
    }
    else{
      is_cathode_disabled=true;
      digitalWrite(EN_CAT,LOW);
    }
    retval=0;
    break;
  }
  case eHVPotEN:{
    uint8_t to_enable;
    memcpy((uint8_t * ) &to_enable, data, len);
    if(to_enable){
      is_potential_disabled=false;
      digitalWrite(EN_POT,HIGH);
    }
    else{
      is_potential_disabled=true;
      digitalWrite(EN_POT,LOW);
    }
    retval=0;
    break;
  }
  case eRampHVCat:{
    step_HV_C=true;
    step_HV_C_time=millis()+CATHODE_VOLTAGE_STEP_PERIOD;
    voltage_cathode=0;
    retval=0;
    break;
  }
  case eRampHVPot:{
    step_HV_P=true;
    step_HV_P_time=millis()+POTENTIAL_VOLTAGE_STEP_PERIOD;
    voltage_potential=0;
    retval=0;
    break;
  }
  case eCancelRamping:{
    step_HV_P=false;
    step_HV_C=false;
    POTChannelProgram(0, 3);
    CATChannelProgram(0, 3);
    retval=0;
    break;
  }
  case ePauseRamping:{
    step_HV_P=false;
    step_HV_C=false;
    retval=0;
    break;
  }
  case eResumeRamping:{
    step_HV_P=true;
    step_HV_C=true;
    step_HV_P_time=millis()+POTENTIAL_VOLTAGE_STEP_PERIOD;
    step_HV_C_time=millis()+CATHODE_VOLTAGE_STEP_PERIOD;
    retval=0;
    break;
  }
  case eHeaterStartupValue:{
    //uint8_t pot_array[4]={254,172,*data,*(data+1)}; //real potentiometer
    uint8_t pot_array[5]={254,172,*data,*(data+1),13}; // emulator code (add carriage return to the end)
    if(pot_array[2] >=0 && pot_array[2]<24){
      //Serial3.write(pot_array,4);
      //Serial2.write(pot_array,4);
      Serial3.write(pot_array,5);
      Serial2.write(pot_array,5);
    }
    retval = 0;
    break;
  }
  case eSetVREF:{
    CATSetVREF();
    POTSetVREF();
    retval=0;
    break;
  }
  case ePacketCount:{
    retval = EBADLEN;
    break;
  }
  default:
    retval=EBADCOMMAND;    
    break;
  }
  return retval;
}

// Fn to handle a local command read.
// This gets called when a local command is received
// with no data (len == 0)
// buffer contains the pointer to where the data
// will be written.
// int returns the number of bytes that were copied into
// the buffer, or EBADCOMMAND if there's no command there
int handleLocalRead(uint8_t localCommand, uint8_t *buffer) {
  int retval = 0;
  switch(localCommand) {
  case ePingPong:
    retval=0;
    break;
  case eSetPriority:
    retval = EBADLEN;
    break;
  case eIntSensorRead: {
    uint32_t TempRead=analogRead(TEMPSENSOR);
    float TempC = (float)(1475 - ((2475 * TempRead) / 4096)) / 10;
    memcpy(buffer,(uint8_t *) &TempC,sizeof(TempC));
    retval=sizeof(TempC);
    break;
  }
  case eThermistorsTest:{
    int len_therms = 4; // length of the thermistor channels to send back?
// Have just plan Serial.read() 
//    retval = Serial2.readBytes(buffer,len_therms);
//    whatToDoIfThermistors(buffer);
    break;}
  case ePacketCount:
    memcpy(buffer, (uint8_t *) &currentPacketCount, sizeof(currentPacketCount));
    retval = sizeof(currentPacketCount);
    break;
  case eThermistors:{
    retval = sizeof(sDCTThermistors); // length of the thermistor channels to send back?
//    float res= 10;
//    float res = return_resistance(CHIP_SELECT_A, 10);
//    float res = measure_channel_2(CHIP_SELECT, 10); // test functions to do the read correctly
//    float res = return_resistance_2(CHIP_SELECT, 10); // read the channel 10 resistance from RTD
    memcpy(buffer,(uint8_t *) &thermistors, sizeof(sDCTThermistors));
    break;}
  case eHVmon:{
    memcpy(buffer, (uint8_t *) &hvmon,sizeof(sDCTHV));
    retval=sizeof(sDCTHV);
    break;
  }
  case eVPGMPotential:{
    memcpy(buffer, (uint8_t *) &voltage_potential,sizeof(voltage_potential));
    retval=sizeof(voltage_potential);
    break;
  }
  case eIPGMPotential:{
    memcpy(buffer, (uint8_t *) &current_potential,sizeof(current_potential));
    retval=sizeof(current_potential);
    break;
  }
  case eVPGMCathode:{
    memcpy(buffer, (uint8_t *) &voltage_cathode,sizeof(voltage_cathode));
    retval=sizeof(voltage_cathode);
    break;
  }
  case eIPGMCathode:{
    memcpy(buffer, (uint8_t *) &current_cathode,sizeof(current_cathode));
    retval=sizeof(current_cathode);
    break;
  }

  case eVMONCathode:{
    memcpy(buffer, (uint8_t *) hvmon.CatVmon,sizeof(hvmon.CatVmon));
    retval=sizeof(hvmon.CatVmon);
    break;
  }
  case eIMONCathode:{
    memcpy(buffer, (uint8_t *) hvmon.CatImon,sizeof(hvmon.CatImon));
    retval=sizeof(hvmon.CatImon);
    break;
  }
  case eVMONPotential:{
    memcpy(buffer, (uint8_t *) hvmon.PotVmon,sizeof(hvmon.PotVmon));
    retval=sizeof(hvmon.PotVmon);
    break;
  }
  case eIMONPotential:{
    memcpy(buffer, (uint8_t *) hvmon.PotImon,sizeof(hvmon.PotImon));
    retval=sizeof(hvmon.PotImon);
    break;
  }
  case ePressure:{
    memcpy(buffer,(uint8_t *)&dct_pressure,sizeof(sDCTPressure));
    retval=sizeof(sDCTPressure);
    break;
  }
  case eHVMonConverted:{
    hvmonconverted.CatV=hvmon.CatVmon*conv_factor_HV;
    hvmonconverted.CatI=hvmon.CatImon*conv_factor_current;
    hvmonconverted.PotV=hvmon.PotVmon*conv_factor_HV;
    hvmonconverted.PotI=hvmon.PotImon*conv_factor_current;
    memcpy(buffer, (uint8_t *) &hvmonconverted,sizeof(sDCTHVConverted));
    retval=sizeof(sDCTHVConverted);
    break;
  }
  case eHVMonOTEN:{
    memcpy(buffer, (uint8_t *) &ot_cathode_read,sizeof(ot_cathode_read));
    memcpy(buffer+sizeof(ot_cathode_read), (uint8_t *) &ot_potential_read,sizeof(ot_potential_read));
    memcpy(buffer+sizeof(ot_cathode_read)+sizeof(ot_potential_read), (uint8_t *) &is_cathode_disabled,sizeof(is_cathode_disabled));
    memcpy(buffer+sizeof(ot_cathode_read)+sizeof(ot_potential_read)+sizeof(is_cathode_disabled), (uint8_t *) &is_potential_disabled,sizeof(is_potential_disabled));
    retval=sizeof(ot_cathode_read)+sizeof(ot_potential_read)+sizeof(is_cathode_disabled)+sizeof(is_potential_disabled);
    break;
  }
  case eReadHeaterResponse:{
    memcpy(buffer, (uint8_t *) &potentiometer1_says,sizeof(potentiometer1_says));
    memcpy(buffer+sizeof(potentiometer1_says), (uint8_t *) &potentiometer2_says,sizeof(potentiometer2_says));
    retval=sizeof(potentiometer1_says)+sizeof(potentiometer2_says);
    break;
  }
  
  case eISR: {
    uint32_t TempRead=analogRead(TEMPSENSOR);
    float TempC = (float)(1475 - ((2475 * TempRead) / 4096)) / 10;
    memcpy(buffer,(uint8_t *) &TempC,sizeof(TempC));
    retval=sizeof(TempC);
    break;
  }
  case eReset: {
    SysCtlReset();
    retval = 0;
    break;
  }
  default:
    retval=EBADCOMMAND;
    break;
  }  
  return retval;
}

// Function to call first when localcommand sent. 
// Store the "result" as retval (which is the bytes read or written, hopefully)
void handleLocalCommand(housekeeping_hdr_t *hdr, uint8_t * data, uint8_t * responsePacketBuffer) {
  int retval=0;
  housekeeping_hdr_t *respHdr = (housekeeping_hdr_t *) responsePacketBuffer;
  uint8_t *respData = responsePacketBuffer + sizeof(housekeeping_hdr_t);
  respHdr->src = myID;
  respHdr->dst = eSFC;
  if (hdr->len) {
    retval = handleLocalWrite(hdr->cmd, data, hdr->len, respData); // retval is negative construct the baderror hdr and send that instead. 
    if(retval>=0) {
//      *respData= 5;
      respHdr->cmd = hdr->cmd;
      respHdr->len = retval; // response bytes of the write.
    }
    else{
      housekeeping_err_t *err = (housekeeping_err_t *) respData;
      buildError(err, respHdr, hdr, retval);
    }  
  } 
  else {
    // local read. by definition these always go downstream.
    retval = handleLocalRead(hdr->cmd, respData);
    if (retval>=0) {
      respHdr->cmd = hdr->cmd;
      respHdr->len = retval; //bytes read
    }
    else {
      housekeeping_err_t *err = (housekeeping_err_t *) respData;
      buildError(err, respHdr, hdr, retval); // the err pointer is pointing to the data of the response packet based on the line above so this fn fills that packet. 
    }
  }
  fillChecksum(responsePacketBuffer);
  // send to SFC
  downStream1.send(responsePacketBuffer, respHdr->len + hdr_size + 1 );
  currentPacketCount++;
}

void handleTestMode(housekeeping_hdr_t *hdr, uint8_t *data, uint8_t * responsePacketBuffer) {
  housekeeping_hdr_t *respHdr = (housekeeping_hdr_t *) responsePacketBuffer;
  uint8_t *respData = responsePacketBuffer + sizeof(housekeeping_hdr_t);
  respHdr->src = myID;
  respHdr->dst = hdr->src;
// if length was actually placed then go into testmode, else build badlength error.
  if (hdr->len) {
   //construct data incoming to be the num testpackets and send the data packet in a while loop and decrement numtestpackets?
    uint16_t numTestPackets = ((uint16_t) (*(data+1) << 8)) | *(data) ; // figure out the correct way to get 2 bytes into a 16_t
    timelastpacket = millis();
    while(numTestPackets){
      if(long (millis()-timelastpacket)>0) { // only send every 50 milliseconds?
        *(respData) = numTestPackets;    
        *(respData+1) = numTestPackets >> 8;
        respHdr->cmd = hdr->cmd;
        respHdr->len = 0x02; // response bytes of the write.
        fillChecksum(responsePacketBuffer);
        // send to SFC
        downStream1.send(responsePacketBuffer, respHdr->len + sizeof(housekeeping_hdr_t) + 1 );  
        numTestPackets--;
        timelastpacket = timelastpacket+TEST_MODE_PERIOD;
        currentPacketCount++;
      }
    }
  }
  else{
    housekeeping_err_t *err = (housekeeping_err_t *) respData;
    buildError(err, respHdr, hdr, EBADLEN); 
    fillChecksum(responsePacketBuffer);
    // send to SFC
    downStream1.send(responsePacketBuffer, respHdr->len + sizeof(housekeeping_hdr_t) + 1 );  
    currentPacketCount++;
  }  
}
void switch_LED(){
  if(is_high){
    is_high=false;
    digitalWrite(LED,LOW);
  }
  else{    
    is_high=true;
    digitalWrite(LED,HIGH);
  }
}
