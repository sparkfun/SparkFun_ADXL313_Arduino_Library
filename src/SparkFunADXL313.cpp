/******************************************************************************
  SparkFunADXL313.cpp
  SparkFun ADXL313 Arduino Library - main source file
  Pete Lewis @ SparkFun Electronics
  Original Creation Date: September 19, 2020
  https://github.com/sparkfun/SparkFun_ADXL313_Arduino_Library

  Development environment specifics:

	IDE: Arduino 1.8.13
	Hardware Platform: SparkFun Redboard Qwiic
	SparkFun 3-Axis Digital Accelerometer Breakout - ADXL313 (Qwiic) Version: 1.0

  Do you like this library? Help support SparkFun. Buy a board!

    SparkFun 3-Axis Digital Accelerometer Breakout - ADXL313 (Qwiic)
    https://www.sparkfun.com/products/17241

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
******************************************************************************/

#include "Arduino.h"
#include "SparkFunADXL313.h"
#include <Wire.h>
#include <SPI.h>

#define ADXL313_TO_READ (6)      // Number of Bytes Read - Two Bytes Per Axis

ADXL313::ADXL313() {
	status = ADXL313_OK;
	error_code = ADXL313_NO_ERROR;
	gains[0] = 0.00376390;		// Original gain 0.00376390
	gains[1] = 0.00376009;		// Original gain 0.00376009
	gains[2] = 0.00349265;		// Original gain 0.00349265
}

//Initializes the sensor with basic settings
//Returns false if sensor is not detected
boolean ADXL313::begin(uint8_t deviceAddress, TwoWire &wirePort)
{
  	_deviceAddress = deviceAddress;
  	_i2cPort = &wirePort;
  	I2C = true;
  	if (isConnected() == false)
    	return (false); //Check for sensor presence
  	return (true); //We're all setup!
}

//Returns true if I2C device ack's
boolean ADXL313::isConnected()
{
  	_i2cPort->beginTransmission((uint8_t)_deviceAddress);
  	if (_i2cPort->endTransmission() != 0)
    	return (false); //Sensor did not ACK
  	return (true);
}

//Initializes the sensor with basic settings via SPI
//Returns false if sensor is not detected
boolean ADXL313::beginSPI(uint8_t CS_pin)
{
	_CS = CS_pin;
	I2C = false;
	SPI.begin();
	SPI.setDataMode(SPI_MODE3);
	pinMode(_CS, OUTPUT);
	digitalWrite(_CS, HIGH);

	// since we can't simply check for an "ACK" like in I2C,
	// we will check PART ID, to verify that it's there and working
  	if (checkPartId() != false) 
    	return (false); //Check for sensor presence

  	return (true); //We're all setup!
}

boolean ADXL313::checkPartId() {
	byte _b;
	readFrom(ADXL313_PARTID, 1, &_b);
	if(_b == ADXL313_PARTID_RSP_EXPECTED)
		return (true);
	return (false);
}

boolean ADXL313::dataReady() {
	return getRegisterBit(ADXL313_INT_SOURCE, ADXL313_INT_DATA_READY_BIT);	// check the dataReady bit 
}

boolean ADXL313::updateIntSourceStatuses() {
	byte _b;
	readFrom(ADXL313_INT_SOURCE, 1, &_b);
	intSource.dataReady = ((_b >> ADXL313_INT_DATA_READY_BIT) & 1);
	intSource.activity = ((_b >> ADXL313_INT_ACTIVITY_BIT) & 1);
	intSource.inactivity = ((_b >> ADXL313_INT_INACTIVITY_BIT) & 1);
	intSource.watermark = ((_b >> ADXL313_INT_WATERMARK_BIT) & 1);
	intSource.overrun = ((_b >> ADXL313_INT_OVERRUN_BIT) & 1);

	return (true);
}


bool ADXL313::measureModeOn() {
	//ADXL313 TURN ON
	//writeTo(ADXL313_POWER_CTL, 0);	// Wakeup
	//writeTo(ADXL313_POWER_CTL, 16);	// Auto_Sleep
	writeTo(ADXL313_POWER_CTL, 8);	// Measure
	return (true);
}

/*********************** READING ACCELERATION ***********************/
/*    Reads Acceleration into Three Class Variables:  x, y and z          */


void ADXL313::readAccel() {
	readFrom(ADXL313_DATA_X0, ADXL313_TO_READ, _buff);	// Read Accel Data from ADXL345

	// Each Axis @ All g Ranges: 10 Bit Resolution (2 Bytes)
	x = (int16_t)((((int)_buff[1]) << 8) | _buff[0]);
	y = (int16_t)((((int)_buff[3]) << 8) | _buff[2]);
	z = (int16_t)((((int)_buff[5]) << 8) | _buff[4]);
}

/*************************** RANGE SETTING **************************/
/*          ACCEPTABLE VALUES: 2g, 4g, 8g, 16g ~ GET & SET          */
void ADXL313::getRangeSetting(byte* rangeSetting) {
	byte _b;
	readFrom(ADXL313_DATA_FORMAT, 1, &_b);
	*rangeSetting = _b & 0b00000011;
}

void ADXL313::setRangeSetting(int val) {
	byte _s;
	byte _b;

	switch (val) {
		case 2:
			_s = 0b00000000;
			break;
		case 4:
			_s = 0b00000001;
			break;
		case 8:
			_s = 0b00000010;
			break;
		case 16:
			_s = 0b00000011;
			break;
		default:
			_s = 0b00000000;
	}
	readFrom(ADXL313_DATA_FORMAT, 1, &_b);
	_s |= (_b & 0b11101100);
	writeTo(ADXL313_DATA_FORMAT, _s);
}

/*************************** SELF_TEST BIT **************************/
/*                            ~ GET & SET                           */
bool ADXL313::getSelfTestBit() {
	return getRegisterBit(ADXL313_DATA_FORMAT, 7);
}

// If Set (1) Self-Test Applied. Electrostatic Force exerted on the sensor
//  causing a shift in the output data.
// If Set (0) Self-Test Disabled.
void ADXL313::setSelfTestBit(bool selfTestBit) {
	setRegisterBit(ADXL313_DATA_FORMAT, 7, selfTestBit);
}

/*************************** SPI BIT STATE **************************/
/*                           ~ GET & SET                            */
bool ADXL313::getSpiBit() {
	return getRegisterBit(ADXL313_DATA_FORMAT, 6);
}

// If Set (1) Puts Device in 3-wire Mode
// If Set (0) Puts Device in 4-wire SPI Mode
void ADXL313::setSpiBit(bool spiBit) {
	setRegisterBit(ADXL313_DATA_FORMAT, 6, spiBit);
}

/*********************** INT_INVERT BIT STATE ***********************/
/*                           ~ GET & SET                            */
bool ADXL313::getInterruptLevelBit() {
	return getRegisterBit(ADXL313_DATA_FORMAT, 5);
}

// If Set (0) Sets the Interrupts to Active HIGH
// If Set (1) Sets the Interrupts to Active LOW
void ADXL313::setInterruptLevelBit(bool interruptLevelBit) {
	setRegisterBit(ADXL313_DATA_FORMAT, 5, interruptLevelBit);
}

/************************* FULL_RES BIT STATE ***********************/
/*                           ~ GET & SET                            */
bool ADXL313::getFullResBit() {
	return getRegisterBit(ADXL313_DATA_FORMAT, 3);
}

// If Set (1) Device is in Full Resolution Mode: Output Resolution Increase with G Range
//  Set by the Range Bits to Maintain a 4mg/LSB Scale Factor
// If Set (0) Device is in 10-bit Mode: Range Bits Determine Maximum G Range
//  And Scale Factor
void ADXL313::setFullResBit(bool fullResBit) {
	setRegisterBit(ADXL313_DATA_FORMAT, 3, fullResBit);
}

/*************************** JUSTIFY BIT STATE **************************/
/*                           ~ GET & SET                            */
bool ADXL313::getJustifyBit() {
	return getRegisterBit(ADXL313_DATA_FORMAT, 2);
}

// If Set (1) Selects the Left Justified Mode
// If Set (0) Selects Right Justified Mode with Sign Extension
void ADXL313::setJustifyBit(bool justifyBit) {
	setRegisterBit(ADXL313_DATA_FORMAT, 2, justifyBit);
}


/****************** GAIN FOR EACH AXIS IN Gs / COUNT *****************/
/*                           ~ SET & GET                            */
void ADXL313::setAxisGains(double *_gains){
	int i;
	for(i = 0; i < 3; i++){
		gains[i] = _gains[i];
	}
}
void ADXL313::getAxisGains(double *_gains){
	int i;
	for(i = 0; i < 3; i++){
		_gains[i] = gains[i];
	}
}

/********************* OFSX, OFSY and OFSZ BYTES ********************/
/*                           ~ SET & GET                            */
// OFSX, OFSY and OFSZ: User Offset Adjustments in Twos Complement Format
// Scale Factor of 15.6mg/LSB
void ADXL313::setAxisOffset(int x, int y, int z) {
	writeTo(ADXL313_OFSX, byte (x));
	writeTo(ADXL313_OFSY, byte (y));
	writeTo(ADXL313_OFSZ, byte (z));
}

void ADXL313::getAxisOffset(int* x, int* y, int*z) {
	byte _b;
	readFrom(ADXL313_OFSX, 1, &_b);
	*x = int (_b);
	readFrom(ADXL313_OFSY, 1, &_b);
	*y = int (_b);
	readFrom(ADXL313_OFSZ, 1, &_b);
	*z = int (_b);
}



/*********************** THRESH_ACT REGISTER ************************/
/*                          ~ SET & GET                             */
// Holds the Threshold Value for Detecting Activity.
// Data Format is Unsigned, so the Magnitude of the Activity Event is Compared
//  with the Value is Compared with the Value in the THRESH_ACT Register.
// The Scale Factor is 62.5mg/LSB.
// Value of 0 may Result in Undesirable Behavior if the Activity Interrupt Enabled.
// It Accepts a Maximum Value of 255.
void ADXL313::setActivityThreshold(int activityThreshold) {
	activityThreshold = constrain(activityThreshold,0,255);
	byte _b = byte (activityThreshold);
	writeTo(ADXL313_THRESH_ACT, _b);
}

// Gets the THRESH_ACT byte
int ADXL313::getActivityThreshold() {
	byte _b;
	readFrom(ADXL313_THRESH_ACT, 1, &_b);
	return int (_b);
}

/********************** THRESH_INACT REGISTER ***********************/
/*                          ~ SET & GET                             */
// Holds the Threshold Value for Detecting Inactivity.
// The Data Format is Unsigned, so the Magnitude of the INactivity Event is
//  Compared with the value in the THRESH_INACT Register.
// Scale Factor is 62.5mg/LSB.
// Value of 0 May Result in Undesirable Behavior if the Inactivity Interrupt Enabled.
// It Accepts a Maximum Value of 255.
void ADXL313::setInactivityThreshold(int inactivityThreshold) {
	inactivityThreshold = constrain(inactivityThreshold,0,255);
	byte _b = byte (inactivityThreshold);
	writeTo(ADXL313_THRESH_INACT, _b);
}

int ADXL313::getInactivityThreshold() {
	byte _b;
	readFrom(ADXL313_THRESH_INACT, 1, &_b);
	return int (_b);
}

/*********************** TIME_INACT RESIGER *************************/
/*                          ~ SET & GET                             */
// Contains an Unsigned Time Value Representing the Amount of Time that
//  Acceleration must be Less Than the Value in the THRESH_INACT Register
//  for Inactivity to be Declared.
// Uses Filtered Output Data* unlike other Interrupt Functions
// Scale Factor is 1sec/LSB.
// Value Must Be Between 0 and 255.
void ADXL313::setTimeInactivity(int timeInactivity) {
	timeInactivity = constrain(timeInactivity,0,255);
	byte _b = byte (timeInactivity);
	writeTo(ADXL313_TIME_INACT, _b);
}

int ADXL313::getTimeInactivity() {
	byte _b;
	readFrom(ADXL313_TIME_INACT, 1, &_b);
	return int (_b);
}



/************************** ACTIVITY BITS ***************************/
/*                                                                  */
bool ADXL313::isActivityXEnabled() {
	return getRegisterBit(ADXL313_ACT_INACT_CTL, 6);
}
bool ADXL313::isActivityYEnabled() {
	return getRegisterBit(ADXL313_ACT_INACT_CTL, 5);
}
bool ADXL313::isActivityZEnabled() {
	return getRegisterBit(ADXL313_ACT_INACT_CTL, 4);
}
bool ADXL313::isInactivityXEnabled() {
	return getRegisterBit(ADXL313_ACT_INACT_CTL, 2);
}
bool ADXL313::isInactivityYEnabled() {
	return getRegisterBit(ADXL313_ACT_INACT_CTL, 1);
}
bool ADXL313::isInactivityZEnabled() {
	return getRegisterBit(ADXL313_ACT_INACT_CTL, 0);
}

void ADXL313::setActivityX(bool state) {
	setRegisterBit(ADXL313_ACT_INACT_CTL, 6, state);
}
void ADXL313::setActivityY(bool state) {
	setRegisterBit(ADXL313_ACT_INACT_CTL, 5, state);
}
void ADXL313::setActivityZ(bool state) {
	setRegisterBit(ADXL313_ACT_INACT_CTL, 4, state);
}
void ADXL313::setActivityXYZ(bool stateX, bool stateY, bool stateZ) {
	setActivityX(stateX);
	setActivityY(stateY);
	setActivityZ(stateZ);
}
void ADXL313::setInactivityX(bool state) {
	setRegisterBit(ADXL313_ACT_INACT_CTL, 2, state);
}
void ADXL313::setInactivityY(bool state) {
	setRegisterBit(ADXL313_ACT_INACT_CTL, 1, state);
}
void ADXL313::setInactivityZ(bool state) {
	setRegisterBit(ADXL313_ACT_INACT_CTL, 0, state);
}
void ADXL313::setInactivityXYZ(bool stateX, bool stateY, bool stateZ) {
	setInactivityX(stateX);
	setInactivityY(stateY);
	setInactivityZ(stateZ);
}

bool ADXL313::isActivityAc() {
	return getRegisterBit(ADXL313_ACT_INACT_CTL, 7);
}
bool ADXL313::isInactivityAc(){
	return getRegisterBit(ADXL313_ACT_INACT_CTL, 3);
}

void ADXL313::setActivityAc(bool state) {
	setRegisterBit(ADXL313_ACT_INACT_CTL, 7, state);
}
void ADXL313::setInactivityAc(bool state) {
	setRegisterBit(ADXL313_ACT_INACT_CTL, 3, state);
}


/************************** LOW POWER BIT ***************************/
/*                                                                  */
bool ADXL313::isLowPower(){
	return getRegisterBit(ADXL313_BW_RATE, 4);
}
void ADXL313::setLowPower(bool state) {
	setRegisterBit(ADXL313_BW_RATE, 4, state);
}

/*************************** RATE BITS ******************************/
/*                                                                  */
double ADXL313::getRate(){
	byte _b;
	readFrom(ADXL313_BW_RATE, 1, &_b);
	_b &= 0b00001111;
	return (pow(2,((int) _b)-6)) * 6.25;
}

void ADXL313::setRate(double rate){
	byte _b,_s;
	int v = (int) (rate / 6.25);
	int r = 0;
	while (v >>= 1)
	{
		r++;
	}
	if (r <= 9) {
		readFrom(ADXL313_BW_RATE, 1, &_b);
		_s = (byte) (r + 6) | (_b & 0b11110000);
		writeTo(ADXL313_BW_RATE, _s);
	}
}

/*************************** BANDWIDTH ******************************/
/*                          ~ SET & GET                             */
void ADXL313::set_bw(byte bw_code){
	// if((bw_code < ADXL313_BW_0_05) || (bw_code > ADXL313_BW_1600)){
	// 	status = false;
	// 	error_code = ADXL313_BAD_ARG;
	// }
	// else{
	// 	writeTo(ADXL313_BW_RATE, bw_code);
	// }
}

byte ADXL313::get_bw_code(){
	byte bw_code;
	readFrom(ADXL313_BW_RATE, 1, &bw_code);
	return bw_code;
}




/************************* TRIGGER CHECK  ***************************/
/*                                                                  */
// Check if Action was Triggered in Interrupts
// Example triggered(interrupts, ADXL313_SINGLE_TAP);
bool ADXL313::triggered(byte interrupts, int mask){
	return ((interrupts >> mask) & 1);
}

/*
 ADXL313_DATA_READY
 ADXL313_SINGLE_TAP
 ADXL313_DOUBLE_TAP
 ADXL313_ACTIVITY
 ADXL313_INACTIVITY
 ADXL313_FREE_FALL
 ADXL313_WATERMARK
 ADXL313_OVERRUNY
 */


byte ADXL313::getInterruptSource() {
	byte _b;
	readFrom(ADXL313_INT_SOURCE, 1, &_b);
	return _b;
}

bool ADXL313::getInterruptSource(byte interruptBit) {
	return getRegisterBit(ADXL313_INT_SOURCE,interruptBit);
}

bool ADXL313::getInterruptMapping(byte interruptBit) {
	return getRegisterBit(ADXL313_INT_MAP,interruptBit);
}

// /*********************** INTERRUPT MAPPING **************************/
// /*         Set the Mapping of an Interrupt to pin1 or pin2          */
// // eg: setInterruptMapping(ADXL313_INT_DOUBLE_TAP_BIT,ADXL313_INT2_PIN);
// void ADXL313::setInterruptMapping(byte interruptBit, bool interruptPin) {
// 	setRegisterBit(ADXL313_INT_MAP, interruptBit, interruptPin);
// }

// void ADXL313::setImportantInterruptMapping(int single_tap, int double_tap, int free_fall, int activity, int inactivity) {
// 	if(single_tap == 1) {
// 		setInterruptMapping( ADXL313_INT_SINGLE_TAP_BIT,   ADXL313_INT1_PIN );}
// 	else if(single_tap == 2) {
// 		setInterruptMapping( ADXL313_INT_SINGLE_TAP_BIT,   ADXL313_INT2_PIN );}

// 	if(double_tap == 1) {
// 		setInterruptMapping( ADXL313_INT_DOUBLE_TAP_BIT,   ADXL313_INT1_PIN );}
// 	else if(double_tap == 2) {
// 		setInterruptMapping( ADXL313_INT_DOUBLE_TAP_BIT,   ADXL313_INT2_PIN );}

// 	if(free_fall == 1) {
// 		setInterruptMapping( ADXL313_INT_FREE_FALL_BIT,   ADXL313_INT1_PIN );}
// 	else if(free_fall == 2) {
// 		setInterruptMapping( ADXL313_INT_FREE_FALL_BIT,   ADXL313_INT2_PIN );}

// 	if(activity == 1) {
// 		setInterruptMapping( ADXL313_INT_ACTIVITY_BIT,   ADXL313_INT1_PIN );}
// 	else if(activity == 2) {
// 		setInterruptMapping( ADXL313_INT_ACTIVITY_BIT,   ADXL313_INT2_PIN );}

// 	if(inactivity == 1) {
// 		setInterruptMapping( ADXL313_INT_INACTIVITY_BIT,   ADXL313_INT1_PIN );}
// 	else if(inactivity == 2) {
// 		setInterruptMapping( ADXL313_INT_INACTIVITY_BIT,   ADXL313_INT2_PIN );}
// }

bool ADXL313::isInterruptEnabled(byte interruptBit) {
	return getRegisterBit(ADXL313_INT_ENABLE,interruptBit);
}

void ADXL313::setInterrupt(byte interruptBit, bool state) {
	setRegisterBit(ADXL313_INT_ENABLE, interruptBit, state);
}

void ADXL313::ActivityINT(bool status) {
	if(status) {
		setInterrupt( ADXL313_INT_ACTIVITY_BIT,   1);
	}
	else {
		setInterrupt( ADXL313_INT_ACTIVITY_BIT,   0);
	}
}
void ADXL313::InactivityINT(bool status) {
	if(status) {
		setInterrupt( ADXL313_INT_INACTIVITY_BIT, 1);
	}
	else {
		setInterrupt( ADXL313_INT_INACTIVITY_BIT, 0);
	}
}

void ADXL313::setRegisterBit(byte regAdress, int bitPos, bool state) {
	byte _b;
	readFrom(regAdress, 1, &_b);
	if (state) {
		_b |= (1 << bitPos);  // Forces nth Bit of _b to 1. Other Bits Unchanged.
	}
	else {
		_b &= ~(1 << bitPos); // Forces nth Bit of _b to 0. Other Bits Unchanged.
	}
	writeTo(regAdress, _b);
}

bool ADXL313::getRegisterBit(byte regAdress, int bitPos) {
	byte _b;
	readFrom(regAdress, 1, &_b);
	return ((_b >> bitPos) & 1);
}

/********************************************************************/
/*                                                                  */
// Print Register Values to Serial Output =
// Can be used to Manually Check the Current Configuration of Device
void ADXL313::printAllRegister() {
	byte _b;
	Serial.print("0x00: ");
	readFrom(0x00, 1, &_b);
	print_byte(_b);
	Serial.println("");
	int i;
	for (i=29;i<=57;i++){
		Serial.print("0x");
		Serial.print(i, HEX);
		Serial.print(": ");
		readFrom(i, 1, &_b);
		print_byte(_b);
		Serial.println("");
	}
}

void print_byte(byte val){
	int i;
	Serial.print("B");
	for(i=7; i>=0; i--){
		Serial.print(val >> i & 1, BIN);
	}
}

/***************** WRITES VALUE TO ADDRESS REGISTER *****************/
void ADXL313::writeTo(byte address, byte val) {
	if(I2C) {
		writeToI2C(address, val);
	}
	else {
		writeToSPI(address, val);
	}
}

/************************ READING NUM BYTES *************************/
/*    Reads Num Bytes. Starts from Address Reg to _buff Array        */
void ADXL313::readFrom(byte address, int num, byte _buff[]) {
	if(I2C) {
		readFromI2C(address, num, _buff);	// If I2C Communication
	}
	else {
		readFromSPI(address, num, _buff);	// If SPI Communication
	}
}

/*************************** WRITE TO I2C ***************************/
/*      Start; Send Register Address; Send Value To Write; End      */
void ADXL313::writeToI2C(byte _address, byte _val) {
	Wire.beginTransmission(_deviceAddress);
	Wire.write(_address);
	Wire.write(_val);
	Wire.endTransmission();
}

/*************************** READ FROM I2C **************************/
/*                Start; Send Address To Read; End                  */
void ADXL313::readFromI2C(byte address, int num, byte _buff[]) {
	Wire.beginTransmission(_deviceAddress);
	Wire.write(address);
	Wire.endTransmission();

//	Wire.beginTransmission(ADXL313_DEVICE);
// Wire.reqeustFrom contains the beginTransmission and endTransmission in it. 
	Wire.requestFrom(_deviceAddress, num);  // Request 6 Bytes

	int i = 0;
	while(Wire.available())
	{
		_buff[i] = Wire.read();				// Receive Byte
		i++;
	}
	if(i != num){
		status = ADXL313_ERROR;
		error_code = ADXL313_READ_ERROR;
	}
//	Wire.endTransmission();
}

/************************** WRITE FROM SPI **************************/
/*         Point to Destination; Write Value; Turn Off              */
void ADXL313::writeToSPI(byte __reg_address, byte __val) {
  digitalWrite(_CS, LOW);
  SPI.transfer(__reg_address);
  SPI.transfer(__val);
  digitalWrite(_CS, HIGH);
}

/*************************** READ FROM SPI **************************/
/*                                                                  */
void ADXL313::readFromSPI(byte __reg_address, int num, byte _buff[]) {
  // Read: Most Sig Bit of Reg Address Set
  char _address = 0x80 | __reg_address;
  // If Multi-Byte Read: Bit 6 Set
  if(num > 1) {
  	_address = _address | 0x40;
  }

  digitalWrite(_CS, LOW);
  SPI.transfer(_address);		// Transfer Starting Reg Address To Be Read
  for(int i=0; i<num; i++){
    _buff[i] = SPI.transfer(0x00);
  }
  digitalWrite(_CS, HIGH);
}