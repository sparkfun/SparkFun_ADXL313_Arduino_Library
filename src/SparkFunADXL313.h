/******************************************************************************
  SparkFunADXL313.h
  SparkFun ADXL313 Arduino Library - ADXL313 Header File
  Pete Lewis @ SparkFun Electronics
  Original Creation Date: September 19, 2020
  https://github.com/sparkfun/SparkFun_ADXL313_Arduino_Library

  This file defines all registers internal to the ADXL313.

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

#ifndef __SparkFunADXL313_H__
#define __SparkFunADXL313_H__

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
  #include "pins_arduino.h"
#endif

#include "ADXL313_Registers.h"
#include "Wire.h"

#define ADXL313_I2C_ADDRESS_DEFAULT 	0x1D
#define ADXL313_I2C_ADDRESS_ALT 0x53
#define ADXL313_CS_PIN_DEFAULT 10

 /************************** INTERRUPT PINS **************************/ //NEED TO CHECK THESE FOR 313!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
#define ADXL313_INT1_PIN		0x00		//INT1: 0
#define ADXL313_INT2_PIN		0x01		//INT2: 1


 /********************** INTERRUPT BIT POSITION **********************/
#define ADXL313_INT_DATA_READY_BIT		0x07
#define ADXL313_INT_ACTIVITY_BIT		0x04
#define ADXL313_INT_INACTIVITY_BIT		0x03
#define ADXL313_INT_WATERMARK_BIT		0x01
#define ADXL313_INT_OVERRUN_BIT			0x00

#define ADXL313_DATA_READY				0x07
#define ADXL313_ACTIVITY				0x04
#define ADXL313_INACTIVITY				0x03
#define ADXL313_WATERMARK				0x01
#define ADXL313_OVERRUN					0x00

 /********************** RANGE SETTINGS OPTIONS **********************/
#define ADXL313_RANGE_05_G		1 // 0-0.5G
#define ADXL313_RANGE_1_G		2 // 0-1G
#define ADXL313_RANGE_2_G		3 // 0-2G
#define ADXL313_RANGE_4_G		4 // 0-4G

 /********************** POWER_CTL BIT POSITION **********************/
#define ADXL313_I2C_DISABLE_BIT		0x06
#define ADXL313_LINK_BIT			0x05
#define ADXL313_AUTO_SLEEP_BIT		0x04
#define ADXL313_MEASURE_BIT			0x03
#define ADXL313_SLEEP_BIT			0x02

 /****************************** ERRORS ******************************/
#define ADXL313_OK			1		// No Error
#define ADXL313_ERROR		0		// Error Exists

#define ADXL313_NO_ERROR	0		// Initial State
#define ADXL313_READ_ERROR	1		// Accelerometer Reading Error
#define ADXL313_BAD_ARG		2		// Bad Argument


//  ADXL313Interrupts
//
//  This is used by the ADXL313 class to hold interrupt settings and statuses from the most recent read of INT_SOURCE. 
//  It is public within that class and the user is expected to write desired values into the settings before calling
//  .setInterrupts();
struct ADXL313IntSource
{
  public:
    bool dataReady;
    bool activity;
	bool inactivity;
	bool watermark;
	bool overrun;
};


class ADXL313
{
public:
	
	// We'll store the accelerometer readings in a series of
	// public class variables. Each sensor gets three variables -- one for each
	// axis. Call readAdxl() first, before using these variables!
	// These values are the RAW signed 16-bit readings from the sensor.
	int16_t x, y, z; // x, y, and z axis readings of the accelerometer

	bool status;					// Set When Error Exists 

	byte error_code;				// Initial State
	double gains[3];				// Counts to Gs

	// INT_SOURCE register bit statuses
	// used to allow a single read of the INT_SOURCE register,
	// and then later check the status of each bit (stored individually in class varaibles)
	ADXL313IntSource intSource;


	// ADXL313 class constructor
	// The constructor will set up with default settings via I2C

	ADXL313();
		
	// begin() -- Initialize the accelerometer
	// This will set up the scale and output rate of each sensor. The values set
	// in the IMUSettings struct will take effect after calling this function.
	// INPUTS:
	// - agAddress - Sets either the I2C address of the accel/gyro or SPI chip 
	//   select pin connected to the CS_XG pin.
	// - mAddress - Sets either the I2C address of the magnetometer or SPI chip 
	//   select pin connected to the CS_M pin.
	// - i2C port (Note, only on "begin()" funtion, for use with I2C com interface)
	//   defaults to Wire, but if hardware supports it, can use other TwoWire ports.
	//   **For SPI use "beginSPI()", and only send first two address arguments.
	bool begin(uint8_t address = ADXL313_I2C_ADDRESS_ALT, TwoWire &wirePort = Wire); //By default use the default I2C addres, and use Wire port
	bool beginSPI(uint8_t CS_pin = ADXL313_CS_PIN_DEFAULT);


	bool isConnected();
	bool checkPartId();

	// dataReady() -- REads the entire INT_Source register, and checks the DATA_READY bit
	// to see if new data is available.
	// **Note, this will also clear any other INT source bits.
	// If you need to know the other int source bits, then use updateIntSourceStatuses()
	// Output:	1 - New data available
	//			0 - No new data available
	bool dataReady();

	// updateIntSourceStatuses() -- Reads the entire INT_Source register, 
	// and stores all of the int statuses in class variables.
	// note, this will clear all INT source bits.
	// Output:	1 - function completed
	//			0 - Communication failure
	bool updateIntSourceStatuses();

	bool measureModeOn();

	bool softReset();

	bool standby();
	
	// readAccel() -- Read the sensors output registers.
	// This function will read all six accelerometer output registers.
	// The readings are stored in the class' x, y, and z variables. Read
	// those _after_ calling readAccel().
	void readAccel();
	
	void setAxisGains(double *_gains);
	void getAxisGains(double *_gains);
	void setAxisOffset(int x, int y, int z);
	void getAxisOffset(int* x, int* y, int*z);
	void setActivityThreshold(int activityThreshold);
	int getActivityThreshold();
	void setInactivityThreshold(int inactivityThreshold);
	int getInactivityThreshold();
	void setTimeInactivity(int timeInactivity);
	int getTimeInactivity();
	
	bool isActivityXEnabled();
	bool isActivityYEnabled();
	bool isActivityZEnabled();
	bool isInactivityXEnabled();
	bool isInactivityYEnabled();
	bool isInactivityZEnabled();
	bool isActivityAc();
	bool isInactivityAc();
	void setActivityAc(bool state);
	void setInactivityAc(bool state);
	
	void setActivityX(bool state);
	void setActivityY(bool state);
	void setActivityZ(bool state);
	void setActivityXYZ(bool stateX, bool stateY, bool stateZ);
	void setInactivityX(bool state);
	void setInactivityY(bool state);
	void setInactivityZ(bool state);
	void setInactivityXYZ(bool stateX, bool stateY, bool stateZ);
	
	bool isActivitySourceOnX();
	bool isActivitySourceOnY();
	bool isActivitySourceOnZ();
	bool isAsleep();
	
	bool isLowPower();
	void setLowPower(bool state);
	double getRate();
	void setRate(double rate);
	void set_bw(byte bw_code);
	byte get_bw_code();  
	
	bool triggered(byte interrupts, int mask);
	
	byte getInterruptSource();
	bool getInterruptSource(byte interruptBit);
	bool getInterruptMapping(byte interruptBit);
	void setInterruptMapping(byte interruptBit, bool interruptPin);
	bool isInterruptEnabled(byte interruptBit);
	void setInterrupt(byte interruptBit, bool state);
	void setImportantInterruptMapping(int single_tap, int double_tap, int free_fall, int activity, int inactivity);
	void InactivityINT(bool status);
	void ActivityINT(bool status);
	
	float getRange();
	void setRange(byte range);
	bool getSelfTestBit();
	void setSelfTestBit(bool selfTestBit);
	bool getSpiBit();
	void setSpiBit(bool spiBit);
	bool getInterruptLevelBit();
	void setInterruptLevelBit(bool interruptLevelBit);
	bool getFullResBit();
	void setFullResBit(bool fullResBit);
	bool getJustifyBit();
	void setJustifyBit(bool justifyBit);
	void printAllRegister();
	
private:

	TwoWire *_i2cPort;
	uint8_t _deviceAddress;

	void writeTo(byte address, byte val);
	void writeToI2C(byte address, byte val);
	void writeToSPI(byte address, byte val);
	void readFrom(byte address, int num, byte buff[]);
	void readFromI2C(byte address, int num, byte buff[]);
	void readFromSPI(byte address, int num, byte buff[]);
	void setRegisterBit(byte regAdress, int bitPos, bool state);
	bool getRegisterBit(byte regAdress, int bitPos);  
	
	byte _buff[6] ;		//	6 Bytes Buffer
	int _CS = ADXL313_CS_PIN_DEFAULT;
	bool I2C = true;
	unsigned long SPIfreq = 5000000;
};
void print_byte(byte val);

#endif // SFE_ADXL313_H //