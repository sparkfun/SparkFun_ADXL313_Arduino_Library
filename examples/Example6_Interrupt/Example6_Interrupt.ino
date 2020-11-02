/******************************************************************************
  Example6_Interrupt.ino
  Shows how to setup a hardware interrupt on the ADXL313.
  We will utilize the autosleep feature of the sensor.
  When it senses inactivity, it will go to sleep.
  When it senses new activity, it will wake up and trigger the INT1 pin.
  We will be listening with our Arduino, which will have a hardware interrupt setup.

  ///// Autosleep setup //////
  First, setup THRESH_INACT, TIME_INACT, and participating axis.
  These settings will determine when the unit will go into autosleep mode and save power!
  We are only going to use the x-axis (and are disabling y-axis and z-axis).
  This is so you can place the board "flat" inside your project,
  and we can ignore gravity on z-axis.

  ///// Interrupt setup //////
  Enable activity interrupt.
  Map activity interrupt to "int pin 1".
  Setup intterupt on Arduino (pin D2, aka INT0)

  SparkFun ADXL313 Arduino Library
  Pete Lewis @ SparkFun Electronics
  Original Creation Date: September 19, 2020
  https://github.com/sparkfun/SparkFun_ADXL313_Arduino_Library

  Do you like this library? Help support SparkFun. Buy a board!

    SparkFun 3-Axis Digital Accelerometer Breakout - ADXL313 (Qwiic)
    https://www.sparkfun.com/products/17241

  Development environment specifics:

	IDE: Arduino 1.8.13
	Hardware Platform: SparkFun Redboard Qwiic
	SparkFun 3-Axis Digital Accelerometer Breakout - ADXL313 (Qwiic) Version: 1.0

  Hardware Connections:
  Use a qwiic cable to connect from the Redboard Qwiic to the ADXL313 breakout (QWIIC).
  You can also choose to wire up the connections using the header pins like so:

  ARDUINO --> ADXL313
  SDA (A4) --> SDA
  SCL (A5) --> SCL
  3.3V --> 3.3V
  GND --> GND
  D2 --> INT1

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
******************************************************************************/

#include <Wire.h>
#include <SparkFunADXL313.h> //Click here to get the library: http://librarymanager/All#SparkFun_ADXL313
ADXL313 myAdxl;

bool awake = true; // global variable to keep track.
bool interruptFlag = false; // global variabl to keep track of new interrupts. Only ever set true by ISR

void setup()
{
  pinMode(2, INPUT); // setup for interrupt
  delay(500);

  Serial.begin(115200);
  Serial.println("Example 6 - Setup Autosleep and then only print values when it's awake.");

  Wire.begin();

  if (myAdxl.begin() == false) //Begin communication over I2C
  {
    Serial.println("The sensor did not respond. Please check wiring.");
    while (1); //Freeze
  }
  Serial.print("Sensor is connected properly.");

  myAdxl.standby(); // Must be in standby before changing settings.
  // This is here just in case we already had sensor powered and/or
  // configured from a previous setup.


  myAdxl.setRange(ADXL313_RANGE_4_G);

  // setup activity sensing options
  myAdxl.setActivityX(true); // enable x-axis participation in detecting inactivity
  myAdxl.setActivityY(false); // disable y-axis participation in detecting inactivity
  myAdxl.setActivityZ(false); // disable z-axis participation in detecting inactivity
  myAdxl.setActivityThreshold(10); // 0-255 (62.5mg/LSB)

  //setup inactivity sensing options
  myAdxl.setInactivityX(true); // enable x-axis participation in detecting inactivity
  myAdxl.setInactivityY(false); // disable y-axis participation in detecting inactivity
  myAdxl.setInactivityZ(false); // disable z-axis participation in detecting inactivity
  myAdxl.setInactivityThreshold(10); // 0-255 (62.5mg/LSB)
  myAdxl.setTimeInactivity(5); // 0-255 (1sec/LSB)



  myAdxl.setInterruptMapping(ADXL313_INT_ACTIVITY_BIT, ADXL313_INT1_PIN); // when activity is detected, it will effect the int1 pin on the sensor
  myAdxl.setInterruptMapping(ADXL313_INT_INACTIVITY_BIT, ADXL313_INT1_PIN);
  // myAdxl.setInterruptMapping(ADXL313_INT_DATA_READY_BIT, ADXL313_INT1_PIN);

  // enable/disable interrupts
  // note, we set them all here, just in case there were previous settings,
  // that need to be changed for this example to work properly.
  myAdxl.InactivityINT(true); // ensable the inactivity interrupt
  myAdxl.ActivityINT(true); // enable the activity interrupt
  myAdxl.DataReadyINT(false); // disable dataReady

  myAdxl.autosleepOn();

  myAdxl.measureModeOn(); // wakes up the sensor from standby and puts it into measurement mode

  // print int enable statuses, to verify we're setup correctly
  Serial.println();
  Serial.print("activity int enable: ");
  Serial.println(myAdxl.isInterruptEnabled(ADXL313_INT_ACTIVITY_BIT));
  Serial.print("inactivity int enable: ");
  Serial.println(myAdxl.isInterruptEnabled(ADXL313_INT_INACTIVITY_BIT));
  Serial.print("dataReady int enable: ");
  Serial.println(myAdxl.isInterruptEnabled(ADXL313_INT_DATA_READY_BIT));
  delay(1000);

  attachInterrupt(digitalPinToInterrupt(2), int1_ISR, RISING); // note, the INT output on the ADXL313 is default active HIGH.
}

void loop()
{
  if (interruptFlag == true) // sensor is awake (this variable is only ever set true in int1_ISR)
  {
    // interrupt has fired
    // check to see what type of detection it was

    myAdxl.updateIntSourceStatuses(); // this will update all class intSource.xxxxx variables by reading int source bits.
    interruptFlag = false;

    if (myAdxl.intSource.activity == true)
    {
      Serial.println("Activity detected.");
      awake = true;
    }
    if (myAdxl.intSource.inactivity == true)
    {
      Serial.println("Inactivity detected.");
      awake = false;
    }
  }

  if (awake == true)
  {
    myAdxl.updateIntSourceStatuses(); // this will update all class intSource.xxxxx variables by reading int source bits.
    if (myAdxl.intSource.dataReady == true)
    {
      myAdxl.readAccel(); // read all 3 axis, they are stored in class variables: myAdxl.x, myAdxl.y and myAdxl.z
      Serial.print("x: ");
      Serial.print(myAdxl.x);
      Serial.print("\ty: ");
      Serial.print(myAdxl.y);
      Serial.print("\tz: ");
      Serial.print(myAdxl.z);
      Serial.println();
    }
  }
  delay(50);
}

// activity or inactivity has caused an interrupt
void int1_ISR()
{
  interruptFlag = true;
}