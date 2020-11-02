/******************************************************************************
  Example3_Autosleep.ino
  Shows how to use Autosleep feature.
  First, setup THRESH_INACT, TIME_INACT, and participating axis.
  These settings will determine when the unit will go into autosleep mode and save power!
  We are only going to use the x-axis (and are disabling y-axis and z-axis).
  This is so you can place the board "flat" inside your project,
  and we can ignore gravity on z-axis.

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

void setup()
{
  Serial.begin(115200);
  Serial.println("Example 3 - Setup Autosleep and then only print values when it's awake.");

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

  myAdxl.ActivityINT(1);
  myAdxl.InactivityINT(1);

  myAdxl.autosleepOn();

  myAdxl.measureModeOn(); // wakes up the sensor from standby and puts it into measurement mode
}

void loop()
{
  myAdxl.updateIntSourceStatuses(); // this will update all class intSource.xxxxx variables by reading int source bits.

  if (myAdxl.intSource.inactivity == true)
  {
    Serial.println("Inactivity detected.");
    delay(1000);
  }
  if (myAdxl.intSource.dataReady) // check data ready interrupt bit
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
  else
  {
    Serial.println("Device is asleep (dataReady is reading false)");
  }
  delay(50);
}