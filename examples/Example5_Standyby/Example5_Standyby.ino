/******************************************************************************
  Example5_Standby.ino
  Shows how to switch the sensor between stanby mode and measure mode.
  This example will put the device in measure mode and print 100 readings to terminal,
  Then enter standby mode for 5 seconds.
  Then loop.
  Note, the typical current required in each mode is as follows:
  Standby: 0.1uA
  Measure: 55-170uA

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
  Serial.println("Example 5 - Standby mode and measure mode.");

  Wire.begin();

  if (myAdxl.begin() == false) //Begin communication over I2C
  {
    Serial.println("The sensor did not respond. Please check wiring.");
    while (1); //Freeze
  }
  Serial.print("Sensor is connected properly.");
}

void loop()
{
  // enter measure mode
  Serial.println("Entering Measure Mode.");
  myAdxl.measureModeOn();

  // print 100 readings to terminal (this should take about 500ms)
  for (int i = 0 ; i < 100 ; i++)
  {
    myAdxl.updateIntSourceStatuses(); // this will update all class intSource.xxxxx variables by reading int source bits.

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
      Serial.println("Waiting for data.");
    }
    delay(50);
  }

  Serial.println("Entering Standby Mode.");
  myAdxl.standby();
  delay(5000); // 5 seconds of standby... really saving power during this time (0.1uA)
}