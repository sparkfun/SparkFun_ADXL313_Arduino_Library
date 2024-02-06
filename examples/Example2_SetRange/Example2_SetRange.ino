/******************************************************************************
  Example2_SetRange.ino
  Set range of the sensor to 4G.
  Then read values of x/y/z axis of the ADXL313 (via I2C), print them to terminal.
  Note, other range options are: 0.5G, 1G[defaut], 2G or 4 G.
  Except for custom range, this example uses default configuration (full resolution, 100Hz datarate).

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
  Serial.println("Example 2 - Set 2G Range and read values from ADXL313");

  Wire.begin();

  if (myAdxl.begin() == false) //Begin communication over I2C
  {
    Serial.println("The sensor did not respond. Please check wiring.");
    while(1); //Freeze
  }
  Serial.print("Sensor is connected properly.");
  
  myAdxl.standby(); // Must be in standby before changing settings.
                    // This is here just in case we already had sensor powered and/or
                    // configured from a previous setup.

  myAdxl.setRange(ADXL313_RANGE_2_G);

  // Try some other range settings by uncommented your choice below
  // myAdxl.setRange(ADXL313_RANGE_05_G);
  // myAdxl.setRange(ADXL313_RANGE_1_G);
  // myAdxl.setRange(ADXL313_RANGE_2_G);
  // myAdxl.setRange(ADXL313_RANGE_4_G);
  
  myAdxl.measureModeOn(); // wakes up the sensor from standby and puts it into measurement mode
}

void loop()
{
  if(myAdxl.dataReady()) // check data ready interrupt, note, this clears all other int bits in INT_SOURCE reg
  {
    myAdxl.readAccel(); // read all 3 axis, they are stored in class variables: myAdxl.x, myAdxl.y and myAdxl.z
    Serial.print("x: ");
    Serial.print(myAdxl.x);
    Serial.print("\ty: ");
    Serial.print(myAdxl.y);
    Serial.print("\tz: ");
    Serial.print(myAdxl.z);
    Serial.print("\trange: ");
    Serial.print(myAdxl.getRange());
    Serial.println();
  }
  else
  {
    Serial.println("Waiting for dataReady.");
  }  
  delay(50);
}