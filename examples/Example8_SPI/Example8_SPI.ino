/******************************************************************************
  Example8_SPI.ino
  Read values of x/y/z axis of the ADXL313 (via SPI), print them to terminal.
  This uses default configuration (1G range, full resolution, 100Hz datarate).

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
  GND --> 3.3V
  GND --> GND

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
******************************************************************************/

#include <SPI.h>
#include <SparkFunADXL313.h>
ADXL313 accel;

void setup()
{
  Serial.begin(115200);
  Serial.println("Reading values from ADXL313");

  if (accel.beginSPI() == false) //Begin communication over SPI
  {
    Serial.println("The sensor did not respond. Please check wiring.");
    while(1); //Freeze
  }
}

void loop()
{
  if(accel.available())
  {
    accel.read();

    Serial.print("x: ");
    Serial.print(accel.x);

    Serial.print("y: ");
    Serial.print(accel.y);

    Serial.print("z: ");
    Serial.print(accel.z);

    Serial.println();
  }
  delay(50);
}