/******************************************************************************
  ADXL313_Registers.h
  SparkFun ADXL313 Arduino Library - ADXL313 Register Map
  Pete Lewis @ SparkFun Electronics
  Original Creation Date: September 19, 2020
  https://github.com/sparkfun/SparkFun_ADXL313_Arduino_Library

  Some of this code was copied/tweaked from an Arduino Library for the ADXL345
  Written by E. Roberts @ SparkFun Electronics
  Created: Jul 13, 2016
  https://github.com/sparkfun/SparkFun_ADXL345_Arduino_Library

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

#ifndef __ADXL313_Registers_H__
#define __ADXL313_Registers_H__

/////////////////////////////////////////
// ADXL313 Registers //
/////////////////////////////////////////
#define ADXL313_DEVID_0				0x00
#define ADXL313_DEVID_1				0x01
#define ADXL313_PARTID				0x02
#define ADXL313_REVID				0x03
#define ADXL313_XID					0x04
#define ADXL313_SOFT_RESET			0x18
#define ADXL313_OFSX				0x1E
#define ADXL313_OFSY				0x1F
#define ADXL313_OFSZ				0x20
#define ADXL313_THRESH_ACT			0x24
#define ADXL313_THRESH_INACT		0x25
#define ADXL313_TIME_INACT			0x26
#define ADXL313_ACT_INACT_CTL		0x27
#define ADXL313_BW_RATE				0x2C
#define ADXL313_POWER_CTL			0x2D
#define ADXL313_INT_ENABLE			0x2E
#define ADXL313_INT_MAP				0x2F
#define ADXL313_INT_SOURCE			0x30
#define ADXL313_DATA_FORMAT			0x31
#define ADXL313_DATA_X0				0x32
#define ADXL313_DATA_X1				0x33
#define ADXL313_DATA_Y0				0x34
#define ADXL313_DATA_Y1				0x35
#define ADXL313_DATA_Z0				0x36
#define ADXL313_DATA_Z1				0x37
#define ADXL313_FIFO_CTL			0x38
#define ADXL313_FIFO_STATUS			0x39

////////////////////////////////
// ADXL313 Responses //
////////////////////////////////
#define ADXL313_DEVID_0_RSP_EXPECTED			0xAD
#define ADXL313_DEVID_1_RSP_EXPECTED			0x1D
#define ADXL313_PARTID_RSP_EXPECTED			0xCB

#endif