/******************************************************************************
  ADXL313_Registers.h
  SparkFun ADXL313 Arduino Library - ADXL313 Register Map
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

#ifndef __ADXL313_Registers_H__
#define __ADXL313_Registers_H__

/////////////////////////////////////////
// ADXL313 Registers //
/////////////////////////////////////////
#define DEVID_0				0x00
#define DEVID_1				0x01
#define PARTID				0x02
#define REVID				0x03
#define XID					0x04
#define SOFT_RESET			0x18
#define OFSX				0x1E
#define OFSY				0x1F
#define OFSZ				0x20
#define THRESH_ACT			0x24
#define THRESH_INACT		0x25
#define TIME_INACT			0x26
#define ACT_INACT_CTL		0x27
#define BW_RATE				0x2C
#define POWER_CTL			0x2D
#define INT_ENABLE			0x2E
#define INT_MAP				0x2F
#define INT_SOURCE			0x30
#define DATA_FORMAT			0x31
#define DATA_X0				0x32
#define DATA_X1				0x33
#define DATA_Y0				0x34
#define DATA_Y1				0x35
#define DATA_Z0				0x36
#define DATA_Z1				0x37
#define FIFO_CTL			0x38
#define FIFO_STATUS			0x39

////////////////////////////////
// ADXL313 Responses //
////////////////////////////////
#define DEVID_0_RSP			0xAD
#define DEVID_0_RSP			0x1D
#define PARTID_RSP			0xCB

#endif