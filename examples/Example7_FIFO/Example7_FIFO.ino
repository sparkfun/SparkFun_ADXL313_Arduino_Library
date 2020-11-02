/******************************************************************************
  Example7_FIFO.ino
  Shows how to setup the FIFO on the ADXL313.
  One key advantage of using the FIFO is that it allows us to
  let the ADXL313 store up to 32 samples in it's FIFO "buffer".
  While it is doing this, we can use our microcontroller to do other things,
  Then, when the FIFO is full (or close to), we can quickly read in the 32 samples.

  In order to use the FIFO in this way, we need to set it up to fire an interrupt
  when it gets "almost full". This threshold of samples is called the "watermark".
  When the watermark level is reached, it will fire the interrupt INT1.
  Our arduino will be waiting to see this INT1 pin go HIGH, and then quickly
  read whatevers in the FIFO and print it to terminal.

  Some timestamps of each stage of this cycle will also be printed.
  This will allow us to fine tune bandwidth and watermark settings.
  The "Entries" of the FIFO_STATUS register will also be printed before each read.
  This will tell us how many samples are currently held in the FIFO.
  This will allow us to read the entire contents and keep an eye on how full it is
  getting before each read. This will help us fine tune how much time we have
  between each read to do other things. (in this example, we are simplly going to do
  a delay adn print dots, but you could choose to do more useful things).

  **SPI app note***
  Note, this example uses I2C to communicate the the sensor.
  If you are going to use SPI, then you will need to add in a sufficient
  delay in between reads (at least 5uSec), to allow the FIFO to "pop" the next
  reading in the data registers. See datasheet page 16 for more info.

  ///// FIFO setup //////
  Stream mode
  Trigger INT1
  Watermark Threshold (aka (samples in FIFO_CTL register)): 30

  ///// Interrupt setup //////
  Enable watermark interrupt.
  Map watermark interrupt to "int pin 1".
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

bool interruptFlag = false; // global variabl to keep track of new interrupts. Only ever set true by ISR
unsigned long lastWatermarkTime; // used for printing timestamps in debug
byte fifoEntriesAmount; // used to know how much is currently in the fifo and make sure to read it all out.

void setup()
{
  pinMode(2, INPUT); // setup for interrupt

  Serial.begin(115200);
  Serial.println("Example 7 - FIFO reading with debug info about timing.");

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

  // set bandwidth
  // note, 12.5Hz was chosen for this example to highlight the FIFO wait/read cycle
  // you can tweak BW and the fifo sample threshhold to suit your application.
  myAdxl.setBandwidth(ADXL313_BW_12_5);

  // setup activity sensing options
  myAdxl.setActivityX(false); // disable x-axis participation in detecting activity
  myAdxl.setActivityY(false); // disable y-axis participation in detecting activity
  myAdxl.setActivityZ(false); // disable z-axis participation in detecting activity

  // setup inactivity sensing options
  myAdxl.setInactivityX(false); // disable x-axis participation in detecting inactivity
  myAdxl.setInactivityY(false); // disable y-axis participation in detecting inactivity
  myAdxl.setInactivityZ(false); // disable z-axis participation in detecting inactivity

  // FIFO SETUP
  myAdxl.setFifoMode(ADXL313_FIFO_MODE_STREAM);
  myAdxl.setFifoSamplesThreshhold(30); // 1-32
  myAdxl.setInterruptMapping(ADXL313_INT_WATERMARK_BIT, ADXL313_INT1_PIN);

  // enable/disable interrupts
  // note, we set them all here, just in case there were previous settings,
  // that need to be changed for this example to work properly.
  myAdxl.InactivityINT(false);  // disable inactivity
  myAdxl.ActivityINT(false);    // disable activity
  myAdxl.DataReadyINT(false);   // disable dataReady
  myAdxl.WatermarkINT(true);    // enable fifo watermark

  myAdxl.autosleepOff(); // just in case it was set from a previous setup

  myAdxl.measureModeOn(); // wakes up the sensor from standby and puts it into measurement mode

  // print statuses, to verify we're setup correctly
  Serial.println();
  Serial.print("activity int enable: ");
  Serial.println(myAdxl.isInterruptEnabled(ADXL313_INT_ACTIVITY_BIT));
  Serial.print("inactivity int enable: ");
  Serial.println(myAdxl.isInterruptEnabled(ADXL313_INT_INACTIVITY_BIT));
  Serial.print("dataReady int enable: ");
  Serial.println(myAdxl.isInterruptEnabled(ADXL313_INT_DATA_READY_BIT));
  Serial.print("FIFO watermark int enable: ");
  Serial.println(myAdxl.isInterruptEnabled(ADXL313_INT_WATERMARK_BIT));
  Serial.print("FIFO watermark Samples Threshhold: ");
  Serial.println(myAdxl.getFifoSamplesThreshhold());
  Serial.print("FIFO mode: ");
  Serial.println(myAdxl.getFifoMode()); // should be "2" for stream mode

  attachInterrupt(digitalPinToInterrupt(2), int1_ISR, RISING); // note, the INT output on the ADXL313 is default active HIGH.

  lastWatermarkTime = micros();

  myAdxl.clearFifo(); // clear FIFO for a fresh start on this example. 
  // The FIFO may have be full from previous use 
  // and then would fail to cause a rising interrupt when starting this example.
}

void loop()
{
  if (interruptFlag == true) // FIFO watermark reached (this variable is only ever set true in int1_ISR)
  {
    interruptFlag = false;
    byte entries = myAdxl.getFifoEntriesAmount(); // this should always be the watermark threshhold amount

    Serial.println();
    Serial.print("Watermark interrupt! Time since last read: ");
    unsigned long timegap = (micros() - lastWatermarkTime);
    Serial.print(timegap);
    Serial.print(" us (");
    Serial.print(timegap/1000);
    Serial.print(" ms)\tEntries: ");
    Serial.println(entries); // keep an eye on this while chooseing BW and fifoSamplesThreshhold
    lastWatermarkTime = micros(); // remember for next interrupt time gap calculation

    while (entries > 0) // clear the entire contents of FIFO
    {
      myAdxl.updateIntSourceStatuses();
      if (myAdxl.intSource.dataReady)
      {
        myAdxl.readAccel(); // read all 3 axis, they are stored in class variables: myAdxl.x, myAdxl.y and myAdxl.z
        Serial.print("entries: ");
        Serial.print(entries);
        Serial.print("\tx: ");
        Serial.print(myAdxl.x);
        Serial.print("\ty: ");
        Serial.print(myAdxl.y);
        Serial.print("\tz: ");
        Serial.print(myAdxl.z);
        Serial.println();
        entries -= 1; // we've read one more entry, so let's keep track and keep going until we're done
      }
      //delayMicroseconds(5); // use this if in SPI mode to ensure enought time for the FIFO to "pop" the next value into data read registers.
    }
  }
  // Put your other code in the following while loop, or put your microcontroller to sleep
  // remember, no long delays, because we don't want to miss samples if we don't have to.
  int uSecTimer = 0; // used to prints some dots during down time.
  while(interruptFlag == false)
  {
    if(uSecTimer == 10000) // every 10ms print a dot
    {
      Serial.print("."); // simply here to show we're still alive and waiting for an interrupt.
      uSecTimer = 0; // reset
    }
    uSecTimer += 1;
    delayMicroseconds(1);
  }
}

// watermark has caused an interrupt
void int1_ISR()
{
  interruptFlag = true;
}