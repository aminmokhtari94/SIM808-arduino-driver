/********************************************************************************
 * Example of GNSS with SoftwareSerial and SIM808-arduino-driver         *
 *                                                                              *
 * Author: Amin Mokhtari                                                        *
 * Source: https://github.com/aminmokhtari94/SIM808-arduino-driver              *
 ********************************************************************************
 * MIT License
 *
 * Copyright (c) 2021 Amin Mokhtari
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *******************************************************************************/
#include <SoftwareSerial.h>

#include "SIM808Driver.h"

#define SIM808_RX_PIN 5
#define SIM808_TX_PIN 4
#define SIM808_RST_PIN -1

SIM808Driver *sim808;

void setup()
{
  // Initialize Serial Monitor for debugging
  Serial.begin(115200);
  while (!Serial)
    ;

  // Initialize a SoftwareSerial
  SoftwareSerial *serial = new SoftwareSerial(SIM808_RX_PIN, SIM808_TX_PIN);
  serial->begin(9600);
  delay(1000);

  // Initialize SIM808 driver with an internal buffer of 200 bytes and a reception buffer of 512 bytes, debug disabled
  // sim808 = new SIM808Driver((Stream *)serial, SIM808_RST_PIN, 200, 512);

  // Equivalent line with the debug enabled on the Serial
  sim808 = new SIM808Driver((Stream *)serial, SIM808_RST_PIN, 1512, 1512, (Stream *)&Serial);

  // Setup module for GPRS communication
  setupModule();
}

SIM808Driver::GnssInfo gnssInfo;
void loop()
{
  if (sim808->getGnssInfo(&gnssInfo) == SIM808Driver::GNSS_FIX)
  {

    Serial.print("utc: ");
    Serial.println(gnssInfo.utc);
    Serial.print("lat: ");
    Serial.println(gnssInfo.latitude);
    Serial.print("lng: ");
    Serial.println(gnssInfo.longitude);
  }
  delay(3000);
}

void setupModule()
{
  // Wait until the module is ready to accept AT commands
  while (!sim808->isReady())
  {
    Serial.println(F("Problem to initialize AT command, retry in 1 sec"));
    delay(1000);
  }
  Serial.println(F("Setup Complete!"));

  sim808->powerOnGNSS();
}
