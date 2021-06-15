/********************************************************************************
 * Execute sanity check on the module and output version (software serial)      *
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

#include "SIM808.h"

#define SIM808_RX_PIN 5
#define SIM808_TX_PIN 4
#define SIM808_RST_PIN 6

SIM808 *sim808;

void setup()
{
  // Initialize Serial Monitor for debugging
  Serial.begin(115200);
  while (!Serial)
    ;
  Serial.println("Start !");

  // Initialize a SoftwareSerial
  SoftwareSerial *serial = new SoftwareSerial(SIM808_RX_PIN, SIM808_TX_PIN);
  serial->begin(9600);
  delay(1000);

  // Initialize SIM808 driver with an internal buffer of 200 bytes and a reception buffer of 512 bytes, debug disabled
  sim808 = new SIM808((Stream *)serial, SIM808_RST_PIN, 200, 512);

  // Equivalent line with the debug enabled on the Serial
  //sim808 = new SIM808((Stream *)serial, SIM808_RST_PIN, 200, 512, (Stream *)&Serial);

  Serial.println("Start of test protocol");

  // Wait until the module is ready to accept AT commands
  while (!sim808->isReady())
  {
    Serial.println(F("Problem to initialize AT command, retry in 1 sec"));
    delay(1000);
  }

  Serial.println("Module ready");

  // Print version
  Serial.print("Module ");
  Serial.println(sim808->getVersion());
  Serial.print("Firmware ");
  Serial.println(sim808->getFirmware());

  // Print SIM card number
  Serial.print("SIM card number ");
  Serial.println(sim808->getSimCardNumber());

  // Wait for the GSM signal
  uint8_t signal = sim808->getSignal();
  while (signal <= 0)
  {
    delay(1000);
    signal = sim808->getSignal();
  }

  if (signal > 5)
  {
    Serial.print(F("Signal OK (strenght: "));
  }
  else
  {
    Serial.print(F("Signal low (strenght: "));
  }
  Serial.print(signal);
  Serial.println(F(")"));
  delay(1000);

  // Wait for operator network registration (national or roaming network)
  NetworkRegistration network = sim808->getRegistrationStatus();
  while (network != REGISTERED_HOME && network != REGISTERED_ROAMING)
  {
    delay(1000);
    network = sim808->getRegistrationStatus();
  }
  Serial.println(F("Network registration OK"));

  Serial.println("End of test protocol");
}

void loop()
{
}
