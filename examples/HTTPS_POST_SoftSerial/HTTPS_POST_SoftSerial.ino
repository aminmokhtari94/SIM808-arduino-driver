/********************************************************************************
 * Example of HTTPS POST with SoftwareSerial and SIM808-arduino-driver         *
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

const char APN[] = "Internet.be";
const char URL[] = "https://postman-echo.com/post";
const char CONTENT_TYPE[] = "application/json";
const char PAYLOAD[] = "{\"name\": \"morpheus\", \"job\": \"leader\"}";

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
  sim808 = new SIM808Driver((Stream *)serial, SIM808_RST_PIN, 200, 512);

  // Equivalent line with the debug enabled on the Serial
  // sim808 = new SIM808Driver((Stream *)&Serial1, SIM808_RST_PIN, 200, 512, (Stream *)&Serial);

  // Setup module for GPRS communication
  setupModule();
}

void loop()
{
  // Establish GPRS connectivity (5 trials)
  bool connected = false;
  for (uint8_t i = 0; i < 5 && !connected; i++)
  {
    delay(1000);
    connected = sim808->connectGPRS();
  }

  // Check if connected, if not reset the module and setup the config again
  if (connected)
  {
    Serial.println(F("GPRS connected !"));
  }
  else
  {
    Serial.println(F("GPRS not connected !"));
    Serial.println(F("Reset the module."));
    sim808->reset();
    setupModule();
    return;
  }

  Serial.println(F("Start HTTP POST..."));

  // Do HTTP POST communication with 10s for the timeout (read and write)
  uint16_t rc = sim808->doPost(URL, CONTENT_TYPE, PAYLOAD, 10000, 10000);
  if (rc == 200)
  {
    // Success, output the data received on the serial
    Serial.print(F("HTTP POST successful ("));
    Serial.print(sim808->getDataSizeReceived());
    Serial.println(F(" bytes)"));
    Serial.print(F("Received : "));
    Serial.println(sim808->getDataReceived());
  }
  else
  {
    // Failed...
    Serial.print(F("HTTP POST error "));
    Serial.println(rc);
  }

  // Close GPRS connectivity (5 trials)
  bool disconnected = sim808->disconnectGPRS();
  for (uint8_t i = 0; i < 5 && !connected; i++)
  {
    delay(1000);
    disconnected = sim808->disconnectGPRS();
  }

  if (disconnected)
  {
    Serial.println(F("GPRS disconnected !"));
  }
  else
  {
    Serial.println(F("GPRS still connected !"));
  }

  // Go into low power mode
  bool lowPowerMode = sim808->setPowerMode(SIM808Driver::POW_MINIMUM);
  if (lowPowerMode)
  {
    Serial.println(F("Module in low power mode"));
  }
  else
  {
    Serial.println(F("Failed to switch module to low power mode"));
  }

  // End of program... wait...
  while (1)
    ;
  delay(10000);
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

  // Wait for the GSM signal
  uint8_t signal = sim808->getSignal();
  while (signal <= 0)
  {
    delay(1000);
    signal = sim808->getSignal();
  }
  Serial.print(F("Signal OK (strenght: "));
  Serial.print(signal);
  Serial.println(F(")"));
  delay(1000);

  // Wait for operator network registration (national or roaming network)
  SIM808Driver::NetworkRegistration network = sim808->getRegistrationStatus();
  while (network != SIM808Driver::NET_REGISTERED_HOME && network != SIM808Driver::NET_REGISTERED_ROAMING)
  {
    delay(1000);
    network = sim808->getRegistrationStatus();
  }
  Serial.println(F("Network registration OK"));
  delay(1000);

  // Setup APN for GPRS configuration
  bool success = sim808->setupGPRS(APN);
  while (!success)
  {
    success = sim808->setupGPRS(APN);
    delay(5000);
  }
  Serial.println(F("GPRS config OK"));
}
