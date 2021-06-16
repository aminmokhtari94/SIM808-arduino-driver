/********************************************************************************
 * SIM808-arduino-driver                                                        *
 * ----------------------                                                       *
 * Arduino driver for GSM/GPRS+GNSS modules SIMCom SIM808/SIM868 to make HTTP/S *
 * connections with GET and POST methods and use GPS/GNSS functions             *
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
#include "SIM808Driver.h"

/**
 * AT commands required (const char in PROGMEM to save memory usage)
 */
const char AT_CMD_BASE[] PROGMEM = "AT"; // Basic AT command to check the link

const char AT_CMD_CSQ[] PROGMEM = "AT+CSQ";       // Check the signal strengh
const char AT_CMD_ATI[] PROGMEM = "ATI";          // Output version of the module
const char AT_CMD_GMR[] PROGMEM = "AT+GMR";       // Output version of the firmware
const char AT_CMD_SIM_CARD[] PROGMEM = "AT+CCID"; // Get Sim Card version

const char AT_CMD_CFUN_TEST[] PROGMEM = "AT+CFUN?"; // Check the current power mode
const char AT_CMD_CFUN0[] PROGMEM = "AT+CFUN=0";    // Switch minimum power mode
const char AT_CMD_CFUN1[] PROGMEM = "AT+CFUN=1";    // Switch normal power mode
const char AT_CMD_CFUN4[] PROGMEM = "AT+CFUN=4";    // Switch sleep power mode

const char AT_CMD_CREG_TEST[] PROGMEM = "AT+CREG?";                           // Check the network registration status
const char AT_CMD_SAPBR_GPRS[] PROGMEM = "AT+SAPBR=3,1,\"Contype\",\"GPRS\""; // Configure the GPRS bearer
const char AT_CMD_SAPBR_APN[] PROGMEM = "AT+SAPBR=3,1,\"APN\",";              // Configure the APN for the GPRS
const char AT_CMD_SAPBR1[] PROGMEM = "AT+SAPBR=1,1";                          // Connect GPRS
const char AT_CMD_SAPBR0[] PROGMEM = "AT+SAPBR=0,1";                          // Disconnect GPRS

const char AT_CMD_HTTPINIT[] PROGMEM = "AT+HTTPINIT";                        // Init HTTP connection
const char AT_CMD_HTTPPARA_CID[] PROGMEM = "AT+HTTPPARA=\"CID\",1";          // Connect HTTP through GPRS bearer
const char AT_CMD_HTTPPARA_URL[] PROGMEM = "AT+HTTPPARA=\"URL\",";           // Define the URL to connect in HTTP
const char AT_CMD_HTTPPARA_USERDATA[] PROGMEM = "AT+HTTPPARA=\"USERDATA\","; // Define the header(s)
const char AT_CMD_HTTPPARA_CONTENT[] PROGMEM = "AT+HTTPPARA=\"CONTENT\",";   // Define the content type for the HTTP POST
const char AT_CMD_HTTPSSL_Y[] PROGMEM = "AT+HTTPSSL=1";                      // Enable SSL for HTTP connection
const char AT_CMD_HTTPSSL_N[] PROGMEM = "AT+HTTPSSL=0";                      // Disable SSL for HTTP connection
const char AT_CMD_HTTPACTION0[] PROGMEM = "AT+HTTPACTION=0";                 // Launch HTTP GET action
const char AT_CMD_HTTPACTION1[] PROGMEM = "AT+HTTPACTION=1";                 // Launch HTTP POST action
const char AT_CMD_HTTPREAD[] PROGMEM = "AT+HTTPREAD";                        // Start reading HTTP return data
const char AT_CMD_HTTPTERM[] PROGMEM = "AT+HTTPTERM";                        // Terminate HTTP connection

const char AT_CMD_CGNSPWR1[] PROGMEM = "AT+CGNSPWR=1";    // Power On GNSS
const char AT_CMD_CGNSPWR0[] PROGMEM = "AT+CGNSPWR=0";    // Power Off GNSS
const char AT_CMD_CGNSPWR_TEST[] PROGMEM = "AT+CGNSPWR?"; // Get power status of GNSS
const char AT_CMD_CGNSURC[] PROGMEM = "AT+CGNSURC=";      // GNSS navigation, GEO-fences and speed alarm URC report
const char AT_CMD_CGNSINF[] PROGMEM = "AT+CGNSINF";       // Get GNSS navigation information parsed from NMEA sentences
const char AT_RSP_CGNSINF[] PROGMEM = "+CGNSINF: ";       // Expected answer CGNSINF
const char AT_RSP_UGNSINF[] PROGMEM = "+UGNSINF: ";       // Expected answer CGNSURC

const char AT_RSP_OK[] PROGMEM = "OK";                // Expected answer OK
const char AT_RSP_DOWNLOAD[] PROGMEM = "DOWNLOAD";    // Expected answer DOWNLOAD
const char AT_RSP_HTTPREAD[] PROGMEM = "+HTTPREAD: "; // Expected answer HTTPREAD

/**
 * Constructor; Init the driver, communication with the module and shared
 * buffer used by the driver (to avoid multiples allocation)
 */
SIM808Driver::SIM808Driver(Stream *_stream, uint8_t _pinRst, uint16_t _internalBufferSize, uint16_t _recvBufferSize, Stream *_debugStream)
{
  // Store local variables
  stream = _stream;
  enableDebug = _debugStream != NULL;
  debugStream = _debugStream;
  pinReset = _pinRst;

  if (pinReset != RESET_PIN_NOT_USED)
  {
    // Setup the reset pin and force a reset of the module
    pinMode(pinReset, OUTPUT);
    reset();
  }

  // Prepare internal buffers
  if (enableDebug)
  {
    debugStream->print(F("SIM808Driver : Prepare internal buffer of "));
    debugStream->print(_internalBufferSize);
    debugStream->println(F(" bytes"));
  }
  internalBufferSize = _internalBufferSize;
  internalBuffer = (char *)malloc(internalBufferSize);

  if (enableDebug)
  {
    debugStream->print(F("SIM808Driver : Prepare reception buffer of "));
    debugStream->print(_recvBufferSize);
    debugStream->println(F(" bytes"));
  }
  recvBufferSize = _recvBufferSize;
  recvBuffer = (char *)malloc(recvBufferSize);
}

/**
 * Destructor; cleanup the memory allocated by the driver
 */
SIM808Driver::~SIM808Driver()
{
  free(internalBuffer);
  free(recvBuffer);
}

/*****************************************************************************************
 * HTTP/S FUNCTIONS
 *****************************************************************************************/
/**
 * Do HTTP/S POST to a specific URL
 */
uint16_t SIM808Driver::doPost(const char *url, const char *contentType, const char *payload, uint16_t clientWriteTimeoutMs, uint16_t serverReadTimeoutMs)
{
  return doPost(url, NULL, contentType, payload, clientWriteTimeoutMs, serverReadTimeoutMs);
}

/**
 * Do HTTP/S POST to a specific URL with headers
 */
uint16_t SIM808Driver::doPost(const char *url, const char *headers, const char *contentType, const char *payload, uint16_t clientWriteTimeoutMs, uint16_t serverReadTimeoutMs)
{
  // Cleanup the receive buffer
  initRecvBuffer();
  dataSize = 0;

  // Initiate HTTP/S session with the module
  uint16_t initRC = initiateHTTP(url, headers);
  if (initRC > 0)
  {
    return initRC;
  }

  // Define the content type
  sendCommand_P(AT_CMD_HTTPPARA_CONTENT, contentType);
  if (!readResponseCheckAnswer_P(DEFAULT_TIMEOUT, AT_RSP_OK))
  {
    if (enableDebug)
      debugStream->println(F("SIM808Driver : doPost() - Unable to define the content type"));
    return 702;
  }

  // Prepare to send the payload
  char *tmpBuf = (char *)malloc(30);
  sprintf(tmpBuf, "AT+HTTPDATA=%d,%d", strlen(payload), clientWriteTimeoutMs);
  sendCommand(tmpBuf);
  free(tmpBuf);
  if (!readResponseCheckAnswer_P(DEFAULT_TIMEOUT, AT_RSP_DOWNLOAD))
  {
    if (enableDebug)
      debugStream->println(F("SIM808Driver : doPost() - Unable to send payload to module"));
    return 707;
  }

  // Write the payload on the module
  if (enableDebug)
  {
    debugStream->print(F("SIM808Driver : doPost() - Payload to send : "));
    debugStream->println(payload);
  }

  purgeSerial();
  stream->write(payload);
  stream->flush();
  delay(500);

  // Start HTTP POST action
  sendCommand_P(AT_CMD_HTTPACTION1);
  if (!readResponseCheckAnswer_P(DEFAULT_TIMEOUT, AT_RSP_OK))
  {
    if (enableDebug)
      debugStream->println(F("SIM808Driver : doPost() - Unable to initiate POST action"));
    return 703;
  }

  // Wait answer from the server
  if (!readResponse(serverReadTimeoutMs))
  {
    if (enableDebug)
      debugStream->println(F("SIM808Driver : doPost() - Server timeout"));
    return 408;
  }

  // Extract status information
  int16_t idxBase = strIndex(internalBuffer, "+HTTPACTION: 1,");
  if (idxBase < 0)
  {
    if (enableDebug)
      debugStream->println(F("SIM808Driver : doPost() - Invalid answer on HTTP POST"));
    return 703;
  }

  // Get the HTTP return code
  uint16_t httpRC = 0;
  httpRC += (internalBuffer[idxBase + 15] - '0') * 100;
  httpRC += (internalBuffer[idxBase + 16] - '0') * 10;
  httpRC += (internalBuffer[idxBase + 17] - '0') * 1;

  if (enableDebug)
  {
    debugStream->print(F("SIM808Driver : doPost() - HTTP status "));
    debugStream->println(httpRC);
  }

  if (httpRC >= 200 && httpRC <= 205)
  {
    // Get the size of the data to receive
    dataSize = 0;
    for (uint16_t i = 0; (internalBuffer[idxBase + 19 + i] - '0') >= 0 && (internalBuffer[idxBase + 19 + i] - '0') <= 9; i++)
    {
      if (i != 0)
      {
        dataSize = dataSize * 10;
      }
      dataSize += (internalBuffer[idxBase + 19 + i] - '0');
    }

    if (enableDebug)
    {
      debugStream->print(F("SIM808Driver : doPost() - Data size received of "));
      debugStream->print(dataSize);
      debugStream->println(F(" bytes"));
    }

    // Ask for reading and detect the start of the reading...
    sendCommand_P(AT_CMD_HTTPREAD);
    if (!readResponseCheckAnswer_P(DEFAULT_TIMEOUT, AT_RSP_HTTPREAD, 2))
    {
      return 705;
    }

    // Read number of bytes defined in the dataSize
    for (uint16_t i = 0; i < dataSize && i < recvBufferSize; i++)
    {
      while (!stream->available())
        ;
      if (stream->available())
      {
        // Load the next char
        recvBuffer[i] = stream->read();
        // If the character is CR or LF, ignore it (it's probably part of the module communication schema)
        if ((recvBuffer[i] == '\r') || (recvBuffer[i] == '\n'))
        {
          i--;
        }
      }
    }

    if (recvBufferSize < dataSize)
    {
      dataSize = recvBufferSize;
      if (enableDebug)
      {
        debugStream->println(F("SIM808Driver : doPost() - Buffer overflow while loading data from HTTP. Keep only first bytes..."));
      }
    }

    // We are expecting a final OK
    if (!readResponseCheckAnswer_P(DEFAULT_TIMEOUT, AT_RSP_OK))
    {
      if (enableDebug)
        debugStream->println(F("SIM808Driver : doPost() - Invalid end of data while reading HTTP result from the module"));
      return 705;
    }

    if (enableDebug)
    {
      debugStream->print(F("SIM808Driver : doPost() - Received from HTTP POST : "));
      debugStream->println(recvBuffer);
    }
  }

  // Terminate HTTP/S session
  uint16_t termRC = terminateHTTP();
  if (termRC > 0)
  {
    return termRC;
  }

  return httpRC;
}

/**
 * Do HTTP/S GET on a specific URL
 */
uint16_t SIM808Driver::doGet(const char *url, uint16_t serverReadTimeoutMs)
{
  return doGet(url, NULL, serverReadTimeoutMs);
}

/**
 * Do HTTP/S GET on a specific URL with headers
 */
uint16_t SIM808Driver::doGet(const char *url, const char *headers, uint16_t serverReadTimeoutMs)
{
  // Cleanup the receive buffer
  initRecvBuffer();
  dataSize = 0;

  // Initiate HTTP/S session
  uint16_t initRC = initiateHTTP(url, headers);
  if (initRC > 0)
  {
    return initRC;
  }

  // Start HTTP GET action
  sendCommand_P(AT_CMD_HTTPACTION0);
  if (!readResponseCheckAnswer_P(DEFAULT_TIMEOUT, AT_RSP_OK))
  {
    if (enableDebug)
      debugStream->println(F("SIM808Driver : doGet() - Unable to initiate GET action"));
    return 703;
  }

  // Wait answer from the server
  if (!readResponse(serverReadTimeoutMs))
  {
    if (enableDebug)
      debugStream->println(F("SIM808Driver : doGet() - Server timeout"));
    return 408;
  }

  // Extract status information
  int16_t idxBase = strIndex(internalBuffer, "+HTTPACTION: 0,");
  if (idxBase < 0)
  {
    if (enableDebug)
      debugStream->println(F("SIM808Driver : doGet() - Invalid answer on HTTP GET"));
    return 703;
  }

  // Get the HTTP return code
  uint16_t httpRC = 0;
  httpRC += (internalBuffer[idxBase + 15] - '0') * 100;
  httpRC += (internalBuffer[idxBase + 16] - '0') * 10;
  httpRC += (internalBuffer[idxBase + 17] - '0') * 1;

  if (enableDebug)
  {
    debugStream->print(F("SIM808Driver : doGet() - HTTP status "));
    debugStream->println(httpRC);
  }

  if (httpRC == 200)
  {
    // Get the size of the data to receive
    dataSize = 0;
    for (uint16_t i = 0; (internalBuffer[idxBase + 19 + i] - '0') >= 0 && (internalBuffer[idxBase + 19 + i] - '0') <= 9; i++)
    {
      if (i != 0)
      {
        dataSize = dataSize * 10;
      }
      dataSize += (internalBuffer[idxBase + 19 + i] - '0');
    }

    if (enableDebug)
    {
      debugStream->print(F("SIM808Driver : doGet() - Data size received of "));
      debugStream->print(dataSize);
      debugStream->println(F(" bytes"));
    }

    // Ask for reading and detect the start of the reading...
    sendCommand_P(AT_CMD_HTTPREAD);
    if (!readResponseCheckAnswer_P(DEFAULT_TIMEOUT, AT_RSP_HTTPREAD, 2))
    {
      return 705;
    }

    // Read number of bytes defined in the dataSize
    for (uint16_t i = 0; i < dataSize && i < recvBufferSize; i++)
    {
      while (!stream->available())
        ;
      if (stream->available())
      {
        // Load the next char
        recvBuffer[i] = stream->read();
        // If the character is CR or LF, ignore it (it's probably part of the module communication schema)
        if ((recvBuffer[i] == '\r') || (recvBuffer[i] == '\n'))
        {
          i--;
        }
      }
    }

    if (recvBufferSize < dataSize)
    {
      dataSize = recvBufferSize;
      if (enableDebug)
      {
        debugStream->println(F("SIM808Driver : doGet() - Buffer overflow while loading data from HTTP. Keep only first bytes..."));
      }
    }

    // We are expecting a final OK
    if (!readResponseCheckAnswer_P(DEFAULT_TIMEOUT, AT_RSP_OK))
    {
      if (enableDebug)
        debugStream->println(F("SIM808Driver : doGet() - Invalid end of data while reading HTTP result from the module"));
      return 705;
    }

    if (enableDebug)
    {
      debugStream->print(F("SIM808Driver : doGet() - Received from HTTP GET : "));
      debugStream->println(recvBuffer);
    }
  }

  // Terminate HTTP/S session
  uint16_t termRC = terminateHTTP();
  if (termRC > 0)
  {
    return termRC;
  }

  return httpRC;
}

/**
 * Meta method to initiate the HTTP/S session on the module
 */
uint16_t SIM808Driver::initiateHTTP(const char *url, const char *headers)
{
  // Init HTTP connection
  sendCommand_P(AT_CMD_HTTPINIT);
  if (!readResponseCheckAnswer_P(DEFAULT_TIMEOUT, AT_RSP_OK))
  {
    if (enableDebug)
      debugStream->println(F("SIM808Driver : initiateHTTP() - Unable to init HTTP"));
    return 701;
  }

  // Use the GPRS bearer
  sendCommand_P(AT_CMD_HTTPPARA_CID);
  if (!readResponseCheckAnswer_P(DEFAULT_TIMEOUT, AT_RSP_OK))
  {
    if (enableDebug)
      debugStream->println(F("SIM808Driver : initiateHTTP() - Unable to define bearer"));
    return 702;
  }

  // Define URL to look for
  sendCommand_P(AT_CMD_HTTPPARA_URL, url);
  if (!readResponseCheckAnswer_P(DEFAULT_TIMEOUT, AT_RSP_OK))
  {
    if (enableDebug)
      debugStream->println(F("SIM808Driver : initiateHTTP() - Unable to define the URL"));
    return 702;
  }

  // Set Headers
  if (headers != NULL)
  {
    sendCommand_P(AT_CMD_HTTPPARA_USERDATA, headers);
    if (!readResponseCheckAnswer_P(DEFAULT_TIMEOUT, AT_RSP_OK))
    {
      if (enableDebug)
        debugStream->println(F("SIM808Driver : initiateHTTP() - Unable to define Headers"));
      return 702;
    }
  }

  // Check if the firmware support HTTPSSL command
  bool isSupportSSL = false;
  char *version = getVersion();
  int16_t rIdx = strIndex(version, "R");
  if (rIdx > 0)
  {
    uint8_t releaseInt = (version[rIdx + 1] - '0') * 10 + (version[rIdx + 2] - '0');

    // The release should be greater or equals to 14 to support SSL stack
    if (releaseInt >= 14)
    {
      isSupportSSL = true;
      if (enableDebug)
        debugStream->println(F("SIM808Driver : initiateHTTP() - Support of SSL enabled"));
    }
    else
    {
      isSupportSSL = false;
      if (enableDebug)
        debugStream->println(F("SIM808Driver : initiateHTTP() - Support of SSL disabled (SIM808 firware below R14)"));
    }
  }

  // Send HTTPSSL command only if the version is greater or equals to 14
  if (isSupportSSL)
  {
    // HTTP or HTTPS
    if (strIndex(url, "https://") == 0)
    {
      sendCommand_P(AT_CMD_HTTPSSL_Y);
      if (!readResponseCheckAnswer_P(DEFAULT_TIMEOUT, AT_RSP_OK))
      {
        if (enableDebug)
          debugStream->println(F("SIM808Driver : initiateHTTP() - Unable to switch to HTTPS"));
        return 702;
      }
    }
    else
    {
      sendCommand_P(AT_CMD_HTTPSSL_N);
      if (!readResponseCheckAnswer_P(DEFAULT_TIMEOUT, AT_RSP_OK))
      {
        if (enableDebug)
          debugStream->println(F("SIM808Driver : initiateHTTP() - Unable to switch to HTTP"));
        return 702;
      }
    }
  }

  return 0;
}

/**
 * Meta method to terminate the HTTP/S session on the module
 */
uint16_t SIM808Driver::terminateHTTP()
{
  // Close HTTP connection
  sendCommand_P(AT_CMD_HTTPTERM);
  if (!readResponseCheckAnswer_P(DEFAULT_TIMEOUT, AT_RSP_OK))
  {
    if (enableDebug)
      debugStream->println(F("SIM808Driver : terminateHTTP() - Unable to close HTTP session"));
    return 706;
  }
  return 0;
}

/**
 * Return the size of data received after the last successful HTTP connection
 */
uint16_t SIM808Driver::getDataSizeReceived()
{
  return dataSize;
}

/**
 * Return the buffer of data received after the last successful HTTP connection
 */
char *SIM808Driver::getDataReceived()
{
  return recvBuffer;
}

/*****************************************************************************************
 * GNSS FUNCTIONS
 *****************************************************************************************/
/**
 * Power On GNSS on the module
*/
bool SIM808Driver::powerOnGNSS()
{
  // Send Power on GNSS cmd
  sendCommand_P(AT_CMD_CGNSPWR1);

  if (!readResponseCheckAnswer_P(DEFAULT_TIMEOUT, AT_RSP_OK))
  {
    if (enableDebug)
      debugStream->println(F("SIM808Driver : powerOnGNSS() - Unable to Power on GNSS"));
    return false;
  }
  return true;
}

/**
 * Power Off GNSS on the module
*/
bool SIM808Driver::powerOffGNSS()
{
  // Send Power off GNSS cmd
  sendCommand_P(AT_CMD_CGNSPWR0);

  if (!readResponseCheckAnswer_P(DEFAULT_TIMEOUT, AT_RSP_OK))
  {
    if (enableDebug)
      debugStream->println(F("SIM808Driver : powerOnGNSS() - Unable to Power off GNSS"));
    return false;
  }
  return true;
}

/**
 *  Get power status of GNSS
*/
SIM808Driver::GnssStatus SIM808Driver::getGnssPowerStatus()
{
  sendCommand_P(AT_CMD_CGNSPWR_TEST);
  if (readResponse(DEFAULT_TIMEOUT))
  {
    // Check if there is an error
    int16_t errIdx = strIndex(internalBuffer, "ERROR");
    if (errIdx > 0)
    {
      if (enableDebug)
        debugStream->println(F("SIM808Driver : getGnssPowerStatus() - Error on getting GNSS Power"));
      return GNSS_ERROR;
    }

    // Extract the value
    int16_t idx = strIndex(internalBuffer, "+CGNSPWR: ");
    char value = internalBuffer[idx + 10];

    if (value == '1')
      return GNSS_POWER_ON;
    else
      return GNSS_POWER_OFF;
  }
  return GNSS_ERROR;
}

/**
 * Turn on navigation data URC report, and report every GNSS FIX
*/
bool SIM808Driver::attachGNSS(uint8_t fix)
{
  // Check if GNSS is On
  if (!getGnssPowerStatus())
  {
    if (enableDebug)
      debugStream->println(F("SIM808Driver : attachGNSS() - GNSS Power in off"));
    return false;
  }

  char buff[3];
  // Send Power off GNSS cmd
  sendCommand_P(AT_CMD_CGNSURC, itoa(fix, buff, 10));

  if (!readResponseCheckAnswer_P(DEFAULT_TIMEOUT, AT_RSP_OK))
  {
    if (enableDebug)
      debugStream->println(F("SIM808Driver : attachGNSS() - Unable to attach GNSS"));
    return false;
  }
  return true;
}

/**
 * Turn off GNSS navigation, GEO-fences and speed alarm URC report 
*/
bool SIM808Driver::detachGNSS()
{
  // Send Power off GNSS cmd
  sendCommand_P(AT_CMD_CGNSURC, "0");

  if (!readResponseCheckAnswer_P(DEFAULT_TIMEOUT, AT_RSP_OK))
  {
    if (enableDebug)
      debugStream->println(F("SIM808Driver : detachGNSS() - Unable to Power off GNSS"));
    return false;
  }
  return true;
}

/**
 * Turn off GNSS navigation, GEO-fences and speed alarm URC report 
*/
SIM808Driver::GnssStatus SIM808Driver::getGnssInfo()
{
  // Check if GNSS is On
  if (!getGnssPowerStatus())
  {
    if (enableDebug)
      debugStream->println(F("SIM808Driver : getGnssInfo() - Unable to get GNSS Info, GNSS Power in off"));
    return GNSS_POWER_OFF;
  }

  // get Info
  sendCommand_P(AT_CMD_CGNSINF);
}

/*****************************************************************************************
 * BASE CONTROLL & CHECK FUNCTIONS
 *****************************************************************************************/
/**
 * Force a reset of the module
 */
void SIM808Driver::reset()
{
  if (pinReset != RESET_PIN_NOT_USED)
  {
    // Some logging
    if (enableDebug)
      debugStream->println(F("SIM808Driver : Reset"));

    // Reset the device
    digitalWrite(pinReset, HIGH);
    delay(500);
    digitalWrite(pinReset, LOW);
    delay(500);
    digitalWrite(pinReset, HIGH);
    delay(1000);
  }
  else
  {
    // Some logging
    if (enableDebug)
      debugStream->println(F("SIM808Driver : Reset requested but reset pin undefined"));

    // Set power to minimum and back it to normal for soft ressetting
    if (setPowerMode(POW_MINIMUM))
    {
      delay(1000);
      setPowerMode(POW_NORMAL);
    }
  }

  // Purge the serial
  stream->flush();
  while (stream->available())
  {
    stream->read();
  }
}

/**
 * Status function: Check if AT command works
 */
bool SIM808Driver::isReady()
{
  sendCommand_P(AT_CMD_BASE);
  return readResponseCheckAnswer_P(DEFAULT_TIMEOUT, AT_RSP_OK);
}

/**
 * Status function: Check the power mode
 */
SIM808Driver::PowerMode SIM808Driver::getPowerMode()
{
  sendCommand_P(AT_CMD_CFUN_TEST);
  if (readResponse(DEFAULT_TIMEOUT))
  {
    // Check if there is an error
    int16_t errIdx = strIndex(internalBuffer, "ERROR");
    if (errIdx > 0)
    {
      return POW_ERROR;
    }

    // Extract the value
    int16_t idx = strIndex(internalBuffer, "+CFUN: ");
    char value = internalBuffer[idx + 7];

    // Prepare the clear output
    switch (value)
    {
    case '0':
      return POW_MINIMUM;
    case '1':
      return POW_NORMAL;
    case '4':
      return POW_SLEEP;
    default:
      return POW_UNKNOWN;
    }
  }
  return POW_ERROR;
}

/**
 * Status function: Get version of the module
 */
char *SIM808Driver::getVersion()
{
  sendCommand_P(AT_CMD_ATI);
  if (readResponse(DEFAULT_TIMEOUT))
  {
    // Extract the value
    int16_t idx = strIndex(internalBuffer, "SIM");
    int16_t idxEnd = strIndex(internalBuffer, "\r", idx + 1);

    // Store it on the recv buffer (not used at the moment)
    initRecvBuffer();
    for (uint16_t i = 0; i < idxEnd - idx; i++)
    {
      recvBuffer[i] = internalBuffer[idx + i];
    }
    return getDataReceived();
  }
  else
  {
    return NULL;
  }
}

/**
 * Status function: Get firmware version
 */
char *SIM808Driver::getFirmware()
{
  sendCommand_P(AT_CMD_GMR);
  if (readResponse(DEFAULT_TIMEOUT))
  {
    // Extract the value
    int16_t idx = strIndex(internalBuffer, "AT+GMR") + 9;
    int16_t idxEnd = strIndex(internalBuffer, "\r", idx + 1);

    // Store it on the recv buffer (not used at the moment)
    initRecvBuffer();
    for (uint16_t i = 0; i < idxEnd - idx; i++)
    {
      recvBuffer[i] = internalBuffer[idx + i];
    }
    return getDataReceived();
  }
  else
  {
    return NULL;
  }
}

/**
 * Status function: Requests the simcard number
 */
char *SIM808Driver::getSimCardNumber()
{
  sendCommand_P(AT_CMD_SIM_CARD);
  if (readResponse(DEFAULT_TIMEOUT))
  {
    int16_t idx = strIndex(internalBuffer, "AT+CCID") + 10;
    int16_t idxEnd = strIndex(internalBuffer, "\r", idx + 1);

    // Store it on the recv buffer (not used at the moment)
    initRecvBuffer();
    for (uint16_t i = 0; i < idxEnd - idx; i++)
    {
      recvBuffer[i] = internalBuffer[idx + i];
    }
    return getDataReceived();
  }
  else
  {
    return NULL;
  }
}

/**
 * Status function: Check if the module is registered on the network
 */
SIM808Driver::NetworkRegistration SIM808Driver::getRegistrationStatus()
{
  sendCommand_P(AT_CMD_CREG_TEST);
  if (readResponse(DEFAULT_TIMEOUT))
  {
    // Check if there is an error
    int16_t errIdx = strIndex(internalBuffer, "ERROR");
    if (errIdx > 0)
    {
      return NET_ERROR;
    }

    // Extract the value
    int16_t idx = strIndex(internalBuffer, "+CREG: ");
    char value = internalBuffer[idx + 9];

    // Prepare the clear output
    switch (value)
    {
    case '0':
      return NET_NOT_REGISTERED;
    case '1':
      return NET_REGISTERED_HOME;
    case '2':
      return NET_SEARCHING;
    case '3':
      return NET_DENIED;
    case '5':
      return NET_REGISTERED_ROAMING;
    default:
      return NET_UNKNOWN;
    }
  }

  return NET_ERROR;
}

/**
 * Setup the GPRS connectivity
 * As input, give the APN string of the operator
 */
bool SIM808Driver::setupGPRS(const char *apn)
{
  // Prepare the GPRS connection as the bearer
  sendCommand_P(AT_CMD_SAPBR_GPRS);
  if (!readResponseCheckAnswer_P(20000, AT_RSP_OK))
  {
    return false;
  }

  // Set the config of the bearer with the APN
  sendCommand_P(AT_CMD_SAPBR_APN, apn);
  return readResponseCheckAnswer_P(20000, AT_RSP_OK);
}

/**
 * Open the GPRS connectivity
 */
bool SIM808Driver::connectGPRS()
{
  sendCommand_P(AT_CMD_SAPBR1);
  // Timout is max 85 seconds according to SIM808 specifications
  // We will wait for 65s to be within uint16_t
  return readResponseCheckAnswer_P(65000, AT_RSP_OK);
}

/**
 * Close the GPRS connectivity
 */
bool SIM808Driver::disconnectGPRS()
{
  sendCommand_P(AT_CMD_SAPBR0);
  // Timout is max 65 seconds according to SIM808 specifications
  return readResponseCheckAnswer_P(65000, AT_RSP_OK);
}

/**
 * Define the power mode
 * Available : POW_MINIMUM, POW_NORMAL, POW_SLEEP
 * Return true is the mode is correctly switched
 */
bool SIM808Driver::setPowerMode(PowerMode powerMode)
{
  // Check if the power mode requested is not ERROR or UNKNOWN
  if (powerMode == POW_ERROR || powerMode == POW_UNKNOWN)
  {
    return false;
  }

  // Check the current power mode
  PowerMode currentPowerMode = getPowerMode();

  // If the current power mode is undefined, abord
  if (currentPowerMode == POW_ERROR || currentPowerMode == POW_UNKNOWN)
  {
    return false;
  }

  // If the current power mode is the same that the requested power mode, say it's OK
  if (currentPowerMode == powerMode)
  {
    return true;
  }

  // If POW_SLEEP or POW_MINIMUM, only POW_NORMAL is allowed
  if ((currentPowerMode == POW_SLEEP || currentPowerMode == POW_MINIMUM) && (powerMode != POW_NORMAL))
  {
    return false;
  }

  // Send the command
  char value;
  switch (powerMode)
  {
  case POW_MINIMUM:
    sendCommand_P(AT_CMD_CFUN0);
    break;
  case POW_SLEEP:
    sendCommand_P(AT_CMD_CFUN4);
    break;
  case POW_NORMAL:
  default:
    sendCommand_P(AT_CMD_CFUN1);
  }

  // Read but don't care about the result
  purgeSerial();

  // Check the current power mode
  currentPowerMode = getPowerMode();

  // If the current power mode is the same that the requested power mode, say it's OK
  return currentPowerMode == powerMode;
}

/**
 * Status function: Check the strengh of the signal
 */
uint8_t SIM808Driver::getSignal()
{
  sendCommand_P(AT_CMD_CSQ);
  if (readResponse(DEFAULT_TIMEOUT))
  {
    int16_t idxBase = strIndex(internalBuffer, "AT+CSQ");
    if (idxBase != 0)
    {
      return 0;
    }
    int16_t idxEnd = strIndex(internalBuffer, ",", idxBase);
    uint8_t value = internalBuffer[idxEnd - 1] - '0';
    if (internalBuffer[idxEnd - 2] != ' ')
    {
      value += (internalBuffer[idxEnd - 2] - '0') * 10;
    }
    if (value > 31)
    {
      return 0;
    }
    return value;
  }
  return 0;
}

/*****************************************************************************************
 * HELPERS
 *****************************************************************************************/
/**
 * Find string "findStr" in another string "str"
 * Returns true if found, false elsewhere
 */
int16_t SIM808Driver::strIndex(const char *str, const char *findStr, uint16_t startIdx)
{
  int16_t firstIndex = -1;
  int16_t sizeMatch = 0;
  for (int16_t i = startIdx; i < strlen(str); i++)
  {
    if (sizeMatch >= strlen(findStr))
    {
      break;
    }
    if (str[i] == findStr[sizeMatch])
    {
      if (firstIndex < 0)
      {
        firstIndex = i;
      }
      sizeMatch++;
    }
    else
    {
      firstIndex = -1;
      sizeMatch = 0;
    }
  }

  if (sizeMatch >= strlen(findStr))
  {
    return firstIndex;
  }
  else
  {
    return -1;
  }
}

/**
 * Init internal buffer
 */
void SIM808Driver::initInternalBuffer()
{
  for (uint16_t i = 0; i < internalBufferSize; i++)
  {
    internalBuffer[i] = '\0';
  }
}

/**
 * Init recv buffer
 */
void SIM808Driver::initRecvBuffer()
{
  // Cleanup the receive buffer
  for (uint16_t i = 0; i < recvBufferSize; i++)
  {
    recvBuffer[i] = 0;
  }
}

/*****************************************************************************************
 * LOW LEVEL FUNCTIONS TO COMMUNICATE WITH THE SIM808 MODULE
 *****************************************************************************************/
/**
 * Send AT command to the module
 */
void SIM808Driver::sendCommand(const char *command)
{
  if (enableDebug)
  {
    debugStream->print(F("SIM808Driver : Send \""));
    debugStream->print(command);
    debugStream->println(F("\""));
  }

  purgeSerial();
  stream->write(command);
  stream->write("\r\n");
  purgeSerial();
}

/**
 * Send AT command coming from the PROGMEM
 */
void SIM808Driver::sendCommand_P(const char *command)
{
  char cmdBuff[32];
  strcpy_P(cmdBuff, command);
  sendCommand(cmdBuff);
}

/**
 * Send AT command to the module with a parameter
 */
void SIM808Driver::sendCommand(const char *command, const char *parameter)
{
  if (enableDebug)
  {
    debugStream->print(F("SIM808Driver : Send \""));
    debugStream->print(command);
    debugStream->print(F("\""));
    debugStream->print(parameter);
    debugStream->print(F("\""));
    debugStream->println(F("\""));
  }

  purgeSerial();
  stream->write(command);
  stream->write("\"");
  stream->write(parameter);
  stream->write("\"");
  stream->write("\r\n");
  purgeSerial();
}

/**
 * Send AT command coming from the PROGMEM with a parameter
 */
void SIM808Driver::sendCommand_P(const char *command, const char *parameter)
{
  char cmdBuff[32];
  strcpy_P(cmdBuff, command);
  sendCommand(cmdBuff, parameter);
}

/**
 * Purge the serial data
 */
void SIM808Driver::purgeSerial()
{
  stream->flush();
  while (stream->available())
  {
    stream->read();
  }
  stream->flush();
}

/**
 * Read from module and expect a specific answer (timeout in millisec)
 */
bool SIM808Driver::readResponseCheckAnswer_P(uint16_t timeout, const char *expectedAnswer, uint8_t crlfToWait)
{
  if (readResponse(timeout, crlfToWait))
  {
    // Prepare the local expected answer
    char rspBuff[16];
    strcpy_P(rspBuff, expectedAnswer);

    // Check if it's the expected answer
    int16_t idx = strIndex(internalBuffer, rspBuff);
    if (idx > 0)
    {
      return true;
    }
  }
  return false;
}

/**
 * Read from the module for a specific number of CRLF
 * True if we have some data
 */
bool SIM808Driver::readResponse(uint16_t timeout, uint8_t crlfToWait)
{
  uint16_t currentSizeResponse = 0;
  bool seenCR = false;
  uint8_t countCRLF = 0;

  // First of all, cleanup the buffer
  initInternalBuffer();

  uint32_t timerStart = millis();

  while (1)
  {
    // While there is data available on the buffer, read it until the max size of the response
    if (stream->available())
    {
      // Load the next char
      internalBuffer[currentSizeResponse] = stream->read();

      // Detect end of transmission (CRLF)
      if (internalBuffer[currentSizeResponse] == '\r')
      {
        seenCR = true;
      }
      else if (internalBuffer[currentSizeResponse] == '\n' && seenCR)
      {
        countCRLF++;
        if (countCRLF == crlfToWait)
        {
          if (enableDebug)
            debugStream->println(F("SIM808Driver : End of transmission"));
          break;
        }
      }
      else
      {
        seenCR = false;
      }

      // Prepare for next read
      currentSizeResponse++;

      // Avoid buffer overflow
      if (currentSizeResponse == internalBufferSize)
      {
        if (enableDebug)
          debugStream->println(F("SIM808Driver : Received maximum buffer size"));
        break;
      }
    }

    // If timeout, abord the reading
    if (millis() - timerStart > timeout)
    {
      if (enableDebug)
        debugStream->println(F("SIM808Driver : Receive timeout"));
      // Timeout, return false to parent function
      return false;
    }
  }

  if (enableDebug)
  {
    debugStream->print(F("SIM808Driver : Receive \""));
    debugStream->print(internalBuffer);
    debugStream->println(F("\""));
  }

  // If we are here, it's OK ;-)
  return true;
}
