#ifndef SMW_SX1276M0_H
#define SMW_SX1276M0_H

/*******************************************************************************
* RoboCore SMW_SX1276M0 Library (v1.0)
* 
* Library to use the SMW_SX1276M0 LoRaWAN module.
* 
* Copyright 2020 RoboCore.
* Written by Francois (13/08/20).
* 
* 
* This file is part of the SMW_SX1276M0 library ("SMW_SX1276M0-lib").
* 
* "SMW_SX1276M0-lib" is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
* 
* "SMW_SX1276M0-lib" is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU Lesser General Public License for more details.
* 
* You should have received a copy of the GNU Lesser General Public License
* along with "SMW_SX1276M0-lib". If not, see <https://www.gnu.org/licenses/>
*******************************************************************************/

#define SMW_SX1276M0_DEBUG

#define SMW_SX1276M0_BUFFER_SIZE            50
#define SMW_SX1276M0_DELAY_INCOMING_DATA    10 // [ms]
#define SMW_SX1276M0_TIMEOUT_READ           10 // [ms]
#define SMW_SX1276M0_TIMEOUT_RESET        5000 // [ms]
#define SMW_SX1276M0_TIMEOUT_WRITE        1000 // [ms]


// --------------------------------------------------
// Libraries

#include <Arduino.h>

extern "C" {
  #include <stdarg.h>
  #include <stdint.h>
  #include <stdlib.h>
}

#include "Buffer.h"


// --------------------------------------------------
// Constants

const char CHAR_EOS = '\0'; // End Of String
const char CHAR_LF = 10; // Line Feed
const char CHAR_CR = 13; // Carriage Return
const char CHAR_SPACE = 32; // ' '
const char CHAR_PLUS = 43; // '+'
const char CHAR_COLON = 58; // ':'
const char CHAR_LT = 60; // '<'
const char CHAR_GT = 62; // '>'


// --------------------------------------------------
// Constants (AT)

const char* const CMD_AT = "AT";

char* const CMD_NONE = nullptr;
const char* const CMD_DEVEUI = "DEVEUI"; // Device EUI (6.2.1)
const char* const CMD_APPEUI = "APPEUI"; // Application EUI (6.2.2)
const char* const CMD_APPKEY = "APPKEY"; // Application Key (6.2.3)
const char* const CMD_NJM = "NJM"; // Join Mode (6.2.5)
const char* const CMD_NJS = "NJS"; // Join Status (6.2.6)
const char* const CMD_JOIN = "JOIN"; // Join (6.2.7)
const char* const CMD_AJOIN = "AJOIN"; // Automatic Join (6.2.9)
const char* const CMD_NWKSKEY = "NWKSKEY"; // Network Session Key (6.2.10)
const char* const CMD_APPSKEY = "APPSKEY"; // Application Session Key (6.2.11)
const char* const CMD_DADDR = "DADDR"; // Device Address (6.2.12)
const char* const CMD_SEND = "SEND"; // Send (7.2.1)
const char* const CMD_SENDB = "SENDB"; // Send B (7.2.2)
const char* const CMD_RECV = "RECV"; // Receive (7.2.3)
const char* const CMD_RECVB = "RECVB"; // Receive B (7.2.4)
const char* const CMD_RSSI = "RSSI"; // RSSI (7.2.5)
const char* const CMD_SNR = "SNR"; // SNR (7.2.6)
const char* const CMD_ADR = "ADR"; // Adaptive Data Rate (8.2.2)
const char* const CMD_DR = "DR"; // Data Rate (8.2.3)
const char* const CMD_RESET = "RESET"; // Reset (9.2.1)
const char* const CMD_VERSION = "VER"; // Version (9.2.3)
const char* const CMD_SLEEP = "SLEEP"; // Alarm (9.2.6)
const char* const CMD_ALARM = "ALARM"; // Alarm (9.2.7)
const char* const CMD_ECHO = "ECHO"; // Echo (9.2.10)

const char* const RSPNS_OK = "OK";
const char* const RSPNS_FAILED = "Failed";
const char* const RSPNS_NOT_FOUND = "Found";
const uint8_t RSPNS_BOOT[] = { 0x07 , '*' };

const char* const RSPNS_EVENT = "[EVENT]";
const char* const RSPNS_JOINED = "JOINED";
const char* const RSPNS_RECV = "RECV";
const char* const RSPNS_SLEEP = "SLEEP";


// --------------------------------------------------
// Constants

#define SMW_SX1276M0_ADR_OFF  0
#define SMW_SX1276M0_ADR_ON   1

#define SMW_SX1276M0_AUTOMATIC_JOIN_OFF  0
#define SMW_SX1276M0_AUTOMATIC_JOIN_ON   1

#define SMW_SX1276M0_ECHO_OFF  0
#define SMW_SX1276M0_ECHO_ON   1

#define SMW_SX1276M0_JOIN_MODE_ABP  0
#define SMW_SX1276M0_JOIN_MODE_OTAA 1
#define SMW_SX1276M0_JOIN_MODE_P2P  2

#define SMW_SX1276M0_JOIN_STATUS_NOT_JOINED 0
#define SMW_SX1276M0_JOIN_STATUS_JOINED     1

enum class CommandResponse : uint8_t { ERROR , OK , FAILED , FAILED_STRING , NOT_FOUND , DATA };
enum class Event : uint8_t { JOINED , RECEIVED , RECEIVED_X , SLEEP , WAKEUP , RESET };


// --------------------------------------------------
// Helper Constants

#define SMW_SX1276M0_SIZE_APPEUI    16
#define SMW_SX1276M0_SIZE_APPKEY    32
#define SMW_SX1276M0_SIZE_APPSKEY   32
#define SMW_SX1276M0_SIZE_DEVEUI    16
#define SMW_SX1276M0_SIZE_DEVADDR    8
#define SMW_SX1276M0_SIZE_NWKSKEY   32
#define SMW_SX1276M0_SIZE_VERSION   10


// --------------------------------------------------
// Class

class SMW_SX1276M0 {
  public:
    void (*event_listener)(Event);
    
    SMW_SX1276M0(Stream (&));
    SMW_SX1276M0(Stream (&), int16_t);
    void flush(void);
    CommandResponse get_ADR(uint8_t (&));
    CommandResponse get_AJoin(uint8_t (&));
    CommandResponse get_Alarm(uint32_t (&));
    CommandResponse get_AppEUI(char (&)[SMW_SX1276M0_SIZE_APPEUI]);
    CommandResponse get_AppKey(char (&)[SMW_SX1276M0_SIZE_APPKEY]);
    CommandResponse get_AppSKey(char (&)[SMW_SX1276M0_SIZE_APPSKEY]);
    CommandResponse get_DevAddr(char (&)[SMW_SX1276M0_SIZE_DEVADDR]);
    CommandResponse get_DevEUI(char (&)[SMW_SX1276M0_SIZE_DEVEUI]);
    CommandResponse get_DR(uint8_t (&));
    CommandResponse get_Echo(uint8_t (&));
    void get_buffer(Buffer (&));
    CommandResponse get_JoinMode(uint8_t (&));
    CommandResponse get_JoinStatus(uint8_t (&));
    CommandResponse get_NwkSKey(char (&)[SMW_SX1276M0_SIZE_NWKSKEY]);
    CommandResponse get_RSSI(double (&));
    CommandResponse get_SNR(double (&));
    CommandResponse get_Version(char (&)[SMW_SX1276M0_SIZE_VERSION]);
    bool isConnected(void);
    bool isSleeping(void);
    void join(void);
    CommandResponse listen(bool = true);
    CommandResponse ping(void);
    CommandResponse readT(void);
    CommandResponse readT(Buffer (&));
    CommandResponse readT(uint8_t (&), Buffer (&));
    CommandResponse readX(void);
    CommandResponse readX(Buffer (&));
    CommandResponse readX(uint8_t (&), Buffer (&));
    CommandResponse reset(void);
    CommandResponse sendT(uint8_t, const char *);
    CommandResponse sendT(uint8_t, const String);
    CommandResponse sendX(uint8_t, const char *);
    CommandResponse sendX(uint8_t, const String);
    CommandResponse set_ADR(uint8_t);
    CommandResponse set_AJoin(uint8_t);
    CommandResponse set_Alarm(uint32_t);
    CommandResponse set_AppEUI(const char *);
    CommandResponse set_AppKey(const char *);
    CommandResponse set_AppSKey(const char *);
    CommandResponse set_DevAddr(const char *);
    CommandResponse set_DevEUI(const char *);
    CommandResponse set_DR(uint8_t);
    CommandResponse set_Echo(uint8_t);
    CommandResponse set_JoinMode(uint8_t);
    CommandResponse set_NwkSKey(const char *);
    void setPinReset(int16_t);
    CommandResponse sleep(uint32_t = 0);

#ifdef SMW_SX1276M0_DEBUG
    void setDebugger(Stream *);
#endif

  private:
    Stream* _stream;
    int16_t _pin_reset;
    Buffer _buffer;
    bool _connected;
    bool _reset;
    bool _sleeping;
    
#ifdef SMW_SX1276M0_DEBUG
    Stream* _stream_debug;
#endif

    void _delay(uint32_t);
    CommandResponse _read_response(uint32_t);
    void _send_command(const char *, uint8_t = 0, ...);
};

// --------------------------------------------------
// --------------------------------------------------

#define FILTER_PRINTABLE    0
#define FILTER_ALPHANUMERIC 1
#define FILTER_ALPHA        2
#define FILTER_HEX          3
#define FILTER_NUMERIC      4

void filter_string(char *, uint8_t, const char *, uint8_t = FILTER_ALPHANUMERIC);

// --------------------------------------------------

void * memmem(const void *, size_t, const void *, size_t);

// --------------------------------------------------

#endif // SMW_SX1276M0_H
