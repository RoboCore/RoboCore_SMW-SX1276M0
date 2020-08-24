/*******************************************************************************
* RoboCore SMW_SX1276M0 Library (v1.0)
* 
* Library to use the SMW_SX1276M0 LoRaWAN module.
* 
* Copyright 2020 RoboCore.
* Written by Francois (24/08/20).
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

// --------------------------------------------------
// Libraries

#include "RoboCore_SMW_SX1276M0.h"

extern "C" {
  #include <string.h>
}

// --------------------------------------------------
// --------------------------------------------------

// Default constructor
//  @param (stream) : the stream to send the data to [Stream *]
SMW_SX1276M0::SMW_SX1276M0(Stream &stream) :
  SMW_SX1276M0(stream, -1)
  {
  // nothing to do here
}

// --------------------------------------------------

// Complete constructor
//  @param (stream)    : the stream to send the data to [Stream *]
//         (pin_reset) : the pin to reset the module [int16_t]
SMW_SX1276M0::SMW_SX1276M0(Stream &stream, int16_t pin_reset) :
  _stream(&stream),
  _pin_reset(pin_reset),
  _buffer(SMW_SX1276M0_BUFFER_SIZE),
  _connected(false),
  _reset(false),
  _sleeping(false),
  event_listener(nullptr)
  {
#ifdef SMW_SX1276M0_DEBUG
    _stream_debug = nullptr;
#endif
}


// --------------------------------------------------
// --------------------------------------------------

// Custom delay in miliseconds
//  @param (duration) : the duration of the delay in miliseconds [uint32_t]
void SMW_SX1276M0::_delay(uint32_t duration){
  uint32_t stop_time = millis() + duration;
  while(millis() < stop_time){
#if defined(ARDUINO_ESP8266_GENERIC) || defined(ARDUINO_ESP8266_NODEMCU) || defined(ARDUINO_ESP8266_THING) || defined(ARDUINO_ESP32_DEV)
// ESP8266 Generic / NodeMCU / Sparkfun The Thing / ESP32 Dev
    yield(); // custom function for a non blocking execution with the ESP family
#else
    // do nothing
#endif
  }
}

// --------------------------------------------------

// Flush the buffered data in the stream
void SMW_SX1276M0::flush(void){
  while(_stream->available()){
    _stream->read();
  }
}

// --------------------------------------------------

// Get the Adaptive Data Rate
//  @param (adr) : the variable to store the result [uint8_t (&)]
//  @returns the type of the response [CommandResponse]
CommandResponse SMW_SX1276M0::get_ADR(uint8_t (&adr)){
  // send the command and read the response
  _send_command(CMD_ADR);
  CommandResponse res = _read_response(SMW_SX1276M0_TIMEOUT_READ);
  
#ifdef SMW_SX1276M0_DEBUG
  _buffer.print(_stream_debug);
#endif

  if(res == CommandResponse::OK){
    if(_buffer.available()){
      adr = _buffer.read() - '0';
    }
  }

  return res;
}

// --------------------------------------------------

// Get the Automatic Join
//  @param (ajoin) : the variable to store the result [uint8_t (&)]
//  @returns the type of the response [CommandResponse]
CommandResponse SMW_SX1276M0::get_AJoin(uint8_t (&ajoin)){
  // send the command and read the response
  _send_command(CMD_AJOIN);
  CommandResponse res = _read_response(SMW_SX1276M0_TIMEOUT_READ);
  
#ifdef SMW_SX1276M0_DEBUG
  _buffer.print(_stream_debug);
#endif

  if(res == CommandResponse::OK){
    if(_buffer.available()){
      ajoin = _buffer.read() - '0';
    }
  }

  return res;
}

// --------------------------------------------------

// Get the RTC wakup time
//  @param (alarm) : the variable to store the result [uint32_t (&)]
//  @returns the type of the response [CommandResponse]
CommandResponse SMW_SX1276M0::get_Alarm(uint32_t (&alarm)){
  // send the command and read the response
  _send_command(CMD_ALARM);
  CommandResponse res = _read_response(SMW_SX1276M0_TIMEOUT_READ);
  
#ifdef SMW_SX1276M0_DEBUG
  _buffer.print(_stream_debug);
#endif

  if(res == CommandResponse::OK){
    if(_buffer.available()){
      alarm = _buffer.read() - '0';
    }
  }

  return res;
}

// --------------------------------------------------

// Get the Application EUI
//  @param (appeui) : the array to store the result [char[n]]
//  @returns the type of the response [CommandResponse]
CommandResponse SMW_SX1276M0::get_AppEUI(char (&appeui)[SMW_SX1276M0_SIZE_APPEUI]){
  // send the command and read the response
  _send_command(CMD_APPEUI);
  CommandResponse res = _read_response(SMW_SX1276M0_TIMEOUT_READ);
  
#ifdef SMW_SX1276M0_DEBUG
  _buffer.print(_stream_debug);
#endif

  if(res == CommandResponse::OK){
    uint8_t length = _buffer.available();
    if(length < SMW_SX1276M0_SIZE_APPEUI){
      appeui[length] = CHAR_EOS;
    }
    _buffer.copy(reinterpret_cast<uint8_t *>(appeui));
  }

  return res;
}

// --------------------------------------------------

// Get the Application Key
//  @param (appkey) : the array to store the result [char[n]]
//  @returns the type of the response [CommandResponse]
CommandResponse SMW_SX1276M0::get_AppKey(char (&appkey)[SMW_SX1276M0_SIZE_APPKEY]){
  // send the command and read the response
  _send_command(CMD_APPKEY);
  CommandResponse res = _read_response(SMW_SX1276M0_TIMEOUT_READ);
  
#ifdef SMW_SX1276M0_DEBUG
  _buffer.print(_stream_debug);
#endif

  if(res == CommandResponse::OK){
    uint8_t length = _buffer.available();
    if(length < SMW_SX1276M0_SIZE_APPKEY){
      appkey[length] = CHAR_EOS;
    }
    _buffer.copy(reinterpret_cast<uint8_t *>(appkey));
  }

  return res;
}

// --------------------------------------------------

// Get the Application Session Key
//  @param (appskey) : the array to store the result [char[n]]
//  @returns the type of the response [CommandResponse]
CommandResponse SMW_SX1276M0::get_AppSKey(char (&appskey)[SMW_SX1276M0_SIZE_APPSKEY]){
  // send the command and read the response
  _send_command(CMD_APPSKEY);
  CommandResponse res = _read_response(SMW_SX1276M0_TIMEOUT_READ);
  
#ifdef SMW_SX1276M0_DEBUG
  _buffer.print(_stream_debug);
#endif

  if(res == CommandResponse::OK){
    uint8_t length = _buffer.available();
    if(length < SMW_SX1276M0_SIZE_APPSKEY){
      appskey[length] = CHAR_EOS;
    }
    _buffer.copy(reinterpret_cast<uint8_t *>(appskey));
  }

  return res;
}

// --------------------------------------------------

// Get the Device Address
//  @param (devaddr) : the array to store the result [char[n]]
//  @returns the type of the response [CommandResponse]
CommandResponse SMW_SX1276M0::get_DevAddr(char (&devaddr)[SMW_SX1276M0_SIZE_DEVADDR]){
  // send the command and read the response
  _send_command(CMD_DADDR);
  CommandResponse res = _read_response(SMW_SX1276M0_TIMEOUT_READ);
  
#ifdef SMW_SX1276M0_DEBUG
  _buffer.print(_stream_debug);
#endif

  if(res == CommandResponse::OK){
    uint8_t length = _buffer.available();
    if(length < SMW_SX1276M0_SIZE_DEVADDR){
      devaddr[length] = CHAR_EOS;
    }
    _buffer.copy(reinterpret_cast<uint8_t *>(devaddr));
  }

  return res;
}

// --------------------------------------------------

// Get the Device EUI
//  @param (deveui) : the array to store the result [char[n]]
//  @returns the type of the response [CommandResponse]
CommandResponse SMW_SX1276M0::get_DevEUI(char (&deveui)[SMW_SX1276M0_SIZE_DEVEUI]){
  // send the command and read the response
  _send_command(CMD_DEVEUI);
  CommandResponse res = _read_response(SMW_SX1276M0_TIMEOUT_READ);
  
#ifdef SMW_SX1276M0_DEBUG
  _buffer.print(_stream_debug);
#endif

  if(res == CommandResponse::OK){
    uint8_t length = _buffer.available();
    if(length < SMW_SX1276M0_SIZE_DEVEUI){
      deveui[length] = CHAR_EOS;
    }
    _buffer.copy(reinterpret_cast<uint8_t *>(deveui));
  }

  return res;
}

// --------------------------------------------------

// Get the Data Rate
//  @param (dr) : the variable to store the result [uint8_t (&)]
//  @returns the type of the response [CommandResponse]
CommandResponse SMW_SX1276M0::get_DR(uint8_t (&dr)){
  // send the command and read the response
  _send_command(CMD_DR);
  CommandResponse res = _read_response(SMW_SX1276M0_TIMEOUT_READ);
  
#ifdef SMW_SX1276M0_DEBUG
  _buffer.print(_stream_debug);
#endif

  if(res == CommandResponse::OK){
    if(_buffer.available()){
      dr = _buffer.read() - '0';
    }
  }

  return res;
}

// --------------------------------------------------

// Get the Echo configuration
//  @param (echo) : the variable to store the result [uint8_t (&)]
//  @returns the type of the response [CommandResponse]
CommandResponse SMW_SX1276M0::get_Echo(uint8_t (&echo)){
  // send the command and read the response
  _send_command(CMD_ECHO);
  CommandResponse res = _read_response(SMW_SX1276M0_TIMEOUT_READ);
  
#ifdef SMW_SX1276M0_DEBUG
  _buffer.print(_stream_debug);
#endif

  if(res == CommandResponse::OK){
    if(_buffer.available()){
      echo = _buffer.read() - '0';
    }
  }

  return res;
}

// --------------------------------------------------

// Get the buffered data
//  @param (buffer) : the variable to store the result [Buffer(&)]
void SMW_SX1276M0::get_buffer(Buffer (&buffer)){
  buffer = _buffer;
}

// --------------------------------------------------

// Get the Join Mode
//  @param (mode) : the variable to store the result [uint8_t (&)]
//  @returns the type of the response [CommandResponse]
CommandResponse SMW_SX1276M0::get_JoinMode(uint8_t (&mode)){
  // send the command and read the response
  _send_command(CMD_NJM);
  CommandResponse res = _read_response(SMW_SX1276M0_TIMEOUT_READ);
  
#ifdef SMW_SX1276M0_DEBUG
  _buffer.print(_stream_debug);
#endif

  if(res == CommandResponse::OK){
    if(_buffer.available()){
      mode = _buffer.read() - '0';
    }
  }

  return res;
}

// --------------------------------------------------

// Get the Join Status
//  @param (status) : the variable to store the result [uint8_t (&)]
//  @returns the type of the response [CommandResponse]
CommandResponse SMW_SX1276M0::get_JoinStatus(uint8_t (&status)){
  // send the command and read the response
  _send_command(CMD_NJS);
  CommandResponse res = _read_response(SMW_SX1276M0_TIMEOUT_READ);
  
#ifdef SMW_SX1276M0_DEBUG
  _buffer.print(_stream_debug);
#endif

  if(res == CommandResponse::OK){
    if(_buffer.available()){
      uint8_t b = _buffer.read();
      status = b - '0';

      _connected = (b == '1') ? true : false; // update
    }
  }

  return res;
}

// --------------------------------------------------

// Get the Network Session Key
//  @param (nwkskey) : the array to store the result [char[n]]
//  @returns the type of the response [CommandResponse]
CommandResponse SMW_SX1276M0::get_NwkSKey(char (&nwkskey)[SMW_SX1276M0_SIZE_NWKSKEY]){
  // send the command and read the response
  _send_command(CMD_APPSKEY);
  CommandResponse res = _read_response(SMW_SX1276M0_TIMEOUT_READ);
  
#ifdef SMW_SX1276M0_DEBUG
  _buffer.print(_stream_debug);
#endif

  if(res == CommandResponse::OK){
    uint8_t length = _buffer.available();
    if(length < SMW_SX1276M0_SIZE_NWKSKEY){
      nwkskey[length] = CHAR_EOS;
    }
    _buffer.copy(reinterpret_cast<uint8_t *>(nwkskey));
  }

  return res;
}

// --------------------------------------------------

// Get the RSSI of the last received data
//  @param (rssi) : the variable to store the result [double (&)]
//  @returns the type of the response [CommandResponse]
CommandResponse SMW_SX1276M0::get_RSSI(double (&rssi)){
  // send the command and read the response
  _send_command(CMD_RSSI);
  CommandResponse res = _read_response(SMW_SX1276M0_TIMEOUT_READ);
  
#ifdef SMW_SX1276M0_DEBUG
  _buffer.print(_stream_debug);
#endif

  if(res == CommandResponse::OK){
    char buffer_value[6] = { '0' }; // default value is 0
    uint8_t index = 0;
    while(_buffer.available()){
      char c = _buffer.read();
      if(isdigit(c) || (c == '-')){
        buffer_value[index++] = c;
      }
      rssi = atof(buffer_value);
    }
  }

  return res;
}

// --------------------------------------------------

// Get the SNR of the last received data
//  @param (snr) : the variable to store the result [double (&)]
//  @returns the type of the response [CommandResponse]
CommandResponse SMW_SX1276M0::get_SNR(double (&snr)){
  // send the command and read the response
  _send_command(CMD_SNR);
  CommandResponse res = _read_response(SMW_SX1276M0_TIMEOUT_READ);
  
#ifdef SMW_SX1276M0_DEBUG
  _buffer.print(_stream_debug);
#endif

  if(res == CommandResponse::OK){
    char buffer_value[6] = { '0' }; // default value is 0
    uint8_t index = 0;
    while(_buffer.available()){
      char c = _buffer.read();
      if(isdigit(c) || (c == '-')){
        buffer_value[index++] = c;
      }
      snr = atof(buffer_value);
    }
  }

  return res;
}

// --------------------------------------------------

// Get the Version
//  @param (version) : the array to store the result [char[n]]
//  @returns the type of the response [CommandResponse]
CommandResponse SMW_SX1276M0::get_Version(char (&version)[SMW_SX1276M0_SIZE_VERSION]){
  // send the command and read the response
  _send_command(CMD_VERSION);
  CommandResponse res = _read_response(SMW_SX1276M0_TIMEOUT_READ);
  
#ifdef SMW_SX1276M0_DEBUG
  _buffer.print(_stream_debug);
#endif

  if(res == CommandResponse::OK){
    uint8_t length = _buffer.available();
    if(length < SMW_SX1276M0_SIZE_VERSION){
      version[length] = CHAR_EOS;
    }
    _buffer.copy(reinterpret_cast<uint8_t *>(version));
  }

  return res;
}

// --------------------------------------------------

// Join the network
//  NOTE: the confirmation is asynchronous (<listen()>)
void SMW_SX1276M0::join(void){
  _send_command(CMD_JOIN);
}

// --------------------------------------------------

// Check if the module is connected to the network
//  @returns true if the device is connected [bool]
//  NOTE: it is better to check via the NJS command
bool SMW_SX1276M0::isConnected(void){
  return _connected;
}

// --------------------------------------------------

// Check if the module is sleeping
//  @returns true if the device is sleeping [bool]
bool SMW_SX1276M0::isSleeping(void){
  return _sleeping;
}

// --------------------------------------------------

// Listen for incoming data
//  @param (call_event) : true to call the event [bool] (default: true)
//  @returns the status of the buffer [CommandResponse]
//  NOTE: return ERROR if no data was read, DATA if there is data in the buffer or OK is an event was called
CommandResponse SMW_SX1276M0::listen(bool call_event){
  _buffer.reset(); // reset the buffer

  // read the incoming data
  uint8_t c;
  uint32_t timeout = millis() + SMW_SX1276M0_DELAY_INCOMING_DATA;
  while(millis() < timeout){
    if(_stream->available()){
      c = _stream->read(); // read the incoming byte
      
#ifdef SMW_SX1276M0_DEBUG
      // debug
      if(_stream_debug){
        if(c > 32){
          _stream_debug->write(c);
        } else {
          _stream_debug->print('(');
          _stream_debug->print('x');
          _stream_debug->print(c, HEX);
          _stream_debug->print(')');
        }
      }
#endif
  
      // check for new line
      if((c == 0) || (c == CHAR_LF) || (c == CHAR_CR)){
//        _delay(1); // wait some time for the next character to arrive
        // wait some time for the data to arrive (without adding more time)
        while(millis() < timeout){
          if(_stream->available()){
            uint8_t p = _stream->peek();
            if((p == 0) || (p == CHAR_LF) || (p == CHAR_CR)){
              _stream->read(); // flush the LF or CR character
            }
            break; // exit the timeout
          }
        }
        break; // line read, move to the next
      } else {
        _buffer.append(c);
        timeout = millis() + SMW_SX1276M0_DELAY_INCOMING_DATA; // give more time for the data to arrive
      }
    }
  }

  // check for a valid buffer
  if(!_buffer.available()){
    return CommandResponse::ERROR; // not the expected result, but treat as no message
  }

  // get a copy of the buffer
  uint8_t data_length = _buffer.available();
  uint8_t data[data_length];
  _buffer.copy(data);
  
#ifdef SMW_SX1276M0_DEBUG
  // debug
  if(_stream_debug){
    _buffer.print(_stream_debug);
  }
#endif

  // check for the event header
  void *ptr = memmem(data, data_length, RSPNS_EVENT, strlen(RSPNS_EVENT));
  if(ptr){
#ifdef SMW_SX1276M0_DEBUG
    if(_stream_debug){
      _stream_debug->println(F("Found E")); // DEBUG
    }
#endif

    // check for sleep
    ptr = memmem(data, data_length, RSPNS_SLEEP, strlen(RSPNS_SLEEP));
    if(ptr){
#ifdef SMW_SX1276M0_DEBUG
    if(_stream_debug){
      _stream_debug->println(F("Found S")); // DEBUG
    }
#endif
      _buffer.reset(); // flush the buffer
      _sleeping = true; // set
      
      // call the event
      if(event_listener && call_event){
        event_listener(Event::SLEEP);
      }
    
      return CommandResponse::OK;
    }
    
    // check for join
    ptr = memmem(data, data_length, RSPNS_JOINED, strlen(RSPNS_JOINED));
    if(ptr){
#ifdef SMW_SX1276M0_DEBUG
    if(_stream_debug){
      _stream_debug->println(F("Found J")); // DEBUG
    }
#endif
      _buffer.reset(); // flush the buffer
      _connected = true; // set
      
      // call the event
      if(event_listener && call_event){
        event_listener(Event::JOINED);
      }
    
      return CommandResponse::OK;
    }
    
    // check for received message
    ptr = memmem(data, data_length, RSPNS_RECV, strlen(RSPNS_RECV));
    if(ptr){
      // flush the start of the message
      uint32_t ignore = reinterpret_cast<uint32_t>(ptr) + strlen(RSPNS_RECV) - reinterpret_cast<uint32_t>(data); // some microcontrollers are 32-bit wide
      for(uint8_t i=0 ; i < ignore ; i++){
        _buffer.read();
      }
#ifdef SMW_SX1276M0_DEBUG
    if(_stream_debug){
      _stream_debug->println(F("Found M")); // DEBUG
      _stream_debug->print(F("Ignore:")); // DEBUG
      _stream_debug->println(ignore); // DEBUG
      _buffer.print(_stream_debug); // DEBUG
    }
#endif

      // get the type of the command received (string or HEX)
      uint8_t type = 0;
      if(_buffer.available()){
        type = _buffer.read();
      }
#ifdef SMW_SX1276M0_DEBUG
    if(_stream_debug){
      _stream_debug->print(F("Type:")); // DEBUG
      _stream_debug->println(type, HEX); // DEBUG
    }
#endif

      // the module seems to trigger the event before actually storing
      //  the message, so a delay prevents an empty return value for a
      //  subsequent reading (it should help when using the library)
      _delay(10);

      if(type == CHAR_SPACE){
        // call the event
        if(event_listener && call_event){
          event_listener(Event::RECEIVED);
        }
      } else if(type == 'B'){
        _buffer.read(); // flush one character
        // call the event
        if(event_listener && call_event){
          event_listener(Event::RECEIVED_X);
        }
      } else {
        return CommandResponse::ERROR; // wrong result
      }
      
      return CommandResponse::DATA;
    }
  }
  
  // check for the reset header
  ptr = memmem(data, data_length, RSPNS_BOOT, sizeof(RSPNS_BOOT));
  if(ptr){
#ifdef SMW_SX1276M0_DEBUG
    if(_stream_debug){
      _stream_debug->println(F("Found R")); // DEBUG
    }
#endif
    _buffer.reset(); // flush the buffer
    _connected = false; // reset

    // check if the module was sleeping
    if(_sleeping){
      _sleeping = false; // reset
      // NOTE: the wakeup reset could be done with "Wakeup by RTC", but
      //       the reset of the module already means it has awoken.
      
      // call the event
      if(event_listener && call_event){
        event_listener(Event::WAKEUP);
      }
    } else {
      _reset = true; // set
      
      // call the event
      if(event_listener && call_event){
        event_listener(Event::RESET); // simple reset
      }
    }
    
    return CommandResponse::OK;
  }

  return CommandResponse::DATA; // there is data in the buffer
}

// --------------------------------------------------

// Ping the module
//  @returns the type of the response [CommandResponse]
CommandResponse SMW_SX1276M0::ping(void){
  _send_command(nullptr); // the command already sends "AT"
  return _read_response(SMW_SX1276M0_TIMEOUT_READ);
}

// --------------------------------------------------

// Read the response of a command
//  @param (timeout) : the time to wait for the response in miliseconds [uint32_t]
//  @returns the type of the response [CommandResponse]
CommandResponse SMW_SX1276M0::_read_response(uint32_t timeout){
  _buffer.reset(); // reset for storing the new response
  
  Buffer buffer_status(25);
  uint8_t c;
  uint8_t status = 0;

  // read the incoming data
  uint32_t stop_time = millis() + timeout;
  while(millis() < stop_time){
    if(_stream->available()){
      c = _stream->read(); // read the incoming byte
      
#ifdef SMW_SX1276M0_DEBUG
      // debug
      if(_stream_debug){
        if(c > 32){
          _stream_debug->write(c);
        } else {
          _stream_debug->print('(');
          _stream_debug->print(c, HEX);
          _stream_debug->print(')');
        }
      }
#endif

      // check the byte
      if(c == CHAR_LT){
        status = 1;
        continue; // skip to next character
      } else if(c == CHAR_GT){
        status = 2;
      }

      // store the byte if necessary
      if(status == 0){
        if((c > 31) && (c < 127)){
          _buffer.append(c);
        }
      } else if(status == 1){
        buffer_status.append(c);
      } else {
        // the remaining data is flushed
      }
    } else {
#if defined(ARDUINO_ESP8266_GENERIC) || defined(ARDUINO_ESP8266_NODEMCU) || defined(ARDUINO_ESP8266_THING) || defined(ARDUINO_ESP32_DEV)
// ESP8266 Generic / NodeMCU / Sparkfun The Thing / ESP32 Dev
      yield(); // custom function for a non blocking execution with the ESP family
#else
      // do nothing
#endif
    }
  }

  // check for a valid buffer
  if(!buffer_status.available()){
    return CommandResponse::ERROR; // wrong result
  }

  // get a copy of the buffer
  uint8_t data_length = buffer_status.available();
  uint8_t data[data_length];
  buffer_status.copy(data);
  
#ifdef SMW_SX1276M0_DEBUG
  // debug
  if(_stream_debug){
    buffer_status.print(_stream_debug);
  }
#endif

  // interpret the data
  if(status == 0){
    return CommandResponse::ERROR; // wrong result
  } else {
    // check for OK
    if(memcmp(data, RSPNS_OK, strlen(RSPNS_OK)) == 0){
      return CommandResponse::OK;
    }

    // check for FAILED
    void *ptr = memmem(data, data_length, RSPNS_FAILED, strlen(RSPNS_FAILED));
    if(ptr){
      if(data_length > strlen(RSPNS_FAILED)){
        // at this time, doesn't store the message of the response
        return CommandResponse::FAILED_STRING;
      }
      return CommandResponse::FAILED;
    }

    // check for NOT FOUND
    ptr = memmem(data, data_length, RSPNS_NOT_FOUND, strlen(RSPNS_NOT_FOUND));
    if(ptr){
      if(data_length == (strlen(RSPNS_NOT_FOUND) + 12)){
        return CommandResponse::NOT_FOUND;
      }
    }

    return CommandResponse::ERROR; // wrong result
  }
}

// --------------------------------------------------

// Read a text message from the module
//  @returns the type of the response [CommandResponse]
//  NOTE: the data must be obtained from the buffer
CommandResponse SMW_SX1276M0::readT(void){
  // send the command and read the response
  _send_command(CMD_RECV);
  return _read_response(SMW_SX1276M0_TIMEOUT_READ);
}

// --------------------------------------------------

// Read a text message from the module
//  @param (buffer) : the buffer to store the payload [Buffer (&)]
//  @returns the type of the response [CommandResponse]
CommandResponse SMW_SX1276M0::readT(Buffer (&buffer)){
  uint8_t port;
  return readT(port, buffer);
}

// --------------------------------------------------

// Read a text message from the module
//  @param (port) : the application port [uint8_t (&)]
//         (buffer) : the buffer to store the payload [Buffer (&)]
//  @returns the type of the response [CommandResponse]
CommandResponse SMW_SX1276M0::readT(uint8_t (&port), Buffer (&buffer)){
  CommandResponse res = readT(); // read the message

  // parse the message
  uint8_t b;
  bool payload = false;
  char sport[5]; // 0 to 9999
  uint8_t index = 0;
  while (_buffer.available()){
    b = _buffer.read();

    // check for delimitter
    if(b == CHAR_COLON){
      payload = true; // set
      buffer.resize(_buffer.available()); // resize the buffer
      continue;
    }

    // parse
    if(!payload){
      if((index < 4) && isdigit(b)){
        sport[index++] = b;
        sport[index] = CHAR_EOS;
      }
    } else {
      buffer.append(b);
    }
  }
  port = atoi(sport); // convert

  return res;
}

// --------------------------------------------------

// Read an hexadecimal message from the module
//  @returns the type of the response [CommandResponse]
//  NOTE: the data must be obtained from the buffer
CommandResponse SMW_SX1276M0::readX(void){
  // send the command and read the response
  _send_command(CMD_RECVB);
  return _read_response(SMW_SX1276M0_TIMEOUT_READ);
}
// --------------------------------------------------

// Read an hexadecimal message from the module
//  @param (buffer) : the buffer to store the payload [Buffer (&)]
//  @returns the type of the response [CommandResponse]
CommandResponse SMW_SX1276M0::readX(Buffer (&buffer)){
  uint8_t port;
  return readX(port, buffer);
}

// --------------------------------------------------

// Read an hexadecimal message from the module
//  @param (port) : the application port [uint8_t (&)]
//         (buffer) : the buffer to store the payload [Buffer (&)]
//  @returns the type of the response [CommandResponse]
CommandResponse SMW_SX1276M0::readX(uint8_t (&port), Buffer (&buffer)){
  CommandResponse res = readX(); // read the message

  // parse the message
  uint8_t b;
  bool payload = false;
  char sport[5]; // 0 to 9999
  uint8_t index = 0;
  while (_buffer.available()){
    b = _buffer.read();

    // check for delimitter
    if(b == CHAR_COLON){
      payload = true; // set
      buffer.resize(_buffer.available()); // resize the buffer
      continue;
    }

    // parse
    if(!payload){
      if((index < 4) && isdigit(b)){
        sport[index++] = b;
        sport[index] = CHAR_EOS;
      }
    } else {
      buffer.append(b);
    }
  }
  port = atoi(sport); // convert

  return res;
}

// --------------------------------------------------

// Reset the module
//  @returns the type of the response [CommandResponse]
CommandResponse SMW_SX1276M0::reset(void){
  if(_pin_reset < 0){
    // the reset pin is not set, so do a software reset
    _send_command(CMD_RESET);
  } else {
    // the reset pin is set, so do a hardware reset
    digitalWrite(_pin_reset, HIGH); // active HIGH
    _delay(2); // 2 ms (minimum is 1 ms)
    digitalWrite(_pin_reset, LOW);
  }

  _reset = false; // reset
  CommandResponse res = CommandResponse::ERROR;
  CommandResponse temp;
  uint8_t count = 0;
  
  // read the incoming data
  uint32_t stop_time = millis() + SMW_SX1276M0_TIMEOUT_RESET;
  while(millis() < stop_time){
    temp = listen(false); // listen for incoming messages

    // check for the reset event
    if(_reset && (temp == CommandResponse::OK)){
      _reset = false; // reset (not really necessary, but useful to prevent further errors)
      res = CommandResponse::OK;
    }

    // leave the loop if there is no more data, but only after reading the reset event (or the result can be inconsistent)
    // NOTE: <listen()> is faster than the reset procedure, so the module must be given some time before returning to the program
    if(temp == CommandResponse::ERROR){
      _delay(SMW_SX1276M0_DELAY_INCOMING_DATA); // give some time for data to arrive
      count++;
      if((count > 100) && (res == CommandResponse::OK)){
        break;
      }
    } else {
      count = 0; // reset
    }
  }

  return res;
}

// --------------------------------------------------

// Send a text message
//  @param (port) : the application port [uint8_t]
//         (data) : the text data to send [char *]
//  @returns the type of the response [CommandResponse]
CommandResponse SMW_SX1276M0::sendT(uint8_t port, const char *data){
  // parse the port
  uint8_t aport = port; // auxiliary variable for <port>
  uint8_t temp[3];
  temp[0] = aport / 100;
  aport %= 100;
  temp[1] = aport / 10;
  temp[2] = aport % 10;

  // set the header (port)
  uint8_t sport[5]; // port stringified
  uint8_t index = 0;
  aport = 0; // reset
  for(uint8_t i=0 ; i < 3 ; i++){
    if((temp[i] > 0) || (aport > 0)){
      sport[index++] = temp[i] + '0'; // convert to ASCII character
    }
    aport += temp[i]; // update (simple)
  }
  sport[index++] = CHAR_COLON;
  sport[index] = CHAR_EOS;
  
  // send the command and read the response
  _send_command(CMD_SEND, 2, sport, data);
  return _read_response(SMW_SX1276M0_TIMEOUT_WRITE); // this command takes almost 1 s to reply
}

// --------------------------------------------------

// Send a text message
//  @param (port) : the application port [uint8_t]
//         (data) : the text data to send [String]
//  @returns the type of the response [CommandResponse]
CommandResponse SMW_SX1276M0::sendT(uint8_t port, const String data){
  uint8_t length = data.length() + 1;
  char cdata[length]; // create a temporary string
  data.toCharArray(cdata, length); // copy the data (with automatic EOS)
  return sendT(port, cdata);
}

// --------------------------------------------------

// Send an hexadecimal message
//  @param (port) : the application port [uint8_t]
//         (data) : the text data to send [char *]
//  @returns the type of the response [CommandResponse]
CommandResponse SMW_SX1276M0::sendX(uint8_t port, const char *data){
  // parse the port
  uint8_t aport = port; // auxiliary variable for <port>
  uint8_t temp[3];
  temp[0] = aport / 100;
  aport %= 100;
  temp[1] = aport / 10;
  temp[2] = aport % 10;

  // set the header (port)
  uint8_t sport[5]; // port stringified
  uint8_t index = 0;
  aport = 0; // reset
  for(uint8_t i=0 ; i < 3 ; i++){
    if((temp[i] > 0) || (aport > 0)){
      sport[index++] = temp[i] + '0'; // convert to ASCII character
    }
    aport += temp[i]; // update (simple)
  }
  sport[index++] = CHAR_COLON;
  sport[index] = CHAR_EOS;
  
  // send the command and read the response
  _send_command(CMD_SENDB, 2, sport, data);
  return _read_response(SMW_SX1276M0_TIMEOUT_WRITE); // this command takes almost 1 s to reply
}

// --------------------------------------------------

// Send an hexadecimal message
//  @param (port) : the application port [uint8_t]
//         (data) : the text data to send [String]
//  @returns the type of the response [CommandResponse]
CommandResponse SMW_SX1276M0::sendX(uint8_t port, const String data){
  uint8_t length = data.length() + 1;
  char cdata[length]; // create a temporary string
  data.toCharArray(cdata, length); // copy the data (with automatic EOS)
  return sendX(port, cdata);
}

// --------------------------------------------------


// Send a command to the module
//  @param (command) : the command to send [char *]
//         (qty)     : the quantity of other parameters to send [uint8_t]
//         (...)     : optional and variable data to send [char *]
void SMW_SX1276M0::_send_command(const char *command, uint8_t qty, ...){
  flush(); // flush the data before sendig the command
  // (it could be done in <readResponse()>, but it might flush some data in some cases - not verified)
  
#ifdef SMW_SX1276M0_DEBUG
  if(_stream_debug){
    _stream_debug->write('[');
    _stream_debug->write(CMD_AT);
  }
#endif
  _stream->write(CMD_AT); // send the <AT> prefix
  
  // check if there is another command
  if(command){
#ifdef SMW_SX1276M0_DEBUG
    if(_stream_debug){
      _stream_debug->write(CHAR_PLUS);
      _stream_debug->write(command);
    }
#endif
    _stream->write(CHAR_PLUS);
    _stream->write(command);

    // check if there are paramenters to send
    if(qty){
#ifdef SMW_SX1276M0_DEBUG
      if(_stream_debug){
        _stream_debug->write(CHAR_SPACE);
      }
#endif
      _stream->write(CHAR_SPACE);
      
      va_list arg_list;
      va_start(arg_list, qty);

      for(uint8_t i=0 ; i < qty ; i++){
        char *data = va_arg(arg_list, char *);
      
#ifdef SMW_SX1276M0_DEBUG
        if(_stream_debug){
          _stream_debug->write(data);
        }
#endif
        _stream->write(data);
      }
      
      va_end(arg_list);
    }
  }
  
#ifdef SMW_SX1276M0_DEBUG
  if(_stream_debug){
    _stream_debug->write(CHAR_CR);
    _stream_debug->write(']');
  }
#endif
  _stream->write(CHAR_CR);
}

// --------------------------------------------------

// Set the Adaptive Data Rate
//  @param (adr) : the data to be sent [uint8_t]
//  @returns the type of the response [CommandResponse]
CommandResponse SMW_SX1276M0::set_ADR(uint8_t adr){
  adr = (adr == SMW_SX1276M0_ADR_ON) ? SMW_SX1276M0_ADR_ON : SMW_SX1276M0_ADR_OFF; // force binary value
  char data[] = { (adr + '0') , CHAR_EOS}; // convert to ASCII character
  
  // send the command and read the response
  _send_command(CMD_ADR, 1, data);
  CommandResponse res = _read_response(SMW_SX1276M0_TIMEOUT_WRITE); // this command takes almost 1 s to reply

  return res;
}

// --------------------------------------------------

// Set the Automatic Join
//  @param (ajoin) : the data to be sent [uint8_t]
//  @returns the type of the response [CommandResponse]
CommandResponse SMW_SX1276M0::set_AJoin(uint8_t ajoin){
  ajoin = (ajoin == SMW_SX1276M0_AUTOMATIC_JOIN_ON) ? SMW_SX1276M0_AUTOMATIC_JOIN_ON : SMW_SX1276M0_AUTOMATIC_JOIN_OFF; // force binary value
  char data[] = { (ajoin + '0') , CHAR_EOS}; // convert to ASCII character
  
  // send the command and read the response
  _send_command(CMD_AJOIN, 1, data);
  CommandResponse res = _read_response(SMW_SX1276M0_TIMEOUT_WRITE); // this command takes almost 1 s to reply

  return res;
}

// --------------------------------------------------

// Set the RTC wakeup time
//  @param (alarm) : the data to be sent [uint32_t]
//  @returns the type of the response [CommandResponse]
CommandResponse SMW_SX1276M0::set_Alarm(uint32_t alarm){
  // check the value
  const uint32_t max_time = 10^9;
  if(alarm >= max_time){
    alarm = max_time;
  }

  // convert to ASCII character
  char data[10];
  itoa(alarm, data, 10); // NOTE: this function is not defined in ANSI-C and is not part of C++, but is supported by some compilers
  data[9] = CHAR_EOS;
  
  // send the command and read the response
  _send_command(CMD_ALARM, 1, data);
  CommandResponse res = _read_response(SMW_SX1276M0_TIMEOUT_WRITE); // this command takes almost 1 s to reply

  return res;
}

// --------------------------------------------------

// Set the Application EUI
//  @param (appeui) : the array with the data to be sent [char *]
//  @returns the type of the response [CommandResponse]
CommandResponse SMW_SX1276M0::set_AppEUI(const char *appeui){
  // filter the data
  char str[SMW_SX1276M0_SIZE_APPEUI + 1];
  str[SMW_SX1276M0_SIZE_APPEUI] = CHAR_EOS;
  filter_string(str, SMW_SX1276M0_SIZE_APPEUI, appeui, FILTER_HEX);
  
  // send the command and read the response
  _send_command(CMD_APPEUI, 1, str);
  CommandResponse res = _read_response(SMW_SX1276M0_TIMEOUT_WRITE); // this command takes almost 1 s to reply

  return res;
}

// --------------------------------------------------

// Set the Application Key
//  @param (appkey) : the array with the data to be sent [char *]
//  @returns the type of the response [CommandResponse]
CommandResponse SMW_SX1276M0::set_AppKey(const char *appkey){
  // filter the data
  char str[SMW_SX1276M0_SIZE_APPKEY + 1];
  str[SMW_SX1276M0_SIZE_APPKEY] = CHAR_EOS;
  filter_string(str, SMW_SX1276M0_SIZE_APPKEY, appkey, FILTER_HEX);
  
  // send the command and read the response
  _send_command(CMD_APPKEY, 1, str);
  CommandResponse res = _read_response(SMW_SX1276M0_TIMEOUT_WRITE); // this command takes almost 1 s to reply

  return res;
}

// --------------------------------------------------

// Set the Application Session Key
//  @param (appskey) : the array with the data to be sent [char *]
//  @returns the type of the response [CommandResponse]
CommandResponse SMW_SX1276M0::set_AppSKey(const char *appskey){
  // filter the data
  char str[SMW_SX1276M0_SIZE_APPSKEY + 1];
  str[SMW_SX1276M0_SIZE_APPSKEY] = CHAR_EOS;
  filter_string(str, SMW_SX1276M0_SIZE_APPSKEY, appskey, FILTER_HEX);
  
  // send the command and read the response
  _send_command(CMD_APPSKEY, 1, str);
  CommandResponse res = _read_response(SMW_SX1276M0_TIMEOUT_WRITE); // this command takes almost 1 s to reply

  return res;
}

// --------------------------------------------------

// Set the debugger of the object
//  @param (debugger) : the stream to print to [Stream *]
#ifdef SMW_SX1276M0_DEBUG
void SMW_SX1276M0::setDebugger(Stream *debugger){
  _stream_debug = debugger;
}
#endif

// --------------------------------------------------

// Set the Device Address
//  @param (devaddr) : the array with the data to be sent [char *]
//  @returns the type of the response [CommandResponse]
CommandResponse SMW_SX1276M0::set_DevAddr(const char *devaddr){
  // filter the data
  char str[SMW_SX1276M0_SIZE_DEVADDR + 1];
  str[SMW_SX1276M0_SIZE_DEVADDR] = CHAR_EOS;
  filter_string(str, SMW_SX1276M0_SIZE_DEVADDR, devaddr, FILTER_HEX);
  
  // send the command and read the response
  _send_command(CMD_DADDR, 1, str);
  CommandResponse res = _read_response(SMW_SX1276M0_TIMEOUT_WRITE); // this command takes almost 1 s to reply

  return res;
}

// --------------------------------------------------

// Set the Device EUI
//  @param (deveui) : the array with the data to be sent [char *]
//  @returns the type of the response [CommandResponse]
CommandResponse SMW_SX1276M0::set_DevEUI(const char *deveui){
  // filter the data
  char str[SMW_SX1276M0_SIZE_DEVEUI + 1];
  str[SMW_SX1276M0_SIZE_DEVEUI] = CHAR_EOS;
  filter_string(str, SMW_SX1276M0_SIZE_DEVEUI, deveui, FILTER_HEX);
  
  // send the command and read the response
  _send_command(CMD_DEVEUI, 1, str);
  CommandResponse res = _read_response(SMW_SX1276M0_TIMEOUT_WRITE); // this command takes almost 1 s to reply

  return res;
}

// --------------------------------------------------

// Set the Data Rate
//  @param (dr) : the data to be sent [uint8_t]
//  @returns the type of the response [CommandResponse]
CommandResponse SMW_SX1276M0::set_DR(uint8_t dr){
  // check the value
  if(dr > 7){
    return CommandResponse::ERROR;
  }
  
  char data[] = { (dr + '0') , CHAR_EOS}; // convert to ASCII character
  
  // send the command and read the response
  _send_command(CMD_DR, 1, data);
  CommandResponse res = _read_response(SMW_SX1276M0_TIMEOUT_WRITE); // this command takes almost 1 s to reply

  return res;
}

// --------------------------------------------------

// Set the Echo configuration
//  @param (echo) : the data to be sent [uint8_t]
//  @returns the type of the response [CommandResponse]
CommandResponse SMW_SX1276M0::set_Echo(uint8_t echo){
  echo = (echo == SMW_SX1276M0_ECHO_ON) ? SMW_SX1276M0_ECHO_ON : SMW_SX1276M0_ECHO_OFF; // force binary value
  char data[] = { (echo + '0') , CHAR_EOS}; // convert to ASCII character
  
  // send the command and read the response
  _send_command(CMD_ECHO, 1, data);
  CommandResponse res = _read_response(SMW_SX1276M0_TIMEOUT_WRITE); // this command takes almost 1 s to reply

  return res;
}

// --------------------------------------------------

// Set the Join Mode
//  @param (mode) : the data to be sent [uint8_t]
//  @returns the type of the response [CommandResponse]
//  NOTE: this command resets the module
CommandResponse SMW_SX1276M0::set_JoinMode(uint8_t mode){
  // check the value
  if(mode > SMW_SX1276M0_JOIN_MODE_P2P){
    return CommandResponse::ERROR;
  }
  
  char data[] = { (mode + '0') , CHAR_EOS}; // convert to ASCII character
  
  // send the command and read the response
  _send_command(CMD_NJM, 1, data);

  _reset = false; // reset
  CommandResponse res = CommandResponse::ERROR;
  CommandResponse temp;
  uint8_t count = 0;
  
  // read the incoming data
  uint32_t stop_time = millis() + SMW_SX1276M0_TIMEOUT_RESET;
  while(millis() < stop_time){
    temp = listen(false); // listen for incoming messages

    // check for the reset event
    if(_reset && (temp == CommandResponse::OK)){
      _reset = false; // reset (not really necessary, but useful to prevent further errors)
      res = CommandResponse::OK;
    }

    // leave the loop if there is no more data, but only after reading the reset event (or the result can be inconsistent)
    // NOTE: <listen()> is faster than the reset procedure, so the module must be given some time before returning to the program
    if(temp == CommandResponse::ERROR){
      _delay(SMW_SX1276M0_DELAY_INCOMING_DATA); // give some time for data to arrive
      count++;
      if((count > 100) && (res == CommandResponse::OK)){
        break;
      }
    } else {
      count = 0; // reset
    }
  }

  return res;
}

// --------------------------------------------------

// Set the Network Session Key
//  @param (nwkskey) : the array with the data to be sent [char *]
//  @returns the type of the response [CommandResponse]
CommandResponse SMW_SX1276M0::set_NwkSKey(const char *nwkskey){
  // filter the data
  char str[SMW_SX1276M0_SIZE_NWKSKEY + 1];
  str[SMW_SX1276M0_SIZE_NWKSKEY] = CHAR_EOS;
  filter_string(str, SMW_SX1276M0_SIZE_NWKSKEY, nwkskey, FILTER_HEX);
  
  // send the command and read the response
  _send_command(CMD_NWKSKEY, 1, str);
  CommandResponse res = _read_response(SMW_SX1276M0_TIMEOUT_WRITE); // this command takes almost 1 s to reply

  return res;
}

// --------------------------------------------------

// Set the pin for reset
//  @param (pin_reset) : the pin to reset the module [int16_t]
void SMW_SX1276M0::setPinReset(int16_t pin_reset){
  _pin_reset = pin_reset;
  pinMode(_pin_reset, OUTPUT);
  digitalWrite(_pin_reset, LOW); // active HIGH
}

// --------------------------------------------------

// Sleep
//  @param (alarm) : the duration of the sleep [uint32_t] (default: 0)
//  @returns the type of the response [CommandResponse]
//  NOTE: <alarm = 0> means that the parameter is ignored
//  NOTE: the confirmation is asynchronous (<listen()>)
CommandResponse SMW_SX1276M0::sleep(uint32_t alarm){
  // update the alarm if necessary
  if(alarm){
    CommandResponse res = set_Alarm(alarm);
    if(res != CommandResponse::OK){
      return res;
    }
  }

  _send_command(CMD_SLEEP); // send the command

  return CommandResponse::OK;
}

// --------------------------------------------------
// --------------------------------------------------

// Filter the characters of a string
//  @param (output) : the output string, already initialized [char *]
//         (length) : the length of the output string [uint8_t]
//         (input)  : the input string to filter [char *]
//         (format) : the format of the filter [uint8_t] (default: FILTER_ALPHANUMERIC)
void filter_string(char *output, uint8_t length, const char *input, uint8_t format){
  uint8_t length_input = strlen(input); // get the length of the input

  // copy the input string
  for(uint8_t i=0 ; i < length ; i++){
    output[i] = CHAR_EOS; // <switch:default> doesn't seem to work for out of bounds access
    
    if(i < length_input){
      switch(format){
        case FILTER_PRINTABLE: {
          if(!iscntrl(input[i])){
            output[i] = input[i];
          }
          break;
        }
        
        case FILTER_ALPHANUMERIC: {
          if(isalnum(input[i])){
            output[i] = input[i];
          }
          break;
        }
        
        case FILTER_ALPHA: {
          if(isalpha(input[i])){
            output[i] = input[i];
          }
          break;
        }
        
        case FILTER_HEX: {
          if(isxdigit(input[i])){
            output[i] = input[i];
          }
          break;
        }
        
        case FILTER_NUMERIC: {
          if(isdigit(input[i])){
            output[i] = input[i];
          }
          break;
        }
      }
    }
  }
}

// --------------------------------------------------

// Find the fist occurrence of a block of data in another block of data
//  @param (haystack) : the block of data for the search [void *]
//         (hlen)     : the length of the haystack block [size_t]
//         (needle)   : the data to search for [void *]
//         (nlen)     : the length of the needle block [size_t]
//  @returns the pointer to the beginning of the sub block or a null pointer [void *]
//  NOTE: credits to <caf> ( https://stackoverflow.com/questions/2188914/c-searching-for-a-string-in-a-file )
void * memmem(const void *haystack, size_t hlen, const void *needle, size_t nlen){
  // check for valid needle length
  if (!nlen){
    return nullptr;
  }
  
  // check for valid haystack length
  if(hlen < nlen){
    return nullptr;
  }
  
  const uint8_t *p = (const uint8_t *)haystack; // [void *] cannot be incremented
  const void *p_void; // auxiliary variable for functions
  size_t plen = hlen;
  
  uint8_t needle_first;
  needle_first = *(uint8_t *)needle;

  // search for the first character of the needle
  while ((plen >= nlen) && (p_void = memchr(p, needle_first, (plen - nlen + 1)))){
    if (!memcmp(p_void, needle, nlen)){
      return (void *)p_void;
    }

    // go to the next address
    p = (const uint8_t *)p_void;
    p++;

    size_t addr_p = reinterpret_cast<size_t>(p); // get the address of <p>
    size_t addr_h = reinterpret_cast<size_t>(haystack); // get the address of <haystack>
    plen = hlen - (addr_p - addr_h); // update the length
  }
  
  return nullptr;
}

// --------------------------------------------------
