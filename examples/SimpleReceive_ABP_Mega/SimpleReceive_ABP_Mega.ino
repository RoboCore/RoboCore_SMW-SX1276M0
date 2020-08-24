/*******************************************************************************
* SMW_SX1276M0 Simple Receive - ABP (v1.0)
* 
* Simple program to test the downlink procedure with ABP.
* This program uses the ATmega2560 (BlackBoard Mega) to communicate
* with the LoRaWAN module.
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

#if !defined(__AVR_ATmega2560__) // Mega 2560
#error Use this example with the ATmega2560
#endif

// --------------------------------------------------
// Libraries

#include "RoboCore_SMW_SX1276M0.h"

// --------------------------------------------------
// Variables

SMW_SX1276M0 lorawan(Serial1);

CommandResponse response;

const char DEVADDR[] = "00000000";
const char APPSKEY[] = "00000000000000000000000000000000";
const char NWKSKEY[] = "00000000000000000000000000000000";

const unsigned long PAUSE_TIME = 300000; // [ms] (5 min)
unsigned long timeout;
int count = 0;

// --------------------------------------------------
// Prototypes

void event_handler(Event);

// --------------------------------------------------
// --------------------------------------------------

void setup() {
  // Start the UART for debugging
  Serial.begin(115200);
  Serial.println(F("--- SMW_SX1276M0 Downlink (ABP) ---"));
  
  // start the UART for the LoRaWAN Bee
  Serial1.begin(115200);

  // set the event handler
  lorawan.event_listener = &event_handler;
  Serial.println(F("Handler set"));

  // read the Device EUI
  char deveui[16];
  response = lorawan.get_DevEUI(deveui);
  if(response == CommandResponse::OK){
    Serial.print(F("DevEUI: "));
    Serial.write(deveui, 16);
    Serial.println();
  } else {
    Serial.println(F("Error getting the Device EUI"));
  }

  // set the Device Address
  response = lorawan.set_DevAddr(DEVADDR);
  if(response == CommandResponse::OK){
    Serial.print(F("Device Address set ("));
    Serial.write(DEVADDR, 8);
    Serial.println(')');
  } else {
    Serial.println(F("Error setting the Device Address"));
  }

  // set the Application Session Key
  response = lorawan.set_AppSKey(APPSKEY);
  if(response == CommandResponse::OK){
    Serial.print(F("Application Session Key set ("));
    Serial.write(APPSKEY, 32);
    Serial.println(')');
  } else {
    Serial.println(F("Error setting the Application Session Key"));
  }

  // set the Network Session Key
  response = lorawan.set_NwkSKey(NWKSKEY);
  if(response == CommandResponse::OK){
    Serial.print(F("Network Session Key set ("));
    Serial.write(NWKSKEY, 32);
    Serial.println(')');
  } else {
    Serial.println(F("Error setting the Network Session Key"));
  }

  // set join mode to ABP
  response = lorawan.set_JoinMode(SMW_SX1276M0_JOIN_MODE_ABP);
  if(response == CommandResponse::OK){
    Serial.println(F("Mode set to ABP"));
  } else {
    Serial.println(F("Error setting the join mode"));
  }

  // join the network (not really necessary in ABP)
  Serial.println(F("Joining the network"));
  lorawan.join();
}

// --------------------------------------------------
// --------------------------------------------------

void loop() {
  // listen for incoming data from the module
  lorawan.listen();

  // send a message
  if(lorawan.isConnected()){
    if(timeout < millis()){
      // update the counter
      count++;
      if(count > 255){
        count = 0; // reset
      }
  
      // convert to HEX
      char data[] = { '0', '0', 0};
      data[0] += (count / 16);
      if(data[0] > '9'){
        data[0] += 7;
      }
      data[1] += (count % 16);
      if(data[1] > '9'){
        data[1] += 7;
      }
  
      // send the message
      Serial.print(F("Data: "));
      Serial.println(data);
      response = lorawan.sendX(1, data);
  
      // update the timeout
      timeout = millis() + PAUSE_TIME;
    }
  } else {
    if(timeout < millis()){
      // show some activity
      Serial.println('.');
    
      // update the timeout
      timeout = millis() + 5000; // 5 s
    }
  }
}

// --------------------------------------------------
// --------------------------------------------------

// Handle the events of the module
//  @param (type) : the type of the event [Event]
void event_handler(Event type){
  // check if join event
  if(type == Event::JOINED){
    Serial.println(F("Joined"));
  } 
  // check if text message received event
  else if(type == Event::RECEIVED){
    Serial.println(F("Text message received"));
    uint8_t port;
    Buffer buffer;
    response = lorawan.readT(port, buffer);
    if(response == CommandResponse::OK){
      Serial.print(F("Message: "));
      while(buffer.available()){
        Serial.write(buffer.read());
      }
      Serial.print(F(" on port "));
      Serial.println(port);
    }
  } 
  // check if hexadecimal message received event
  else if(type == Event::RECEIVED_X){
    Serial.println(F("HEX message received"));
    uint8_t port;
    Buffer buffer;
    response = lorawan.readX(port, buffer);
    if(response == CommandResponse::OK){
      Serial.print(F("Message: "));
      while(buffer.available()){
        Serial.write(buffer.read());
      }
      Serial.print(F(" on port "));
      Serial.println(port);
    }
  }
}

// --------------------------------------------------
