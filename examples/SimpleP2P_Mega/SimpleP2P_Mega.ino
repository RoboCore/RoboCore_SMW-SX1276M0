/*******************************************************************************
* SMW_SX1276M0 Simple P2P (v1.0)
* 
* Simple program to test sending a message with P2P.
* This program uses the ATmega2560 (BlackBoard Mega) to communicate
* with the LoRaWAN module.
* 
* Copyright 2023 RoboCore.
* Written by Francois (16/01/2023).
* 
* Note: do not send a hexadecimal message "05" on port 1 ("AT+SENDB 1:05")
*       when on P2P mode. This triggers a key reset ("PKI completed") on
*       the receiver node.
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

#include <RoboCore_SMW_SX1276M0.h>

// --------------------------------------------------
// Variables

SMW_SX1276M0 lorawan(Serial1);

CommandResponse response;

const char DEVADDR[] = "00000000"; // the address of the current module
const char DEVADDR_P2P[] = "00000000"; // the destination address
const char APPSKEY[] = "00000000000000000000000000000000";
const char NWKSKEY[] = "00000000000000000000000000000000";

const unsigned long PAUSE_TIME = 30000; // [ms] (30 s)
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
  Serial.println(F("--- SMW_SX1276M0 P2P ---"));

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

  // set the P2P destination address
  response = lorawan.set_P2P_DevAddr(DEVADDR_P2P);
  if(response == CommandResponse::OK){
    Serial.print(F("P2P address set ("));
    Serial.write(DEVADDR_P2P, 8);
    Serial.println(')');
  } else {
    Serial.println(F("Error setting the P2P address"));
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

  // set the P2P Sync Word (optional)
  const uint8_t SYNC_WORD = 18;
  response = lorawan.set_P2P_SyncWord(SYNC_WORD);
  if(response == CommandResponse::OK){
    Serial.print(F("P2P Sync Word set ("));
    Serial.print(SYNC_WORD);
    Serial.println(')');
  } else {
    Serial.println(F("Error setting the P2P Sync Word"));
  }

  // set the join mode to P2P
  response = lorawan.set_JoinMode(SMW_SX1276M0_JOIN_MODE_P2P);
  if(response == CommandResponse::OK){
    Serial.println(F("Mode set to P2P"));
  } else {
    Serial.println(F("Error setting the join mode"));
  }
  
}

// --------------------------------------------------
// --------------------------------------------------

void loop() {
  // listen for incoming data from the module
  lorawan.listen();

  // send a message
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
    response = lorawan.sendX(10, data); // DO NOT SEND "05" on port 1

    // update the timeout
    timeout = millis() + PAUSE_TIME;
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

    // give some time to flush the remaining data from the event
    delay(50);
    lorawan.flush();

    // read the message
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

    // give some time to flush the remaining data from the event
    delay(50);
    lorawan.flush();

    // read the message
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
