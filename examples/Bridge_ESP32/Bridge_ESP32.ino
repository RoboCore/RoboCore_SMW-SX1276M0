/*******************************************************************************
* SMW_SX1276M0 Bridge (v1.0)
* 
* Simple program to bridge the computer to the LoRaWAN module.
* This program uses the ESP32 to communicate with the LoRaWAN module.
* 
* Copyright 2020 RoboCore.
* Written by Francois (22/07/20).
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

#if !defined(ARDUINO_ESP32_DEV) // ESP32
#error Use this example with the ESP32
#endif

// --------------------------------------------------
// Libraries

#include "RoboCore_SMW_SX1276M0.h"

// --------------------------------------------------
// Variables

#include <HardwareSerial.h>
HardwareSerial LoRaSerial(2);
#define RXD2 16
#define TXD2 17

SMW_SX1276M0 lorawan(LoRaSerial);

// --------------------------------------------------
// --------------------------------------------------

void setup() {
  // start the UART for the computer
  Serial.begin(115200);
  Serial.println(F("--- SMW_SX1276M0 Bridge ---"));
  
  // start the UART for the LoRaWAN Bee
  LoRaSerial.begin(115200, SERIAL_8N1, RXD2, TXD2);
}

// --------------------------------------------------
// --------------------------------------------------

void loop() {
  // SMW_SX1276M0 to computer
  if(LoRaSerial.available()){
    Serial.write(LoRaSerial.read());
  }

  // computer to SMW_SX1276M0
  if(Serial.available()){
    LoRaSerial.write(Serial.read());
  }
}

// --------------------------------------------------
// --------------------------------------------------
