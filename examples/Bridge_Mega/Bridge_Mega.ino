/*******************************************************************************
* SMW_SX1276M0 Bridge (v1.0)
* 
* Simple program to bridge the computer to the LoRaWAN module.
* This program uses the ATmega2560 (BlackBoard Mega) to communicate
* with the LoRaWAN module.
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

#if !defined(__AVR_ATmega2560__) // Mega 2560
#error Use this example with the ATmega2560
#endif

// --------------------------------------------------
// Libraries

#include "RoboCore_SMW_SX1276M0.h"

// --------------------------------------------------
// Variables

SMW_SX1276M0 lorawan(Serial1);

// --------------------------------------------------
// --------------------------------------------------

void setup() {
  // start the UART for the computer
  Serial.begin(115200);
  Serial.println(F("--- SMW_SX1276M0 Bridge ---"));
  
  // start the UART for the LoRaWAN Bee
  Serial1.begin(115200);
}

// --------------------------------------------------
// --------------------------------------------------

void loop() {
  // SMW_SX1276M0 to computer
  if(Serial1.available()){
    Serial.write(Serial1.read());
  }

  // computer to SMW_SX1276M0
  if(Serial.available()){
    Serial1.write(Serial.read());
  }
}

// --------------------------------------------------
// --------------------------------------------------
