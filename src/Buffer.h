#ifndef BUFFER_H
#define BUFFER_H

/*******************************************************************************
* RoboCore Buffer Library (v1.0)
* 
* Library to manipulate buffers.
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

#define BUFFER_DEBUG

// --------------------------------------------------
// Dependencies

#ifdef BUFFER_DEBUG
#include <Stream.h>
#endif

extern "C" {
  #include <stdint.h>
}

// -----------------------------------------------------------------

class Buffer {
  public:
    Buffer();
    Buffer(uint8_t);
    Buffer(const Buffer&);
    ~Buffer();
    void append(uint8_t);
    uint8_t available(void);
    void copy(uint8_t *);
    bool isFull(void);
    uint8_t peek(void);
    uint8_t read(void);
    void reset(void);
    void resize(uint8_t);
    uint8_t size(void);

    Buffer& operator=(const Buffer&);

    const uint8_t& operator[](uint8_t) const;

#ifdef BUFFER_DEBUG
    void print(Stream *);
#endif
  
  private:
    uint8_t _index;
    uint8_t _size;
    uint8_t *_buffer;
};

// -----------------------------------------------------------------

#endif // BUFFER_H
