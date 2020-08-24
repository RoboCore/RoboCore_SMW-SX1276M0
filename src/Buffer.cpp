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

#include "Buffer.h"

// --------------------------------------------------
// Dependencies

extern "C" {
  #include <string.h>
  #include <stdlib.h>
}

// --------------------------------------------------
// --------------------------------------------------

// Constructor (default)
Buffer::Buffer() : Buffer(1) {
  // nothing to do here
}

// --------------------------------------------------

// Constructor
//  @param (size) : the size of the buffer in bytes [uint8_t]
Buffer::Buffer(uint8_t size) :
  _size(size)
  {
  // check the size
  if(_size == 0){
    _size = 1; // force the minimum size
  }
    
//  _buffer = (uint8_t *)malloc(_size * sizeof(uint8_t)); // 15/04/20 : old version
  _buffer = new uint8_t[_size]; // allocate the memory
  reset();
}

// --------------------------------------------------

// Copy constructor
//  @param (buffer) : the buffer to copy [Buffer]
Buffer::Buffer(const Buffer& buffer) :
  _size(buffer._size),
  _index(buffer._index)
  {
  _buffer = new uint8_t[_size]; // allocate the memory
  for(uint8_t i=0 ; i < _size ; i++){
    _buffer[i] = buffer._buffer[i];
  }
}

// --------------------------------------------------

// Destructor
Buffer::~Buffer(){
//  free(_buffer); // 15/04/20 : old version
  delete[] _buffer; // free the memory
}

// --------------------------------------------------
// --------------------------------------------------

// Operator = (assignment)
Buffer& Buffer::operator=(const Buffer& buffer){
  if(this == &buffer){
    return *this;
  }

  if(_buffer){
    delete[] _buffer;
  }
  _size = buffer._size;
  _index = buffer._index;
  _buffer = new uint8_t[_size]; // allocate the memory
  for(uint8_t i=0 ; i < _size ; i++){
    _buffer[i] = buffer._buffer[i];
  }

  return *this;
}

// --------------------------------------------------

// Operator [] (subscript)
//  @returns the value of the last index if out of bounds [const uint8_t]
const uint8_t& Buffer::operator[](uint8_t index) const {
  // check the index
  if(index >= _index){
    return _buffer[_index - 1]; // return from the last index
  }

  return _buffer[index];
}

// --------------------------------------------------
// --------------------------------------------------

// Append a byte to the buffer
//  @param (b) the byte to append [uint8_t]
void Buffer::append(uint8_t b){
  if(!isFull()){
    _buffer[_index++] = b;
  }
  if(_index > _size){
    _index = _size; // udpate
  }
}

// --------------------------------------------------

// Check if there is data available
//  @returns the quantity of bytes stored [uint8_t]
uint8_t Buffer::available(void){
  return _index;
}

// --------------------------------------------------

// Get a copy of the buffer
//  @param (data) : the array to copy to [uint8_t *]
void Buffer::copy(uint8_t *data){
  memcpy(data, _buffer, _index);
}

// --------------------------------------------------

// Check if the buffer is full
//  @returns [bool]
bool Buffer::isFull(void){
  if(_index == _size){
    return true;
  }
  return false;
}

// --------------------------------------------------

// Check the first byte in the buffer
//  @returns [uint8_t]
uint8_t Buffer::peek(void){
  if(_index == 0){
    return 0;
  }

  return _buffer[0];
}


// --------------------------------------------------

#ifdef BUFFER_DEBUG
// Print the buffer to a stream
//  @param (stream) : the stream to print to [Stream *]
void Buffer::print(Stream *stream){
  if(stream){
    stream->print("\nBuffer: ");
    stream->print(_size);
    stream->print('|');
    stream->print(_index); // is the same as <available()>
    stream->print('|');
    for(uint8_t i=0 ; i < _index ; i++){
      stream->write(_buffer[i]);
    }
    stream->println();
  }
}
#endif

// --------------------------------------------------

// Read the first byte in the buffer
//  @returns [uint8_t]
uint8_t Buffer::read(void){
  uint8_t ret = peek();

  // shift the buffer
  if(_index > 0){
    for(uint8_t i=0 ; i < (_index - 1) ; i++){
      _buffer[i] = _buffer[i+1];
    }
    _index--; // update
    _buffer[_index] = 0; // reset
  }
  
  return ret;
}

// --------------------------------------------------

// Reset the buffer
void Buffer::reset(void){
  _index = 0;
  for(uint8_t i=0 ; i < _size ; i++){
    _buffer[i] = 0;
  }
}


// --------------------------------------------------

// Resize the buffer
//  @param (size) : the size of the buffer in bytes [uint8_t]
void Buffer::resize(uint8_t size){
  // check the new size
  if(size == 0){
    return;
  }
  
  // allocate the memory
  uint8_t *_new_buffer = new uint8_t[size];

  // copy the data
  uint8_t copy_size = (size < _size) ? size : _size;
  for(uint8_t i=0 ; i < copy_size ; i++){
    _new_buffer[i] = _buffer[i];
  }

  // validate the index
  if(_index > size){
    _index = size;
  }

  // update the references
  delete[] _buffer;
  _buffer = _new_buffer;
  _size = size;
}

// --------------------------------------------------

// Get the size of the buffer
//  @returns the size of the buffer [uint8_t]
uint8_t Buffer::size(void){
  return _size;
}

// --------------------------------------------------
