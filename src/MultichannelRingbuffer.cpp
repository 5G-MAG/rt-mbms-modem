// 5G-MAG Reference Tools
// MBMS Modem Process
//
// Copyright (C) 2021 Klaus Kühnhammer (Österreichische Rundfunksender GmbH & Co KG)
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Affero General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Affero General Public License for more details.
//
// You should have received a copy of the GNU Affero General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//

#include "MultichannelRingbuffer.h"

#include <memory>
#include "spdlog/spdlog.h"

MultichannelRingbuffer::MultichannelRingbuffer(size_t size, size_t channels)
  : _size( size )
  , _channels( channels )
  , _used( 0 )
  , _head( 0 )
{
  for (auto ch = 0; ch < _channels; ch++) {
    auto buf = (char*)srsran_vec_malloc( _size);
    //auto buf = (char*)malloc(_size);
    if (buf == nullptr) {
      throw "Could not allocate memory";
    }
    _buffers.push_back(buf);
  }
  spdlog::debug("Created {}-channel ringbuffer with size {}", _channels, _size );
}

MultichannelRingbuffer::~MultichannelRingbuffer()
{
  for (auto buffer : _buffers) {
    if (buffer) free(buffer);
  }
}

auto MultichannelRingbuffer::read_head() -> std::vector<void*>
{
//  _mutex.lock();
  std::lock_guard<std::mutex> lock(_mutex);
  std::vector<void*> buffers(_channels, nullptr);
  for (auto ch = 0; ch < _channels; ch++) {
    buffers[ch] = (void*)(_buffers[ch]); // Return the beggining of the buffer;
  }
  _head = 0;
  _used = 0;
  return buffers;
}

auto MultichannelRingbuffer::write_head(size_t* writeable) -> std::vector<void*>
{
//  _mutex.lock();
  std::lock_guard<std::mutex> lock(_mutex);
  std::vector<void*> buffers(_channels, nullptr);
  if (_size == _used) { // In this case we return a nullptr. Because read_head and read functions we should never see a situation where we try to read this returned nullptr. Usually never reach exactly te end of the buffer. 
    *writeable = 0;
  } else {
    auto tail = (_head + _used) % _size;
    if (tail < _head) {
      *writeable = _head - tail;
    } else {
      *writeable = _size - tail;
    }
    for (auto ch = 0; ch < _channels; ch++) {
      buffers[ch] = (void*)(_buffers[ch] + tail);
    }
  }

  return buffers;
}

auto MultichannelRingbuffer::commit(size_t written) -> void
{
  assert(written >= 0);
  assert(written <= free_size());
  std::lock_guard<std::mutex> lock(_mutex);
  _used += written;
//  _mutex.unlock();
}

auto MultichannelRingbuffer::read(std::vector<char*> dest, size_t size) -> void
{
  assert(dest.size() >= _channels);
  assert(size <= used_size());
  assert(size >= 0);

  std::lock_guard<std::mutex> lock(_mutex);
  auto end = (_head + size) % _size;

  if (end <= _head) {
    auto first_part = _size - _head;
    auto second_part = size - first_part;
    for (auto ch = 0; ch < _channels; ch++) {
      memcpy(dest[ch],              _buffers[ch] + _head, first_part);
      memcpy(dest[ch] + first_part, _buffers[ch],         second_part);
    }
  } else {
    for (auto ch = 0; ch < _channels; ch++) {
      memcpy(dest[ch], _buffers[ch] + _head, size);
    }
  }
  _head = (_head + size) % _size;
  _used -= size;
}
