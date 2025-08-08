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

#pragma once
#include <stddef.h>
#include <vector>
#include <mutex>

#include "srsran/srsran.h"
#include "srsran/phy/common/phy_common.h"

class MultichannelRingbuffer {
 public:
    explicit MultichannelRingbuffer(size_t size, size_t channels);
    virtual ~MultichannelRingbuffer();

    inline size_t free_size() { std::lock_guard<std::mutex> lock(_mutex);  return _size - _used; }
    inline size_t used_size() { std::lock_guard<std::mutex> lock(_mutex); return _used; }
    inline size_t capacity() { std::lock_guard<std::mutex> lock(_mutex);return _size; }

    inline void clear() {std::lock_guard<std::mutex> lock(_mutex); _head = _used = 0; };

    std::vector<void*> read_head();
    std::vector<void*> write_head(size_t* writeable);
    void commit(size_t written);

    void read(std::vector<char*> dest, size_t bytes);

 private:
    std::vector<char*> _buffers;
    size_t _size;
    size_t _channels;
    size_t _head;
    size_t _used;
    std::mutex _mutex;
};
