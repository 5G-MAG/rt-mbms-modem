// 5G-MAG Reference Tools
// MBMS Modem Process
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

#ifndef MBMS_RT_SUBFRAME_CLOCK_H
#define MBMS_RT_SUBFRAME_CLOCK_H

#include <cstdint>
#include <mutex>
#include <optional>
#include <vector>

namespace MBMS_RT {

/**
 * Ties subframes to the receiver's sample clock, so that a time label a cell broadcasts for the start of a given frame
 * (SIB16 timeReferenceInfo, 3GPP TS 36.331 V19.4.0 clause 6.3.2) can be related to the receiver's own timeline.
 *
 * The main loop calls note() once per subframe with the time of that subframe's first sample on the sample clock
 * (SdrReader::sample_clock_ns(), taken before the subframe is read). request() asks for the clock at a future or recent
 * subframe; it is answered from the last `kHistory` subframes if that subframe has passed, otherwise as soon as it arrives.
 */
class SubframeClock {
public:
  static constexpr size_t kHistory = 2048; // 2 s of subframes, well inside one SFN cycle of 10.24 s

  struct Request {
    uint32_t tti             = 0; // absolute subframe, 10 * SFN + subframe
    uint16_t sfn             = 0;
    int64_t  label_ns        = 0; // the time the cell gave for that subframe (nanoseconds from the label's origin)
    bool     label_local_clock = false;
  };
  struct Anchor {
    Request  request;
    uint64_t sample_clock_ns = 0; // receive sample clock at the first sample of the subframe
    int64_t  host_time_ns    = 0; // host clock when the anchor was completed (informational only)
  };

  SubframeClock() : _ring(kHistory) {}

  void note(uint32_t tti, uint64_t sample_clock_ns, int64_t host_time_ns = 0)
  {
    std::lock_guard<std::mutex> lock(_mutex);
    _ring[_next % kHistory] = {tti, sample_clock_ns, true};
    _next++;
    if (_pending && _pending->tti == tti) {
      _anchor  = Anchor{*_pending, sample_clock_ns, host_time_ns};
      _pending.reset();
    }
  }

  void request(const Request& r, int64_t host_time_ns = 0)
  {
    std::lock_guard<std::mutex> lock(_mutex);
    const size_t n = _next < kHistory ? _next : kHistory;
    for (size_t i = 1; i <= n; i++) { // newest first
      const Entry& e = _ring[(_next - i) % kHistory];
      if (e.valid && e.tti == r.tti) {
        _anchor = Anchor{r, e.clock_ns, host_time_ns};
        _pending.reset();
        return;
      }
    }
    _pending = r;
  }

  std::optional<Anchor> latest() const
  {
    std::lock_guard<std::mutex> lock(_mutex);
    return _anchor;
  }

private:
  struct Entry {
    uint32_t tti      = 0;
    uint64_t clock_ns = 0;
    bool     valid    = false;
  };
  mutable std::mutex     _mutex;
  std::vector<Entry>     _ring;
  size_t                 _next = 0;
  std::optional<Request> _pending;
  std::optional<Anchor>  _anchor;
};

} // namespace MBMS_RT

#endif // MBMS_RT_SUBFRAME_CLOCK_H
