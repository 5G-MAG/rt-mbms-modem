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

#ifndef MBMS_RT_SIB16_TIME_H
#define MBMS_RT_SIB16_TIME_H

#include <cstdint>

// Arithmetic of the SIB16 time reference, free of modem state so that it can be tested on its own.
// Baseline: 3GPP TS 36.331 V19.4.0 (Release 19).

namespace MBMS_RT {
namespace sib16_time {

/// timeInfoUTC counts UTC seconds in 10 ms units since 1900-01-01 (clause 6.3.1); Unix time starts 2,208,988,800 s later.
inline int64_t time_info_utc_to_unix_ms(uint64_t time_info_utc_10ms)
{
  return (int64_t)time_info_utc_10ms * 10 - 2208988800LL * 1000;
}

/// ReferenceTime-r15 in nanoseconds from the origin of the time field (clause 6.3.2): the value in 0.25 us units is
/// refDays*86400*1000*4000 + refSeconds*1000*4000 + refMilliSeconds*4000 + refQuarterMicroSeconds.
inline int64_t reference_time_ns(uint32_t ref_days, uint32_t ref_seconds, uint16_t ref_milli_seconds, uint16_t ref_quarter_us)
{
  const int64_t units = (int64_t)ref_days * 86400LL * 1000 * 4000 + (int64_t)ref_seconds * 1000LL * 4000 +
                        (int64_t)ref_milli_seconds * 4000LL + ref_quarter_us;
  return units * 250;
}

/**
 * The SFN (0..1023) whose start the SIB16 time names, for a SIB16 decoded while the receiver's SFN is `rx_sfn`.
 *
 * Clause 5.2.3b (MBMS-dedicated cell): n is the order of entry of the SI message in the schedulingInfoList of SIB1-MBMS,
 * x = (n - 1)*w with w = si-WindowLength, and "the SI-window starts always at the subframe #a, where a = x mod 10, in the
 * radio frame for which SFN mod T = FLOOR(x/10), where T is the si-Periodicity". Clause 6.3.2: without referenceSFN the time
 * is "the time at the SFN boundary at or immediately after the ending boundary of the SI-window". The boundary is thus the
 * first frame start at or after x + w ms into the period, i.e. SFN mod T = (FLOOR(x/10) + CEIL((a + w)/10)) mod T. The result
 * is the first such SFN at or after `rx_sfn`, which assumes the SI message is decoded inside its window, before the boundary.
 * Returns false for an impossible schedule (n or w or T zero).
 */
inline bool boundary_sfn(uint32_t n, uint32_t w_ms, uint32_t period_rf, uint32_t rx_sfn, uint16_t* out)
{
  if (n == 0 || w_ms == 0 || period_rf == 0) {
    return false;
  }
  const uint32_t x            = (n - 1) * w_ms;
  const uint32_t a            = x % 10;
  const uint32_t boundary_mod = (x / 10 + (a + w_ms + 9) / 10) % period_rf;
  const uint32_t rx_mod       = rx_sfn % period_rf;
  *out                        = (uint16_t)((rx_sfn + (boundary_mod + period_rf - rx_mod) % period_rf) % 1024);
  return true;
}

} // namespace sib16_time
} // namespace MBMS_RT

#endif // MBMS_RT_SIB16_TIME_H
