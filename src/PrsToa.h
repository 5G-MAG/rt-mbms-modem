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

#ifndef MBMS_RT_PRS_TOA_H
#define MBMS_RT_PRS_TOA_H

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "PrsSignal.h"

namespace MBMS_RT {

/**
 * Time of arrival of the positioning reference signals of several cells, measured on the samples the modem receives.
 *
 * The application layer (a UE that has received the assistance data of the broadcast positioning proposal over FLUTE, slide 15 of that deck) pushes the PRS
 * information of the reference cell and its neighbours down with set_assistance(). From then on every subframe the modem receives is offered with push();
 * at each PRS occasion of the reference cell the three consecutive subframes around it (neighbours' subframes arrive up to about +-0.5 ms away from the
 * serving cell's) are correlated with each configured cell's PRS on a worker thread, so the receive loop never waits. The result is each cell's time of
 * arrival against the start of the middle subframe on the modem's own grid, which the serving cell's synchronisation defines; the same offset is in every
 * cell's figure, so differences between cells (the RSTD of TS 36.214 clause 5.1.12) do not contain it.
 *
 * Supported: cells whose PRS occasions are the reference cell's own (no subframe or slot offsets between cells) and one subframe per occasion (N_PRS 1),
 * which is what an MBMS-dedicated cell can have (PRS is defined for 15 kHz only and the cell acquisition subframes are its only 15 kHz subframes).
 */
class PrsToa {
 public:
  struct CellInfo {
    uint32_t  cell_id = 0;
    prs::Cfg  prs;
  };
  struct Assistance {
    uint32_t              reference_cell_id = 0;
    std::vector<CellInfo> cells;   // including the reference cell
  };
  struct CellToa {
    uint32_t cell_id   = 0;
    uint32_t prs_id    = 0;
    bool     present   = false;   // false: PRS of this cell muted in this occasion, or the correlation failed
    double   toa_ns    = 0.0;     // against the nominal start of the middle subframe
    double   ratio     = 0.0;     // correlation peak over the median elsewhere
    double   coherence = 0.0;     // peak over the norms of the received segment and the reference, 1 for a perfect copy
  };
  struct Measurement {
    uint32_t             tti = 0;               // of the PRS subframe
    uint64_t             sample_clock_ns = 0;   // receiver sample clock at the first sample of that subframe
    uint32_t             sf_len = 0;
    double               sample_rate_hz = 0.0;
    std::vector<CellToa> cells;
  };
  struct Stats {
    uint64_t windows = 0;     // measurements made
    uint64_t dropped = 0;     // occasions skipped because the previous measurement was still running
    uint64_t gaps = 0;        // occasions skipped because the three subframes were not consecutive or differed in length
    uint64_t unsupported = 0; // occasions skipped because the sampling is not one the correlator handles
    uint64_t discarded = 0;   // measurements finished after the assistance data had changed, and so dropped
    double   last_ms = 0.0;   // how long the latest measurement took on the worker thread
  };

  PrsToa();
  ~PrsToa();
  PrsToa(const PrsToa&) = delete;
  PrsToa& operator=(const PrsToa&) = delete;

  /** Takes the PRS information; returns false and says why if it cannot be used. An empty cell list clears it. */
  bool set_assistance(const Assistance& a, std::string& why);
  bool has_assistance() const { return _have.load(std::memory_order_relaxed); }
  Assistance assistance() const;

  /** Offer the samples of subframe `tti` (`sf_len` of them) together with the receiver sample clock at their start and what the cell looks like. */
  void push(uint32_t tti, uint64_t sample_clock_ns, const std::complex<float>* samples, uint32_t sf_len, const prs::CellParams& cell);

  /** The most recent measurements, oldest first. */
  std::vector<Measurement> measurements() const;
  Stats stats() const;

 private:
  struct Sf {
    uint32_t tti = 0;
    uint64_t clock = 0;
    std::vector<std::complex<float>> samples;
  };
  struct Job {
    std::vector<std::complex<float>> window;   // three subframes
    uint32_t tti = 0;
    uint64_t clock = 0;
    uint32_t sf_len = 0;
    prs::CellParams cell;
    Assistance assistance;
    uint64_t generation = 0;   // of the assistance data it was made under
  };

  void worker_loop();
  Measurement measure(const Job& j) const;

  mutable std::mutex      _mutex;
  Assistance              _assistance;
  uint64_t                _generation = 0;   // counts changes of the assistance data; results of an older one are not kept
  std::atomic<bool>       _have{false};
  std::deque<Sf>          _history;
  std::deque<Measurement> _results;
  Stats                   _stats;

  std::mutex              _job_mutex;
  std::condition_variable _job_cv;
  bool                    _has_job = false;
  bool                    _busy = false;
  bool                    _stop = false;
  Job                     _job;
  std::thread             _worker;
};

}  // namespace MBMS_RT

#endif  // MBMS_RT_PRS_TOA_H
