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
 * Supported: cells on the reference cell's carrier, whose PRS occasion overlaps the reference cell's (TS 37.355: within half a subframe), with the subframe offset of the assistance data applied to their own numbering, and one subframe per occasion (N_PRS 1); the slot number offset is only for the CRS and is not used,
 * which is what an MBMS-dedicated cell can have (PRS is defined for 15 kHz only and the cell acquisition subframes are its only 15 kHz subframes).
 */
class PrsToa {
 public:
  struct CellInfo {
    uint32_t  cell_id = 0;
    prs::Cfg  prs;
    // Search window of a neighbour (TS 37.355 expectedRSTD and expectedRSTD-Uncertainty): its PRS is looked for within +-uncertainty of the reference cell's arrival
    // plus the expected RSTD. Both NaN: the whole +-0.75 ms.
    double    expected_rstd_ns = std::nan("");
    double    expected_uncertainty_ns = std::nan("");
    // Whole subframes from the beginning of frame SFN = 0 of the reference cell to the beginning of the next frame SFN = 0 of this cell (LPPe subframeOffset = Tnc - Tref, so
    // its subframe number is the reference cell's minus this offset). The cell's
    // occasion, muting bit and replica are those of its own subframe number at the reference cell's occasion. On an MBMS-dedicated carrier PRS is only in the cell
    // acquisition subframes (15 kHz only, frames n_f mod 4 = 0): a cell whose own number is not one of them has no PRS there and is reported as skipped.
    uint32_t  sfn_offset = 0;
    int       cp = -1;   // this cell's cyclic prefix (TS 37.355 cpLength of OTDOA assistance data, per cell): 0 extended, 1 normal, -1 the serving cell's
  };
  struct Assistance {
    uint32_t              reference_cell_id = 0;
    std::vector<CellInfo> cells;   // including the reference cell
  };
  struct CellToa {
    uint32_t cell_id   = 0;
    uint32_t prs_id    = 0;
    bool     present   = false;   // false: PRS of this cell muted in this occasion, or the correlation failed
    bool     skipped   = false;   // MBMS-dedicated carrier: this cell's own subframe number at the reference occasion is not a cell acquisition subframe, so it cannot carry PRS now
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
    uint64_t skipped_cells = 0; // cells not measured in an occasion because, in their own numbering, it is not a cell acquisition subframe (MBMS-dedicated carrier)
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
