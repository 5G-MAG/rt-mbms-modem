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

#include "PrsToa.h"

#include <chrono>
#include <cmath>
#include <limits>

namespace MBMS_RT {

namespace {
constexpr size_t kKeptResults = 32;

bool power_of_two(uint32_t v) { return v && !(v & (v - 1)); }
}  // namespace

PrsToa::PrsToa() { _worker = std::thread([this] { worker_loop(); }); }

PrsToa::~PrsToa() {
  {
    std::lock_guard<std::mutex> lock(_job_mutex);
    _stop = true;
  }
  _job_cv.notify_all();
  if (_worker.joinable()) _worker.join();
}

bool PrsToa::set_assistance(const Assistance& a, std::string& why) {
  if (a.cells.empty()) {
    std::lock_guard<std::mutex> lock(_mutex);
    _assistance = Assistance();
    _generation++;
    _results.clear();
    _have.store(false);
    _history.clear();
    return true;
  }
  const CellInfo* ref = nullptr;
  for (const auto& c : a.cells) {
    if (c.cell_id == a.reference_cell_id) ref = &c;
  }
  if (!ref) {
    why = "the reference cell is not among the cells";
    return false;
  }
  for (size_t i = 0; i < a.cells.size(); i++) {
    const auto& c = a.cells[i];
    for (size_t j = 0; j < i; j++) {
      if (a.cells[j].cell_id == c.cell_id) {
        why = "cell " + std::to_string(c.cell_id) + " is listed twice";
        return false;
      }
    }
    uint32_t period = 0, offset = 0;
    if (!prs::period_offset(c.prs.config_index, &period, &offset)) {
      why = "cell " + std::to_string(c.cell_id) + ": PRS configuration index " + std::to_string(c.prs.config_index) + " is reserved or out of range";
      return false;
    }
    if (c.prs.nof_subframes != 1) {
      why = "cell " + std::to_string(c.cell_id) + ": only one PRS subframe per occasion (N_PRS 1) is supported";
      return false;
    }
    if (c.prs.nof_prb != 6 && c.prs.nof_prb != 15 && c.prs.nof_prb != 25 && c.prs.nof_prb != 50 && c.prs.nof_prb != 75 && c.prs.nof_prb != 100) {
      why = "cell " + std::to_string(c.cell_id) + ": PRS bandwidth " + std::to_string(c.prs.nof_prb) + " PRB is not 6, 15, 25, 50, 75 or 100";
      return false;
    }
    if (c.prs.id != prs::kPrsIdFromCell && c.prs.id > 4095) {
      why = "cell " + std::to_string(c.cell_id) + ": PRS ID above 4095";
      return false;
    }
    if (!c.prs.muting.empty()) {
      size_t n = c.prs.muting.size();
      bool ok = n >= 2 && n <= 1024 && (n & (n - 1)) == 0;
      for (char ch : c.prs.muting) ok = ok && (ch == '0' || ch == '1');
      if (!ok) {
        why = "cell " + std::to_string(c.cell_id) + ": the muting sequence must be 2, 4, ..., 1024 characters of 0 and 1";
        return false;
      }
    }
    if (std::isfinite(c.expected_rstd_ns) != std::isfinite(c.expected_uncertainty_ns) ||
        (std::isfinite(c.expected_uncertainty_ns) && (c.expected_uncertainty_ns < 0.0 || std::fabs(c.expected_rstd_ns) > 1e6))) {
      why = "cell " + std::to_string(c.cell_id) + ": the search window needs both an expected RSTD (within +-1 ms) and a non-negative uncertainty, or neither";
      return false;
    }
    if (c.prs.config_index != ref->prs.config_index) {
      why = "cell " + std::to_string(c.cell_id) + ": a PRS configuration index different from the reference cell's (subframe offsets between cells) is not supported";
      return false;
    }
  }
  std::lock_guard<std::mutex> lock(_mutex);
  // A new set that differs from the current one only in the search windows (the usual refresh: the expected RSTDs follow the RTDs) changes what is looked for where, not
  // what was measured, so the results and the measurement in progress stay valid.
  bool windows_only = _have.load() && a.reference_cell_id == _assistance.reference_cell_id && a.cells.size() == _assistance.cells.size();
  for (size_t i = 0; windows_only && i < a.cells.size(); i++) {
    const CellInfo &x = a.cells[i], &y = _assistance.cells[i];
    windows_only = x.cell_id == y.cell_id && x.cp == y.cp && x.prs.config_index == y.prs.config_index && x.prs.nof_prb == y.prs.nof_prb &&
                   x.prs.nof_subframes == y.prs.nof_subframes && x.prs.id == y.prs.id && x.prs.muting == y.prs.muting;
  }
  const bool keep = windows_only && !a.cells.empty();
  _assistance = a;
  for (auto& c : _assistance.cells) c.prs.enabled = true;
  if (keep) return true;
  _generation++;     // what was measured under the previous data no longer applies
  _results.clear();
  _have.store(true);
  _history.clear();
  return true;
}

PrsToa::Assistance PrsToa::assistance() const {
  std::lock_guard<std::mutex> lock(_mutex);
  return _assistance;
}

void PrsToa::push(uint32_t tti, uint64_t sample_clock_ns, const std::complex<float>* samples, uint32_t sf_len, const prs::CellParams& cell) {
  if (!_have.load(std::memory_order_relaxed)) return;
  Job job;
  bool dispatch = false;
  {
    std::lock_guard<std::mutex> lock(_mutex);
    if (_assistance.cells.empty()) return;
    Sf sf;
    sf.tti = tti % prs::kSubframesPerSfnCycle;
    sf.clock = sample_clock_ns;
    sf.samples.assign(samples, samples + sf_len);
    _history.push_back(std::move(sf));
    while (_history.size() > 3) _history.pop_front();
    if (_history.size() < 3) return;
    const Sf &a = _history[0], &b = _history[1], &c = _history[2];
    if ((a.tti + 1) % prs::kSubframesPerSfnCycle != b.tti || (b.tti + 1) % prs::kSubframesPerSfnCycle != c.tti ||
        a.samples.size() != b.samples.size() || b.samples.size() != c.samples.size()) {
      // Only an occasion in the middle is a loss; ordinary gaps between occasions are not counted.
      const prs::Cfg* rc = nullptr;
      for (const auto& ci : _assistance.cells)
        if (ci.cell_id == _assistance.reference_cell_id) rc = &ci.prs;
      if (rc && prs::active(*rc, b.tti)) _stats.gaps++;
      return;
    }
    const prs::Cfg* ref_cfg = nullptr;
    for (const auto& ci : _assistance.cells)
      if (ci.cell_id == _assistance.reference_cell_id) ref_cfg = &ci.prs;
    if (!ref_cfg || !prs::active(*ref_cfg, b.tti)) return;
    // an MBMS-dedicated cell has 15 kHz subframes only where it has cell acquisition subframes
    if (cell.mbms_dedicated && !(b.tti % 10 == 0 && prs::cas_frame(cell, b.tti / 10))) return;
    const uint32_t n_fft = sf_len / 15;
    if (sf_len % 15 != 0 || !power_of_two(n_fft) || n_fft < 128 || n_fft > 2048) {
      _stats.unsupported++;
      return;
    }
    job.window.reserve(3 * sf_len);
    for (const Sf* s : {&a, &b, &c}) job.window.insert(job.window.end(), s->samples.begin(), s->samples.end());
    job.tti = b.tti;
    job.clock = b.clock;
    job.sf_len = sf_len;
    job.cell = cell;
    job.assistance = _assistance;
    job.generation = _generation;
    dispatch = true;
  }
  if (!dispatch) return;
  {
    std::lock_guard<std::mutex> lock(_job_mutex);
    if (_has_job || _busy) {
      std::lock_guard<std::mutex> l2(_mutex);
      _stats.dropped++;
      return;
    }
    _job = std::move(job);
    _has_job = true;
  }
  _job_cv.notify_one();
}

PrsToa::Measurement PrsToa::measure(const Job& j) const {
  Measurement m;
  m.tti = j.tti;
  m.sample_clock_ns = j.clock;
  m.sf_len = j.sf_len;
  const uint32_t n_fft = j.sf_len / 15;
  const double fs = 15000.0 * n_fft;
  m.sample_rate_hz = fs;
  const int half = static_cast<int>(0.75e-3 * fs);
  const prs::Prepared prepared = prs::prepare(j.window, j.sf_len);   // the window's spectrum, shared by all cells
  // The reference cell is measured first: the search window of a neighbour is centred on the reference cell's arrival plus the expected RSTD.
  std::vector<const CellInfo*> order;
  for (const auto& ci : j.assistance.cells)
    if (ci.cell_id == j.assistance.reference_cell_id) order.push_back(&ci);
  for (const auto& ci : j.assistance.cells)
    if (ci.cell_id != j.assistance.reference_cell_id) order.push_back(&ci);
  std::vector<CellToa> results;
  bool   have_ref = false;
  double ref_toa_ns = 0.0;
  for (const CellInfo* cip : order) {
    const CellInfo& ci = *cip;
    CellToa out;
    out.cell_id = ci.cell_id;
    out.prs_id = (ci.prs.id == prs::kPrsIdFromCell) ? ci.cell_id : ci.prs.id;
    if (!prs::active(ci.prs, j.tti) || ci.prs.nof_prb > j.cell.nof_prb) {
      results.push_back(out);   // muted in this occasion
      continue;
    }
    prs::CellParams cp = j.cell;
    cp.cell_id = ci.cell_id;
    if (ci.cp >= 0) cp.normal_cp = (ci.cp == 1);
    const std::vector<prs::cplx> ref = prs::subframe_waveform(ci.prs, cp, (j.tti / 10) % 1024, j.tti % 10, n_fft);
    int peak_lo = std::numeric_limits<int>::min(), peak_hi = std::numeric_limits<int>::max();
    const bool is_ref = ci.cell_id == j.assistance.reference_cell_id;
    if (!is_ref && have_ref && std::isfinite(ci.expected_rstd_ns) && std::isfinite(ci.expected_uncertainty_ns)) {
      const double centre_ns = ref_toa_ns + ci.expected_rstd_ns;
      peak_lo = static_cast<int>(std::floor(static_cast<double>(j.sf_len) + (centre_ns - ci.expected_uncertainty_ns) * 1e-9 * fs));
      peak_hi = static_cast<int>(std::ceil(static_cast<double>(j.sf_len) + (centre_ns + ci.expected_uncertainty_ns) * 1e-9 * fs));
    }
    const prs::Toa t = prs::correlate(prepared, j.window, ref, static_cast<int>(j.sf_len) - half, static_cast<int>(j.sf_len) + half, peak_lo, peak_hi);
    if (t.ok) {
      out.present = true;
      out.toa_ns = (t.lag - static_cast<double>(j.sf_len)) / fs * 1e9;
      out.ratio = t.ratio;
      out.coherence = t.coherence;
      if (is_ref) {
        have_ref = true;
        ref_toa_ns = out.toa_ns;
      }
    }
    results.push_back(out);
  }
  // report in the order of the assistance data
  for (const auto& ci : j.assistance.cells)
    for (const auto& r : results)
      if (r.cell_id == ci.cell_id) {
        m.cells.push_back(r);
        break;
      }
  return m;
}

void PrsToa::worker_loop() {
  for (;;) {
    Job job;
    {
      std::unique_lock<std::mutex> lock(_job_mutex);
      _job_cv.wait(lock, [this] { return _has_job || _stop; });
      if (_stop) return;
      job = std::move(_job);
      _has_job = false;
      _busy = true;
    }
    const auto t0 = std::chrono::steady_clock::now();
    Measurement m = measure(job);
    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    {
      std::lock_guard<std::mutex> lock(_mutex);
      if (job.generation != _generation) {
        _stats.discarded++;   // made under assistance data that has since been replaced or cleared
      } else {
        _results.push_back(std::move(m));
        while (_results.size() > kKeptResults) _results.pop_front();
        _stats.windows++;
        _stats.last_ms = ms;
      }
    }
    {
      std::lock_guard<std::mutex> lock(_job_mutex);
      _busy = false;
    }
  }
}

std::vector<PrsToa::Measurement> PrsToa::measurements() const {
  std::lock_guard<std::mutex> lock(_mutex);
  return std::vector<Measurement>(_results.begin(), _results.end());
}

PrsToa::Stats PrsToa::stats() const {
  std::lock_guard<std::mutex> lock(_mutex);
  return _stats;
}

}  // namespace MBMS_RT
