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

#ifndef MBMS_RT_SYNC_SIGNAL_H
#define MBMS_RT_SYNC_SIGNAL_H

/**
 * Primary and secondary synchronization signals of an MBMS-dedicated cell (3GPP TS 36.211 V19.3.0 clauses 6.11.1 and 6.11.2), receive side: the signal a cell sends in
 * its cell acquisition subframe, built in the receiver, and the time of arrival of several cells that overlap in one received window.
 *
 * This is the alternative the broadcast positioning proposal names for cells whose cell acquisition subframes are not SFN'ed (slide 13, Note 1: "ToA may also be
 * obtained using the PSS/SSS"): the time of arrival of each cell is taken from its PSS and SSS instead of from PRS. It is the proposal's alternative, not a measurement
 * the specification sets requirements for (TS 36.133 clause 8.1.2.5 writes the RSTD requirements for PRS).
 *
 * The sequences and their mapping are those of bps/sync.py of rt-mbms-examples/positioning-assistance, which was compared with Table 6.11.2.1-1 for all 168 groups;
 * test/sync_signal_test.cc compares this implementation with the golden vectors written by tools/gen_sync_golden.py there. Header only, on top of PrsSignal.h.
 */

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdint>
#include <limits>
#include <vector>

#include "PrsSignal.h"

namespace MBMS_RT {
namespace sync {

using cplx = prs::cplx;

constexpr uint32_t kSyncLen = 62;

/** Clause 6.11.1.1, Table 6.11.1.1-1: the Zadoff-Chu root index for N_ID^(2) 0, 1, 2. */
inline uint32_t pss_root(uint32_t n_id_2) {
  static const uint32_t roots[3] = {25, 29, 34};
  return roots[n_id_2 % 3];
}

/** Clause 6.11.1.1: d(n) = exp(-j pi u n (n + 1) / 63) for n = 0..30 and exp(-j pi u (n + 1) (n + 2) / 63) for n = 31..61. */
inline std::vector<cplx> pss_sequence(uint32_t n_id_2) {
  const double u = pss_root(n_id_2);
  std::vector<cplx> d(kSyncLen);
  for (uint32_t n = 0; n < kSyncLen; n++) {
    const double num = n <= 30 ? u * n * (n + 1) : u * (n + 1) * (n + 2);
    d[n] = cplx(static_cast<float>(std::cos(-M_PI * num / 63.0)), static_cast<float>(std::sin(-M_PI * num / 63.0)));
  }
  return d;
}

/** The m-sequence of clause 6.11.2.1: x(i + 5) = (sum of x(i + t) for t in taps) mod 2, x(0..3) = 0, x(4) = 1; returns 1 - 2 x(i) for i = 0..30. */
inline std::vector<float> m_sequence(const std::vector<int>& taps) {
  int x[31] = {0, 0, 0, 0, 1};
  for (int i = 0; i < 26; i++) {
    int s = 0;
    for (int t : taps) s += x[i + t];
    x[i + 5] = s % 2;
  }
  std::vector<float> out(31);
  for (int i = 0; i < 31; i++) out[i] = 1.0f - 2.0f * static_cast<float>(x[i]);
  return out;
}

/** Clause 6.11.2.1: m0 and m1 from N_ID^(1) (Table 6.11.2.1-1, as its formula). */
inline void sss_indices(uint32_t n_id_1, uint32_t* m0, uint32_t* m1) {
  const uint32_t qp = n_id_1 / 30;
  const uint32_t q = (n_id_1 + qp * (qp + 1) / 2) / 30;
  const uint32_t mp = n_id_1 + q * (q + 1) / 2;
  *m0 = mp % 31;
  *m1 = (*m0 + mp / 31 + 1) % 31;
}

/** Clause 6.11.2.1: the SSS d(0..61) of subframe 0 (`subframe_5` false) or subframe 5. An MBMS-dedicated cell sends it in slot 0 of subframe 0. */
inline std::vector<float> sss_sequence(uint32_t n_id_1, uint32_t n_id_2, bool subframe_5 = false) {
  static const std::vector<float> s_tilde = m_sequence({2, 0}), c_tilde = m_sequence({3, 0}), z_tilde = m_sequence({4, 2, 1, 0});
  uint32_t m0 = 0, m1 = 0;
  sss_indices(n_id_1, &m0, &m1);
  std::vector<float> d(kSyncLen);
  for (uint32_t n = 0; n < 31; n++) {
    const float s0 = s_tilde[(n + m0) % 31], s1 = s_tilde[(n + m1) % 31];
    const float c0 = c_tilde[(n + n_id_2) % 31], c1 = c_tilde[(n + n_id_2 + 3) % 31];
    const float z0 = z_tilde[(n + m0 % 8) % 31], z1 = z_tilde[(n + m1 % 8) % 31];
    if (!subframe_5) {
      d[2 * n] = s0 * c0;
      d[2 * n + 1] = s1 * c1 * z0;
    } else {
      d[2 * n] = s1 * c0;
      d[2 * n + 1] = s0 * c1 * z1;
    }
  }
  return d;
}

/** Slot 0 of a cell acquisition subframe, laid out symbol * 12 * nof_prb + subcarrier: the PSS in the last OFDM symbol and the SSS in the one before it, on
 *  k = n - 31 + N_RB^DL N_sc^RB / 2, n = 0..61 (clauses 6.11.1.2 and 6.11.2.2, MBMS-dedicated cell). Nothing else is generated. */
inline std::vector<cplx> slot0_grid(uint32_t cell_id, uint32_t nof_prb, bool normal_cp) {
  const uint32_t nsym = normal_cp ? 7 : 6, width = 12 * nof_prb;
  std::vector<cplx> grid(static_cast<size_t>(nsym) * width, cplx(0, 0));
  const uint32_t n1 = (cell_id % 504) / 3, n2 = cell_id % 3;
  const std::vector<cplx> pss = pss_sequence(n2);
  const std::vector<float> sss = sss_sequence(n1, n2, false);
  for (uint32_t n = 0; n < kSyncLen; n++) {
    const uint32_t k = n + 6 * nof_prb - 31;
    grid[(nsym - 1) * width + k] = pss[n];
    grid[(nsym - 2) * width + k] = cplx(sss[n], 0.0f);
  }
  return grid;
}

/** Time-domain waveform of the cell acquisition subframe as far as it is generated here (slot 0 with the PSS and SSS, slot 1 empty), for an FFT of `n_fft`
 *  points, in the same convention as prs::subframe_waveform (DC unused, cyclic prefix scaled to n_fft). */
inline std::vector<cplx> cas_waveform(uint32_t cell_id, uint32_t nof_prb, bool normal_cp, uint32_t n_fft) {
  const uint32_t nsym = normal_cp ? 7 : 6, width = 12 * nof_prb;
  const uint32_t cp0 = normal_cp ? 160 * n_fft / 2048 : 512 * n_fft / 2048;
  const uint32_t cp = normal_cp ? 144 * n_fft / 2048 : 512 * n_fft / 2048;
  const std::vector<cplx> grid = slot0_grid(cell_id, nof_prb, normal_cp);
  std::vector<cplx> out;
  for (uint32_t slot = 0; slot < 2; slot++) {
    for (uint32_t l = 0; l < nsym; l++) {
      std::vector<cplx> spec(n_fft, cplx(0.0f, 0.0f));
      if (slot == 0) {
        for (uint32_t k = 0; k < width; k++) {
          const int krel = static_cast<int>(k) - static_cast<int>(6 * nof_prb);
          const uint32_t bin = krel < 0 ? n_fft + krel : krel + 1;
          spec[bin] = grid[l * width + k];
        }
      }
      prs::fft(spec, true);
      const float scale = 1.0f / std::sqrt(static_cast<float>(n_fft));
      const uint32_t pre = (l == 0) ? cp0 : cp;
      for (uint32_t i = n_fft - pre; i < n_fft; i++) out.push_back(spec[i] * scale);
      for (uint32_t i = 0; i < n_fft; i++) out.push_back(spec[i] * scale);
    }
  }
  return out;
}

/** `ref` delayed by `lag` samples (not necessarily whole), as the first `n` samples of a window, through an FFT of `m` points (m >= n + ref.size()). */
inline std::vector<cplx> delayed(const std::vector<cplx>& ref, double lag, size_t n, size_t m) {
  std::vector<cplx> spec(m, cplx(0, 0));
  std::copy(ref.begin(), ref.end(), spec.begin());
  prs::fft(spec, false);
  for (size_t i = 0; i < m; i++) {
    const double f = (i < m / 2) ? static_cast<double>(i) : static_cast<double>(i) - static_cast<double>(m);
    const double ph = -2.0 * M_PI * f * lag / static_cast<double>(m);
    spec[i] *= cplx(static_cast<float>(std::cos(ph)), static_cast<float>(std::sin(ph)));
  }
  prs::fft(spec, true);
  std::vector<cplx> out(n);
  const float inv_m = 1.0f / static_cast<float>(m);
  for (size_t i = 0; i < n; i++) out[i] = spec[i] * inv_m;
  return out;
}

/**
 * The time of arrival of every cell whose synchronization signals overlap in `window`, one result per reference in `refs` (each reference is that cell's
 * cas_waveform). Different cells' sequences are not orthogonal, so correlating each on its own pulls every peak towards the others and a much stronger cell can
 * capture the correlation of a weaker one; cells that share N_ID^(2) also share their PSS. The cells are therefore taken strongest first: the delay and complex
 * amplitude of the strongest are estimated, its signal is subtracted, and the next is looked for in what remains; two passes then redo each cell with the
 * others removed. The first sample of a reference is searched for between `lag_lo` and `lag_hi`. `ratio` and `coherence` of each result are those of its last
 * correlation, against what the others leave.
 */
inline std::vector<prs::Toa> measure_cells(const std::vector<cplx>& window, const std::vector<std::vector<cplx>>& refs, int lag_lo, int lag_hi) {
  const size_t cells = refs.size();
  std::vector<prs::Toa> toa(cells);
  if (cells == 0 || window.empty()) return toa;
  size_t longest = 0;
  for (const auto& r : refs) longest = std::max(longest, r.size());
  struct Fit {
    prs::Toa         toa;
    std::vector<cplx> contrib;
    double           strength = 0.0;
  };
  const auto fit = [&](size_t i, const std::vector<cplx>& target) {
    Fit f;
    const prs::Prepared prep = prs::prepare(target, longest);
    f.toa = prs::correlate(prep, target, refs[i], lag_lo, lag_hi);
    if (!f.toa.ok) return f;
    const std::vector<cplx> shifted = delayed(refs[i], f.toa.lag, target.size(), prep.m);
    std::complex<double> num(0, 0);
    double den = 0.0;
    for (size_t k = 0; k < target.size(); k++) {
      num += std::complex<double>(std::conj(shifted[k])) * std::complex<double>(target[k]);
      den += std::norm(shifted[k]);
    }
    if (den <= 0.0) {
      f.toa.ok = false;
      return f;
    }
    const std::complex<double> amp = num / den;
    f.contrib.resize(target.size());
    for (size_t k = 0; k < target.size(); k++) f.contrib[k] = cplx(static_cast<float>(amp.real()), static_cast<float>(amp.imag())) * shifted[k];
    f.strength = std::abs(amp) * std::sqrt(den);
    return f;
  };
  if (cells == 1) {
    toa[0] = fit(0, window).toa;
    return toa;
  }
  std::vector<std::vector<cplx>> contrib(cells, std::vector<cplx>(window.size(), cplx(0, 0)));
  std::vector<cplx> residual = window;
  std::vector<bool> done(cells, false);
  const auto subtract = [&](size_t i, bool add) {
    for (size_t k = 0; k < residual.size(); k++) residual[k] += add ? contrib[i][k] : -contrib[i][k];
  };
  for (size_t round = 0; round < cells; round++) {   // strongest first
    size_t best = cells;
    Fit best_fit;
    for (size_t i = 0; i < cells; i++) {
      if (done[i]) continue;
      Fit f = fit(i, residual);
      if (best == cells || f.strength > best_fit.strength) {
        best = i;
        best_fit = std::move(f);
      }
    }
    done[best] = true;
    toa[best] = best_fit.toa;
    if (best_fit.toa.ok) {
      contrib[best] = std::move(best_fit.contrib);
      subtract(best, false);
    }
  }
  for (int pass = 0; pass < 2; pass++) {   // refine each cell with the others removed
    for (size_t i = 0; i < cells; i++) {
      if (!toa[i].ok) continue;
      subtract(i, true);
      Fit f = fit(i, residual);
      if (f.toa.ok) {
        toa[i] = f.toa;
        contrib[i] = std::move(f.contrib);
      }
      subtract(i, false);
    }
  }
  return toa;
}

}  // namespace sync
}  // namespace MBMS_RT

#endif  // MBMS_RT_SYNC_SIGNAL_H
