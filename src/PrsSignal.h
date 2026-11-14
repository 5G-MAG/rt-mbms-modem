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

#ifndef MBMS_RT_PRS_SIGNAL_H
#define MBMS_RT_PRS_SIGNAL_H

/**
 * Positioning reference signals (3GPP TS 36.211 V19.3.0 clause 6.10.4), receive side: the signal a cell sends, built in the receiver, and the time of
 * arrival measured by correlating a received window with it. Header only and free of any other library, so the tests need nothing of the modem.
 *
 * The sequence, mapping, occasions and muting are the same as rt-mbms-tx's lib/src/phy/ch_estimation/prs.c; both are compared with the golden vectors written
 * by tools/gen_prs_golden.py of rt-mbms-examples/positioning-assistance (test/prs_signal_test.cc here, lib/test/prs/prs_test.c there).
 */

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

namespace MBMS_RT {
namespace prs {

using cplx = std::complex<float>;

constexpr uint32_t kSubframesPerSfnCycle = 10240;
constexpr uint32_t kMaxDlRb = 110;      // N_RB^max,DL of the sequence definition
constexpr uint32_t kPrsIdFromCell = 0xFFFF;

/** PRS information of one cell (TS 37.355 PRS-Info; slides 27 and 28 of the broadcast positioning proposal). */
struct Cfg {
  bool        enabled       = false;
  uint32_t    id            = kPrsIdFromCell;  // PRS ID; the cell identity when kPrsIdFromCell
  uint32_t    config_index  = 0;               // I_PRS, Table 6.10.4.3-1
  uint32_t    nof_subframes = 1;               // N_PRS
  uint32_t    nof_prb       = 25;              // N_RB^PRS
  std::string muting;                          // '0' and '1'; '0' mutes the occasion; empty: none
};

/** What of the cell the PRS pattern depends on. */
struct CellParams {
  uint32_t nof_prb        = 25;
  bool     normal_cp      = false;
  bool     mbms_dedicated = true;
  uint32_t nof_ports      = 1;
  uint32_t cell_id        = 0;
};

inline bool period_offset(uint32_t config_index, uint32_t* period, uint32_t* offset) {
  struct Row { uint32_t first, last, period; };
  static const Row rows[] = {{0, 159, 160}, {160, 479, 320}, {480, 1119, 640}, {1120, 2399, 1280}, {2400, 2404, 5},
                             {2405, 2414, 10}, {2415, 2434, 20}, {2435, 2474, 40}, {2475, 2554, 80}};
  for (const auto& r : rows) {
    if (config_index >= r.first && config_index <= r.last) {
      if (period) *period = r.period;
      if (offset) *offset = config_index - r.first;
      return true;
    }
  }
  return false;
}

/** True if PRS is transmitted in the subframe `tti` = 10 n_f + subframe (modulo 10240): one of the N_PRS consecutive subframes of an occasion, not muted.
 *  `muting_origin` is the subframe, in the numbering of the cell the configuration belongs to, at which the assistance data reference cell's SFN = 0 begins
 *  (0 for the reference cell itself): TS 37.355 counts the first bit of the muting sequence from the first occasion that starts after that point. */
inline bool active(const Cfg& cfg, uint32_t tti, uint32_t muting_origin = 0) {
  uint32_t period = 0, offset = 0;
  if (!cfg.enabled || !period_offset(cfg.config_index, &period, &offset)) return false;
  uint32_t sf = tti % kSubframesPerSfnCycle;
  for (uint32_t j = 0; j < cfg.nof_subframes; j++) {
    uint32_t start = (sf + kSubframesPerSfnCycle - j) % kSubframesPerSfnCycle;
    if ((start + kSubframesPerSfnCycle - offset) % period != 0) continue;
    if (cfg.muting.empty()) return true;
    // occasions are counted from the reference cell's SFN 0 within each cycle of 10240 subframes (T_PRS divides 10240), so a long muting sequence only ever
    // reaches its first 10240 / T_PRS bits; `gap` is the distance from the origin to the first occasion at or after it
    const uint32_t origin = muting_origin % kSubframesPerSfnCycle;
    const uint32_t gap = (offset % period + period - origin % period) % period;
    const uint32_t occasion = (((start + kSubframesPerSfnCycle - origin) % kSubframesPerSfnCycle) - gap) / period;
    return cfg.muting[occasion % cfg.muting.size()] == '1';
  }
  return false;
}

/** Frame in which an MBMS-dedicated cell has a cell acquisition subframe (clause 6.6.4.1). */
inline bool cas_frame(const CellParams& c, uint32_t n_f) { return c.nof_prb >= 25 ? (n_f % 4 == 0) : (n_f % 8 == 4); }

/** Resource elements of a slot that carry the PSS, SSS or the PBCH core set (PRS is not mapped onto them), laid out symbol * 12 * nof_prb + subcarrier. */
inline std::vector<char> sync_pbch_mask(const CellParams& c, uint32_t n_f, uint32_t n_s) {
  const uint32_t nsym = c.normal_cp ? 7 : 6, width = 12 * c.nof_prb, centre = 6 * c.nof_prb;
  std::vector<char> mask(width * nsym, 0);
  bool present, sync_slot;
  if (c.mbms_dedicated) {
    present = (n_f % 4 == 0);   // clauses 6.11.1.2 and 6.6.4: whatever the bandwidth; cas_frame() is the frame set of the cell acquisition subframes (6.6.4.1)
    sync_slot = (n_s == 0);
  } else {
    present = true;
    sync_slot = (n_s == 0 || n_s == 10);
  }
  if (!present) return mask;
  if (sync_slot) {
    for (uint32_t k = centre - 31; k < centre + 31; k++) {
      mask[(nsym - 1) * width + k] = 1;  // PSS
      mask[(nsym - 2) * width + k] = 1;  // SSS
    }
  }
  if (n_s == 1) {
    for (uint32_t l = 0; l < 4; l++)
      for (uint32_t k = centre - 36; k < centre + 36; k++) mask[l * width + k] = 1;  // PBCH core set
  }
  return mask;
}

/** Gold sequence of clause 7.2 (Nc = 1600). */
inline void gold_bits(uint32_t c_init, uint8_t* out, uint32_t len) {
  uint32_t x1 = 1, x2 = c_init;
  for (uint32_t n = 0; n < 1600 + len; n++) {
    if (n >= 1600) out[n - 1600] = static_cast<uint8_t>((x1 ^ x2) & 1u);
    uint32_t n1 = ((x1 >> 3) ^ x1) & 1u;
    uint32_t n2 = ((x2 >> 3) ^ (x2 >> 2) ^ (x2 >> 1) ^ x2) & 1u;
    x1 = (x1 >> 1) | (n1 << 30);
    x2 = (x2 >> 1) | (n2 << 30);
  }
}

inline uint32_t c_init(uint32_t n_id_prs, uint32_t n_s, uint32_t l, bool normal_cp) {
  uint32_t group = n_id_prs / 512, rest = n_id_prs % 512;
  return (1u << 28) * group + (1u << 10) * (7 * (n_s + 1) + l + 1) * (2 * rest + 1) + 2 * rest + (normal_cp ? 1u : 0u);
}

/** The PRS of one slot as a grid laid out symbol * 12 * nof_prb + subcarrier, zero outside the PRS elements (clause 6.10.4.2, no frequency hopping). */
inline std::vector<cplx> slot_grid(const Cfg& cfg, const CellParams& c, uint32_t n_f, uint32_t n_s) {
  const bool normal = c.normal_cp;
  const uint32_t nsym = normal ? 7 : 6, width = 12 * c.nof_prb;
  const bool four = (c.nof_ports == 4);
  std::vector<uint32_t> symbols;
  if (normal) {
    if (n_s % 2 == 0) symbols = {3, 5, 6};
    else symbols = four ? std::vector<uint32_t>{2, 3, 5, 6} : std::vector<uint32_t>{1, 2, 3, 5, 6};
  } else {
    if (n_s % 2 == 0) symbols = {4, 5};
    else symbols = four ? std::vector<uint32_t>{2, 4, 5} : std::vector<uint32_t>{1, 2, 4, 5};
  }
  std::vector<cplx> grid(width * nsym, cplx(0.0f, 0.0f));
  const std::vector<char> mask = sync_pbch_mask(c, n_f, n_s);
  const uint32_t id = (cfg.id == kPrsIdFromCell) ? c.cell_id : cfg.id;
  const uint32_t v_shift = id % 6, base = normal ? 6 : 5;
  std::vector<uint8_t> bits(4 * kMaxDlRb);
  const float s = 0.70710678118654752f;
  for (uint32_t l : symbols) {
    gold_bits(c_init(id, n_s, l, normal), bits.data(), 4 * kMaxDlRb);
    for (uint32_t m = 0; m < 2 * cfg.nof_prb; m++) {
      uint32_t k = 6 * (m + c.nof_prb - cfg.nof_prb) + ((base - l + v_shift) % 6);
      uint32_t idx = m + kMaxDlRb - cfg.nof_prb;
      if (mask[l * width + k]) continue;
      grid[l * width + k] = cplx((1.0f - 2.0f * bits[2 * idx]) * s, (1.0f - 2.0f * bits[2 * idx + 1]) * s);
    }
  }
  return grid;
}

/** In-place radix-2 FFT (inverse unnormalised when `inverse`); the size must be a power of two. */
inline void fft(std::vector<cplx>& a, bool inverse) {
  const size_t n = a.size();
  for (size_t i = 1, j = 0; i < n; i++) {
    size_t bit = n >> 1;
    for (; j & bit; bit >>= 1) j ^= bit;
    j ^= bit;
    if (i < j) std::swap(a[i], a[j]);
  }
  for (size_t len = 2; len <= n; len <<= 1) {
    const double ang = 2.0 * M_PI / static_cast<double>(len) * (inverse ? 1.0 : -1.0);
    const cplx wl(static_cast<float>(std::cos(ang)), static_cast<float>(std::sin(ang)));
    for (size_t i = 0; i < n; i += len) {
      cplx w(1.0f, 0.0f);
      for (size_t k = 0; k < len / 2; k++) {
        const cplx u = a[i + k], v = a[i + k + len / 2] * w;
        a[i + k] = u + v;
        a[i + k + len / 2] = u - v;
        // recompute the twiddle exactly every 64 steps to keep float error down
        w = ((k + 1) % 64 == 0) ? cplx(static_cast<float>(std::cos(ang * (k + 1))), static_cast<float>(std::sin(ang * (k + 1)))) : w * wl;
      }
    }
  }
}

/** Time-domain waveform of one PRS subframe (two slots), for an FFT of `n_fft` points (sample rate 15 kHz x n_fft). DC unused, the subcarrier mapping of an
 *  LTE downlink: negative frequencies in the upper bins. Cyclic prefix: 160/144 samples (first/other symbols of a slot) at 2048 points for normal CP, 512 for
 *  extended CP, scaled to n_fft. */
inline std::vector<cplx> subframe_waveform(const Cfg& cfg, const CellParams& c, uint32_t n_f, uint32_t subframe, uint32_t n_fft) {
  const uint32_t nsym = c.normal_cp ? 7 : 6, width = 12 * c.nof_prb;
  const uint32_t cp0 = c.normal_cp ? 160 * n_fft / 2048 : 512 * n_fft / 2048;
  const uint32_t cp = c.normal_cp ? 144 * n_fft / 2048 : 512 * n_fft / 2048;
  std::vector<cplx> out;
  for (uint32_t slot = 0; slot < 2; slot++) {
    const std::vector<cplx> grid = slot_grid(cfg, c, n_f, 2 * subframe + slot);
    for (uint32_t l = 0; l < nsym; l++) {
      std::vector<cplx> spec(n_fft, cplx(0.0f, 0.0f));
      for (uint32_t k = 0; k < width; k++) {
        const int krel = static_cast<int>(k) - static_cast<int>(6 * c.nof_prb);
        const uint32_t bin = krel < 0 ? n_fft + krel : krel + 1;
        spec[bin] = grid[l * width + k];
      }
      fft(spec, true);
      const float scale = 1.0f / std::sqrt(static_cast<float>(n_fft));   // ifft (1/N) times sqrt(N)
      const uint32_t pre = (l == 0) ? cp0 : cp;
      for (uint32_t i = n_fft - pre; i < n_fft; i++) out.push_back(spec[i] * scale);
      for (uint32_t i = 0; i < n_fft; i++) out.push_back(spec[i] * scale);
    }
  }
  return out;
}

/** Result of one correlation. */
struct Toa {
  bool   ok       = false;
  double lag      = 0.0;   // position of the reference's first sample in the window, in samples, with sub-sample resolution
  double peak     = 0.0;   // correlation magnitude at the peak
  double ratio    = 0.0;   // peak over the median magnitude elsewhere in the search range
  double coherence = 0.0;  // peak over (norm of the received segment x norm of the reference): 1 for a perfect copy
};

/** The spectrum of a window zero-padded for correlation with references of `ref_len` samples; computed once and shared by every cell's correlation. */
struct Prepared {
  size_t            m = 0;
  size_t            n = 0;
  std::vector<cplx> spec;
};

inline Prepared prepare(const std::vector<cplx>& window, size_t ref_len) {
  Prepared p;
  p.n = window.size();
  p.m = 1;
  while (p.m < window.size() + ref_len) p.m <<= 1;
  p.spec.assign(p.m, cplx(0, 0));
  std::copy(window.begin(), window.end(), p.spec.begin());
  fft(p.spec, false);
  return p;
}

/** Correlate the prepared window with `ref` and find the strongest match with the first reference sample between lag_lo and lag_hi (inclusive, in samples).
 *  The coarse peak is found on the integer lags by an FFT correlation; its position between samples is then found by evaluating the correlation at fractional
 *  lags within 0.6 sample of it, in the frequency domain over the bins where the reference has energy, followed by a parabola through the best three points. */
inline Toa correlate(const Prepared& prep, const std::vector<cplx>& window, const std::vector<cplx>& ref, int lag_lo, int lag_hi,
                     int peak_lo = std::numeric_limits<int>::min(), int peak_hi = std::numeric_limits<int>::max()) {
  Toa out;
  const int n = static_cast<int>(window.size()), r = static_cast<int>(ref.size());
  lag_lo = std::max(lag_lo, 0);
  lag_hi = std::min(lag_hi, n - r);
  if (r == 0 || lag_lo > lag_hi || prep.n != window.size() || prep.m < window.size() + ref.size()) return out;
  const size_t m = prep.m;
  std::vector<cplx> s(m, cplx(0, 0));
  std::copy(ref.begin(), ref.end(), s.begin());
  fft(s, false);
  std::vector<cplx> cspec(m);
  for (size_t i = 0; i < m; i++) cspec[i] = prep.spec[i] * std::conj(s[i]);
  std::vector<cplx> c = cspec;
  fft(c, true);
  const float inv_m = 1.0f / static_cast<float>(m);
  std::vector<float> mag(lag_hi - lag_lo + 1);
  // The peak is looked for between peak_lo and peak_hi (a search window narrower than the range), the background it is judged against over the whole range.
  const int plo = std::max(lag_lo, peak_lo), phi = std::min(lag_hi, peak_hi);
  if (plo > phi) return out;
  int best = plo - lag_lo;
  for (int lag = lag_lo; lag <= lag_hi; lag++) mag[lag - lag_lo] = std::abs(c[lag]) * inv_m;
  for (int lag = plo; lag <= phi; lag++)
    if (mag[lag - lag_lo] > mag[best]) best = lag - lag_lo;
  // median of the magnitudes away from the peak
  std::vector<float> rest;
  rest.reserve(mag.size());
  for (size_t i = 0; i < mag.size(); i++)
    if (std::abs(static_cast<int>(i) - best) > 40) rest.push_back(mag[i]);
  float median = 0.0f;
  if (!rest.empty()) {
    std::nth_element(rest.begin(), rest.begin() + rest.size() / 2, rest.end());
    median = rest[rest.size() / 2];
  }
  // fractional lag: the correlation at lag tau is (1/m) sum over bins of cspec[i] exp(+j 2 pi f_i tau / m), tau = coarse + 0.1 (d - 6), d = 0..12
  const int coarse = lag_lo + best;
  constexpr int kSteps = 13;
  float smax = 0.0f;
  for (size_t i = 0; i < m; i++) smax = std::max(smax, std::abs(s[i]));
  std::vector<std::complex<double>> acc(kSteps, std::complex<double>(0, 0));
  for (size_t i = 0; i < m; i++) {
    if (std::abs(s[i]) <= 1e-3f * smax) continue;
    const double f = (i < m / 2) ? static_cast<double>(i) : static_cast<double>(i) - static_cast<double>(m);
    const double w = 2.0 * M_PI * f / static_cast<double>(m);
    std::complex<double> ph = std::polar(1.0, w * (coarse - 0.6));
    const std::complex<double> step = std::polar(1.0, w * 0.1);
    const std::complex<double> cs(cspec[i]);
    for (int d = 0; d < kSteps; d++) {
      acc[d] += cs * ph;
      ph *= step;
    }
  }
  std::vector<double> val(kSteps);
  for (int d = 0; d < kSteps; d++) val[d] = std::abs(acc[d]) / static_cast<double>(m);
  int bi = 0;
  for (int d = 1; d < kSteps; d++)
    if (val[d] > val[bi]) bi = d;
  double frac = 0.0;
  if (bi > 0 && bi + 1 < kSteps) {
    const double a = val[bi - 1], b = val[bi], cc = val[bi + 1], den = a - 2 * b + cc;
    if (den != 0.0) frac = 0.5 * (a - cc) / den;
  }
  out.lag = coarse + 0.1 * (bi - 6) + 0.1 * frac;
  out.peak = val[bi];
  out.ratio = median > 0.0f ? out.peak / median : 0.0;
  // coherence: peak over the norms of the received segment at the coarse lag and of the reference
  double ex = 0.0, er = 0.0;
  for (int i = 0; i < r; i++) {
    ex += std::norm(window[coarse + i]);
    er += std::norm(ref[i]);
  }
  out.coherence = (ex > 0.0 && er > 0.0) ? out.peak / std::sqrt(ex * er) : 0.0;
  out.ok = true;
  return out;
}

/** As above for a single reference, preparing the window itself. */
inline Toa correlate(const std::vector<cplx>& window, const std::vector<cplx>& ref, int lag_lo, int lag_hi) {
  return correlate(prepare(window, ref.size()), window, ref, lag_lo, lag_hi);
}

}  // namespace prs
}  // namespace MBMS_RT

#endif  // MBMS_RT_PRS_SIGNAL_H
