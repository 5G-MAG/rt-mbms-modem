// Test of src/PrsToa.{h,cpp}: the ring of subframes, the trigger at a PRS occasion, muting, the checks on the assistance data and the measurement on
// synthetic subframes with known delays, all through push() as the modem's receive loop calls it.

#include <chrono>
#include <cmath>
#include <cstdio>
#include <random>
#include <thread>

#include "../src/PrsToa.h"

using namespace MBMS_RT;
using namespace MBMS_RT::prs;

static int failures = 0;
#define CHECK(cond, ...)                                  \
  do {                                                    \
    if (!(cond)) {                                        \
      printf("FAIL %s:%d: ", __FILE__, __LINE__);         \
      printf(__VA_ARGS__);                                \
      printf("\n");                                       \
      failures++;                                         \
    }                                                     \
  } while (0)

static const uint32_t kNfft = 512, kSfLen = 7680;
static const double kFs = 7.68e6;

struct Syn { uint32_t cell_id; double delay_s; double power_db; };

// three consecutive subframes holding the cells' PRS subframe (subframe 0, n_f of the middle one) at fractional delays, plus noise
static std::vector<std::complex<float>> three_subframes(const std::vector<Syn>& cells, uint32_t mid_tti, double snr_db, uint32_t seed) {
  const size_t n_win = 3 * kSfLen;
  size_t m = 1; while (m < n_win) m <<= 1;
  std::vector<cplx> sum(m, cplx(0, 0));
  for (const auto& c : cells) {
    Cfg cfg; cfg.enabled = true; cfg.id = c.cell_id; cfg.config_index = 2435; cfg.nof_prb = 25;
    CellParams cp; cp.nof_prb = 25; cp.normal_cp = false; cp.mbms_dedicated = true; cp.nof_ports = 1; cp.cell_id = c.cell_id;
    auto w = subframe_waveform(cfg, cp, (mid_tti / 10) % 1024, mid_tti % 10, kNfft);
    std::vector<cplx> pad(m, cplx(0, 0));
    for (size_t i = 0; i < w.size(); i++) pad[kSfLen + i] = w[i] * static_cast<float>(std::pow(10.0, c.power_db / 20.0));
    fft(pad, false);
    for (size_t i = 0; i < m; i++) {
      const double f = (i < m / 2) ? static_cast<double>(i) : static_cast<double>(i) - static_cast<double>(m);
      const double ph = -2.0 * M_PI * f * c.delay_s * kFs / static_cast<double>(m);
      sum[i] += pad[i] * cplx(static_cast<float>(std::cos(ph)), static_cast<float>(std::sin(ph)));
    }
  }
  fft(sum, true);
  std::vector<std::complex<float>> win(n_win);
  std::mt19937 rng(seed);
  std::normal_distribution<double> nd(0.0, std::sqrt(std::pow(10.0, -snr_db / 10.0) / 2.0));
  for (size_t i = 0; i < n_win; i++) win[i] = sum[i] / static_cast<float>(m) + std::complex<float>(static_cast<float>(nd(rng)), static_cast<float>(nd(rng)));
  return win;
}

static void feed(PrsToa& p, uint32_t mid_tti, const std::vector<std::complex<float>>& w, const CellParams& cell) {
  for (int i = 0; i < 3; i++) {
    const uint32_t tti = (mid_tti + 10240 - 1 + i) % 10240;
    p.push(tti, 1000000000ull + tti * 1000000ull, w.data() + i * kSfLen, kSfLen, cell);
  }
}

static bool wait_for(PrsToa& p, size_t n) {
  for (int i = 0; i < 200; i++) {
    if (p.measurements().size() >= n) return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  return false;
}

static PrsToa::Assistance make_assistance(const std::vector<uint32_t>& ids, uint32_t ref = 1) {
  PrsToa::Assistance a; a.reference_cell_id = ref;
  for (uint32_t id : ids) { PrsToa::CellInfo c; c.cell_id = id; c.prs.enabled = true; c.prs.config_index = 2435; c.prs.nof_prb = 25; a.cells.push_back(c); }
  return a;
}

int main() {
  CellParams cell; cell.nof_prb = 25; cell.normal_cp = false; cell.mbms_dedicated = true; cell.nof_ports = 1; cell.cell_id = 1;
  std::vector<Syn> cells = {{1, 12.0e-6, 0}, {2, 150.8e-6, 0}, {3, -38.4e-6, 0}, {4, 63.1e-6, 0}};
  std::string why;

  // --- the assistance checks
  {
    PrsToa p;
    CHECK(!p.has_assistance(), "no assistance at the start");
    CHECK(p.set_assistance(make_assistance({1, 2, 3, 4}), why), "a good assistance is accepted: %s", why.c_str());
    CHECK(p.has_assistance(), "assistance present");
    auto bad = make_assistance({1, 2}); bad.reference_cell_id = 9;
    CHECK(!p.set_assistance(bad, why) && !why.empty(), "an unknown reference cell is refused");
    CHECK(p.has_assistance(), "a refused assistance leaves the previous one");
    auto dup = make_assistance({1, 2, 2});
    CHECK(!p.set_assistance(dup, why), "a cell listed twice is refused");
    auto other = make_assistance({1, 2}); other.cells[1].prs.config_index = 2436;
    CHECK(!p.set_assistance(other, why), "a different configuration index (a subframe offset between cells) is refused: %s", why.c_str());
    auto n2 = make_assistance({1, 2}); n2.cells[1].prs.nof_subframes = 2;
    CHECK(!p.set_assistance(n2, why), "N_PRS 2 is refused");
    auto bw = make_assistance({1, 2}); bw.cells[0].prs.nof_prb = 20;
    CHECK(!p.set_assistance(bw, why), "a PRS bandwidth of 20 PRB is refused");
    auto res = make_assistance({1, 2}); res.cells[0].prs.config_index = 2555; res.cells[1].prs.config_index = 2555;
    CHECK(!p.set_assistance(res, why), "a reserved configuration index is refused");
    auto mut = make_assistance({1, 2}); mut.cells[1].prs.muting = "011";
    CHECK(!p.set_assistance(mut, why), "a 3 bit muting sequence is refused");
    mut.cells[1].prs.muting = "0x";
    CHECK(!p.set_assistance(mut, why), "a muting sequence with other characters is refused");
    auto big = make_assistance({1, 2}); big.cells[1].prs.id = 4096;
    CHECK(!p.set_assistance(big, why), "a PRS ID above 4095 is refused");
    PrsToa::Assistance none;
    CHECK(p.set_assistance(none, why) && !p.has_assistance(), "an empty assistance clears it");
  }

  // --- measurement through push(), including the wrap at tti 10240
  {
    PrsToa p;
    CHECK(p.set_assistance(make_assistance({1, 2, 3, 4}), why), "%s", why.c_str());
    size_t expected = 0;
    for (uint32_t mid : {0u, 40u}) {   // tti 0: the previous subframe is tti 10239
      auto w = three_subframes(cells, mid, 30.0, 7 + mid);
      feed(p, mid, w, cell);
      CHECK(wait_for(p, ++expected), "measurement %zu expected", expected);
    }
    auto ms = p.measurements();
    if (ms.size() >= 2) {
      for (const auto& m : ms) {
        CHECK(m.cells.size() == 4 && m.sf_len == kSfLen && std::fabs(m.sample_rate_hz - kFs) < 1.0, "measurement shape");
        CHECK(m.tti == 0 || m.tti == 40, "occasion tti %u", m.tti);
        CHECK(m.sample_clock_ns == 1000000000ull + m.tti * 1000000ull, "the sample clock is that of the PRS subframe: %llu", (unsigned long long)m.sample_clock_ns);
        for (size_t i = 0; i < 4; i++) {
          CHECK(m.cells[i].present && m.cells[i].cell_id == cells[i].cell_id && m.cells[i].prs_id == cells[i].cell_id, "cell %zu present", i);
          // each cell's own ToA carries the interference of the others, a few ns; the differences are what is used
          CHECK(std::fabs(m.cells[i].toa_ns - cells[i].delay_s * 1e9) < 5.0, "tti %u cell %zu: toa %.2f ns, truth %.2f ns", m.tti, i + 1, m.cells[i].toa_ns, cells[i].delay_s * 1e9);
          CHECK(m.cells[i].coherence > 0.1 && m.cells[i].ratio > 5.0, "cell %zu quality %.3f / %.1f", i + 1, m.cells[i].coherence, m.cells[i].ratio);
        }
      }
    }
    auto st = p.stats();
    printf("a four cell measurement took %.0f ms on the worker\n", st.last_ms);
    CHECK(st.last_ms > 0.0 && st.last_ms < 2000.0, "a measurement took %.0f ms", st.last_ms);
    CHECK(st.windows == 2 && st.gaps == 0 && st.unsupported == 0, "stats: %llu windows, %llu gaps", (unsigned long long)st.windows, (unsigned long long)st.gaps);
  }

  // --- not an occasion, a gap, a different subframe length, a non-CAS frame
  {
    PrsToa p;
    CHECK(p.set_assistance(make_assistance({1, 2}), why), "%s", why.c_str());
    auto w = three_subframes({cells[0], cells[1]}, 40, 30.0, 3);
    feed(p, 41, w, cell);                                  // the middle subframe is not an occasion (tti 41)
    std::this_thread::sleep_for(std::chrono::milliseconds(400));
    CHECK(p.measurements().empty() && p.stats().windows == 0 && p.stats().dropped == 0, "a window whose middle subframe is not an occasion is not even attempted");
    feed(p, 80, w, cell);                                  // a real occasion ...
    CHECK(wait_for(p, 1), "the occasion at tti 80 is measured");
    auto n = p.measurements().size();
    // a gap: tti 119, 121 (not 120) around an occasion at 120
    p.push(119, 0, w.data(), kSfLen, cell);
    p.push(121, 0, w.data() + kSfLen, kSfLen, cell);
    p.push(122, 0, w.data() + 2 * kSfLen, kSfLen, cell);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    CHECK(p.measurements().size() == n, "a window with a gap is not measured");
    // occasion in a frame that is not a cell acquisition frame of a dedicated cell: tti 120 is frame 12 (12 % 4 == 0 is fine); use a cell with 25 PRB and tti 150
    CellParams ded = cell;
    auto w2 = three_subframes({cells[0], cells[1]}, 40, 30.0, 4);
    feed(p, 160, w2, ded);
    CHECK(wait_for(p, n + 1), "tti 160 (frame 16) is a cell acquisition occasion");
    // different length for one of the three: ignored
    std::vector<std::complex<float>> shorter(w.begin(), w.begin() + kSfLen - 1);
    auto before = p.measurements().size();
    p.push(199, 0, w.data(), kSfLen, cell);
    p.push(200, 0, shorter.data(), kSfLen - 1, cell);
    p.push(201, 0, w.data(), kSfLen, cell);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    CHECK(p.measurements().size() == before, "subframes of different lengths are not measured");
    // Only an occasion that falls in the middle of a broken window is counted as a gap: the length mismatch around tti 200 is one; the window around
    // 119, 121, 122 never held the occasion at 120, so nothing is known to be lost there.
    CHECK(p.stats().gaps == 1, "gaps counted: %llu", (unsigned long long)p.stats().gaps);
  }

  // --- an ordinary cell: the cell acquisition guard does not apply, so only the occasion check keeps a non-occasion window from being measured
  {
    PrsToa p;
    CHECK(p.set_assistance(make_assistance({1, 2}), why), "%s", why.c_str());
    CellParams ordinary = cell; ordinary.mbms_dedicated = false;
    auto w = three_subframes({cells[0], cells[1]}, 40, 30.0, 14);
    feed(p, 41, w, ordinary);
    std::this_thread::sleep_for(std::chrono::milliseconds(400));
    CHECK(p.measurements().empty() && p.stats().windows == 0 && p.stats().dropped == 0, "an ordinary cell's non-occasion window is not attempted");
    feed(p, 80, w, ordinary);
    CHECK(wait_for(p, 1), "and its occasion is measured");
  }

  // --- a gap right next to an occasion: the first or the last of the three subframes is not the neighbour it should be
  {
    PrsToa p;
    CHECK(p.set_assistance(make_assistance({1, 2}), why), "%s", why.c_str());
    auto w = three_subframes({cells[0], cells[1]}, 40, 30.0, 12);
    p.push(38, 0, w.data(), kSfLen, cell);                 // 38, 40, 41: 39 is missing before the occasion at 40
    p.push(40, 0, w.data() + kSfLen, kSfLen, cell);
    p.push(41, 0, w.data() + 2 * kSfLen, kSfLen, cell);
    p.push(79, 0, w.data(), kSfLen, cell);                 // 79, 80, 82: 81 is missing after the occasion at 80
    p.push(80, 0, w.data() + kSfLen, kSfLen, cell);
    p.push(82, 0, w.data() + 2 * kSfLen, kSfLen, cell);
    std::this_thread::sleep_for(std::chrono::milliseconds(400));
    CHECK(p.measurements().empty() && p.stats().gaps == 2, "both broken windows are refused and counted: %llu gaps, %zu measurements", (unsigned long long)p.stats().gaps, p.measurements().size());
  }

  // --- a configuration whose occasions are not all cell acquisition subframes: index 2400 (T_PRS 5) is active at tti 10, frame 1, which a dedicated
  //     cell sends as MBSFN; an ordinary cell sends it as a 15 kHz subframe
  {
    PrsToa p;
    auto a = make_assistance({1, 2});
    for (auto& c : a.cells) c.prs.config_index = 2400;
    CHECK(p.set_assistance(a, why), "%s", why.c_str());
    auto w = three_subframes({cells[0], cells[1]}, 40, 30.0, 13);          // content does not matter for the trigger
    feed(p, 10, w, cell);                                  // dedicated: no measurement
    std::this_thread::sleep_for(std::chrono::milliseconds(400));
    CHECK(p.measurements().empty(), "no PRS measurement in a subframe an MBMS-dedicated cell sends as MBSFN");
    CellParams plain = cell; plain.mbms_dedicated = false;
    feed(p, 10, w, plain);                                 // an ordinary cell: measured
    CHECK(wait_for(p, 1), "an ordinary cell has PRS in that subframe");
  }

  // --- a busy worker drops the next occasion and says so
  {
    PrsToa p;
    CHECK(p.set_assistance(make_assistance({1, 2, 3, 4}), why), "%s", why.c_str());
    auto w0 = three_subframes(cells, 0, 30.0, 21);
    auto w1 = three_subframes(cells, 40, 30.0, 22);
    feed(p, 0, w0, cell);
    feed(p, 40, w1, cell);      // straight after: the first measurement (four cells) takes far longer than the loop to hand over three subframes
    CHECK(wait_for(p, 1), "the first occasion is measured");
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    auto st = p.stats();
    CHECK(st.windows + st.dropped == 2 && st.dropped == 1 && st.windows == 1, "one measured and one dropped: %llu and %llu", (unsigned long long)st.windows, (unsigned long long)st.dropped);
  }

  // --- muting: cell 2 muted at the second occasion
  {
    PrsToa p;
    auto a = make_assistance({1, 2});
    a.cells[1].prs.muting = "10";          // occasion 0 (tti 0) sent, occasion 1 (tti 40) muted
    CHECK(p.set_assistance(a, why), "%s", why.c_str());
    auto w0 = three_subframes({cells[0], cells[1]}, 0, 30.0, 5);
    auto w1 = three_subframes({cells[0]}, 40, 30.0, 6);
    feed(p, 0, w0, cell);
    CHECK(wait_for(p, 1), "first occasion");
    feed(p, 40, w1, cell);
    CHECK(wait_for(p, 2), "second occasion");
    auto ms = p.measurements();
    CHECK(ms.size() == 2 && ms[0].cells[1].present && !ms[1].cells[1].present && ms[1].cells[0].present,
          "cell 2 is reported in the first occasion and muted in the second");
  }

  // --- samples at a rate the correlator does not handle are counted, not measured
  {
    PrsToa p;
    CHECK(p.set_assistance(make_assistance({1, 2}), why), "%s", why.c_str());
    std::vector<std::complex<float>> junk(3 * 11520);
    CellParams c75 = cell; c75.nof_prb = 75;
    for (int i = 0; i < 3; i++) p.push(39 + i, 0, junk.data() + i * 11520, 11520, c75);   // 1536-point FFT
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    CHECK(p.stats().unsupported == 1 && p.measurements().empty(), "a 1536-point rate is counted as unsupported: %llu", (unsigned long long)p.stats().unsupported);
  }

  // --- a change of the assistance data drops what was measured under the old one, and a measurement still running when it changes is discarded
  {
    PrsToa p;
    CHECK(p.set_assistance(make_assistance({1, 2}), why), "%s", why.c_str());
    auto w = three_subframes({cells[0], cells[1]}, 0, 30.0, 31);
    feed(p, 0, w, cell);
    CHECK(wait_for(p, 1), "a measurement under the first assistance data");
    auto other = make_assistance({1, 2});
    other.cells[1].prs.id = 77;                               // the same cells, a different PRS ID
    CHECK(p.set_assistance(other, why), "%s", why.c_str());
    CHECK(p.measurements().empty(), "the measurements of the old assistance data are gone after a change");
    // a measurement in flight: feed an occasion, change the data at once, the result must not appear
    auto w2 = three_subframes({cells[0], cells[1]}, 40, 30.0, 32);
    feed(p, 40, w2, cell);
    CHECK(p.set_assistance(make_assistance({1, 2}), why), "%s", why.c_str());
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    CHECK(p.measurements().empty(), "a measurement finished after the change is not kept (%zu kept)", p.measurements().size());
    CHECK(p.stats().discarded == 1, "and it is counted: %llu", (unsigned long long)p.stats().discarded);
    // clearing drops results ...
    auto w3 = three_subframes({cells[0], cells[1]}, 80, 30.0, 33);
    feed(p, 80, w3, cell);
    CHECK(wait_for(p, 1), "a measurement under the current data");
    PrsToa::Assistance none;
    CHECK(p.set_assistance(none, why), "clearing");
    CHECK(p.measurements().empty(), "clearing drops the results");
    // ... and a measurement still running when the data are cleared is discarded too
    CHECK(p.set_assistance(make_assistance({1, 2}), why), "%s", why.c_str());
    auto w4 = three_subframes({cells[0], cells[1]}, 120, 30.0, 34);
    feed(p, 120, w4, cell);
    CHECK(p.set_assistance(none, why), "clearing while a measurement runs");
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    CHECK(p.measurements().empty(), "a measurement finished after the clearing is not kept (%zu kept)", p.measurements().size());
    CHECK(p.stats().discarded == 2, "and it is counted: %llu", (unsigned long long)p.stats().discarded);
  }

  // --- nothing is kept without assistance
  {
    PrsToa p;
    auto w = three_subframes({cells[0]}, 40, 30.0, 8);
    feed(p, 40, w, cell);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    CHECK(p.measurements().empty() && p.stats().windows == 0, "no measurement without assistance");
  }

  printf("prs_toa_test: %d failures\n", failures);
  return failures ? 1 : 0;
}
