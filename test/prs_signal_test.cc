// Test of src/PrsSignal.h: the PRS mapping, time-domain waveform, occasions and muting against the golden vectors written by
// rt-mbms-examples/positioning-assistance/tools/gen_prs_golden.py (the same file rt-mbms-tx's lib/test/prs/prs_test.c reads), and the time of arrival
// estimator on synthetic windows with known delays.
//
//   case <name> <n_id_prs> <n_s> <n_rb_dl> <n_rb_prs> <four_ports> <n_f> <mbms_dedicated> <normal_cp> <count>   then <count> lines "k l re im"
//   wave <name> <n_id_prs> <n_rb_dl> <n_f> <subframe> <normal_cp> <n_samples> <energy> <count>                   then <count> lines "index re im"
//   active <index> <n_prs> <muting or -> <count> <sum>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <map>
#include <random>
#include <sstream>
#include <string>

#include "../src/PrsSignal.h"

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

static uint32_t fft_size(uint32_t n_rb) {
  switch (n_rb) {
    case 6: return 128;
    case 15: return 256;
    case 25: return 512;
    case 50: return 1024;
    case 75: return 1536;   // not a power of two: not used here
    default: return 2048;
  }
}

static void run_case(std::istringstream& hdr, std::ifstream& f, const std::string& name) {
  unsigned nid, n_s, n_rb_dl, n_rb_prs, four, n_f, ded, normal_cp, count;
  hdr >> nid >> n_s >> n_rb_dl >> n_rb_prs >> four >> n_f >> ded >> normal_cp >> count;
  Cfg cfg;
  cfg.enabled = true; cfg.id = nid; cfg.config_index = 2435; cfg.nof_prb = n_rb_prs;
  CellParams c;
  c.nof_prb = n_rb_dl; c.normal_cp = normal_cp != 0; c.mbms_dedicated = ded != 0; c.nof_ports = four ? 4 : 2; c.cell_id = 1;
  std::vector<cplx> g = slot_grid(cfg, c, n_f, n_s);
  size_t nonzero = 0;
  for (auto& v : g) nonzero += (v != cplx(0, 0));
  CHECK(nonzero == count, "%s: %zu non-zero elements, golden has %u", name.c_str(), nonzero, count);
  const uint32_t width = 12 * n_rb_dl;
  for (unsigned i = 0; i < count; i++) {
    unsigned k, l; double re, im;
    f >> k >> l >> re >> im;
    cplx v = g[l * width + k];
    CHECK(std::fabs(v.real() - re) < 1e-6 && std::fabs(v.imag() - im) < 1e-6, "%s: k=%u l=%u is %f%+fi, golden %f%+fi", name.c_str(), k, l, v.real(), v.imag(), re, im);
  }
}

static void run_wave(std::istringstream& hdr, std::ifstream& f, const std::string& name) {
  unsigned nid, n_rb_dl, n_f, subframe, normal_cp, n_samples, count; double energy;
  hdr >> nid >> n_rb_dl >> n_f >> subframe >> normal_cp >> n_samples >> energy >> count;
  Cfg cfg; cfg.enabled = true; cfg.id = nid; cfg.config_index = 2435; cfg.nof_prb = n_rb_dl;
  CellParams c; c.nof_prb = n_rb_dl; c.normal_cp = normal_cp != 0; c.mbms_dedicated = true; c.nof_ports = 1; c.cell_id = 1;
  std::vector<cplx> w = subframe_waveform(cfg, c, n_f, subframe, fft_size(n_rb_dl));
  CHECK(w.size() == n_samples, "%s: %zu samples, golden %u", name.c_str(), w.size(), n_samples);
  double e = 0; for (auto& v : w) e += std::norm(v);
  CHECK(std::fabs(e - energy) < 1e-4 * energy, "%s: energy %f, golden %f", name.c_str(), e, energy);
  for (unsigned i = 0; i < count; i++) {
    unsigned idx; double re, im;
    f >> idx >> re >> im;
    CHECK(idx < w.size() && std::fabs(w[idx].real() - re) < 2e-5 && std::fabs(w[idx].imag() - im) < 2e-5, "%s: sample %u is %f%+fi, golden %f%+fi", name.c_str(), idx, w[idx].real(), w[idx].imag(), re, im);
  }
}

static void run_active(std::istringstream& hdr) {
  unsigned index, n_prs, count; unsigned long long sum; std::string muting;
  hdr >> index >> n_prs >> muting >> count >> sum;
  Cfg cfg; cfg.enabled = true; cfg.config_index = index; cfg.nof_subframes = n_prs; if (muting != "-") cfg.muting = muting;
  unsigned long long s = 0; unsigned n = 0;
  for (uint32_t t = 0; t < 10240; t++) if (active(cfg, t)) { n++; s += t; }
  CHECK(n == count && s == sum, "active %u N_PRS %u muting %s: %u subframes (sum %llu), golden %u (sum %llu)", index, n_prs, muting.c_str(), n, s, count, sum);
  CHECK(active(cfg, 10240 + 40) == active(cfg, 40), "index %u: tti does not wrap", index);
}

// A window of three subframes holding the PRS subframes of several cells, each at its own fractional delay (against the middle subframe's nominal start)
// and power, plus white Gaussian noise of variance 10^(-snr/10) per resource element.
struct Syn { uint32_t cell_id; double delay_s; double power_db; };

static std::vector<cplx> make_window(const std::vector<Syn>& cells, uint32_t n_rb, double snr_db, uint32_t seed, uint32_t* sf_len_out) {
  const uint32_t n_fft = fft_size(n_rb), sf_len = 15 * n_fft;
  *sf_len_out = sf_len;
  const size_t n_win = 3 * sf_len;
  size_t m = 1; while (m < n_win) m <<= 1;
  std::vector<cplx> sum(m, cplx(0, 0));
  const double fs = 15000.0 * n_fft;
  for (const auto& c : cells) {
    Cfg cfg; cfg.enabled = true; cfg.id = c.cell_id; cfg.config_index = 2435; cfg.nof_prb = n_rb;
    CellParams cp; cp.nof_prb = n_rb; cp.normal_cp = false; cp.mbms_dedicated = true; cp.nof_ports = 1; cp.cell_id = c.cell_id;
    std::vector<cplx> w = subframe_waveform(cfg, cp, 0, 0, n_fft);
    std::vector<cplx> pad(m, cplx(0, 0));
    for (size_t i = 0; i < w.size(); i++) pad[sf_len + i] = w[i] * static_cast<float>(std::pow(10.0, c.power_db / 20.0));
    fft(pad, false);
    for (size_t i = 0; i < m; i++) {
      const double f = (i < m / 2) ? static_cast<double>(i) : static_cast<double>(i) - static_cast<double>(m);
      const double ph = -2.0 * M_PI * f * c.delay_s * fs / static_cast<double>(m);   // delay in samples: delay_s * fs
      sum[i] += pad[i] * cplx(static_cast<float>(std::cos(ph)), static_cast<float>(std::sin(ph)));
    }
  }
  fft(sum, true);
  std::vector<cplx> win(n_win);
  std::mt19937 rng(seed);
  std::normal_distribution<double> nd(0.0, std::sqrt(std::pow(10.0, -snr_db / 10.0) / 2.0));
  for (size_t i = 0; i < n_win; i++) win[i] = sum[i] / static_cast<float>(m) + cplx(static_cast<float>(nd(rng)), static_cast<float>(nd(rng)));
  return win;
}

// ToA of every cell relative to cell 0, in seconds
static std::vector<double> estimate(const std::vector<cplx>& win, const std::vector<Syn>& cells, uint32_t n_rb, uint32_t sf_len) {
  const uint32_t n_fft = fft_size(n_rb);
  const double fs = 15000.0 * n_fft;
  const int half = static_cast<int>(0.75e-3 * fs);
  std::vector<double> toa;
  for (const auto& c : cells) {
    Cfg cfg; cfg.enabled = true; cfg.id = c.cell_id; cfg.config_index = 2435; cfg.nof_prb = n_rb;
    CellParams cp; cp.nof_prb = n_rb; cp.normal_cp = false; cp.mbms_dedicated = true; cp.nof_ports = 1; cp.cell_id = c.cell_id;
    Toa t = correlate(win, subframe_waveform(cfg, cp, 0, 0, n_fft), static_cast<int>(sf_len) - half, static_cast<int>(sf_len) + half);
    CHECK(t.ok, "correlation for cell %u failed", c.cell_id);
    toa.push_back((t.lag - sf_len) / fs);
  }
  std::vector<double> rel;
  for (size_t i = 1; i < toa.size(); i++) rel.push_back(toa[i] - toa[0]);
  return rel;
}

static void test_estimator() {
  const uint32_t n_rb = 25;
  // delays against the reference cell: propagation plus a transmit offset, within the +-0.5 ms of the closest-subframe definition
  std::vector<Syn> cells = {{1, 0.0, 0}, {2, 138.8e-6, 0}, {3, -50.4e-6, 0}, {4, 51.1e-6, 0}};
  uint32_t sf_len;
  {
    // one cell alone: the estimator itself is exact to a fraction of a nanosecond
    std::vector<Syn> alone = {{1, 77.3e-6, 0}};
    auto win = make_window(alone, n_rb, 30.0, 11, &sf_len);
    const double fs = 15000.0 * fft_size(n_rb); const int half = static_cast<int>(0.75e-3 * fs);
    Cfg cfg; cfg.enabled = true; cfg.id = 1; cfg.config_index = 2435; cfg.nof_prb = n_rb;
    CellParams cp; cp.nof_prb = n_rb; cp.normal_cp = false; cp.mbms_dedicated = true; cp.nof_ports = 1; cp.cell_id = 1;
    Toa t = correlate(win, subframe_waveform(cfg, cp, 0, 0, fft_size(n_rb)), static_cast<int>(sf_len) - half, static_cast<int>(sf_len) + half);
    CHECK(t.ok && std::fabs((t.lag - sf_len) / fs - 77.3e-6) < 0.1e-9, "single cell: error %.3f ns", ((t.lag - sf_len) / fs - 77.3e-6) * 1e9);
    CHECK(t.coherence > 0.9, "a clean single cell has coherence close to 1: %.3f", t.coherence);
  }
  {
    // four cells overlapping: the other cells' PRS leak into each correlation, 1 to 3 ns here (a single cell is exact), so the bound is 5 ns
    auto win = make_window(cells, n_rb, 30.0, 11, &sf_len);
    auto rel = estimate(win, cells, n_rb, sf_len);
    for (size_t i = 0; i < rel.size(); i++)
      CHECK(std::fabs(rel[i] - (cells[i + 1].delay_s - cells[0].delay_s)) < 5e-9, "30 dB: cell %zu error %.2f ns", i + 2, (rel[i] - (cells[i + 1].delay_s - cells[0].delay_s)) * 1e9);
  }
  {
    double sum2 = 0; int n = 0;
    for (uint32_t seed = 1; seed <= 4; seed++) {
      auto win = make_window(cells, n_rb, 10.0, seed, &sf_len);
      auto rel = estimate(win, cells, n_rb, sf_len);
      for (size_t i = 0; i < rel.size(); i++) { double e = rel[i] - (cells[i + 1].delay_s - cells[0].delay_s); sum2 += e * e; n++; }
    }
    double rms = std::sqrt(sum2 / n);
    CHECK(rms < 8e-9, "10 dB per resource element: rms error %.2f ns", rms * 1e9);   // measured about 3 ns; the bound leaves room for draws
  }
  {
    // a cell 15 dB weaker, and a negative delay for the reference: the found delays are still right
    std::vector<Syn> mixed = {{1, 20.0e-6, 0}, {2, 100.0e-6, -15}, {3, -30.0e-6, -10}};
    auto win = make_window(mixed, n_rb, 20.0, 5, &sf_len);
    auto rel = estimate(win, mixed, n_rb, sf_len);
    for (size_t i = 0; i < rel.size(); i++)
      CHECK(std::fabs(rel[i] - (mixed[i + 1].delay_s - mixed[0].delay_s)) < 25e-9, "unequal powers: cell %zu error %.2f ns", i + 2, (rel[i] - (mixed[i + 1].delay_s - mixed[0].delay_s)) * 1e9);
  }
  {
    // a window without the cell: the quality figures tell
    std::vector<Syn> only1 = {{1, 0.0, 0}};
    auto win = make_window(only1, n_rb, 10.0, 3, &sf_len);
    Cfg cfg; cfg.enabled = true; cfg.id = 2; cfg.config_index = 2435; cfg.nof_prb = n_rb;
    CellParams cp; cp.nof_prb = n_rb; cp.normal_cp = false; cp.mbms_dedicated = true; cp.nof_ports = 1; cp.cell_id = 2;
    const double fs = 15000.0 * fft_size(n_rb); const int half = static_cast<int>(0.75e-3 * fs);
    Toa absent = correlate(win, subframe_waveform(cfg, cp, 0, 0, fft_size(n_rb)), static_cast<int>(sf_len) - half, static_cast<int>(sf_len) + half);
    cfg.id = 1; cp.cell_id = 1;
    Toa present = correlate(win, subframe_waveform(cfg, cp, 0, 0, fft_size(n_rb)), static_cast<int>(sf_len) - half, static_cast<int>(sf_len) + half);
    CHECK(present.coherence > 3.0 * absent.coherence, "coherence of the present cell %.3f against %.3f for an absent one", present.coherence, absent.coherence);
    CHECK(present.ratio > 2.0 * absent.ratio, "peak ratio %.1f against %.1f", present.ratio, absent.ratio);
  }
  {
    // lag range checks
    std::vector<Syn> one = {{1, 0.0, 0}};
    auto win = make_window(one, n_rb, 30.0, 2, &sf_len);
    Cfg cfg; cfg.enabled = true; cfg.id = 1; cfg.config_index = 2435; cfg.nof_prb = n_rb;
    CellParams cp; cp.nof_prb = n_rb; cp.normal_cp = false; cp.mbms_dedicated = true; cp.nof_ports = 1; cp.cell_id = 1;
    auto ref = subframe_waveform(cfg, cp, 0, 0, 512);
    CHECK(!correlate(win, ref, 5000, 4000).ok, "an empty search range must fail");
    CHECK(!correlate(win, std::vector<cplx>(), 0, 10).ok, "an empty reference must fail");
    Toa t = correlate(win, ref, -100, static_cast<int>(sf_len) + 1000);
    CHECK(t.ok && std::fabs(t.lag - sf_len) < 0.05, "the lag range is clamped to the window: lag %.3f", t.lag);
  }
}

// The PSS, SSS and PBCH-core exclusion of an MBMS-dedicated cell is in the frames with n_f mod 4 = 0 whatever the bandwidth (TS 36.211 clauses 6.11.1.2 and 6.6.4), not in
// the frames of the cell acquisition subframes (clause 6.6.4.1: n_f mod 8 = 4 below 25 PRB).
static void test_mask_frames() {
  for (uint32_t prb : {6u, 15u, 25u, 50u}) {
    CellParams c; c.nof_prb = prb; c.normal_cp = true; c.mbms_dedicated = true;
    const uint32_t width = 12 * prb, centre = 6 * prb;
    for (uint32_t n_f = 0; n_f < 16; n_f++) {
      const bool want = n_f % 4 == 0;
      auto m0 = sync_pbch_mask(c, n_f, 0);
      CHECK((m0[6 * width + centre] != 0) == want && (m0[5 * width + centre] != 0) == want, "%u PRB frame %u: PSS/SSS mask %d, wanted %d", prb, n_f, m0[6 * width + centre] != 0, want);
      auto m1 = sync_pbch_mask(c, n_f, 1);
      CHECK((m1[centre - 36] != 0 && m1[3 * width + centre + 35] != 0) == want, "%u PRB frame %u: PBCH core mask wanted %d", prb, n_f, want);
      auto m10 = sync_pbch_mask(c, n_f, 10);
      CHECK(m10[6 * width + centre] == 0, "%u PRB frame %u: no PSS in slot 10 of an MBMS-dedicated cell", prb, n_f);
    }
  }
}

// The muting sequence starts at the first occasion at or after the reference cell's SFN = 0, given in the cell's own subframe numbers (TS 37.355, prs-MutingInfo)
static void test_muting_origin() {
  Cfg cfg; cfg.enabled = true; cfg.config_index = 2475; cfg.nof_subframes = 1; cfg.muting = "10";   // T_PRS 80, offset 0
  CHECK(!active(cfg, 80, 0) && active(cfg, 0, 0) && active(cfg, 160, 0), "origin 0: occasion 0 at 0 sent, occasion 1 at 80 muted, occasion 2 at 160 sent");
  CHECK(active(cfg, 80, 40) && !active(cfg, 160, 40) && active(cfg, 240, 40), "origin 40: the first occasion is 80 (sent), then 160 muted, 240 sent");
  CHECK(active(cfg, 80, 80) && !active(cfg, 160, 80), "origin 80: an occasion at the origin is the first one");
  CHECK(active(cfg, 80, 41) && !active(cfg, 160, 41) && active(cfg, 240, 41), "origin 41: the first occasion is 80, bit 0 (sent), then 160 muted");
  CHECK(active(cfg, 0, 10200) && !active(cfg, 80, 10200), "origin 10200: the first occasion after it is subframe 0 of the next cycle (sent), then 80 muted");
  cfg.config_index = 2435; cfg.muting = "0110";   // T_PRS 40, offset 0
  const bool want[] = {false, true, true, false, false, true, true, false};
  for (uint32_t k = 0; k < 8; k++) CHECK(active(cfg, 40 * (k + 2), 60) == want[k], "T_PRS 40, origin 60: occasion %u sent %d, wanted %d", k, active(cfg, 40 * (k + 2), 60), want[k]);
}

int main(int argc, char** argv) {
  if (argc < 2) { printf("usage: prs_signal_test golden.txt\n"); return 2; }
  std::ifstream f(argv[1]);
  if (!f) { printf("cannot open %s\n", argv[1]); return 2; }
  std::string line; int cases = 0, waves = 0, actives = 0;
  while (std::getline(f, line)) {
    if (line.empty() || line[0] == '#') continue;
    std::istringstream hdr(line);
    std::string kind; hdr >> kind;
    if (kind == "case") { std::string name; hdr >> name; run_case(hdr, f, name); cases++; }
    else if (kind == "wave") { std::string name; hdr >> name; run_wave(hdr, f, name); waves++; }
    else if (kind == "active") { run_active(hdr); actives++; }
  }
  CHECK(cases >= 45, "only %d slot cases read", cases);
  CHECK(waves >= 4, "only %d waveform cases read", waves);
  CHECK(actives >= 12, "only %d activity cases read", actives);
  test_estimator();
  test_mask_frames();
  test_muting_origin();
  printf("prs_signal_test: %d slot cases, %d waveforms, %d activity cases, %d failures\n", cases, waves, actives, failures);
  return failures ? 1 : 0;
}
