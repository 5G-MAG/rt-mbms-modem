// Test of src/SyncSignal.h: the PSS and SSS grid and waveform of a cell acquisition subframe, and the time of arrival of overlapping cells taken from them, against the
// golden vectors written by rt-mbms-examples/positioning-assistance/tools/gen_sync_golden.py (TS 36.211 V19.3.0 clauses 6.11.1 and 6.11.2).
//
//   grid <name> <cell_id> <n_rb_dl> <normal_cp> <count>                     then <count> lines "k l re im"
//   wave <name> <cell_id> <n_rb_dl> <normal_cp> <n_samples> <energy> <count>  then <count> lines "index re im"
//   window <name> <n_rb_dl> <normal_cp> <n_cells> <n_samples>                then <n_cells> lines "cell_id delay_ns power_db", then <n_samples> lines "re im"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <random>
#include <sstream>
#include <string>

#include "../src/SyncSignal.h"

using namespace MBMS_RT;
using prs::cplx;

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
    default: return 2048;
  }
}

static void run_grid(std::istringstream& hdr, std::ifstream& f, const std::string& name) {
  unsigned cell, n_rb, normal, count;
  hdr >> cell >> n_rb >> normal >> count;
  const std::vector<cplx> g = sync::slot0_grid(cell, n_rb, normal != 0);
  size_t nonzero = 0;
  for (auto& v : g) nonzero += (v != cplx(0, 0));
  CHECK(nonzero == count, "%s: %zu non-zero elements, golden has %u", name.c_str(), nonzero, count);
  const uint32_t width = 12 * n_rb;
  for (unsigned i = 0; i < count; i++) {
    unsigned k, l; double re, im;
    f >> k >> l >> re >> im;
    const cplx v = g[l * width + k];
    CHECK(std::fabs(v.real() - re) < 1e-6 && std::fabs(v.imag() - im) < 1e-6, "%s: k=%u l=%u is %f%+fi, golden %f%+fi", name.c_str(), k, l, v.real(), v.imag(), re, im);
  }
}

static void run_wave(std::istringstream& hdr, std::ifstream& f, const std::string& name) {
  unsigned cell, n_rb, normal, n_samples, count; double energy;
  hdr >> cell >> n_rb >> normal >> n_samples >> energy >> count;
  const std::vector<cplx> w = sync::cas_waveform(cell, n_rb, normal != 0, fft_size(n_rb));
  CHECK(w.size() == n_samples, "%s: %zu samples, golden %u", name.c_str(), w.size(), n_samples);
  double e = 0;
  for (auto& v : w) e += std::norm(v);
  CHECK(std::fabs(e - energy) < 1e-4 * energy, "%s: energy %f, golden %f", name.c_str(), e, energy);
  for (unsigned i = 0; i < count; i++) {
    unsigned idx; double re, im;
    f >> idx >> re >> im;
    CHECK(idx < w.size() && std::fabs(w[idx].real() - re) < 2e-5 && std::fabs(w[idx].imag() - im) < 2e-5, "%s: sample %u is %f%+fi, golden %f%+fi", name.c_str(), idx, w[idx].real(), w[idx].imag(), re, im);
  }
}

// a window written by the Python reference, no noise: the delays come back
static void run_window(std::istringstream& hdr, std::ifstream& f, const std::string& name) {
  unsigned n_rb, normal, n_cells, n_samples;
  hdr >> n_rb >> normal >> n_cells >> n_samples;
  const uint32_t n_fft = fft_size(n_rb), sf_len = 15 * n_fft;
  CHECK(n_samples == 3 * sf_len, "%s: %u samples for a subframe of %u", name.c_str(), n_samples, sf_len);
  std::vector<unsigned> ids(n_cells);
  std::vector<double> delay_ns(n_cells), power_db(n_cells);
  for (unsigned i = 0; i < n_cells; i++) f >> ids[i] >> delay_ns[i] >> power_db[i];
  std::vector<cplx> window(n_samples);
  for (auto& v : window) { double re, im; f >> re >> im; v = cplx(static_cast<float>(re), static_cast<float>(im)); }
  std::vector<std::vector<cplx>> refs;
  for (unsigned id : ids) refs.push_back(sync::cas_waveform(id, n_rb, normal != 0, n_fft));
  const double fs = 15000.0 * n_fft;
  const int half = static_cast<int>(0.75e-3 * fs);
  const auto toa = sync::measure_cells(window, refs, static_cast<int>(sf_len) - half, static_cast<int>(sf_len) + half);
  for (unsigned i = 0; i < n_cells; i++) {
    const double got_ns = (toa[i].lag - sf_len) / fs * 1e9;
    CHECK(toa[i].ok && std::fabs(got_ns - delay_ns[i]) < 2.0, "%s: cell %u measured %.2f ns, written %.2f ns", name.c_str(), ids[i], got_ns, delay_ns[i]);
  }
}

// cells overlapping at different delays and powers in one noisy window built here (the generator of the window is this code, so this tests the estimator, not the signal)
static void test_noisy_overlap() {
  const uint32_t n_rb = 25, n_fft = 512, sf_len = 15 * n_fft;
  const double fs = 15000.0 * n_fft;
  struct C { uint32_t id; double delay_ns, power_db; };
  const std::vector<C> cells = {{1, 0.0, 0.0}, {2, 1500.0, -6.0}, {3, -2500.0, -10.0}, {304, 3200.0, -12.0}};
  const double snr_db = 25.0;
  std::mt19937 rng(7);
  std::normal_distribution<double> nd(0.0, std::sqrt(std::pow(10.0, -snr_db / 10.0) / 2.0));
  std::vector<std::vector<cplx>> refs;
  for (const auto& c : cells) refs.push_back(sync::cas_waveform(c.id, n_rb, true, n_fft));
  const size_t n_win = 3 * sf_len;
  size_t m = 1;
  while (m < n_win + refs[0].size()) m <<= 1;
  std::vector<cplx> window(n_win, cplx(0, 0));
  for (size_t i = 0; i < cells.size(); i++) {
    const std::vector<cplx> d = sync::delayed(refs[i], sf_len + cells[i].delay_ns * 1e-9 * fs, n_win, m);
    const float a = static_cast<float>(std::pow(10.0, cells[i].power_db / 20.0));
    for (size_t k = 0; k < n_win; k++) window[k] += a * d[k];
  }
  for (auto& v : window) v += cplx(static_cast<float>(nd(rng)), static_cast<float>(nd(rng)));
  const int half = static_cast<int>(0.75e-3 * fs);
  const auto toa = sync::measure_cells(window, refs, static_cast<int>(sf_len) - half, static_cast<int>(sf_len) + half);
  for (size_t i = 0; i < cells.size(); i++) {
    const double got = (toa[i].lag - sf_len) / fs * 1e9;
    CHECK(toa[i].ok && std::fabs(got - cells[i].delay_ns) < 25.0, "four cells at %.0f dB: cell %u measured %.1f ns, built at %.1f ns", snr_db, cells[i].id, got, cells[i].delay_ns);
  }
  // a single cell is measured as well
  const auto one = sync::measure_cells(window, {refs[0]}, static_cast<int>(sf_len) - half, static_cast<int>(sf_len) + half);
  CHECK(one.size() == 1 && one[0].ok, "a single cell is measured");
  // and nothing is invented for an empty window or no cells
  CHECK(sync::measure_cells({}, refs, 0, 10).size() == refs.size() && !sync::measure_cells({}, refs, 0, 10)[0].ok, "an empty window gives no result");
  CHECK(sync::measure_cells(window, {}, 0, 10).empty(), "no cells give no results");
}

int main(int argc, char** argv) {
  if (argc < 2) { printf("usage: sync_signal_test golden.txt\n"); return 2; }
  std::ifstream f(argv[1]);
  if (!f) { printf("cannot open %s\n", argv[1]); return 2; }
  std::string line;
  int grids = 0, waves = 0, windows = 0;
  while (std::getline(f, line)) {
    if (line.empty() || line[0] == '#') continue;
    std::istringstream hdr(line);
    std::string kind, name;
    hdr >> kind >> name;
    if (kind == "grid") { run_grid(hdr, f, name); grids++; }
    else if (kind == "wave") { run_wave(hdr, f, name); waves++; }
    else if (kind == "window") { run_window(hdr, f, name); windows++; }
  }
  CHECK(grids >= 10, "only %d grids read", grids);
  CHECK(waves >= 6, "only %d waveforms read", waves);
  CHECK(windows >= 2, "only %d windows read", windows);
  test_noisy_overlap();
  printf("sync_signal_test: %d grids, %d waveforms, %d windows, %d failures\n", grids, waves, windows, failures);
  return failures ? 1 : 0;
}
