// 5G-MAG Reference Tools
// MBMS Modem Process
//
// Copyright (C) 2021 Klaus Kühnhammer (Österreichische Rundfunksender GmbH & Co KG)
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

/**
 * @file main.cpp
 * @brief Contains the program entry point, command line parameter handling, and the main runloop for data processing.
 */

/** \mainpage 5G-MAG Reference Tools - MBMS Modem
 *
 * This is the documentation for the FeMBMS receiver. Please see main.cpp for for the runloop and main processing logic as a starting point.
 *
 */

#include <algorithm>
#include <argp.h>

#include <chrono>
#include <cstdlib>
#include <libconfig.h++>

#include "CasFrameProcessor.h"
#include "Gw.h"
#include "SdrReader.h"
#include "MbsfnFrameProcessor.h"
#include "MeasurementFileWriter.h"
#include "Phy.h"
#include "RestHandler.h"
#include "Rrc.h"
#include "Version.h"
#include "spdlog/async.h"
#include "spdlog/spdlog.h"
#include "spdlog/sinks/syslog_sink.h"
#include "srsran/srsran.h"
#include "srsran/upper/pdcp.h"
#include "srsran/rlc/rlc.h"
#include "thread_pool.hpp"

using libconfig::Config;
using libconfig::FileIOException;
using libconfig::ParseException;
using libconfig::Setting;

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

static void print_version(FILE *stream, struct argp_state *state);
void (*argp_program_version_hook)(FILE *, struct argp_state *) = print_version;
const char *argp_program_bug_address = "5G-MAG Reference Tools <reference-tools@5g-mag.com>";
static char doc[] = "5G-MAG-RT MBMS Modem Process";  // NOLINT

static struct argp_option options[] = {  // NOLINT
    {"config", 'c', "FILE", 0, "Configuration file (default: /etc/5gmag-rt.conf)", 0},
    {"log-level", 'l', "LEVEL", 0,
     "Log verbosity: 0 = trace, 1 = debug, 2 = info, 3 = warn, 4 = error, 5 = "
     "critical, 6 = none. Default: 2.",
     0},
    {"srsran-log-level", 's', "LEVEL", 0,
     "Log verbosity for srsran: 0 = debug, 1 = info, 2 = warn, 3 = error, 4 = "
     "none, Default: 4.",
     0},
    {"sample-file", 'f', "FILE", 0,
     "Sample file in 4 byte float interleaved format to read I/Q data from. If "
     "present, the data from this file will be decoded instead of live SDR "
     "data. The channel bandwith must be specified with the --file-bandwidth "
     "flag, and the sample rate of the file must be suitable for this "
     "bandwidth.",
     0},
    {"write-sample-file", 'w', "FILE", 0,
     "Create a sample file in 4 byte float interleaved format containing the "
     "raw received I/Q data.",
     0},
    {"file-bandwidth", 'b', "BANDWIDTH (MHz)", 0,
     "Required if decoding data from a file, to specify the channel bandwidth of the "
     "recorded data in MHz here (e.g. 5). Optional in live-SDR mode, where bandwidth is "
     "otherwise detected automatically via blind cell search; pass this to force the "
     "cell-search width instead (e.g. to match a fixed-rate simulated RF bridge).",
     0},
    {"override_nof_prb", 'p', "# PRB", 0,
     "Override the number of PRB received in the MIB, regardless of what was actually "
     "broadcast. Does not affect cell-search width (see --file-bandwidth/-b for that).",
     0},
    {"sdr_devices", 'd', nullptr, 0,
     "Prints a list of all available SDR devices", 0},
    {"repeat", 'r', nullptr, 0,
     "Replay the sample file endlessly (default: false)", 0},

    {nullptr, 0, nullptr, 0, nullptr, 0}};

/**
 * Holds all options passed on the command line
 */
struct arguments {
  const char *config_file = {};  /**< file path of the config file. */
  unsigned log_level = 2;        /**< log level */
  unsigned srs_log_level = 4;    /**< srsLTE log level */
  int8_t override_nof_prb = -1;  /**< ovride PRB number */
  const char *sample_file = {};  /**< file path of the sample file. */
  uint8_t file_bw = 0;           /**< bandwidth of the sample file */
  const char
      *write_sample_file = {};   /**< file path of the created sample file. */
  bool list_sdr_devices = false;
  bool repeat_sample_file = false;
};

/**
 * Parses the command line options into the arguments struct.
 */
static auto parse_opt(int key, char *arg, struct argp_state *state) -> error_t {
  auto arguments = static_cast<struct arguments *>(state->input);
  switch (key) {
    case 'c':
      arguments->config_file = arg;
      break;
    case 'l':
      arguments->log_level = static_cast<unsigned>(strtoul(arg, nullptr, 10));
      break;
    case 's':
      arguments->srs_log_level =
          static_cast<unsigned>(strtoul(arg, nullptr, 10));
      break;
    case 'f':
      arguments->sample_file = arg;
      break;
    case 'w':
      arguments->write_sample_file = arg;
      break;
    case 'b':
      arguments->file_bw = static_cast<uint8_t>(strtoul(arg, nullptr, 10));
      break;
    case 'p':
      arguments->override_nof_prb =
          static_cast<int8_t>(strtol(arg, nullptr, 10));
      break;
    case 'd':
      arguments->list_sdr_devices = true;
      break;
    case 'r':
      arguments->repeat_sample_file = true;
      break;
    case ARGP_KEY_ARG:
      argp_usage(state);
      break;
    default:
      return ARGP_ERR_UNKNOWN;
  }
  return 0;
}

static struct argp argp = {options, parse_opt, nullptr, doc,
                           nullptr, nullptr,   nullptr};

/**
 * Print the program version in MAJOR.MINOR.PATCH format.
 */
void print_version(FILE *stream, struct argp_state * /*state*/) {
  fprintf(stream, "%s.%s.%s\n", std::to_string(VERSION_MAJOR).c_str(),
          std::to_string(VERSION_MINOR).c_str(),
          std::to_string(VERSION_PATCH).c_str());
}

static Config cfg;  /**< Global configuration object. */

static unsigned sample_rate = 1920000;  /**< Sample rate of the SDR (6 PRB narrowband search default) */
static unsigned search_sample_rate = 1920000;  /**< Sample rate of the SDR during blind cell search */
static unsigned frequency = 667000000;  /**< Center freqeuncy the SDR is tuned to */
static uint32_t bandwidth = 10000000;   /**< Low pass filter bandwidth for the SDR */
static double gain = 0.9;               /**< Overall system gain for the SDR */
static std::string antenna = "LNAW";    /**< Antenna input to be used */
static bool use_agc = false;

static unsigned mbsfn_nof_prb = 0;
static unsigned cas_nof_prb = 0;

/**
 * Restart flag. Setting this to true triggers resynchronization using the params set in the following parameters:
 * @see sample_rate
 * @see frequency
 * @see bandwith
 * @see gain
 * @see antenna
 */
static bool restart = false;

/**
 * Set new SDR parameters and initialize resynchronisation. This function is used by the RESTful API handler
 * to modify the SDR params.
 *
 * @param ant  Name of the antenna input (For LimeSDR Mini: LNAW, LNAL)
 * @param fc   Center frequency to tune to (in Hz)
 * @param gain Total system gain to set [0..1]
 * @param sr   Sample rate (in Hz)
 * @param bw   Low pass filter bandwidth (in Hz)
 */
void set_params(const std::string& ant, unsigned fc, double g, unsigned sr, unsigned bw) {
  sample_rate = sr;
  frequency = fc;
  bandwidth = bw;
  antenna = ant;
  gain = g;
  spdlog::info("RESTful API requesting new parameters: fc {}, bw {}, rate {}, gain {}, antenna {}",
      frequency, bandwidth, sample_rate, gain, antenna);

  restart = true;
}

/**
 * Load the TV Service Configuration MO (ETSI TS 103 720 clause 5.10, MO
 * urn:oma:mo:ext-3gpp-tv-config:1.0, defined in ETSI TS 124 117) from
 * modem.conf's optional [modem.tv_config] section. This is the static,
 * config-file equivalent of RestHandler's PUT /tv_config - lets a deployment
 * provision PLMN/EARFCN/TMGI info before the REST API is even reachable,
 * most importantly so the initial search frequency (below) can come from a
 * single, spec-defined source instead of the flat, easily-drifting
 * modem.sdr.center_frequency_hz value compared against whatever a live
 * MBMS-ROM-Info redirect later advertises.
 */
std::vector<Phy::TvConfigPlmn> load_tv_config_from_cfg(const Config& cfg) {
  std::vector<Phy::TvConfigPlmn> plmns;
  if (!cfg.exists("modem.tv_config")) {
    return plmns;
  }
  auto parse_tmgi_list = [](const Setting& s) {
    std::vector<Phy::TvConfigTmgi> out;
    for (int k = 0; k < s.getLength(); k++) {
      Phy::TvConfigTmgi tmgi;
      s[k].lookupValue("tmgi", tmgi.tmgi);
      s[k].lookupValue("usd", tmgi.usd);
      int tsi_val = 0;
      s[k].lookupValue("tsi", tsi_val);
      tmgi.tsi = static_cast<uint32_t>(tsi_val);
      out.push_back(tmgi);
    }
    return out;
  };
  const Setting& tv_config = cfg.lookup("modem.tv_config");
  for (int i = 0; i < tv_config.getLength(); i++) {
    const Setting& p = tv_config[i];
    Phy::TvConfigPlmn plmn;
    p.lookupValue("plmn_id", plmn.plmn_id);
    if (p.exists("ran_info")) {
      const Setting& ran_info = p["ran_info"];
      for (int j = 0; j < ran_info.getLength(); j++) {
        plmn.earfcns.push_back((uint32_t)(int)ran_info[j]);
      }
    }
    if (p.exists("tmgi_list_for_sa")) {
      plmn.tmgis_for_sa = parse_tmgi_list(p["tmgi_list_for_sa"]);
    }
    if (p.exists("tmgi_list_for_service")) {
      plmn.tmgis_for_service = parse_tmgi_list(p["tmgi_list_for_service"]);
    }
    plmns.push_back(plmn);
  }
  return plmns;
}

/**
 *  Main entry point for the program.
 *  
 * @param argc  Command line agument count
 * @param argv  Command line arguments
 * @return 0 on clean exit, -1 on failure
 */
auto main(int argc, char **argv) -> int {
  struct arguments arguments;
  /* Default values */
  arguments.config_file = "/etc/5gmag-rt.conf";
  arguments.sample_file = nullptr;
  arguments.write_sample_file = nullptr;
  argp_parse(&argp, argc, argv, 0, nullptr, &arguments);

  // Read and parse the configuration file
  try {
    cfg.readFile(arguments.config_file);
  } catch(const FileIOException &fioex) {
    spdlog::error("I/O error while reading config file at {}. Exiting.", arguments.config_file);
    exit(1);
  } catch(const ParseException &pex) {
    spdlog::error("Config parse error at {}:{} - {}. Exiting.",
        pex.getFile(), pex.getLine(), pex.getError());
    exit(1);
  }

  // Set up logging
  std::string ident = "modem";
  auto syslog_logger = spdlog::syslog_logger_mt("syslog", ident, LOG_PID | LOG_PERROR | LOG_CONS );

  spdlog::set_level(
      static_cast<spdlog::level::level_enum>(arguments.log_level));
  spdlog::set_pattern("[%H:%M:%S.%f %z] [%^%l%$] [thr %t] %v");

  spdlog::set_default_logger(syslog_logger);
  spdlog::info("5g-mag-rt modem v{}.{}.{} starting up", VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH);

  // Init and tune the SDR
  auto rx_channels = 1;
  cfg.lookupValue("modem.sdr.rx_channels", rx_channels);
  spdlog::info("Initialising SDR with {} RX channel(s)", rx_channels);
  SdrReader sdr(cfg, rx_channels);
  if (arguments.list_sdr_devices) {
    sdr.enumerateDevices();
    exit(0);
  }

  std::string sdr_dev = "driver=lime";
  cfg.lookupValue("modem.sdr.device_args", sdr_dev);
  if (!sdr.init(sdr_dev, arguments.sample_file, arguments.write_sample_file, arguments.repeat_sample_file)) {
    spdlog::error("Failed to initialize I/Q data source.");
    exit(1);
  }

  cfg.lookupValue("modem.sdr.search_sample_rate_hz", sample_rate);
  search_sample_rate = sample_rate;

  if (arguments.sample_file != nullptr && arguments.file_bw) {
    // Sample files are captured at a fixed rate determined by the channel bandwidth given via
    // --file-bandwidth (there's no "reduced-bandwidth blind search" possible on a file, unlike
    // with a live SDR where modem.sdr.search_sample_rate_hz picks a deliberately narrow search
    // rate). Phy::cell_search() below sizes its FFT/frame lengths from cs_nof_prb = file_bw * 5,
    // so SdrReader must be tuned to that same native rate from the start; otherwise its ring
    // buffer pacing/watermark math (based on _sampleRate) runs against a smaller rate than the
    // sample counts srsran's cell-search actually requests for that PRB count, starving
    // MultichannelRingbuffer reads once the mismatch is large enough (e.g. at 100 PRB with a
    // 25 PRB search_sample_rate_hz).
    sample_rate = search_sample_rate = (unsigned)srsran_sampling_freq_hz(arguments.file_bw * 5);
  }

  unsigned long long center_frequency = frequency;
  if (!cfg.lookupValue("modem.sdr.center_frequency_hz", center_frequency)) {
    spdlog::error("Unable to parse center_frequency_hz - values must have a ‘L’ character appended");
    exit(1);
  }
  // We needed unsigned long long for correct parsing,
  // but unsigned is required
  if (center_frequency <= UINT_MAX) {
     frequency = static_cast<unsigned>(center_frequency);
  } else {
    spdlog::error("Configured center_frequency_hz is {}, maximal value supported is {}.",
        center_frequency, UINT_MAX);
    exit(1);
  }


  /* If a TV Service Configuration MO (ETSI TS 103 720 clause 5.10) has been
   * provisioned, its first PLMN's first RANInfo/EARFCN is the spec-intended
   * source for which frequency to search on - takes priority over the flat
   * modem.sdr.center_frequency_hz above, which has no way to stay consistent
   * with what the network actually broadcasts/advertises via ROM-Info. */
  std::vector<Phy::TvConfigPlmn> initial_tv_config = load_tv_config_from_cfg(cfg);
  for (const auto& plmn : initial_tv_config) {
    if (!plmn.earfcns.empty()) {
      double freq_mhz = srsran_band_fd(plmn.earfcns[0]);
      if (freq_mhz > 0.0) {
        frequency = static_cast<unsigned>(freq_mhz * 1e6);
        spdlog::info("TV Service Configuration MO: using EARFCN={} ({:.3f} MHz) from PLMN '{}' as "
                     "initial search frequency, overriding modem.sdr.center_frequency_hz",
                     plmn.earfcns[0], freq_mhz, plmn.plmn_id);
      } else {
        spdlog::warn("TV Service Configuration MO: EARFCN={} from PLMN '{}' does not resolve to a "
                     "frequency, ignoring", plmn.earfcns[0], plmn.plmn_id);
      }
      break;
    }
  }

  cfg.lookupValue("modem.sdr.normalized_gain", gain);
  cfg.lookupValue("modem.sdr.antenna", antenna);
  cfg.lookupValue("modem.sdr.use_agc", use_agc);

  if (!sdr.tune(frequency, sample_rate, bandwidth, gain, antenna, use_agc)) {
    spdlog::error("Failed to set initial center frequency. Exiting.");
    exit(1);
  }

  set_srsran_verbose_level(arguments.log_level <= 1 ? SRSRAN_VERBOSE_DEBUG : SRSRAN_VERBOSE_NONE);
  srsran_use_standard_symbol_size(true);

  // TEST-ONLY (2026-07-19): seeds the MBSFN retune logic below (~line 612) with a
  // target PRB width directly, bypassing the normal "learn it by decoding SIB13"
  // path. Exists because this ZMQ-simulation test harness has no rate negotiation
  // between the eNB's TX and this modem's RX: once the eNB genuinely widens its own
  // TX rate for a wider pmch_bandwidth, this modem's CAS/PDCCH decode - and by
  // extension its ability to ever decode the real SIB13 value - breaks immediately
  // (confirmed via a clean A/B test, see SIB13_MBSFN_TEST_RESULTS.md), a real
  // chicken-and-egg deadlock this override exists purely to sidestep for testing.
  // Default 0 = no override, existing dynamic-discovery behavior unchanged. A real
  // UE has no equivalent shortcut; this has no bearing on real deployment.
  unsigned mbsfn_prb_test_override = 0;
  cfg.lookupValue("modem.phy.mbsfn_prb_test_override", mbsfn_prb_test_override);

  // Create a thread pool for the frame processors
  unsigned thread_cnt = 4;
  cfg.lookupValue("modem.phy.threads", thread_cnt);
  int phy_prio = 10;
  cfg.lookupValue("modem.phy.thread_priority_rt", phy_prio);
  thread_pool pool{ thread_cnt + 1, phy_prio };

  bool enable_measurement_file = false;
  cfg.lookupValue("modem.measurement_file.enabled", enable_measurement_file);
  MeasurementFileWriter measurement_file(cfg);

  // Create the layer components: Phy, RLC, RRC and GW
  //
  // cs_nof_prb sizes the initial cell-search/MIB-decode engine. Blind by default: when no
  // --file-bandwidth/-b is given, it's fixed at the library's own narrowband cell-search/MIB-decode
  // default (SRSRAN_CS_NOF_PRB == SRSRAN_UE_MIB_NOF_PRB == 6), since PSS/SSS/PBCH always live in the
  // central 6 PRB regardless of the true system bandwidth; MIB's own bw_idx field then reveals the
  // real bandwidth (see the searching-state handling below, which already reconfigures everything
  // off phy.nr_prb() rather than off this guess). File-source mode still requires -b (raw capture
  // files carry no embedded rate metadata). -b remains available in live-SDR mode too, as an
  // explicit override for setups that need a specific, fixed cell-search width -- e.g. a simulated
  // RF bridge whose sample-rate decimation ratio must stay integral and known in advance.
  constexpr uint8_t kLiveCellSearchNofPrb = 6;
  uint8_t cs_nof_prb = arguments.file_bw ? arguments.file_bw * 5 : kLiveCellSearchNofPrb;
  Phy phy(
      cfg,
      std::bind(&SdrReader::get_samples, &sdr, _1, _2, _3),  // NOLINT
      cs_nof_prb,
      arguments.override_nof_prb,
      rx_channels);

  phy.init();
  phy.set_tv_config(std::move(initial_tv_config));

  srsran::pdcp pdcp(nullptr, "PDCP");
  srsran::rlc rlc("RLC");
  srsran::timer_handler timers;

  Rrc rrc(cfg, phy, rlc);
  Gw gw(cfg, phy);
  gw.init();

  rlc.init(&pdcp, &rrc, &timers, 0 /* RB_ID_SRB0 */);
  pdcp.init(&rlc, &rrc,  &gw);

  auto srs_level = srslog::basic_levels::none;
  switch (arguments.srs_log_level) {
    case 0: srs_level = srslog::basic_levels::debug; break;
    case 1: srs_level = srslog::basic_levels::info; break;
    case 2: srs_level = srslog::basic_levels::warning; break;
    case 3: srs_level = srslog::basic_levels::error; break;
    case 4: srs_level = srslog::basic_levels::none; break;
  }

  // Configure srsLTE logging
 auto& mac_log = srslog::fetch_basic_logger("MAC", false);
  mac_log.set_level(srs_level);
 auto& phy_log = srslog::fetch_basic_logger("PHY", false);
  phy_log.set_level(srs_level);
 auto& rlc_log = srslog::fetch_basic_logger("RLC", false);
  rlc_log.set_level(srs_level);
 auto& asn1_log = srslog::fetch_basic_logger("ASN1", false);
  asn1_log.set_level(srs_level);


  state_t state = searching;

  // Create the RESTful API handler
  std::string uri = "http://0.0.0.0:3010/modem-api/";
  cfg.lookupValue("modem.restful_api.uri", uri);
  spdlog::info("Starting RESTful API handler at {}", uri);
  RestHandler rest_handler(cfg, uri, state, sdr, phy, set_params);

  // Initialize one CAS and thered_cnt MBSFN frame processors
  CasFrameProcessor cas_processor(cfg, phy, rlc, rest_handler, rx_channels);
  if (!cas_processor.init()) {
    spdlog::error("Failed to create CAS processor. Exiting.");
    exit(1);
  }
  
  // We need the cas processor to be accesible within the rest_handler object to gather all the values display in the rt-mbms-application
  rest_handler.set_cas_processor(&cas_processor);
  
  std::vector<MbsfnFrameProcessor*> mbsfn_processors;
  for (int i = 0; i < thread_cnt; i++) {
    auto p = new MbsfnFrameProcessor(cfg, rlc, phy, mac_log, rest_handler, rx_channels);
    if (!p->init()) {
      spdlog::error("Failed to create MBSFN processor. Exiting.");
      exit(1);
    }
    mbsfn_processors.push_back(p);
  }

  /* set_cell() (main thread) reallocates rx/FFT buffers inside the processor
   * (srsran_ue_dl_set_cell() -> ofdm.c's srsran_ofdm_rx_set_prb_symbol_sz(),
   * plus the CIR FFTW plan) that an already-dispatched, fire-and-forget
   * process() call (running on a pool worker thread) may still be reading/
   * writing - a genuine, confirmed double-free/heap-corruption race
   * (2026-07-18: "double free or corruption" crashes on both the
   * cas_processor and mbsfn retune paths). A mutex was tried first and
   * caused a guaranteed deadlock instead (get_rx_buffer_and_lock() locks the
   * same non-recursive mutex synchronously on the main thread before
   * pool.push() even runs, so a second lock attempt in set_cell() on that
   * same thread self-deadlocks). Waiting on the std::future returned by
   * pool.push() has no such risk: it just blocks the main thread until that
   * specific worker task's function body has returned, which is exactly the
   * point at which it's done touching the processor's buffers. */
  std::future<void> cas_future;
  std::vector<std::future<void>> mbsfn_futures(thread_cnt);

  rest_handler.start(); // Start the listener, we need to do it after storing the cas into the rest_handler, otherwise we will get segfault.
  // Start receiving sample data
  sdr.start();

  // Variables to store the measure BLER of PDSCH and MCH/MCCH. 
  uint32_t mch_bler_global = 0;
  uint32_t mcch_bler_global = 0;
  uint32_t pdsch_bler_global = 0;
  uint32_t mch_total_global = 0;
  uint32_t mcch_total_global = 0;
  uint32_t pdsch_total_global = 0;
  
  // Variables to store the times the receiver losses the signal and the lost subframes between resynchronizations.
  uint32_t sync_losses = 0;
  uint32_t lost_subframes = 0;
  uint32_t measurements = 0.0f;

  uint32_t tti = 0;

  float measurement_interval_f = 5;
  cfg.lookupValue("modem.measurement_file.interval_secs", measurement_interval_f);
  uint32_t measurement_interval = measurement_interval_f * 1000;
  uint32_t tick = 0;

  // Initial state: searching a cell
  state = searching;

  uint8_t mb_idx = 0;
  /* Last (N, M) this dispatch loop saw on the active MCH's own PMCH entry (not PMCH0's -- see
   * the mb_idx-advance block's own comment on why mcch_for_ti.pmch_info_list[0] there needs the
   * same per-mch_idx lookup as here), so a LIVE change (embms.pmch1.time_interleaving_n/m via the
   * control socket) can force mb_idx back to a known-good realignment point immediately, rather
   * than waiting for mch_subframe_idx to naturally satisfy the NEW block_len's own boundary
   * condition -- which, mid-block, can take up to a full extra block's worth of subframes,
   * during which mb_idx stays frozen on whatever instance the OLD cadence last pinned, handing
   * it subframes that no longer correspond to a clean block start under the new N*M. Sentinel
   * 0xFF so the very first tti forces a (harmless, mb_idx already 0) realignment too. */
  uint8_t main_last_ti_n = 0xFF, main_last_ti_m = 0xFF;

  // Elevate execution to real time scheduling. Moved here (2026-07-26), as the LAST setup
  // step before the main loop -- not before phy.init(), and not right after it either (both
  // tried and still too early). ALL one-time setup work that can legitimately take a while
  // happens above this point: phy.init() (FFTW planning, cell-search/MIB-sync engine
  // allocation), plus CasFrameProcessor::init() and 4x MbsfnFrameProcessor::init() (each
  // doing their own FFTW planning and buffer allocation). Elevating to SCHED_FIFO before ANY
  // of that finishes risks the kernel's RLIMIT_RTTIME safety limit (confirmed 200ms on this
  // host via `ulimit -a`): any SCHED_FIFO thread that runs that long without a blocking
  // syscall gets SIGKILLed unconditionally, with no log output and no core dump. Confirmed
  // live, 2026-07-26: moving this past phy.init() alone fixed the first crash point but
  // exposed an identical crash later, right after CasFrameProcessor/MbsfnFrameProcessor
  // setup -- moving it past ALL one-time setup, not just phy.init(), is the actual fix.
  // Real-time priority is only needed for the ongoing per-subframe loop below.
  struct sched_param thread_param = {};
  thread_param.sched_priority = 20;
  cfg.lookupValue("modem.phy.main_thread_priority_rt", thread_param.sched_priority);

  if (thread_param.sched_priority > 0) {
  spdlog::info("Raising main thread to realtime scheduling priority {}", thread_param.sched_priority);

  // SCHED_FIFO, not SCHED_RR -- see SdrReader.cpp's reader-thread priority elevation for why.
  int error = pthread_setschedparam(pthread_self(), SCHED_FIFO, &thread_param);
  if (error != 0) {
    spdlog::error("Cannot set main thread priority to realtime: {}. Thread will run at default priority.", strerror(error));
  }
  } else {
    spdlog::info("main_thread_priority_rt=0, skipping realtime scheduling for main thread");
  }

  // Start the main processing loop
  for (;;) { // Only one main loop, any time therè's a change of state we force next iteration with continue. This way there's no need of nested loops within the cases.
    switch (state) {
      case processing: {  // processing
        tti = (tti + 1) % 10240; // Clamp the TTI

        if (phy.rom_redirect_pending()) {
          auto [rom_earfcn, rom_prb] = phy.consume_rom_redirect();
          if (rom_earfcn != 0) {
            double freq_mhz = srsran_band_fd(rom_earfcn);
            // freq_mhz > 0.0 does not catch EARFCNs that land in inter-band gap ranges;
            // those resolve against a dummy sentinel with a nonzero frequency. Acceptable
            // because ROM targets in practice are always valid broadcast EARFCNs.
            if (freq_mhz > 0.0) {
              /* Cross-check against the provisioned TV Service Configuration MO
               * (ETSI TS 103 720 clause 5.10), if any - a live RRC-signalled ROM
               * redirect pointing somewhere the MO's RANInfo never provisioned is
               * legitimate (RRC signaling is dynamic, the MO is a coarser
               * provisioning hint) but worth surfacing, since in practice it
               * usually means the two are out of sync rather than a deliberate
               * cross-carrier move. Purely informational - the redirect is still
               * honoured either way. */
              auto provisioned = phy.tv_config_earfcns();
              if (!provisioned.empty() &&
                  std::find(provisioned.begin(), provisioned.end(), rom_earfcn) == provisioned.end()) {
                spdlog::warn("ROM redirect: EARFCN={} is not in the provisioned TV Service "
                             "Configuration MO's RANInfo list - honouring it anyway, but this "
                             "usually indicates the MO and the live SIB13 ROM-Info have drifted "
                             "out of sync", rom_earfcn);
              }
              unsigned rom_freq_hz = static_cast<unsigned>(freq_mhz * 1e6);
              if (rom_freq_hz != frequency) {
                spdlog::info("ROM redirect: retuning SDR to EARFCN={} ({:.3f} MHz, {} PRB)",
                             rom_earfcn, freq_mhz, rom_prb);
                frequency = rom_freq_hz;
                restart = true;
              } else {
                spdlog::debug("ROM redirect: EARFCN={} matches current frequency, no retune needed", rom_earfcn);
              }
            } else {
              spdlog::warn("ROM redirect: cannot resolve EARFCN={} to a frequency, ignoring", rom_earfcn);
            }
          }
        }

        if (phy.is_cas_subframe(tti)) {
          // Get the samples from the SDR interface, hand them to a CAS processor, and start it
          // on a thread from the pool.
          // TEMPORARY DIAGNOSTIC (CAS_TIMING_DIAG=1, 2026-07-22): times how long this specific
          // call blocks acquiring cas_processor's lock -- tests whether the main thread stalls
          // here waiting for the previous occasion's still-in-flight (queued or executing)
          // process() call to release it, which get_next_frame()'s own polling-wait for fresh
          // samples (already ruled out as decimation-related) would misattribute to sample
          // unavailability rather than lock contention.
          bool cas_timing_diag = getenv("CAS_TIMING_DIAG") != nullptr;
          std::chrono::steady_clock::time_point lock_wait_start;
          if (cas_timing_diag) lock_wait_start = std::chrono::steady_clock::now();
          cf_t** cas_rx_buffer = cas_processor.get_rx_buffer_and_lock();
          if (cas_timing_diag) {
            auto us = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - lock_wait_start).count();
            if (us > 1000) {
              fprintf(stderr, "CAS_TIMING_DIAG tti=%u lock_wait_us=%lld\n", tti, (long long)us);
            }
          }
          if (!restart && phy.get_next_frame(cas_rx_buffer, cas_processor.rx_buffer_size())) {
            spdlog::debug("sending tti {} to regular processor", tti);
            cas_future = pool.push([ObjectPtr = &cas_processor, tti, &rest_handler] {
                if (ObjectPtr->process(tti)) {
                // Set constellation diagram data and rx params for CAS in the REST API handler
                rest_handler.add_cinr_value(ObjectPtr->cinr_db());
                }
            });


            // Re-tune whenever the EFFECTIVE MBSFN width (max(actual PMCH width, carrier))
            // has changed from what the SDR is currently tuned for - this covers both
            // directions: widening when the PMCH grows past the carrier (Rel-19
            // extended-bandwidth case), and narrowing back down when it later shrinks
            // to fit inside the carrier again (e.g. a live pmch_bandwidth SET back to 0
            // after a wideband test). A PMCH that fits inside the carrier (Rel-17
            // pmch_bandwidth sub-allocation, e.g. 40 PRB inside a 50-PRB / 10 MHz
            // carrier) must NOT shrink the SDR below the carrier's own width: keep the
            // CAS sampling at the carrier and decode the PMCH as a sub-band of that same
            // grid, since the eNB encodes with symbol stride = carrier nof_prb.
            /* Guard against re-triggering on every CAS occasion: mbsfn_nof_prb tracks
             * "which effective width am I already retuned for", so this only re-enters
             * when that width actually changes (2026-07-18). Confirmed live,
             * 2026-07-18: without the narrow-back branch below, reverting
             * pmch_bandwidth to 0 after a wideband test left the SDR/CAS FFT
             * permanently stuck at the wider grid (wrong RE-per-symbol count, visibly
             * broken CAS composition) until a full process restart - the widen-only
             * condition never re-enters once already retuned wide, even though the
             * eNB is signalling a narrower width again. */
            unsigned target_mbsfn_prb = std::max<unsigned>({phy.nof_mbsfn_prb(), cas_nof_prb, mbsfn_prb_test_override});
            if (target_mbsfn_prb != mbsfn_nof_prb)
            {
              mbsfn_nof_prb = target_mbsfn_prb;

              unsigned new_srate;
              if (mbsfn_nof_prb > cas_nof_prb) {
                // Wider than the carrier: adjust the SDR's sample rate to fit the
                // wider MBSFN bandwidth, using its own (possibly reduced) SCS.
                srsran_scs_t mbsfn_scs = SRSRAN_SCS_15KHZ;
                switch (phy.mbsfn_subcarrier_spacing()) {
                  case Phy::SubcarrierSpacing::df_7kHz5:     mbsfn_scs = SRSRAN_SCS_7KHZ5;      break;
                  case Phy::SubcarrierSpacing::df_2kHz5:     mbsfn_scs = SRSRAN_SCS_2KHZ5;      break;
                  case Phy::SubcarrierSpacing::df_1kHz25:    mbsfn_scs = SRSRAN_SCS_1KHZ25;     break;
                  case Phy::SubcarrierSpacing::df_370Hz:     mbsfn_scs = SRSRAN_SCS_370HZ;      break;
                  case Phy::SubcarrierSpacing::df_370Hz_sl4: mbsfn_scs = SRSRAN_SCS_370HZ_SL4;  break;
                  case Phy::SubcarrierSpacing::df_370Hz_sl2: mbsfn_scs = SRSRAN_SCS_370HZ_SL2;  break;
                  default: break;
                }
                new_srate = (unsigned)srsran_sampling_freq_hz_scs(mbsfn_nof_prb, mbsfn_scs);
              } else {
                // Back down to (at most) the carrier's own width - same rate cell
                // search originally tuned to, standard 15kHz-numerology CAS.
                new_srate = srsran_sampling_freq_hz(mbsfn_nof_prb);
              }
              // Only touch the SDR hardware (stop/tune/start) and force a full
              // resync when the actual sample rate needs to change. The gating
              // condition above (target_mbsfn_prb != mbsfn_nof_prb) is about the
              // logical PRB width, not the rate - a widened mbsfn_prb can easily
              // map to the SAME rate the initial cell search already tuned to
              // (e.g. both bucket to 15.36 MHz), in which case doing a full
              // stop/tune/start anyway was destroying an already-good sample-level
              // timing/phase reference for no hardware-level reason, forcing an
              // unnecessary resync. Confirmed live 2026-07-19: this was corrupting
              // CAS/PDCCH decode (~28x worse sync_error, 0% PDCCH success) even
              // though every FFT/width-sizing candidate checked out correct - see
              // SIB13_MBSFN_TEST_RESULTS.md.
              bool rate_change_needed = (new_srate != sample_rate);
              if (rate_change_needed) {
                spdlog::info("Setting sample rate {} Mhz for MBSFN with {} PRB / {} Mhz channel width", new_srate/1000000.0, mbsfn_nof_prb,
                    mbsfn_nof_prb * 0.2);
                sdr.stop();
                bandwidth = (mbsfn_nof_prb * 200000) * 1.2;
                sdr.tune(frequency, new_srate, bandwidth, gain, antenna, use_agc);
              } else {
                spdlog::info("MBSFN width changed to {} PRB but sample rate ({} Mhz) is unchanged - reconfiguring PHY/CAS only, no SDR retune", mbsfn_nof_prb, new_srate/1000000.0);
              }

              // ... configure the PHY and CAS processor to decode a narrow CAS and wider MBSFN, and move back to syncing state
              // after reconfiguring and restarting the SDR.
              // Wait for the CAS process() just dispatched above (line ~568) to actually
              // finish before resizing its buffers here - see the cas_future/mbsfn_futures
              // doc comment near their declaration for why this, not a mutex.
              if (cas_future.valid()) {
                cas_future.wait();
              }
              phy.set_cell();
              cas_processor.set_cell(phy.cell(), phy.mbsfn_scs());

              if (rate_change_needed) {
                sdr.start();
                sample_rate = new_srate;
                spdlog::info("Synchronizing subframe after PRB extension");
                state = syncing;
              }
            }
          } else {
            // Failed to receive data, or sync lost. Go back to searching state.
            spdlog::warn("Synchronization lost while processing. Going back to searching state.");
            sync_losses++;
            state = syncing;
            // get_rx_buffer_and_lock() above locked cas_processor for this tti, but
            // process() was never dispatched to release it - unlock it here instead of
            // deferring to the blanket unlock that used to live in case syncing, which
            // couldn't tell a genuinely dangling lock from one a worker thread already
            // released (confirmed via ThreadSanitizer, 2026-07-22).
            cas_processor.unlock();
          }
        } else {
          // All other frames in FeMBMS dedicated mode are MBSFN frames.
          spdlog::debug("sending tti {} to mbsfn proc {}", tti, mb_idx);

          /* TS 36.213 §11.1 (Rel-19 time interleaving): peek at whether time
           * interleaving is active for this tti and, if so, whether this is
           * the LAST subframe of its N*M-subframe block - see the comment at
           * the mb_idx advance below for why. A block spans N*M subframes,
           * not N: within it, subframe s belongs to slot m=s%M with
           * redundancy version n=(s%(N*M))/M (see srsran_pmch_decode's
           * comment in pmch.c for the full derivation) - M independent,
           * pipelined transport blocks are in flight across the whole
           * block, not just one, so the worker instance (and its per-slot
           * softbuffer array in MbsfnFrameProcessor) must stay fixed for
           * the full N*M span, not just N. Calls the same public Phy method
           * MbsfnFrameProcessor::process() calls internally; its one side
           * effect (scheduling an MCCH re-read at a modification-period
           * boundary) is idempotent, gated on a flag that's already true
           * after the first of the two calls, so calling it here too is
           * safe. Only trust time_interleaving_n/m/mch_subframe_idx when
           * enable && !is_mcch, matching how MbsfnFrameProcessor.cpp itself
           * gates on these fields - other struct fields are not guaranteed
           * initialized otherwise. */
          unsigned           peek_area = 0;
          srsran_mbsfn_cfg_t peek_cfg  = phy.mbsfn_config_for_tti(tti, peek_area);
          bool ti_last_of_block        = true; /* default: advance every TTI, matching the old behavior */
          /* mbsfn_config_for_tti() only populates time_interleaving_n/m/mch_subframe_idx
           * when sf_idx falls inside a PMCH's own data allocation - gap subframes within
           * an otherwise TI-active MCH's schedule (CAS, additionalNonMBSFNSubframes) leave
           * peek_cfg.time_interleaving_n at its unconditional default (1), so the block-
           * boundary check below can't see them. Left unhandled, such a gap subframe fell
           * through to the "advance every TTI" default - a spurious rotation that
           * permanently offset which absolute mch_subframe_idx block boundary this
           * worker's pinning aligns to for the rest of that scheduling period (confirmed
           * empirically via PMCH_TI_DIAG: the very first 1-2 real TI subframes after each
           * MCCH occasion landed on an already-abandoned instance). Query the MCCH content
           * directly (stable across the whole scheduling period, unlike the per-tti peek)
           * to tell "TI genuinely isn't configured" (old behavior: advance every TTI) apart
           * from "TI is configured but this specific subframe isn't a real block position"
           * (must not advance - see the fuller rationale in "Round 19"/finding #3 of the
           * project roadmap). */
          const srsran::mcch_msg_t& mcch_for_ti = phy.current_mcch();
          /* pmch_info_list[peek_cfg.pmch_idx], NOT pmch_info_list[0] -- this gap-detection check
           * exists specifically for a TI-active MCH other than PMCH0 (PMCH0 always carries MCCH,
           * and TS 36.300 SS15.3.3 forbids TI there once a second PMCH exists -- rrc.cc's
           * reconfigure_embms() enforces that on the TX side, so PMCH0's own time_interleaving_n
           * is always <=1 in any config where this matters at all). Hardcoding index 0 here meant
           * this check could only ever fire for PMCH0's own (always-disabled) TI, silently never
           * protecting PMCH1+'s real gap subframes -- confirmed live 2026-08-05: exactly the
           * "spurious rotation" this whole mechanism exists to prevent, just unprotected for every
           * PMCH but the one that structurally can never need it. */
          uint8_t active_pmch_idx = (peek_cfg.pmch_idx < mcch_for_ti.nof_pmch_info) ? peek_cfg.pmch_idx : 0;
          /* Live TI reconfiguration (embms.pmch1.time_interleaving_n/m via the control socket,
           * confirmed possible while running -- rrc::reconfigure_embms()): force an immediate
           * mb_idx realignment rather than letting the OLD-cadence pinning free-run until
           * mch_subframe_idx happens to satisfy the NEW block_len's own boundary, which mid-block
           * can take up to a full extra block's worth of subframes of misrouted work first (see
           * this variable's own declaration for the fuller rationale). Read from the stable
           * broadcast MCCH content, not the per-tti peek_cfg (only valid on real, non-gap
           * subframes), so a change is caught on the very next tti regardless of whether it lands
           * on a gap or a real data subframe. */
          if (mcch_for_ti.nof_pmch_info > 0) {
            const auto& active_pmch_info = mcch_for_ti.pmch_info_list[active_pmch_idx];
            if (active_pmch_info.time_interleaving_n != main_last_ti_n ||
                active_pmch_info.time_interleaving_m != main_last_ti_m) {
              mb_idx          = 0;
              main_last_ti_n = active_pmch_info.time_interleaving_n;
              main_last_ti_m = active_pmch_info.time_interleaving_m;
            }
          }
          bool ti_configured_for_active_mch =
              !peek_cfg.is_mcch && mcch_for_ti.nof_pmch_info > 0 &&
              mcch_for_ti.pmch_info_list[active_pmch_idx].time_interleaving_n > 1;
          if (ti_configured_for_active_mch && !(peek_cfg.enable && peek_cfg.time_interleaving_n > 1)) {
            ti_last_of_block = false;
          } else if (peek_cfg.enable && !peek_cfg.is_mcch && peek_cfg.time_interleaving_n > 1) {
            /* Clamp defensively, consistent with MbsfnFrameProcessor.cpp's
             * own clamp on the same broadcast-derived field: an out-of-range
             * M here would only mis-time the worker advance, not corrupt
             * memory, but the two must agree on what M means. */
            uint8_t ti_m = peek_cfg.time_interleaving_m;
            if (ti_m == 0) {
              ti_m = 1;
            } else if (ti_m > SRSRAN_PMCH_MAX_TI_M) {
              ti_m = SRSRAN_PMCH_MAX_TI_M;
            }
            uint32_t block_len  = (uint32_t)peek_cfg.time_interleaving_n * (uint32_t)ti_m;
            ti_last_of_block = (peek_cfg.mch_subframe_idx % block_len) == (block_len - 1);
          }

          /* TEMPORARY (RACE_DIAG2, 2026-07-17): is_mbsfn_subframe(tti) below is an
           * INDEPENDENT classification from mbsfn_config_for_tti(tti,...) (already
           * computed above as peek_cfg) - both must agree for a muted-CAS sf=0 to be
           * both dispatched and correctly configured. Log any live disagreement to
           * test whether this is the source of the CAS-muting sf=0 equalization
           * anomaly (see SIB13_MBSFN_TEST_RESULTS.md's Finding 3 for context) - the
           * anomaly is now masked by MbsfnFrameProcessor's validity gate, but the
           * root cause is still open. */
          if (getenv("RACE_DIAG2") && (tti % 10) == 0) {
            bool is_mbsfn_now = phy.is_mbsfn_subframe(tti);
            if (is_mbsfn_now != peek_cfg.enable) {
              fprintf(stderr, "RACE_DIAG2_MISMATCH tti=%u is_mbsfn_subframe=%d peek_cfg.enable=%d peek_cfg.is_mcch=%d mb_idx=%u\n",
                      tti, (int)is_mbsfn_now, (int)peek_cfg.enable, (int)peek_cfg.is_mcch, mb_idx);
            }
          }
          // Get the samples from the SDR interface, hand them to an MNSFN processor, and start it
          // on a thread from the pool. Getting the buffer pointer from the pool also locks this processor.
          if (!restart && phy.get_next_frame(mbsfn_processors[mb_idx]->get_rx_buffer_and_lock(), mbsfn_processors[mb_idx]->rx_buffer_size())) {
            if (getenv("MCCH_SCHED_DIAG")) {
              static uint64_t dispatch_call_count = 0;
              dispatch_call_count++;
              bool mc = phy.mcch_configured();
              bool imf = phy.is_mbsfn_subframe(tti);
              if (dispatch_call_count % 2000 == 1 || (mc && imf)) {
                fprintf(stderr, "MCCH_SCHED_DIAG4 tti=%u mcch_configured=%d is_mbsfn_subframe=%d call=%llu\n",
                        tti, (int)mc, (int)imf, (unsigned long long)dispatch_call_count);
              }
            }
            if (phy.mcch_configured() && phy.is_mbsfn_subframe(tti)) {
              // If data frm SIB1/SIB13 has been received in CAS, configure the processors accordingly
              auto cell = phy.cell();
              // MBSFN processor grid width = the WIDER of carrier and PMCH. For a
              // pmch_bandwidth sub-allocation (PMCH < carrier) this keeps the grid at
              // the carrier, so pmch.c's symbol stride (= cell.nof_prb) matches the
              // eNB's while mbsfn_prb stays the narrower PMCH RE-allocation width; for
              // the extended-BW case (PMCH > carrier) it grows the grid to the PMCH.
              cell.nof_prb = (cell.nof_prb > cell.mbsfn_prb) ? cell.nof_prb : cell.mbsfn_prb;
              /* Reconfigure not just on first-ever use but whenever the effective
               * width has changed since this instance's last configuration -
               * pmch_bandwidth (hence mbsfn_prb, hence this computed width) can
               * legitimately change value mid-session (e.g. a live SET). Each of
               * the thread_cnt worker instances used to latch "configured" once
               * and never again, silently keeping a stale FFT/buffer width and
               * corrupting every subsequent decode with NaN post-equalization
               * power once the actual width diverged (confirmed live 2026-07-17,
               * see MbsfnFrameProcessor::configured_nof_prb()'s doc comment). */
              if (!mbsfn_processors[mb_idx]->mbsfn_configured() ||
                  mbsfn_processors[mb_idx]->configured_nof_prb() != cell.nof_prb) {
                srsran_scs_t scs = SRSRAN_SCS_15KHZ;
                switch (phy.mbsfn_subcarrier_spacing()) {
                  case Phy::SubcarrierSpacing::df_15kHz:  scs = SRSRAN_SCS_15KHZ;  break;
                  case Phy::SubcarrierSpacing::df_7kHz5:  scs = SRSRAN_SCS_7KHZ5;  break;
                  case Phy::SubcarrierSpacing::df_2kHz5:     scs = SRSRAN_SCS_2KHZ5;      break;
                  case Phy::SubcarrierSpacing::df_1kHz25:    scs = SRSRAN_SCS_1KHZ25;     break;
                  case Phy::SubcarrierSpacing::df_370Hz:     scs = SRSRAN_SCS_370HZ;      break;
                  case Phy::SubcarrierSpacing::df_370Hz_sl4: scs = SRSRAN_SCS_370HZ_SL4;  break;
                  case Phy::SubcarrierSpacing::df_370Hz_sl2: scs = SRSRAN_SCS_370HZ_SL2;  break;
                }
                if (getenv("RACE_DIAG2")) {
                  fprintf(stderr, "RECONFIG tti=%u mb_idx=%u nof_prb=%u mbsfn_prb=%u was_configured=%d\n",
                          tti, mb_idx, cell.nof_prb, cell.mbsfn_prb,
                          (int)mbsfn_processors[mb_idx]->mbsfn_configured());
                }
                // Wait for this instance's last dispatched process() (if any) to
                // finish before resizing its buffers - same race/fix as cas_future,
                // see the doc comment near mbsfn_futures' declaration.
                if (mbsfn_futures[mb_idx].valid()) {
                  mbsfn_futures[mb_idx].wait();
                }
                mbsfn_processors[mb_idx]->set_cell(cell, scs);
                mbsfn_processors[mb_idx]->configure_mbsfn(phy.mbsfn_area_id(), scs);
              }
              if (getenv("RACE_DIAG2")) {
                fprintf(stderr, "DISPATCH tti=%u mb_idx=%u is_mcch=%d nof_prb=%u mbsfn_prb=%u\n",
                        tti, mb_idx, (int)peek_cfg.is_mcch, cell.nof_prb, cell.mbsfn_prb);
              }
              mbsfn_futures[mb_idx] = pool.push([ObjectPtr = mbsfn_processors[mb_idx], tti] {
                ObjectPtr->process(tti);
              });
            } else {
              // Nothing to do yet, we lack the data from SIB1/SIB13
              // Discard the samples and unlock the processor.
              rest_handler.record_subframe_event(tti, RestHandler::SF_EVENT_GAP, RestHandler::SF_STATUS_IDLE);
              mbsfn_processors[mb_idx]->unlock();
            }
          } else {
            // Failed to receive data, or sync lost. Go back to searching state.
            spdlog::warn("Synchronization lost while processing. Going back to searching state.");
            sync_losses++;
            state = syncing;
            // get_rx_buffer_and_lock() above locked mbsfn_processors[mb_idx] for this
            // tti, but process() was never dispatched to release it - unlock it here,
            // same reasoning as the cas_processor case above.
            mbsfn_processors[mb_idx]->unlock();
          }
          /* Only advance the round-robin worker index when this tti was the
           * last subframe of its N*M-subframe time-interleaving block (or
           * time interleaving isn't active, in which case every subframe is
           * independent and this is unconditional exactly as before).
           * Keeping mb_idx unchanged for the continuation subframes of a
           * block is what makes all N*M of them land on the SAME
           * MbsfnFrameProcessor instance - required for that instance's
           * per-slot softbuffer array / srsran_pmch_t per-slot state to
           * accumulate across the block at all: the block interleaves M
           * independently-pipelined transport blocks (see pmch.c), so the
           * instance must stay fixed for the WHOLE block, not just one
           * slot's own N subframes (see the roadmap's finding #3a for the
           * original bug this fixes, and its later generalization from N to
           * N*M once the M-slot pipelining model was corrected). */
          if (getenv("PMCH_TI_DIAG")) {
            fprintf(stderr,
                    "TI_DIAG_MAIN tti=%u mb_idx_before=%u enable=%d is_mcch=%d ti_n=%u ti_m=%u "
                    "mch_subframe_idx=%u ti_last_of_block=%d ti_configured=%d mcch_nof_pmch=%u mcch_ti_n=%u\n",
                    tti, mb_idx, (int)peek_cfg.enable, (int)peek_cfg.is_mcch, peek_cfg.time_interleaving_n,
                    peek_cfg.time_interleaving_m, peek_cfg.mch_subframe_idx, (int)ti_last_of_block,
                    (int)ti_configured_for_active_mch, mcch_for_ti.nof_pmch_info,
                    mcch_for_ti.nof_pmch_info > 0 ? mcch_for_ti.pmch_info_list[0].time_interleaving_n : 0);
          }
          if (ti_last_of_block) {
            mb_idx = static_cast<int>((mb_idx + 1) % thread_cnt);
          }
        }
      }
      break;
      
      case searching: {
        /* TEMPORARY (2026-07-26): drop to normal scheduling for the ENTIRE searching state,
         * not just the post-cell-search reconfiguration block. Confirmed live: dropping
         * priority only around the reconfiguration in main.cpp (below) was NOT enough --
         * phy.cell_search() itself calls srsran_ue_sync_set_cell()/srsran_ue_mib_set_cell()
         * internally, right after a successful MIB decode and before returning here, which
         * is exactly the same class of one-time FFTW-replanning work already fixed twice
         * today elsewhere, and it was still running at already-elevated SCHED_FIFO. Simplest
         * correct fix: real-time priority isn't actually needed until a cell is found and
         * locked (case processing), not while searching or reconfiguring at all. Idempotent
         * to call every time this case is entered (already-SCHED_OTHER stays SCHED_OTHER).*/
        struct sched_param normal_param = {};
        normal_param.sched_priority = 0;
        pthread_setschedparam(pthread_self(), SCHED_OTHER, &normal_param);

        if (restart) { // Triggered from the rt-mbms-application
          sdr.stop();
          sample_rate = search_sample_rate;  // sample rate for searching
          sdr.tune(frequency, sample_rate, bandwidth, gain, antenna, use_agc);
          sdr.start();
        }

        // We're at the search sample rate, and there's no point in creating a sample file. rtop the sample writer, if enabled.
        //sdr.disableSampleFileWriting();

        // In searching state, clear the receive buffer and try to find a cell at the configured frequency and synchronize with it
        restart = false;
       // sdr.clear_buffer();
        bool cell_found = phy.cell_search();
        if (cell_found) {
          // A cell has been found. We now know the required number of PRB = bandwidth of the carrier. Set the approproiate
          // sample rate...
          cas_nof_prb = phy.nr_prb();
          // TEST-ONLY: if mbsfn_prb_test_override is set, go straight to the wide
          // rate here instead of narrowing to cas_nof_prb first and letting the
          // in-flight retune logic (below, ~line 612) widen it a second time -
          // that two-step narrow-then-rewiden dance was observed to leave the
          // modem stuck re-syncing indefinitely (dispatch loop never resumes,
          // CAS/PDCCH total count frozen) after a live retune, whereas tuning
          // directly to the target width from a cold cell-search does not hit
          // this problem. See SIB13_MBSFN_TEST_RESULTS.md.
          mbsfn_nof_prb = std::max<unsigned>(cas_nof_prb, mbsfn_prb_test_override);

          if (arguments.sample_file && arguments.file_bw) {
            // Samples files are recorded at a fixed sample rate that can be determined from the bandwidth command line argument.
            // If we're decoding from file, do not readjust the rate to match the CAS PRBs, but stay at this rate and instead configure the
            // PHY to decode a narrow CAS from a wider channel: CAS/cell-access signalling always stays at a
            // traditional LTE bandwidth (nof_prb, from 1.4 up to 20 MHz), while PMCH (mbsfn_nof_prb) can
            // legitimately run wider, non-standard bandwidths (e.g. 6/7/8 MHz-equivalent PRB counts) - this
            // is the normal 5G Terrestrial Broadcast / FeMBMS case, not a mismatch to reconcile away. See
            // srsran_cell_isvalid()'s doc comment (lib/srsran/lib/src/phy/common/phy_common.c) for the
            // buffer-capacity reasoning on why mbsfn_prb no longer has to equal nof_prb.
            mbsfn_nof_prb = arguments.file_bw * 5;
            phy.set_nof_mbsfn_prb(mbsfn_nof_prb);
            phy.set_cell();
          } else {
            // When decoding from the air, configure the SDR accordingly. RF
            // backends that can't actually retune (the ZMQ test bridge,
            // soapy-zmq-bridge/ZmqRxDevice) now decimate down to whatever
            // rate is requested here when it's narrower than the wire's
            // native rate (see that bridge's `native_srate` device arg /
            // Decimator), so it's safe to genuinely retune down again.
            //
            // mbsfn_nof_prb, not cas_nof_prb: normally equal at this point, but
            // TEST-ONLY mbsfn_prb_test_override (see above) can make it wider -
            // mirrors the wide/narrow branching in the in-flight retune logic
            // below (~line 618) so this initial tune lands directly on the
            // target rate instead of narrowing first and widening again later.
            unsigned new_srate;
            if (mbsfn_nof_prb > cas_nof_prb) {
              new_srate = (unsigned)srsran_sampling_freq_hz_scs(mbsfn_nof_prb, phy.mbsfn_scs());
            } else {
              new_srate = srsran_sampling_freq_hz(cas_nof_prb);
            }
            // Skip the stop/tune/start cycle entirely when cell search already
            // landed on the right rate (e.g. modem.sdr.search_sample_rate_hz was
            // configured to match the wide target directly). A retune at an
            // unchanged rate still tears down and rebuilds the SDR's internal
            // timing/phase state, which was found to corrupt the channel
            // estimate used by CAS/PDCCH for the remainder of the session even
            // though PBCH/MIB (more error-tolerant) kept decoding fine. See
            // SIB13_MBSFN_TEST_RESULTS.md.
            if (new_srate != sample_rate) {
              spdlog::info("Setting sample rate {} Mhz for {} PRB / {} Mhz channel width", new_srate/1000000.0, mbsfn_nof_prb,
                  mbsfn_nof_prb * 0.2);
              sdr.stop();
              sdr.clear_buffer();
              bandwidth = (mbsfn_nof_prb * 200000) * 1.2;
              sdr.tune(frequency, new_srate, bandwidth, gain, antenna, use_agc);
              //sleep(1);

              sdr.start();
              sample_rate = new_srate;
            } else {
              spdlog::info("Cell search already at the target sample rate ({} Mhz) for {} PRB - reconfiguring PHY/CAS only, no SDR retune",
                  new_srate/1000000.0, mbsfn_nof_prb);
            }
            // Mirrors the sample_file branch above and the in-flight retune logic
            // below: the SDR's raw rate alone doesn't configure the PHY/CAS FFT
            // grid width. Since this initial tune now lands directly on the wide
            // target (see mbsfn_nof_prb computation above), the in-flight retune
            // check further below will see no change and never fire, so this call
            // has to happen here instead, or the CAS/PBCH FFT stays sized for the
            // narrow carrier while the SDR itself is already sampling wide.
            phy.set_nof_mbsfn_prb(mbsfn_nof_prb);
            phy.set_cell();
            cas_processor.set_cell(phy.cell(), phy.mbsfn_scs());
          }
          // Restore real-time priority now that searching (cell_search() itself, plus the
          // retune/reconfiguration above) is done and we're about to move to syncing/
          // processing -- see the drop at the top of this case for why it was dropped.
          if (thread_param.sched_priority > 0) {
            if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &thread_param) != 0) {
              spdlog::error("Cannot restore main thread to realtime priority after cell search.");
            }
          }
          spdlog::debug("Synchronizing subframe");
          // ... and move to syncing state.
          state = syncing;
        } else {
          //sleep(1);
        }
      } 
      break;
      case syncing: {
        // In syncing state, we already know the cell we want to camp on, and the SDR is tuned to the required
        // sample rate for its number of PRB / bandwidth. We now synchronize PSS/SSS and receive the MIB once again
        // at this sample rate.
        if (restart) { // The only way of going to search is restarting
          state = searching; // Jump to searching where the restart is actually handled.
          continue; // We need to skip the next step and go to the next iteration.
        }
        
        bool sfn_sync = false;
        sfn_sync = phy.synchronize_subframe();
        
        if (sfn_sync) {
          // We're locked on to the cell, and have succesfully received the MIB at the target sample rate.
          lost_subframes += (((phy.tti() < tti) * 10240 +  phy.tti())-tti) * cas_processor.is_started() ; // cas_processor has started when it has a valid cell set. 
          spdlog::info("Decoded MIB at target sample rate, TTI is {}. Subframe synchronized, sync lost in TTI {}, {} subframes lost, {} total subframe lost, sync losses {}.", phy.tti(), tti, (((phy.tti() < tti) * 10240 +  phy.tti())-tti) * cas_processor.is_started(), lost_subframes, sync_losses);

          // Set the cell parameters in the CAS processor, and set started to true.
          // A sync-loss-triggered re-sync (not just first-ever startup) can reach
          // here with a previous process() dispatch still in flight - wait for it
          // first, same reasoning as the other set_cell() call sites above.
          if (cas_future.valid()) {
            cas_future.wait();
          }
          cas_processor.set_cell(phy.cell(), phy.mbsfn_scs());

          // No blanket unlock needed here: cas_processor and mbsfn_processors[mb_idx] each
          // unlock themselves immediately at their own sync-loss detection site (in case
          // processing above) whenever get_rx_buffer_and_lock() wasn't followed by a
          // dispatched process() call. Unconditionally unlocking every processor here was
          // undefined behaviour - unlock of an already-unlocked mutex, or worse, of one
          // still held by an in-flight worker thread - confirmed via ThreadSanitizer,
          // 2026-07-22 (5 "unlock of an unlocked mutex" reports, all this call site).

          // Get the initial TTI / subframe ID (= system frame number * 10 + subframe number)
          tti = phy.tti();
          // Reset the RRC
          rrc.reset();

          // Ready to receive actual data. Go to processing state.
          state = processing;

          // If sample file creation is enabled, start writing out samples now that we're at the target sample rate
          sdr.enableSampleFileWriting();
        }
      }
      break;
      default:
        //we are not supposed to be here.
      break;
    }
    
    tick++;
    if (tick%measurement_interval == 0) {
      // It's time to output rx info to the measurement file and to syslog.
      // Collect the relevant info and write it out.
      std::vector<std::string> cols;
      cols.reserve(11);

      if (state == processing) {
        spdlog::info("CINR {:.2f} dB", rest_handler.cinr_db() );
        cols.push_back(std::to_string((float)rest_handler.cinr_db()));

        // Wait to finish and lock, we don't want to update total and errors independently. Yes, it's a blocking solution, but, what other way is possible?
        cas_processor.lock();

        spdlog::info("PDSCH: MCS {}, BLER {}",
            rest_handler._pdsch.mcs,
            ((rest_handler._pdsch.errors > 0 && rest_handler._pdsch.total > 0) ? (rest_handler._pdsch.errors * 1.0) / (rest_handler._pdsch.total * 1.0) : 0));
        
        cols.push_back(std::to_string(rest_handler._pdsch.mcs));
        cols.push_back(std::to_string(((rest_handler._pdsch.errors * 1.0) / (rest_handler._pdsch.total * 1.0))));

        pdsch_bler_global += rest_handler._pdsch.errors;
        pdsch_total_global += rest_handler._pdsch.total;
        rest_handler._pdsch.errors = 0;
        rest_handler._pdsch.total = 0;
        // We are done with the CAS
        cas_processor.unlock();
        
        // Wait for all the mbsfn processors to finish, to avoid having an update on total or errors while accesing to the values that leads to having a wrong BLER.
        for (int i = 0; i < thread_cnt; i++) {
            mbsfn_processors[i]->lock();
        }
        spdlog::info("MCCH: MCS {}, BLER {}",
            rest_handler._mcch.mcs,
            ((rest_handler._mcch.errors > 0 && rest_handler._mcch.total > 0) ? (rest_handler._mcch.errors * 1.0) / (rest_handler._mcch.total * 1.0) : 0));

        cols.push_back(std::to_string(rest_handler._mcch.mcs));
        cols.push_back(std::to_string(((rest_handler._mcch.errors * 1.0) / (rest_handler._mcch.total * 1.0))));

        cols.push_back(std::to_string(tti)); // Current TTI
        cols.emplace_back(std::string("")); // Blank to maintain the column, only used when desync.
        cols.push_back(std::to_string(lost_subframes)); // Total amount of lost subframes

        auto mch_info = phy.mch_info();
        int mch_idx = 0;
        std::for_each(std::begin(mch_info), std::end(mch_info), [&cols, &mch_idx, &rest_handler, &mch_bler_global, &mch_total_global](Phy::mch_info_t const& mch) {

            spdlog::info("MCH {}: MCS {}, BLER {}",
                mch_idx,
                mch.mcs,
                ((rest_handler._mch[mch_idx].errors > 0 && rest_handler._mch[mch_idx].total > 0) ? (rest_handler._mch[mch_idx].errors * 1.0) / (rest_handler._mch[mch_idx].total * 1.0) : 0));

            cols.push_back(std::to_string(mch_idx));
            cols.push_back(std::to_string(mch.mcs));
            cols.push_back(std::to_string((rest_handler._mch[mch_idx].errors * 1.0) / (rest_handler._mch[mch_idx].total * 1.0)));

            int mtch_idx = 0;
            std::for_each(std::begin(mch.mtchs), std::end(mch.mtchs), [&mtch_idx, &mch_bler_global, &mch_total_global](Phy::mtch_info_t const& mtch) {
              spdlog::info("    MTCH {}: LCID {}, TMGI 0x{}, {}",
                mtch_idx,
                mtch.lcid,
                mtch.tmgi,
                mtch.dest);
              mtch_idx++;
                });

              mch_bler_global  += rest_handler._mch[mch_idx].errors;
              mch_total_global += rest_handler._mch[mch_idx].total;
              rest_handler._mch[mch_idx].errors = 0;
              rest_handler._mch[mch_idx].total = 0;
              mch_idx++;
            });

        mcch_bler_global += rest_handler._mcch.errors;
        mcch_total_global += rest_handler._mcch.total;
        rest_handler._mcch.errors = 0;
        rest_handler._mcch.total = 0;
        // We can unlock cas and mbsfn processor at this point, every variable has been saved in the rest_handler.
        for (int i = 0; i < thread_cnt; i++) {
          mbsfn_processors[i]->unlock();
        }
      } else if (state == syncing) { // In syncing and searching states we place in every row and column NaN, this way is easier to process after, since every time measured theres always a row in the csv.
        cols.emplace_back(std::string("NOT SYNC - SYNCING...")); 
        cols.emplace_back(std::string("nan")); 
        cols.emplace_back(std::string("nan")); 
        cols.emplace_back(std::string("nan")); 
        cols.emplace_back(std::string("nan")); 
        cols.emplace_back(std::string("nan")); 
        cols.push_back(std::to_string(sync_losses)); 
        cols.push_back(std::to_string(lost_subframes));
        cols.emplace_back(std::string("nan")); 
        cols.emplace_back(std::string("nan")); 
        cols.emplace_back(std::string("nan")); 

      } else {
        cols.emplace_back(std::string("SEARCHING FOR A CELL...")); 
        cols.emplace_back(std::string("nan")); 
        cols.emplace_back(std::string("nan")); 
        cols.emplace_back(std::string("nan")); 
        cols.emplace_back(std::string("nan")); 
        cols.emplace_back(std::string("nan")); 
        cols.emplace_back(std::string(""));
        cols.push_back(std::to_string(lost_subframes));
        cols.emplace_back(std::string("nan")); 
        cols.emplace_back(std::string("nan")); 
        cols.emplace_back(std::string("nan")); 
      }
    
      if (enable_measurement_file) {
        measurement_file.WriteLogValues(cols);
      }
      measurements++;
      
      spdlog::info("------ Global statistics ------ \n\t\tMCH BLER {}, \n\t\tMCH TOTAL ERRORS {}, \n\t\tMCCH BLER: {}, \n\t\tPDSCH BLER: {}, \n\t\tSYNC LOSSES: {}, \n\t\tSF PROCESSED: {}", 
          ((mch_bler_global > 0 && mch_total_global > 0) ? (mch_bler_global * 1.0) / (mch_total_global * 1.0) : 0),
          mch_bler_global,
          ((mcch_bler_global > 0 && mcch_total_global > 0) ? (mcch_bler_global * 1.0) / (mcch_total_global * 1.0) : 0),
          ((pdsch_bler_global > 0 && pdsch_total_global > 0) ? (pdsch_bler_global * 1.0) / (pdsch_total_global * 1.0) : 0),
          sync_losses, 
          tick);
      
      spdlog::info("Total subframe lost {}, sync losses {}.", lost_subframes, sync_losses);

    }
  }

  // Main loop ended by signal. Free the MBSFN processors, and bail.
  for (int i = 0; i < thread_cnt; i++) {
    delete( mbsfn_processors[i] );
  }
exit:
  return 0;
}
