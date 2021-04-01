// OBECA - Open Broadcast Edge Cache Appliance
// Receive Process
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

/** \mainpage OBECA - Open Broadcast Edge Cache Applicance, receive process
 *
 * This is the documentation for the FeMBMS receiver. Please see main.cpp for for the runloop and main processing logic as a starting point.
 *
 */

#include <argp.h>

#include <cstdlib>
#include <libconfig.h++>

#include "CasFrameProcessor.h"
#include "Gw.h"
#include "LimeSdrReader.h"
#include "MbsfnFrameProcessor.h"
#include "MeasurementFileWriter.h"
#include "Phy.h"
#include "RestHandler.h"
#include "Rrc.h"
#include "Version.h"
#include "spdlog/async.h"
#include "spdlog/spdlog.h"
#include "spdlog/sinks/syslog_sink.h"
#include "srslte/srslte.h"
#include "srslte/upper/pdcp.h"
#include "srslte/upper/rlc.h"
#include "thread_pool.hpp"

using libconfig::Config;
using libconfig::FileIOException;
using libconfig::ParseException;

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

static void print_version(FILE *stream, struct argp_state *state);
void (*argp_program_version_hook)(FILE *, struct argp_state *) = print_version;
const char *argp_program_bug_address = "Austrian Broadcasting Services <obeca@ors.at>";
static char doc[] = "OBECA Receive Process";  // NOLINT

static struct argp_option options[] = {  // NOLINT
    {"config", 'c', "FILE", 0, "Configuration file (default: /etc/rp.conf)", 0},
    {"log-level", 'l', "LEVEL", 0,
     "Log verbosity: 0 = trace, 1 = debug, 2 = info, 3 = warn, 4 = error, 5 = "
     "critical, 6 = none. Default: 2.",
     0},
    {"srslte-log-level", 's', "LEVEL", 0,
     "Log verbosity for srslte: 0 = debug, 1 = info, 2 = warn, 3 = error, 4 = "
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
     "If decoding data from a file, specify the channel bandwidth of the "
     "recorded data in MHz here (e.g. 5)",
     0},
    {"override_nof_prb", 'p', "# PRB", 0,
     "Override the number of PRB received in the MIB", 0},
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
  uint8_t file_bw = 5;           /**< bandwidth of the sample file */
  const char
      *write_sample_file = {};   /**< file path of the created sample file. */
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

static unsigned sample_rate = 7680000;  /**< Sample rate of the SDR */
static unsigned frequency = 667000000;  /**< Center freqeuncy the SDR is tuned to */
static uint32_t bandwidth = 10000000;   /**< Low pass filter bandwidth for the SDR */
static double gain = 0.9;               /**< Overall system gain for the SDR */
static std::string antenna = "LNAW";    /**< Antenna input to be used */

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
  if (g > 1) { g = 1; }
  if (g < 0) { g = 0; }
  gain = g;
  spdlog::info("RESTful API requesting new parameters: fc {}, bw {}, rate {}, gain {}, antenna {}",
      frequency, bandwidth, sample_rate, gain, antenna);

  restart = true;
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
  arguments.config_file = "/etc/rp.conf";
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
  std::string ident = "rp";
  auto syslog_logger = spdlog::syslog_logger_mt("syslog", ident, LOG_PID | LOG_PERROR);

  spdlog::set_level(
      static_cast<spdlog::level::level_enum>(arguments.log_level));
  spdlog::set_pattern("[%H:%M:%S.%f %z] [%^%l%$] [thr %t] %v");

  spdlog::set_default_logger(syslog_logger);
  spdlog::info("ofr-lrp v{}.{} starting up", VERSION_MAJOR, VERSION_MINOR);


  // Init and tune the SDR
  spdlog::info("Initialising LimeSDR");
  LimeSdrReader lime(cfg);

  if (!lime.init(0, arguments.sample_file, arguments.write_sample_file)) {
    spdlog::error("Failed to initialize I/Q data source.");
    exit(1);
  }

  cfg.lookupValue("sdr.search_sample_rate_hz", sample_rate);
  cfg.lookupValue("sdr.center_frequency_hz", frequency);
  cfg.lookupValue("sdr.normalized_gain", gain);
  cfg.lookupValue("sdr.antenna", antenna);

  if (!lime.tune(frequency, sample_rate, bandwidth, gain, antenna)) {
    spdlog::error("Failed to set initial center frequency. Exiting.");
    exit(1);
  }

  srslte_verbose = arguments.log_level == 1 ? 2 : 0;
  srslte_use_standard_symbol_size(true);

  // Create a thread pool for the frame processors
  unsigned thread_cnt = 4;
  cfg.lookupValue("phy.threads", thread_cnt);
  int phy_prio = 10;
  cfg.lookupValue("phy.thread_priority_rt", phy_prio);
  thread_pool pool{ thread_cnt + 1, phy_prio };

  // Elevate execution to real time scheduling
  struct sched_param thread_param = {};
  thread_param.sched_priority = 20;
  cfg.lookupValue("phy.main_thread_priority_rt", thread_param.sched_priority);

  spdlog::info("Raising main thread to realtime scheduling priority {}", thread_param.sched_priority);

  int error = pthread_setschedparam(pthread_self(), SCHED_RR, &thread_param);
  if (error != 0) {
    spdlog::error("Cannot set main thread priority to realtime: {}. Thread will run at default priority.", strerror(error));
  }

  bool enable_measurement_file = false;
  cfg.lookupValue("measurement_file.enabled", enable_measurement_file);
  MeasurementFileWriter measurement_file(cfg);

  // Create the layer components: Phy, RLC, RRC and GW
  Phy phy(
      cfg,
      std::bind(&LimeSdrReader::getSamples, &lime, _1, _2, _3),  // NOLINT
      arguments.file_bw * 5,
      arguments.override_nof_prb);

  phy.init();

  srslte::pdcp pdcp(nullptr, "PDCP");
  srslte::rlc rlc("RLC");
  srslte::timer_handler timers;

  Rrc rrc(cfg, phy, rlc);
  Gw gw(cfg, phy);
  gw.init();

  rlc.init(&pdcp, &rrc, &timers, 0 /* RB_ID_SRB0 */);
  pdcp.init(&rlc, &rrc,  &gw);

  auto srs_level = srslte::LOG_LEVEL_WARNING;
  switch (arguments.srs_log_level) {
    case 0: srs_level = srslte::LOG_LEVEL_DEBUG; break;
    case 1: srs_level = srslte::LOG_LEVEL_INFO; break;
    case 2: srs_level = srslte::LOG_LEVEL_WARNING; break;
    case 3: srs_level = srslte::LOG_LEVEL_ERROR; break;
    case 4: srs_level = srslte::LOG_LEVEL_NONE; break;
  }

  // Configure srsLTE logging
  srslte::log_ref rlc_log = srslte::logmap::get("RLC");
  rlc_log->set_level(srs_level);
  rlc_log->set_hex_limit(128);

  srslte::log_ref mac_log = srslte::logmap::get("MAC");
  mac_log->set_level(srs_level);
  mac_log->set_hex_limit(128);

  state_t state = searching;

  // Create the RESTful API handler
  std::string uri = "http://0.0.0.0:3000/rp-api/";
  cfg.lookupValue("restful_api.uri", uri);
  spdlog::info("Starting RESTful API handler at {}", uri);
  RestHandler rest_handler(cfg, uri, state, lime, phy, set_params);

  // Initialize one CAS and thered_cnt MBSFN frame processors
  CasFrameProcessor cas_processor(cfg, phy, rlc, rest_handler);
  if (!cas_processor.init()) {
    spdlog::error("Failed to create CAS processor. Exiting.");
    exit(1);
  }

  std::vector<MbsfnFrameProcessor*> mbsfn_processors;
  for (int i = 0; i < thread_cnt; i++) {
    auto p = new MbsfnFrameProcessor(cfg, rlc, phy, mac_log, rest_handler);
    if (!p->init()) {
      spdlog::error("Failed to create MBSFN processor. Exiting.");
      exit(1);
    }
    mbsfn_processors.push_back(p);
  }

  // Start receiving sample data
  lime.start();

  uint32_t tti = 0;

  uint32_t measurement_interval = 5;
  cfg.lookupValue("measurement_file.interval_secs", measurement_interval);
  measurement_interval *= 1000;
  uint32_t tick = 0;

  // Initial state: searching a cell
  state = searching;

  // Start the main processing loop
  for (;;) {
    if (state == searching) {
      // In searching state, clear the receive buffer and try to find a cell at the configured frequency and synchronize with it
      restart = false;
      lime.clear_buffer();
      bool cell_found = phy.cell_search();
      if (cell_found) {
        // A cell has been found. We now know the required number of PRB = bandwidth of the carrier. Set the approproiate
        // sample rate...
        cas_nof_prb = mbsfn_nof_prb = phy.nr_prb();
        unsigned new_srate = srslte_sampling_freq_hz(cas_nof_prb);
        spdlog::info("Setting sample rate {} Mhz for {} PRB / {} Mhz channel width", new_srate/1000000.0, phy.nr_prb(),
            phy.nr_prb() * 0.2);
        lime.stop();

        bandwidth = (cas_nof_prb * 200000) * 1.2;
        lime.tune(frequency, new_srate, bandwidth, gain, antenna);
        lime.start();
        spdlog::debug("Synchronizing subframe");
        // ... and move to syncing state.
        state = syncing;
      } else {
        sleep(1);
      }
    } else if (state == syncing) {
      // In syncing state, we already know the cell we want to camp on, and the SDR is tuned to the required
      // sample rate for its number of PRB / bandwidth. We now synchronize PSS/SSS and receive the MIB once again 
      // at this sample rate.
      unsigned max_frames = 200;
      bool sfn_sync = false;
      while (!sfn_sync && max_frames-- > 0) {
        sfn_sync = phy.synchronize_subframe();
      }

      if (max_frames == 0 && !sfn_sync) {
        // Failed. Back to square one: search state.
        spdlog::warn("Synchronization failed. Going back to search state.");
        state = searching;
        sleep(1);
      }

      if (sfn_sync) {
        // We're locked on to the cell, and have succesfully received the MIB at the target sample rate.
        spdlog::info("Decoded MIB at target sample rate, TTI is {}. Subframe synchronized.", phy.tti());

        // Set the cell parameters in the CAS and MBSFN processors
        cas_processor.set_cell(phy.cell());

        for (int i = 0; i < thread_cnt; i++) {
          mbsfn_processors[i]->unlock();
        }

        // Get the initial TTI / subframe ID (= system frame number * 10 + subframe number)
        tti = phy.tti();
        // Reset the RRC
        rrc.reset();

        // Ready to receive actual data. Go to processing state.
        state = processing;
      }
    } else {  // processing
      int mb_idx = 0;
      while (state == processing) {
        tti = (tti + 1) % 10240; // Clamp the TTI
        if (tti%40 == 0) { 
          // This is subframe 0 in a radio frame divisible by 4, and hence a CAS frame. 
          // Get the samples from the SDR interface, hand them to a CAS processor, and start it
          // on a thread from the pool.
          if (!restart && phy.get_next_frame(cas_processor.rx_buffer(), cas_processor.rx_buffer_size())) {
            spdlog::debug("sending tti {} to regular processor", tti);
            pool.push([ObjectPtr = &cas_processor, tti] {
                ObjectPtr->process(tti);
                });

            // Set constellation diagram data and rx params for CAS in the REST API handler
            rest_handler._ce_values = std::move(cas_processor.ce_values());
            rest_handler._pdsch.SetData(cas_processor.pdsch_data());

            if (phy.nof_mbsfn_prb() != mbsfn_nof_prb)
            {
              mbsfn_nof_prb = phy.nof_mbsfn_prb();
              // Adjust the sample rate to fit the wider MBSFN bandwidth
              unsigned new_srate = srslte_sampling_freq_hz(mbsfn_nof_prb);
              spdlog::info("Setting sample rate {} Mhz for MBSFN with {} PRB / {} Mhz channel width", new_srate/1000000.0, mbsfn_nof_prb,
                  mbsfn_nof_prb * 0.2);
              lime.stop();

              bandwidth = (mbsfn_nof_prb * 200000) * 1.2;
              lime.tune(frequency, new_srate, bandwidth, gain, antenna);
              lime.start();
              spdlog::debug("Synchronizing subframe after PRB extension");
              // ... and move to syncing state.
              phy.set_cell();
              cas_processor.set_cell(phy.cell());
              state = syncing;
            }
          } else {
            // Failed to receive data, or sync lost. Go back to searching state.
            state = searching;
            sleep(1);

            lime.tune(frequency, sample_rate, bandwidth, gain, antenna);
            rrc.reset();
            phy.reset();
          }
        } else {
          // All other frames in FeMBMS dedicated mode are MBSFN frames.
          spdlog::debug("sending tti {} to mbsfn proc {}", tti, mb_idx);

          // Get the samples from the SDR interface, hand them to an MNSFN processor, and start it
          // on a thread from the pool. Getting the buffer pointer from the pool also locks this processor.
          if (!restart && phy.get_next_frame(mbsfn_processors[mb_idx]->get_rx_buffer_and_lock(), mbsfn_processors[mb_idx]->rx_buffer_size())) {
            if (phy.mcch_configured()) {
              // If data frm SIB1/SIB13 has been received in CAS, configure the processors accordingly
              if (!mbsfn_processors[mb_idx]->mbsfn_configured()) {
                srslte_scs_t scs;
                switch (phy.mbsfn_subcarrier_spacing()) {
                  case Phy::SubcarrierSpacing::df_15kHz:  scs = SRSLTE_SCS_15KHZ; break;
                  case Phy::SubcarrierSpacing::df_7kHz5:  scs = SRSLTE_SCS_7KHZ5; break;
                  case Phy::SubcarrierSpacing::df_1kHz25: scs = SRSLTE_SCS_1KHZ25; break;
                }
                auto cell = phy.cell();
                cell.nof_prb = cell.mbsfn_prb;
                mbsfn_processors[mb_idx]->set_cell(cell);
                mbsfn_processors[mb_idx]->configure_mbsfn(phy.mbsfn_area_id(), scs);
              }
              pool.push([ObjectPtr = mbsfn_processors[mb_idx], tti] {
                ObjectPtr->process(tti);
              });
            } else {
              // Nothing to do yet, we lack the data from SIB1/SIB13
              // Discard the samples and unlock the processor.
              mbsfn_processors[mb_idx]->unlock();
            }
          } else {
            // Failed to receive data, or sync lost. Go back to searching state.
            spdlog::warn("Synchronization lost while processing. Going back to searching state.");
            state = searching;
            lime.tune(frequency, sample_rate, bandwidth, gain, antenna);

            sleep(1);
            rrc.reset();
            phy.reset();
          }
          mb_idx = static_cast<int>((mb_idx + 1) % thread_cnt);
        }

        tick++;
        if (tick%measurement_interval == 0) {
          // It's time to output rx info to the measurement file and to syslog.
          // Collect the relevant info and write it out.
          std::vector<std::string> cols;

          spdlog::info("CINR {:.2f} dB", cas_processor.cinr_db() );
          cols.push_back(std::to_string(cas_processor.cinr_db()));

          spdlog::info("PDSCH: MCS {}, BLER {}, BER {}",
              rest_handler._pdsch.mcs,
              ((rest_handler._pdsch.errors * 1.0) / (rest_handler._pdsch.total * 1.0)),
              rest_handler._pdsch.ber);
          cols.push_back(std::to_string(rest_handler._pdsch.mcs));
          cols.push_back(std::to_string(((rest_handler._pdsch.errors * 1.0) / (rest_handler._pdsch.total * 1.0))));
          cols.push_back(std::to_string(rest_handler._pdsch.ber));

          spdlog::info("MCCH: MCS {}, BLER {}, BER {}",
              rest_handler._mcch.mcs,
              ((rest_handler._mcch.errors * 1.0) / (rest_handler._mcch.total * 1.0)),
              "-");

          cols.push_back(std::to_string(rest_handler._mcch.mcs));
          cols.push_back(std::to_string(((rest_handler._mcch.errors * 1.0) / (rest_handler._mcch.total * 1.0))));
          cols.emplace_back("-");

          auto mch_info = phy.mch_info();
          int mch_idx = 0;
          std::for_each(std::begin(mch_info), std::end(mch_info), [&cols, &mch_idx, &rest_handler](Phy::mch_info_t const& mch) {
              spdlog::info("MCH {}: MCS {}, BLER {}, BER {}",
                  mch_idx,
                  mch.mcs,
                  (rest_handler._mch[mch_idx].errors * 1.0) / (rest_handler._mch[mch_idx].total * 1.0),
                  "-");
              cols.push_back(std::to_string(mch_idx));
              cols.push_back(std::to_string(mch.mcs));
              cols.push_back(std::to_string((rest_handler._mch[mch_idx].errors * 1.0) / (rest_handler._mch[mch_idx].total * 1.0)));
              cols.emplace_back("-");

              int mtch_idx = 0;
              std::for_each(std::begin(mch.mtchs), std::end(mch.mtchs), [&mtch_idx](Phy::mtch_info_t const& mtch) {
                spdlog::info("    MTCH {}: LCID {}, TMGI 0x{}, {}",
                  mtch_idx,
                  mtch.lcid,
                  mtch.tmgi,
                  mtch.dest);
                mtch_idx++;
                  });
                mch_idx++;
              });
          spdlog::info("-----");
          if (enable_measurement_file) {
            measurement_file.WriteLogValues(cols);
          }
        }
      }
    }
  }

  // Main loop ended by signal. Free the MBSFN processors, and bail.
  for (int i = 0; i < thread_cnt; i++) {
    delete( mbsfn_processors[i] );
  }
exit:
  return 0;
}
