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

#include <argp.h>

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
     "If decoding data from a file, specify the channel bandwidth of the "
     "recorded data in MHz here (e.g. 5)",
     0},
    {"override_nof_prb", 'p', "# PRB", 0,
     "Override the number of PRB received in the MIB", 0},
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

static unsigned sample_rate = 7680000;  /**< Sample rate of the SDR */
static unsigned search_sample_rate = 7680000;  /**< Sample rate of the SDR */
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


  cfg.lookupValue("modem.sdr.normalized_gain", gain);
  cfg.lookupValue("modem.sdr.antenna", antenna);
  cfg.lookupValue("modem.sdr.use_agc", use_agc);

  if (!sdr.tune(frequency, sample_rate, bandwidth, gain, antenna, use_agc)) {
    spdlog::error("Failed to set initial center frequency. Exiting.");
    exit(1);
  }

  set_srsran_verbose_level(arguments.log_level <= 1 ? SRSRAN_VERBOSE_DEBUG : SRSRAN_VERBOSE_NONE);
  srsran_use_standard_symbol_size(true);

  // Create a thread pool for the frame processors
  unsigned thread_cnt = 4;
  cfg.lookupValue("modem.phy.threads", thread_cnt);
  int phy_prio = 10;
  cfg.lookupValue("modem.phy.thread_priority_rt", phy_prio);
  thread_pool pool{ thread_cnt + 1, phy_prio };

  // Elevate execution to real time scheduling
  struct sched_param thread_param = {};
  thread_param.sched_priority = 20;
  cfg.lookupValue("modem.phy.main_thread_priority_rt", thread_param.sched_priority);

  spdlog::info("Raising main thread to realtime scheduling priority {}", thread_param.sched_priority);

  int error = pthread_setschedparam(pthread_self(), SCHED_RR, &thread_param);
  if (error != 0) {
    spdlog::error("Cannot set main thread priority to realtime: {}. Thread will run at default priority.", strerror(error));
  }

  bool enable_measurement_file = false;
  cfg.lookupValue("modem.measurement_file.enabled", enable_measurement_file);
  MeasurementFileWriter measurement_file(cfg);

  // Create the layer components: Phy, RLC, RRC and GW
  Phy phy(
      cfg,
      std::bind(&SdrReader::get_samples, &sdr, _1, _2, _3),  // NOLINT
      arguments.file_bw ? arguments.file_bw * 5 : 25,
      arguments.override_nof_prb,
      rx_channels);

  phy.init();

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
  
  // We need the cas processor to be accesible within the rest_handler object to gather all the values display in the rt-wui
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
  // Start the main processing loop
  for (;;) { // Only one main loop, any time therè's a change of state we force next iteration with continue. This way there's no need of nested loops within the cases.
    switch (state) {
      case processing: {  // processing
        tti = (tti + 1) % 10240; // Clamp the TTI
        unsigned sfn = tti / 10;
        if (phy.is_cas_subframe(tti)) {
          // Get the samples from the SDR interface, hand them to a CAS processor, and start it
          // on a thread from the pool.
          if (!restart && phy.get_next_frame(cas_processor.get_rx_buffer_and_lock(), cas_processor.rx_buffer_size())) {
            spdlog::debug("sending tti {} to regular processor", tti);
            pool.push([ObjectPtr = &cas_processor, tti, &rest_handler] {
                if (ObjectPtr->process(tti)) {
                // Set constellation diagram data and rx params for CAS in the REST API handler
                rest_handler.add_cinr_value(ObjectPtr->cinr_db());
                }
            });


            if (phy.nof_mbsfn_prb() != mbsfn_nof_prb)
            {
              // Handle the non-LTE bandwidths (6, 7 and 8 MHz). In these cases, CAS stays at the original bandwidth, but the MBSFN
              // portion of the frames can be wider. We need to...

              mbsfn_nof_prb = phy.nof_mbsfn_prb();

              // ...adjust the SDR's sample rate to fit the wider MBSFN bandwidth...
              unsigned new_srate = srsran_sampling_freq_hz(mbsfn_nof_prb);
              spdlog::info("Setting sample rate {} Mhz for MBSFN with {} PRB / {} Mhz channel width", new_srate/1000000.0, mbsfn_nof_prb,
                  mbsfn_nof_prb * 0.2);
              sdr.stop();

              bandwidth = (mbsfn_nof_prb * 200000) * 1.2;
              sdr.tune(frequency, new_srate, bandwidth, gain, antenna, use_agc);

              // ... configure the PHY and CAS processor to decode a narrow CAS and wider MBSFN, and move back to syncing state
              // after reconfiguring and restarting the SDR.
              phy.set_cell();
              cas_processor.set_cell(phy.cell());

              sdr.start();
              spdlog::info("Synchronizing subframe after PRB extension");
              state = syncing;
            }
          } else {
            // Failed to receive data, or sync lost. Go back to searching state.
            spdlog::warn("Synchronization lost while processing. Going back to searching state.");
            sync_losses++;
            state = syncing;
            auto t2 = std::chrono::high_resolution_clock::now();
          }
        } else {
          // All other frames in FeMBMS dedicated mode are MBSFN frames.
          spdlog::debug("sending tti {} to mbsfn proc {}", tti, mb_idx);

          // Get the samples from the SDR interface, hand them to an MNSFN processor, and start it
          // on a thread from the pool. Getting the buffer pointer from the pool also locks this processor.
          auto t1 = std::chrono::high_resolution_clock::now();
          auto t2 = t1;
          if (!restart && phy.get_next_frame(mbsfn_processors[mb_idx]->get_rx_buffer_and_lock(), mbsfn_processors[mb_idx]->rx_buffer_size())) {
            t2 = std::chrono::high_resolution_clock::now();
            if (phy.mcch_configured() && phy.is_mbsfn_subframe(tti)) {
              // If data frm SIB1/SIB13 has been received in CAS, configure the processors accordingly
              if (!mbsfn_processors[mb_idx]->mbsfn_configured()) {
                srsran_scs_t scs = SRSRAN_SCS_15KHZ;
                switch (phy.mbsfn_subcarrier_spacing()) {
                  case Phy::SubcarrierSpacing::df_15kHz:  scs = SRSRAN_SCS_15KHZ; break;
                  case Phy::SubcarrierSpacing::df_7kHz5:  scs = SRSRAN_SCS_7KHZ5; break;
                  case Phy::SubcarrierSpacing::df_1kHz25: scs = SRSRAN_SCS_1KHZ25; break;
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
            spdlog::warn("Synchronization lost while processing. Going back to searching state, we were waiting {} microseconds.", std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count());
            sync_losses++; 
            state = syncing;
          }
          mb_idx = static_cast<int>((mb_idx + 1) % thread_cnt);
        }
      }
      break;
      
      case searching: {
        if (restart) { // Triggered from the rt-wui
          sdr.stop();
          sample_rate = search_sample_rate;  // sample rate for searching
          sdr.tune(frequency, sample_rate, bandwidth, gain, antenna, use_agc);
          sdr.start();
        }

        // We're at the search sample rate, and there's no point in creating a sample file. rtop the sample writer, if enabled.
        sdr.disableSampleFileWriting();

        // In searching state, clear the receive buffer and try to find a cell at the configured frequency and synchronize with it
        restart = false;
       // sdr.clear_buffer();
        bool cell_found = phy.cell_search();
        if (cell_found) {
          // A cell has been found. We now know the required number of PRB = bandwidth of the carrier. Set the approproiate
          // sample rate...
          cas_nof_prb = mbsfn_nof_prb = phy.nr_prb();

          if (arguments.sample_file && arguments.file_bw) {
            // Samples files are recorded at a fixed sample rate that can be determined from the bandwidth command line argument.
            // If we're decoding from file, do not readjust the rate to match the CAS PRBs, but stay at this rate and instead configure the
            // PHY to decode a narrow CAS from a wider channel.
            mbsfn_nof_prb = arguments.file_bw * 5;
            phy.set_nof_mbsfn_prb(mbsfn_nof_prb);
            phy.set_cell();
          } else {
            // When decoding from the air, configure the SDR accordingly
            unsigned new_srate = srsran_sampling_freq_hz(cas_nof_prb);
            spdlog::info("Setting sample rate {} Mhz for {} PRB / {} Mhz channel width", new_srate/1000000.0, phy.nr_prb(),
                phy.nr_prb() * 0.2);
            sdr.stop();
            sdr.clear_buffer();
            bandwidth = (cas_nof_prb * 200000) * 1.2;
            sdr.tune(frequency, new_srate, bandwidth, gain, antenna, use_agc);
            //sleep(1);

            sdr.start();
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

          // Set the cell parameters in the CAS processor, and set started to true
          cas_processor.set_cell(phy.cell());

          for (int i = 0; i < thread_cnt; i++) {
            mbsfn_processors[i]->unlock();
          }
          cas_processor.unlock(); // We need to unlock because we felt here from processing after calling get_rx_buffer_and_lock()

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
        //spdlog::info("CINR {:.2f} dB", rest_handler.cinr_db() );
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
