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

#pragma once
#include <string>
#include <vector>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <libconfig.h++>

#include "SdrReader.h"
#include "Phy.h"

#include "cpprest/json.h"
#include "cpprest/http_listener.h"
#include "cpprest/uri.h"
#include "cpprest/asyncrt_utils.h"
#include "cpprest/filestream.h"
#include "cpprest/containerstream.h"
#include "cpprest/producerconsumerstream.h"

const int CINR_RAVG_CNT = 20;
typedef enum { searching, syncing, processing } state_t;

class CasFrameProcessor; // Forward declaration of CasFrameProcessor to avoid circular references.
class MbsfnFrameProcessor;

/**
 *  The RESTful API handler. Supports GET and PUT verbs for SDR parameters, and GET for reception info
 */
class RestHandler {
  public:
    /**
     *  Definition of the callback for setting new reception parameters
     */
    typedef std::function<void(const std::string& antenna, unsigned fcen, double gain, unsigned sample_rate, unsigned bandwidth)> set_params_t;

    /**
     *  What a logged subframe was used for. CAS occasions only occur in subframe 0,
     *  once every 40ms (see TS 36.331's dedicated-cell CAS scheduling) - everything
     *  else is either MCCH, MCH, or a gap (e.g. additionalNonMBSFNSubframes, or a
     *  subframe the MBSFN config isn't decoded yet to classify).
     */
    enum SubframeEventType : uint8_t { SF_EVENT_CAS = 0, SF_EVENT_MCCH = 1, SF_EVENT_MCH = 2, SF_EVENT_GAP = 3 };

    /**
     *  Outcome of processing a logged subframe. IDLE means the subframe was processed
     *  but had nothing to decode (e.g. a CAS occasion with no SI message pending, or a
     *  time-interleaved MCH block still accumulating) - not a failure.
     */
    enum SubframeEventStatus : uint8_t { SF_STATUS_IDLE = 0, SF_STATUS_OK = 1, SF_STATUS_FAIL = 2 };

    struct SubframeEvent {
      uint32_t sfn;
      uint8_t  sf;
      uint8_t  type;
      uint8_t  status;
    };

    /**
     *  Record one subframe's scheduling outcome for the CAS/MCCH/MCH activity matrix.
     */
    void record_subframe_event(uint32_t tti, uint8_t type, uint8_t status);

    /**
     *  Snapshot of the recent subframe event log, oldest first.
     */
    std::vector<SubframeEvent> subframe_log_snapshot();

    /**
     *  Default constructor.
     *
     *  @param cfg Config singleton reference
     *  @param url URL to open the server on
     *  @param state Reference to the main loop sate
     *  @param sdr Reference to the SDR reader
     *  @param set_params Set parameters callback
     */
    RestHandler(const libconfig::Config& cfg, const std::string& url, state_t& state,
        SdrReader& sdr, Phy& phy, set_params_t set_params);
    /**
     *  Default destructor.
     */
    virtual ~RestHandler();

    /**
     *  Start function for the listener.
     */
    void start() { _listener->open().wait(); }

    /**
     *  RX Info pertaining to an SCH (MCCH/MCH or PDSCH)
     */
    class ChannelInfo {
      public:
        void SetData( std::vector<uint8_t> data) {
          std::lock_guard<std::mutex> lock(_data_mutex);
          _data = data;
        };
        std::vector<uint8_t> GetData() { 
          std::lock_guard<std::mutex> lock(_data_mutex);
          return _data; 
        };
        bool present = false;
        int mcs = 0;
        double ber;
        float evm_rms = 0.0f;
        unsigned total = 0;
        unsigned errors = 0;
      private:
        std::vector<uint8_t> _data = {};
        std::mutex _data_mutex;
    };

    /**
     *  Frequency domain subcarrier CE values (CAS)
     */
    std::vector<uint8_t> _ce_values = {};

    /**
     *  Frequency domain subcarrier CE values (MBSFN - MCCH/MCH)
     */
    std::vector<uint8_t> _ce_values_mbsfn = {};

    /**
     *  Time domain channel impulse response of the CAS.
     */
    std::vector<uint8_t> _cir_values = {};

    /**
     *  Time domain channel impulse response of the mbsfn subframes
     */
    std::vector<uint8_t> _cir_values_mbsfn = {};


    /**
     *  Correlaton samples from the sync functions.
     */
    std::vector<uint8_t> _corr_values = {};


    /**
     *  Correlaton samples from the sync functions.
     */
    std::vector<uint8_t> _corr_values_mbsfn = {};

    /**
     *  RX info for PDSCH
     */
    ChannelInfo _pdsch;

    /**
     *  RX info for PDCCH. MCS/BER/EVM don't apply to a control channel (no
     *  MCS, no CRC in the usual sense) - only total/errors and the raw
     *  constellation are meaningful: total = CAS occasions attempted,
     *  errors = occasions where no DCI candidate was found (this conflates
     *  "genuinely nothing scheduled" with "a candidate was sent but missed",
     *  since blind decoding can't distinguish the two - see box tooltip).
     */
    ChannelInfo _pdcch;

    /**
     *  RX info for MCCH
     */
    ChannelInfo _mcch;

    /**
     *  RX info for MCHs
     */
    std::map<uint32_t, ChannelInfo> _mch;

    /**
     *  Current instantaneous CINR value
     */
    float cinr_db() { return _cinr_db.size() ? _cinr_db.back() : 0.0f; };
    
    /**
     *  Current average CINR value
     */
    float cinr_db_avg() { return _cinr_db.size() ? (std::accumulate(_cinr_db.begin(), _cinr_db.end(), 0.0f) / (_cinr_db.size() * 1.0f)) : 0.0f; };
    
    void add_cinr_value( float cinr);

    /**
     *  Save the pointer to the CasFrameProcessor
     */
    void set_cas_processor (CasFrameProcessor* cas_processor) { _cas_processor = cas_processor; };
    
    /**
     *  Save the pointer to the vector cointaining the MbsfnFrameProcessors
     */
    void set_mbsfn_processor (MbsfnFrameProcessor* mbsfn_processor) { _mbsfn_processors.push_back(mbsfn_processor); };


  private:
    // We need access to the processors to get the values to be displayed in the rt-wui.
    CasFrameProcessor* _cas_processor;
    std::vector<MbsfnFrameProcessor*> _mbsfn_processors; 
    
    std::vector<float>  _cinr_db;

    /* ~5s of history at 1ms/TTI, matching rt-wui's SUBFRAME_MATRIX_WINDOW_SECONDS -
     * the matrix's column width is derived from its canvas width divided by this
     * many frames, so buffering more than the display window ever uses is waste. */
    static constexpr size_t SUBFRAME_LOG_CAPACITY = 5000;
    std::deque<SubframeEvent> _subframe_log;
    std::mutex                _subframe_log_mutex;

    void get(web::http::http_request message);
    void put(web::http::http_request message);

    /**
     *  Aggregate JSON of everything decoded from SIB1-MBMS/SIB13/SIB15/SIB16
     *  and the current MCCH-derived PMCH schedule, for the SIB Inspection page.
     */
    web::json::value sib_info_json();

    const libconfig::Config& _cfg;

    std::unique_ptr<web::http::experimental::listener::http_listener> _listener;

    state_t& _state;
    SdrReader& _sdr;
    Phy& _phy;

    set_params_t _set_params;

    bool _require_bearer_token = false;
    std::string _api_key;
};

