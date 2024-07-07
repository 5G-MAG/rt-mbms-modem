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

#include <cstdint>
#include <string>
#include <thread>
#include <vector>
#include <map>
#include "srsran/srsran.h"
#include "srsran/rlc/rlc.h"
#include "srsran/upper/pdcp.h"
#include "srsran/mac/pdu.h"
#include <libconfig.h++>
#include "Phy.h"
#include "RestHandler.h"

/**
 *  Frame processor for MBSFN subframes. Handles the complete processing chain for
 *  a CAS subframe: calls FFT and channel estimation, decodes PDSCH and passes received PDUs to RLC.
 */
class MbsfnFrameProcessor {
  public:
    /**
     *  Default constructor.
     *
     *  @param cfg Config singleton reference
     *  @param phy PHY reference
     *  @param rlc RLC reference
     *  @param log_h srsLTE log handle for the MCH MAC msg decoder
     *  @param rest RESTful API handler reference
     */
    MbsfnFrameProcessor(const libconfig::Config& cfg, srsran::rlc& rlc, Phy& phy, srslog::basic_logger& log_h, RestHandler& rest, unsigned rx_channels )
      : _cfg(cfg)
      , _rlc(rlc)
      , _phy(phy)
      , _rest(rest)
      , mch_mac_msg(20, log_h)
      , _rx_channels(rx_channels)
      {}

    /**
     *  Default destructor.
     */
    virtual ~MbsfnFrameProcessor();

    /**
     *  Initialize signal- and softbuffers, init all underlying components. 
     *  Must be called once before the first call to process().
     */
    bool init();

    /**
     *  Process the sample data in the signal buffer. Data must already be present in the buffer
     *  obtained through the handle returnd by rx_buffer()
     *
     *  @param tti TTI of the subframe the data belongs to
     */
    int process(uint32_t tti);

    /**
     *  Set the parameters for the cell (Nof PRB, etc).
     * 
     *  @param cell The cell we're camping on
     */
    void set_cell(srsran_cell_t cell);

    /**
     *  Get a handle of the signal buffer to store samples for processing in, 
     *  and lock this processor.
     *
     *  The processor unlocks itself after (failed or successful) frame processing in process().
     *  If process() is not called by the application after calling this method, it must unlock the 
     *  processor itself by calling unlock()
     */
    cf_t** get_rx_buffer_and_lock() { _mutex.lock(); return _signal_buffer_rx; }

    /**
     *  Size of the signal buffer
     */
    uint32_t rx_buffer_size() { return _signal_buffer_max_samples; }

    /**
     *  Set MBSFN parameters: area ID and subcarrier spacing
     */
    void configure_mbsfn(uint8_t area_id, srsran_scs_t subcarrier_spacing);

    /**
     *  Returns tru if MBSFN params have already been configured
     */
    bool mbsfn_configured() { return _mbsfn_configured; }

    /**
     *  Unlock the processor
     *
     *  @see get_rx_buffer_and_lock() 
     */
    void unlock() { _mutex.unlock(); }

    /**
     *  Lock the processor
     *
     *  Used when getting the BLER values.
     */
    void lock() { _mutex.lock(); }

    /**
     *  Get the constellation diagram data (I/Q data of the subcarriers after CE)
     */
    const std::vector<uint8_t> mch_data() const;

    /**
     *  Get the CINR estimate (in dB)
     */
    float cinr_db() { return _ue_dl.chest_res.snr_db; }

  private:
    const libconfig::Config& _cfg;
    srsran::rlc& _rlc;
    Phy& _phy;

    srsran_cell_t _cell;

    cf_t*    _signal_buffer_rx[SRSRAN_MAX_PORTS] = {};
    uint32_t _signal_buffer_max_samples          = 0;

    static const uint32_t  _payload_buffer_sz = SRSRAN_MAX_BUFFER_SIZE_BYTES;
    uint8_t                _payload_buffer[_payload_buffer_sz];
    srsran_softbuffer_rx_t _softbuffer;

    srsran_ue_dl_t     _ue_dl     = {};
    srsran_ue_dl_cfg_t _ue_dl_cfg = {};
    srsran_dl_sf_cfg_t _sf_cfg = {};
    srsran_pmch_cfg_t  _pmch_cfg  = {};

    uint8_t _area_id = 1;
    bool _mbsfn_configured = false;

    srsran::mch_pdu mch_mac_msg;
    std::mutex _mutex;

    RestHandler& _rest;

    unsigned _rx_channels;

    static std::mutex _sched_stop_mutex;
    static std::map<uint8_t, uint16_t> _sched_stops;

    static std::mutex _rlc_mutex;
    static int _current_mcs;
};
