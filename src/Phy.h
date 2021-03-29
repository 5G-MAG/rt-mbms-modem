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

#pragma once

#include <lime/LimeSuite.h>
#include <functional>
#include <cstdint>
#include <string>
#include <map>
#include <vector>
#include <thread>
#include <libconfig.h++>

#include "srslte/srslte.h"
#include "srslte/interfaces/rrc_interface_types.h"
#include "srslte/common/gen_mch_tables.h"
#include "srslte/phy/common/phy_common.h"

constexpr unsigned int MAX_PRB = 100;

/**
 *  The PHY component. Handles synchronisation and is the central hub for
 *  lower level processing
 *
 */
class Phy {
 public:
    /**
     *  Definition of the callback function used to fetch samples from the SDR
     */
    typedef std::function<int(cf_t* data, uint32_t nsamples, srslte_timestamp_t* rx_time)> get_samples_t;

    /**
     *  Default constructor.
     *
     *  @param cfg Config singleton reference
     *  @param cb  Sample recv callback
     *  @param cs_nof_prb  Nr of PRBs to use during cell search
     *  @param override_nof_prb  If set, overrides the nof PRB received in the MIB
     */
    Phy(const libconfig::Config& cfg, get_samples_t cb, uint8_t cs_nof_prb, int8_t override_nof_prb);
    
    /**
     *  Default destructor.
     */
    virtual ~Phy();
    
    /**
     *  Initialize the underlying components.
     */
    bool init();

    /**
     *  Search for a cell
     *
     *  Returns true if a cell has been found and the MIB could be decoded, false otherwise.
     */
    bool cell_search();

    /**
     *  Synchronizes PSS/SSS and tries to deocode the MIB.
     *
     *  Returns true on success, false otherwise.
     */
    bool synchronize_subframe();

    /**
     * Get the sample data for the next subframe.
     */
    bool get_next_frame(cf_t** buffer, uint32_t size);

    /**
     * Get the current cell.
     */
    srslte_cell_t cell() { return _cell; }

    /**
     * Get the current number of PRB.
     */
    unsigned nr_prb() { return _cell.nof_prb; }

    /**
     * Get the current subframe TTI
     */
    uint32_t tti() { return _tti; }
    
    /**
     * Get the current CFO value
     */
    float cfo() { return srslte_ue_sync_get_cfo(&_ue_sync);}

    /**
     * Set the CFO value from channel estimation
     */
    void set_cfo_from_channel_estimation(float cfo) { srslte_ue_sync_set_cfo_ref(&_ue_sync, cfo); }

    /**
     * Set the values received in SIB13
     */
    void set_mch_scheduling_info(const srslte::sib13_t& sib13);

    /**
     * Set MBSFN configuration values
     */
    void set_mbsfn_config(const srslte::mcch_msg_t& mcch);

    /**
     * Clear configuration values
     */
    void reset() { _mcch_configured = _mch_configured = false; }

    /**
     * Return true if MCCH has been configured
     */
    bool mcch_configured() { return _mcch_configured; }

    /**
     * Returns the current MBSFN area ID
     */
    uint8_t mbsfn_area_id() { return _sib13.mbsfn_area_info_list[0].mbsfn_area_id; }

    /**
     * Returns the MBSFN configuration (MCS, etc) for the subframe with the passed TTI
     */
    srslte_mbsfn_cfg_t mbsfn_config_for_tti(uint32_t tti, unsigned& area);

    /**
     * Enable MCCH decoding
     */
    void set_decode_mcch(bool d) { _decode_mcch = d; }

    typedef struct {
      std::string tmgi;
      std::string dest;
      int lcid;
    } mtch_info_t;
    typedef struct {
      int mcs;
      std::vector< mtch_info_t > mtchs;
    } mch_info_t;

    const std::vector< mch_info_t>& mch_info() { return _mch_info;  }

    void set_dest_for_lcid(int lcid, std::string dest) { _dests[_mcs][lcid] = dest; }

    enum class SubcarrierSpacing {
      df_15kHz,
      df_7kHz5,
      df_1kHz25
    };

    SubcarrierSpacing mbsfn_subcarrier_spacing() {
      switch (_sib13.mbsfn_area_info_list[0].subcarrier_spacing) {
        case srslte::mbsfn_area_info_t::subcarrier_spacing_t::khz_minus1dot25: return SubcarrierSpacing::df_1kHz25;
        case srslte::mbsfn_area_info_t::subcarrier_spacing_t::khz_minus7dot5: return SubcarrierSpacing::df_7kHz5;
        default: return SubcarrierSpacing::df_15kHz;
      }
    }

    float mbsfn_subcarrier_spacing_khz() {
      switch (_sib13.mbsfn_area_info_list[0].subcarrier_spacing) {
        case srslte::mbsfn_area_info_t::subcarrier_spacing_t::khz_minus1dot25: return 1.25;
        case srslte::mbsfn_area_info_t::subcarrier_spacing_t::khz_minus7dot5: return 7.5;
        default: return 15;
      }
    }

    srslte::mcch_msg_t& mcch() { return _mcch; }

    int _mcs = 0;
    get_samples_t _sample_cb;

 private:
    const libconfig::Config& _cfg;
    srslte_ue_sync_t _ue_sync = {};
    srslte_ue_cellsearch_t _cell_search = {};
    srslte_ue_mib_sync_t  _mib_sync = {};
    srslte_ue_mib_t  _mib = {};
    srslte_cell_t _cell = {};

    bool _decode_mcch = false;

    cf_t* _mib_buffer[SRSLTE_MAX_CHANNELS] = {};
    uint32_t _buffer_max_samples = 0;
    uint32_t _tti = 0;

    uint8_t  _mcch_table[10] = {};
    bool _mcch_configured = false;
    srslte::sib13_t _sib13 = {};
    srslte::mcch_msg_t _mcch = {};

    bool _mch_configured = false;

    uint8_t _cs_nof_prb;

    std::vector< mch_info_t > _mch_info;

    std::map< int, std::map< int, std::string >> _dests;

    int8_t _override_nof_prb;
};
