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

#include <cstdint>
#include <string>
#include <vector>
#include <thread>
#include "Phy.h"
#include "RestHandler.h"
#include <libconfig.h++>
#include "srslte/srslte.h"
#include "srslte/upper/rlc.h"
#include "srslte/upper/pdcp.h"

/**
 *  Frame processor for CAS subframes. Handles the complete processing chain for
 *  a CAS subframe: calls FFT and channel estimation, decodes PCFICH and PDCCH and gets DCI(s),
 *  decodes PDSCH and passes received PDUs to RLC.
 */
class CasFrameProcessor {
 public:
   /**
    *  Default constructor.
    *
    *  @param cfg Config singleton reference
    *  @param phy PHY reference
    *  @param rlc RLC reference
    *  @param rest RESTful API handler reference
    */
   CasFrameProcessor(const libconfig::Config& cfg, Phy& phy, srslte::rlc& rlc, RestHandler& rest)
     : _cfg(cfg)
       , _phy(phy)
       , _rest(rest)
       , _rlc(rlc) {}

   /**
    *  Default destructor.
    */
   virtual ~CasFrameProcessor();

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
   bool process(uint32_t tti);

   /**
    *  Set the parameters for the cell (Nof PRB, etc).
    * 
    *  @param cell The cell we're camping on
    */
   void set_cell(srslte_cell_t cell);

   /**
    *  Get a handle of the signal buffer to store samples for processing in
    */
   cf_t** rx_buffer() { return _signal_buffer_rx; }

   /**
    *  Size of the signal buffer
    */
   uint32_t rx_buffer_size() { return _signal_buffer_max_samples; }

   /**
    *  Get the CE values (time domain) for displaying the spectrum
    *  of the received signal
    */
   std::vector<uint8_t> ce_values();

   /**
    *  Get the constellation diagram data (I/Q data of the subcarriers after CE)
    */
   std::vector<uint8_t> pdsch_data();

   /**
    *  Get the CINR estimate (in dB)
    */
   float cinr_db() { return _ue_dl.chest_res.snr_db; }

 private:
   const libconfig::Config& _cfg;
    srslte::rlc& _rlc;
    Phy& _phy;
    RestHandler& _rest;

    cf_t*    _signal_buffer_rx[SRSLTE_MAX_PORTS] = {};
    uint32_t _signal_buffer_max_samples          = 0;

    srslte_softbuffer_rx_t _softbuffer;
    uint8_t* _data[SRSLTE_MAX_CODEWORDS];

    srslte_ue_dl_t     _ue_dl     = {};
    srslte_ue_dl_cfg_t _ue_dl_cfg = {};
    srslte_dl_sf_cfg_t _sf_cfg = {};

    srslte_cell_t _cell;
};
