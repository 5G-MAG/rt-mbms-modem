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
#include <libconfig.h++>
#include "srsran/srsran.h"
#include "srsran/rlc/rlc.h"
#include "srsran/asn1/rrc.h"

#include "Phy.h"

/**
 *  Simple RRC component between PHY and RLC.
 *
 *  Handles only reception of SIB1 (incl. SIB13) in CAS, and MCCH PDUs.
 */
class Rrc : public srsue::rrc_interface_rlc, public srsue::rrc_interface_pdcp {
 public:
    /**
     *  Default constructor.
     *
     *  @param cfg Config singleton reference
     *  @param rlc RLC reference
     *  @param rlc PHY reference
     */
    Rrc(const libconfig::Config& cfg, Phy& phy, srsran::rlc& rlc)
      : _cfg(cfg)
      , _rlc(rlc)
      , _phy(phy) {}
    virtual ~Rrc() = default;

    void max_retx_attempted() override {}; // Unused

    typedef enum {
      ACQUIRE_SIB,
      ACQUIRE_AREA_CONFIG,
      STREAMING
    } rrc_state_t;
    rrc_state_t state() { return _state; }
    void reset() { _state = ACQUIRE_SIB;  }


    /**
     *  Handle a MCH PDU. 
     *
     *  Automatically creates MRB bearers for all discovered LCIDs, and sets the MBSFN configuration
     *  in PHY.
     */
    void write_pdu_mch(uint32_t lcid, srsran::unique_byte_buffer_t pdu) override;

    /**
     *  Handle SIB1(SIB13) from BCCH/DLSCH, and set the scheduling info in PHY.
     */
    void write_pdu_bcch_dlsch(srsran::unique_byte_buffer_t pdu) override;
    void write_pdu(uint32_t lcid, srsran::unique_byte_buffer_t pdu) override {}; // Unused
    void write_pdu_bcch_bch(srsran::unique_byte_buffer_t pdu) override {}; // Unused
    void write_pdu_pcch(srsran::unique_byte_buffer_t pdu) override {}; // Unused
    const char* get_rb_name(uint32_t lcid) override { return "RB" + lcid; };
    void protocol_failure() override {};
    void notify_pdcp_integrity_error(uint32_t lcid) override {};

 private:
    void handle_sib1(const asn1::rrc::sib_type1_mbms_r14_s& sib1);
    rrc_state_t _state = ACQUIRE_SIB;

    const libconfig::Config& _cfg;
    srsran::rlc& _rlc;
    Phy& _phy;
};
