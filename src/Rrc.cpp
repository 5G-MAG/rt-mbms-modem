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

#include "Rrc.h"
#include "spdlog/spdlog.h"
#include "srslte/asn1/rrc_asn1_utils.h"

using asn1::rrc::mcch_msg_type_c;
using asn1::rrc::bcch_dl_sch_msg_mbms_s;
using asn1::rrc::bcch_dl_sch_msg_s;
using asn1::rrc::bcch_dl_sch_msg_type_mbms_r14_c;
using asn1::rrc::sib_type1_mbms_r14_s;
using asn1::rrc::sib_type_mbms_r14_e;
using asn1::rrc::sched_info_mbms_r14_s;
using asn1::rrc::sys_info_r8_ies_s;
using asn1::rrc::sib_info_item_c;

void Rrc::write_pdu_mch(uint32_t /*lcid*/, srslte::unique_byte_buffer_t pdu) {
  spdlog::trace("rrc: write_pdu_mch");
  if (pdu->N_bytes <= 0 || pdu->N_bytes >= SRSLTE_MAX_BUFFER_SIZE_BITS) {
    return;
  }
  asn1::cbit_ref bref(pdu->msg, pdu->N_bytes);
  asn1::rrc::mcch_msg_s msg;
  if (msg.unpack(bref) != asn1::SRSASN_SUCCESS ||
      msg.msg.type().value != mcch_msg_type_c::types_opts::c1) {
    spdlog::error("Failed to unpack MCCH message");
    return;
  }
  asn1::json_writer json_writer;
  msg.to_json(json_writer);
  spdlog::debug("BCCH-DLSCH message content:\n{}", json_writer.to_string());

  srslte::mcch_msg_t mcch = srslte::make_mcch_msg(msg);

  // add bearers for all LCIDs
  for (uint32_t i = 0; i < mcch.nof_pmch_info; i++) {
    for (uint32_t j = 0; j < mcch.pmch_info_list[i].nof_mbms_session_info; j++) {
      uint32_t lcid = mcch.pmch_info_list[i].mbms_session_info_list[j].lc_ch_id;
      if (!_rlc.has_bearer_mrb(i, lcid)) {
        _rlc.add_bearer_mrb(i, lcid);
      }
    }
  }

  _phy.set_mbsfn_config(mcch);
  _phy.set_decode_mcch(false);
  _state = STREAMING;
}

void Rrc::write_pdu_bcch_dlsch(srslte::unique_byte_buffer_t pdu) {
  // Stop BCCH search after successful reception of 1 BCCH block
  // mac->bcch_stop_rx();

  bcch_dl_sch_msg_mbms_s dlsch_msg;
  asn1::cbit_ref    dlsch_bref(pdu->msg, pdu->N_bytes);
  asn1::SRSASN_CODE err = dlsch_msg.unpack(dlsch_bref);

  if (err != asn1::SRSASN_SUCCESS || dlsch_msg.msg.type().value != bcch_dl_sch_msg_type_mbms_r14_c::types_opts::c1) {
    spdlog::debug("Could not unpack BCCH DL-SCH MBMS message ({} B), trying as BCCH DL-SCH.", pdu->N_bytes);

    bcch_dl_sch_msg_s dlsch_msg1;
    asn1::cbit_ref    dlsch_bref(pdu->msg, pdu->N_bytes);
    asn1::SRSASN_CODE err = dlsch_msg1.unpack(dlsch_bref);

    asn1::json_writer json_writer;
    dlsch_msg1.to_json(json_writer);
    spdlog::debug("BCCH-DLSCH message content:\n{}", json_writer.to_string());
    return;
  }

  asn1::json_writer json_writer;
  dlsch_msg.to_json(json_writer);
  spdlog::debug("BCCH-DLSCH MBMS message content:\n{}", json_writer.to_string());

  if (dlsch_msg.msg.c1().type() == bcch_dl_sch_msg_type_mbms_r14_c::c1_c_::types::sib_type1_mbms_r14) {
    spdlog::debug("Processing SIB1-MBMS (1/1)");
    handle_sib1(dlsch_msg.msg.c1().sib_type1_mbms_r14());
  } else {
    sys_info_r8_ies_s::sib_type_and_info_l_& sib_list =
        dlsch_msg.msg.c1().sys_info_mbms_r14().crit_exts.sys_info_r8().sib_type_and_info;
    for (uint32_t i = 0; i < sib_list.size(); ++i) {
      switch (sib_list[i].type().value) {
        case sib_info_item_c::types::sib2:
          spdlog::debug("Handling SIB2\n");
          //handle_sib2();
          break;
        case sib_info_item_c::types::sib13_v920:
          spdlog::debug("Handling SIB13\n");
          _phy.set_mch_scheduling_info( srslte::make_sib13(sib_list[i].sib13_v920()));
          if (!_rlc.has_bearer_mrb(0)) {
            _rlc.add_bearer_mrb(0);
          }
  _phy.set_decode_mcch(true);
  _state = ACQUIRE_AREA_CONFIG;
          //handle_sib13();
          break;
        default:
          spdlog::debug("SIB{} is not supported\n", sib_list[i].type().to_number());
      }
    }
  }
}

void Rrc::handle_sib1(const sib_type1_mbms_r14_s& sib1) {
  spdlog::debug("SIB1-MBMS received, si_window={}",
                sib1.si_win_len_r14.to_number());

  // Print SIB scheduling info
  for (auto& i : sib1.sched_info_list_mbms_r14) {
    sched_info_mbms_r14_s::si_periodicity_r14_e_ p = i.si_periodicity_r14;
    for (auto t : i.sib_map_info_r14) {
      spdlog::info("SIB scheduling info, sib_type={}, si_periodicity={}",
                   t.to_number(), p.to_number());
    }
  }

  _phy.set_mch_scheduling_info( srslte::make_sib13(sib1.sib_type13_r14));
  if (!_rlc.has_bearer_mrb(0, 0)) {
    _rlc.add_bearer_mrb(0, 0);
  }

  _phy.set_decode_mcch(true);
  _state = ACQUIRE_AREA_CONFIG;
}

