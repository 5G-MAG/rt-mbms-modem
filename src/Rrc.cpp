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

#include <cstdio>

#include "Rrc.h"
#include "spdlog/fmt/fmt.h"
#include "spdlog/spdlog.h"
#include "srsran/asn1/rrc_utils.h"

using asn1::rrc::mcch_msg_type_c;
using asn1::rrc::bcch_dl_sch_msg_mbms_s;
using asn1::rrc::bcch_dl_sch_msg_s;
using asn1::rrc::bcch_dl_sch_msg_type_mbms_r14_c;
using asn1::rrc::sib_type1_mbms_r14_s;
using asn1::rrc::sib_type_mbms_r14_e;
using asn1::rrc::sched_info_mbms_r14_s;
using asn1::rrc::sys_info_r8_ies_s;
using asn1::rrc::sib_info_item_c;

void Rrc::write_pdu_mch(uint32_t /*lcid*/, srsran::unique_byte_buffer_t pdu) {
  spdlog::trace("rrc: write_pdu_mch");
  if (getenv("PMCH_TI_DIAG")) {
    char hex[64] = {0};
    for (uint32_t i = 0; i < pdu->N_bytes && i < 16; i++) {
      snprintf(hex + i * 2, 3, "%02x", pdu->msg[i]);
    }
    fprintf(stderr, "TI_DIAG_RRC_WPM N_bytes=%u first16=%s\n", pdu->N_bytes, hex);
  }
  if (pdu->N_bytes <= 0 || pdu->N_bytes >= SRSRAN_MAX_BUFFER_SIZE_BITS) {
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

  srsran::mcch_msg_t mcch = srsran::make_mcch_msg(msg);

  // Remove bearers that exist in the previous MCCH but are absent in the new one.
  if (_state == STREAMING) {
    const srsran::mcch_msg_t& prev = _phy.current_mcch();
    for (uint32_t i = 0; i < prev.nof_pmch_info; i++) {
      for (uint32_t j = 0; j < prev.pmch_info_list[i].nof_mbms_session_info; j++) {
        uint32_t old_lcid = prev.pmch_info_list[i].mbms_session_info_list[j].lc_ch_id;
        bool found = false;
        if (i < mcch.nof_pmch_info) {
          for (uint32_t k = 0; k < mcch.pmch_info_list[i].nof_mbms_session_info; k++) {
            if (mcch.pmch_info_list[i].mbms_session_info_list[k].lc_ch_id == old_lcid) {
              found = true;
              break;
            }
          }
        }
        if (!found && _rlc.has_bearer_mrb(i, old_lcid)) {
          spdlog::info("rrc: removing stale RLC MRB pmch={} lcid={}", i, old_lcid);
          _rlc.del_bearer_mrb(i, old_lcid);
        }
      }
    }
  }

  // Add bearers for all LCIDs in the new MCCH.
  if (getenv("PMCH_TI_DIAG")) {
    fprintf(stderr, "TI_DIAG_ADDBEARER nof_pmch_info=%u\n", mcch.nof_pmch_info);
    for (uint32_t i = 0; i < mcch.nof_pmch_info; i++) {
      fprintf(stderr, "TI_DIAG_ADDBEARER  pmch[%u].nof_mbms_session_info=%u\n", i,
              mcch.pmch_info_list[i].nof_mbms_session_info);
    }
  }
  for (uint32_t i = 0; i < mcch.nof_pmch_info; i++) {
    for (uint32_t j = 0; j < mcch.pmch_info_list[i].nof_mbms_session_info; j++) {
      uint32_t lcid = mcch.pmch_info_list[i].mbms_session_info_list[j].lc_ch_id;
      if (getenv("PMCH_TI_DIAG")) {
        fprintf(stderr, "TI_DIAG_ADDBEARER  session[%u] lcid=%u has_bearer=%d\n", j, lcid,
                (int)_rlc.has_bearer_mrb(i, lcid));
      }
      if (!_rlc.has_bearer_mrb(i, lcid)) {
        _rlc.add_bearer_mrb(i, lcid);
      }
    }
  }

  if (_state == STREAMING) {
    /* Detect MCCH content changes; log any difference at info level. */
    const srsran::mcch_msg_t& prev = _phy.current_mcch();
    bool changed = false;
    for (uint32_t i = 0; i < mcch.nof_pmch_info && i < prev.nof_pmch_info; i++) {
      const auto& cur  = mcch.pmch_info_list[i];
      const auto& old  = prev.pmch_info_list[i];
      if (cur.sf_alloc_end        != old.sf_alloc_end        ||
          cur.mch_sched_period    != old.mch_sched_period    ||
          cur.data_mcs            != old.data_mcs            ||
          cur.time_interleaving_n != old.time_interleaving_n ||
          cur.time_interleaving_m != old.time_interleaving_m ||
          cur.cyclic_shift_alpha  != old.cyclic_shift_alpha  ||
          cur.freq_interleaving   != old.freq_interleaving   ||
          cur.use_mcs_table2      != old.use_mcs_table2) {
        changed = true;
        break;
      }
    }
    if (changed || mcch.nof_pmch_info != prev.nof_pmch_info) {
      spdlog::info("MCCH changed — applying updated PMCH parameters");
    } else {
      spdlog::debug("MCCH re-read: no change detected");
    }
  }

  _phy.set_mbsfn_config(mcch);
  _phy.set_decode_mcch(false);
  _state = STREAMING;
}

void Rrc::write_pdu_bcch_dlsch(srsran::unique_byte_buffer_t pdu) {
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
    for (auto& sib : sib_list) {
      switch (sib.type().value) {
        case sib_info_item_c::types::sib2:
          spdlog::debug("Handling SIB2\n");
          //handle_sib2();
          break;
        case sib_info_item_c::types::sib13_v920: {
          spdlog::debug("Handling SIB13\n");
          const auto& sib13 = sib.sib13_v920();
          _phy.set_mch_scheduling_info(srsran::make_sib13(sib13));
          if (!_rlc.has_bearer_mrb(0, 0)) {
            _rlc.add_bearer_mrb(0, 0);
          }
          _phy.set_decode_mcch(true);
          _state = ACQUIRE_AREA_CONFIG;
          if (sib13.mbms_rom_info_list_r16_present && sib13.mbms_rom_info_list_r16.size() > 0) {
            for (const auto& ri : sib13.mbms_rom_info_list_r16) {
              spdlog::info("MBMS-ROM-Info-r16: EARFCN={} BW={}PRB{}", ri.rom_freq_r16, ri.bw_r16.to_number(),
                           ri.subcarrier_spacing_r16_present
                               ? fmt::format(" SCS={}kHz", ri.subcarrier_spacing_r16.to_number())
                               : std::string(""));
            }
            const auto& ri0 = sib13.mbms_rom_info_list_r16[0];
            _phy.set_rom_redirect(ri0.rom_freq_r16, ri0.bw_r16.to_number());
          }
          break;
        }
        case sib_info_item_c::types::sib15_v1130: {
          const auto& sib15 = sib.sib15_v1130();
          if (sib15.mbms_sai_intra_freq_r11_present) {
            for (const auto& sai : sib15.mbms_sai_intra_freq_r11) {
              spdlog::info("SIB15: intra-freq MBMS-SAI={}", sai);
            }
          }
          if (sib15.mbms_sai_inter_freq_list_r11_present) {
            for (const auto& entry : sib15.mbms_sai_inter_freq_list_r11) {
              for (const auto& sai : entry.mbms_sai_list_r11) {
                spdlog::info("SIB15: inter-freq EARFCN={} MBMS-SAI={}", entry.dl_carrier_freq_r11, sai);
              }
            }
          }
          break;
        }
        case sib_info_item_c::types::sib16_v1130: {
          const auto& sib16 = sib.sib16_v1130();
          if (sib16.time_info_r11_present) {
            const auto& ti = sib16.time_info_r11;
            /* time_info_utc_r11 is a 48-bit GPS epoch count in units of 10 ms (TS 36.331 §6.3.4).
             * Maximum valid value: 2^48-1 ≈ 281 trillion; check for the common sentinel 0. */
            if (ti.time_info_utc_r11 == 0) {
              spdlog::warn("SIB16: time_info_utc_r11 is zero — ignoring GPS time");
            } else {
              uint64_t gps_10ms = ti.time_info_utc_r11;
              spdlog::info("SIB16: GPS time {} * 10ms ({}s since GPS epoch)", gps_10ms, gps_10ms / 100);
              if (ti.leap_seconds_r11_present) {
                spdlog::info("SIB16: UTC-GPS leap seconds = {}", ti.leap_seconds_r11);
              }
              if (ti.local_time_offset_r11_present) {
                spdlog::info("SIB16: local time offset = {} * 15min", ti.local_time_offset_r11);
              }
            }
          }
          break;
        }
        default:
          spdlog::debug("SIB{} is not supported\n", sib.type().to_number());
      }
    }
  }
}

void Rrc::handle_sib1(const sib_type1_mbms_r14_s& sib1) {
  spdlog::debug("SIB1-MBMS received, si_window={}",
                sib1.si_win_len_r14.to_number());

  // Detect SI schedule changes via sys_info_value_tag_r14 (TS 36.331 §5.2.1.2)
  uint8_t current_tag = sib1.sys_info_value_tag_r14;
  if (_last_sys_info_value_tag != kValueTagUnset && _last_sys_info_value_tag != current_tag) {
    spdlog::warn("SIB1-MBMS: sys_info_value_tag changed {} → {} — SI schedule may have changed, restart recommended",
                 _last_sys_info_value_tag, current_tag);
  }
  _last_sys_info_value_tag = current_tag;

  // Log cell identity (PLMN, TAC, cell ID)
  const auto& cai = sib1.cell_access_related_info_r14;
  for (const auto& plmn : cai.plmn_id_list_r14) {
    if (plmn.mcc_present) {
      spdlog::info("SIB1-MBMS: PLMN MCC={}{}{} MNC={}{}{}", plmn.mcc[0], plmn.mcc[1], plmn.mcc[2],
                   plmn.mnc[0], plmn.mnc[1], plmn.mnc.size() > 2 ? std::to_string(plmn.mnc[2]) : "");
    } else {
      spdlog::info("SIB1-MBMS: PLMN MNC={}{}{}", plmn.mnc[0], plmn.mnc[1],
                   plmn.mnc.size() > 2 ? std::to_string(plmn.mnc[2]) : "");
    }
  }
  spdlog::info("SIB1-MBMS: TAC=0x{:04x} CellID=0x{:07x}", cai.tac_r14.to_number(),
               cai.cell_id_r14.to_number());

  // Print SIB scheduling info
  for (auto& i : sib1.sched_info_list_mbms_r14) {
    sched_info_mbms_r14_s::si_periodicity_r14_e_ p = i.si_periodicity_r14;
    for (auto t : i.sib_map_info_r14) {
      spdlog::info("SIB scheduling info, sib_type={}, si_periodicity={}",
                   t.to_number(), p.to_number());
    }
  }

  if (sib1.non_crit_ext_present && sib1.non_crit_ext.cas_muting_cfg_r19_present) {
    const auto& cas = sib1.non_crit_ext.cas_muting_cfg_r19;
    _phy.set_cas_muting(true, cas.k_cas_r19, cas.n_cas_r19.to_number());
    spdlog::info("CAS muting configured: KCAS={} NCAS={}", cas.k_cas_r19, cas.n_cas_r19.to_number());
  }

  const auto& sib13 = sib1.sib_type13_r14;
  _phy.set_mch_scheduling_info(srsran::make_sib13(sib13));
  if (!_rlc.has_bearer_mrb(0, 0)) {
    _rlc.add_bearer_mrb(0, 0);
  }
  if (sib13.mbms_rom_info_list_r16_present && sib13.mbms_rom_info_list_r16.size() > 0) {
    for (const auto& ri : sib13.mbms_rom_info_list_r16) {
      spdlog::info("MBMS-ROM-Info-r16: EARFCN={} BW={}PRB{}", ri.rom_freq_r16, ri.bw_r16.to_number(),
                   ri.subcarrier_spacing_r16_present
                       ? fmt::format(" SCS={}kHz", ri.subcarrier_spacing_r16.to_number())
                       : std::string(""));
    }
    const auto& ri0 = sib13.mbms_rom_info_list_r16[0];
    _phy.set_rom_redirect(ri0.rom_freq_r16, ri0.bw_r16.to_number());
  }

  _phy.set_decode_mcch(true);
  _state = ACQUIRE_AREA_CONFIG;
}

