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

#include <chrono>
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
using asn1::rrc::sib_type12_r9_s;

namespace {

/* Reverse of rt-mbms-tx's tools/encode_sib12_alert.py: TS 23.038 GSM 7-bit
 * default alphabet, packed 8 septets per 7 octets. Only the same subset the
 * encoder tool supports is mapped back; anything outside that table decodes
 * to '?', matching the encoder's own fallback so round-tripping is exact for
 * every character the encoder can actually produce.
 *
 * One entry per septet code (0..127), NOT a single flat byte string: several
 * entries (£, è, Δ, etc.) are multi-byte in UTF-8, so indexing a flat
 * "const char*" table by septet code silently reads the wrong byte the
 * moment any earlier entry is multi-byte - caught via a round-trip test
 * against the encoder tool's own output before this array form was used.
 * Bytes given as \x escapes (not literal UTF-8 source text or \u) so this is
 * unambiguous regardless of source-file/compiler charset handling. */
const char* kGsm7Basic[128] = {
    "@", "\xc2\xa3", "$", "\xc2\xa5", "\xc3\xa8", "\xc3\xa9", "\xc3\xb9", "\xc3\xac",
    "\xc3\xb2", "\xc3\x87", "\n", "\xc3\x98", "\xc3\xb8", "\r", "\xc3\x85", "\xc3\xa5",
    "\xce\x94", "_", "\xce\xa6", "\xce\x93", "\xce\x9b", "\xce\xa9", "\xce\xa0", "\xce\xa8",
    "\xce\xa3", "\xce\x98", "\xce\x9e", "\x1b", "?", "?", "?", "?",
    " ", "!", "\"", "#", "\xc2\xa4", "%", "&", "'",
    "(", ")", "*", "+", ",", "-", ".", "/",
    "0", "1", "2", "3", "4", "5", "6", "7",
    "8", "9", ":", ";", "<", "=", ">", "?",
    "\xc2\xa1", "A", "B", "C", "D", "E", "F", "G",
    "H", "I", "J", "K", "L", "M", "N", "O",
    "P", "Q", "R", "S", "T", "U", "V", "W",
    "X", "Y", "Z", "\xc3\x84", "\xc3\x96", "\xc3\x91", "\xc3\x9c", "\xc2\xa7",
    "\xc2\xbf", "a", "b", "c", "d", "e", "f", "g",
    "h", "i", "j", "k", "l", "m", "n", "o",
    "p", "q", "r", "s", "t", "u", "v", "w",
    "x", "y", "z", "\xc3\xa4", "\xc3\xb6", "\xc3\xb1", "\xc3\xbc", "\xc3\xa0",
};

std::string decode_gsm7(const uint8_t* data, uint32_t len) {
  std::string out;
  uint32_t bitpos = 0;
  uint32_t nbits   = len * 8;
  while (bitpos + 7 <= nbits) {
    uint32_t byte_idx = bitpos / 8;
    uint32_t offset    = bitpos % 8;
    uint32_t code;
    if (offset <= 1) {
      code = (data[byte_idx] >> offset) & 0x7F;
    } else {
      code = ((data[byte_idx] >> offset) | (data[byte_idx + 1] << (8 - offset))) & 0x7F;
    }
    out += kGsm7Basic[code];
    bitpos += 7;
  }
  return out;
}

std::string decode_ucs2(const uint8_t* data, uint32_t len) {
  /* UTF-16-BE -> UTF-8, restricted to the BMP (no surrogate pairs) - matches
   * the encoder tool, which only ever emits plain text.encode('utf-16-be'). */
  std::string out;
  for (uint32_t i = 0; i + 1 < len; i += 2) {
    uint32_t cp = ((uint32_t)data[i] << 8) | data[i + 1];
    if (cp < 0x80) {
      out += (char)cp;
    } else if (cp < 0x800) {
      out += (char)(0xC0 | (cp >> 6));
      out += (char)(0x80 | (cp & 0x3F));
    } else {
      out += (char)(0xE0 | (cp >> 12));
      out += (char)(0x80 | ((cp >> 6) & 0x3F));
      out += (char)(0x80 | (cp & 0x3F));
    }
  }
  return out;
}

/* rt-mbms-tx's tools/encode_sib12_alert.py (the only thing that produces
 * SIB12 content anywhere in this project) emits exactly two DCS byte values:
 * 0x48 for GSM 7-bit default alphabet, 0x18 for UCS-2 - verified directly
 * against that tool's own output rather than re-deriving TS 23.038's general
 * DCS bit-group table, which has several groups and is easy to mis-parse.
 * Match those two literal values; anything else falls back to GSM7 (the
 * encoder's own default) rather than guessing at an unhandled DCS group. */
constexpr uint8_t kDcsUcs2 = 0x18;

/* Mirrors mbms-control-portal's lib/cap.js ALERT_TYPES table (message
 * identifier -> CAP category) - that Node app is the actual CBE originating
 * these alerts in this project, so its table is the ground truth for what
 * msg_id values mean here, not a generic 3GPP-wide registry. */
std::string pws_alert_label(uint32_t msg_id) {
  switch (msg_id) {
    case 0x1100: return "ETWS: Earthquake";
    case 0x1102: return "ETWS: Tsunami";
    case 0x1104: return "ETWS: Test";
    case 0x1112: return "CMAS: Presidential Alert";
    case 0x1113: return "CMAS: Extreme";
    case 0x1115: return "CMAS: Severe";
    case 0x111b: return "CMAS: AMBER Alert";
    default:     return "";
  }
}

Phy::PwsAlert decode_pws_alert(const sib_type12_r9_s& sib12) {
  Phy::PwsAlert alert;
  alert.msg_id             = (uint32_t)sib12.msg_id_r9.to_number();
  alert.serial_number      = (uint32_t)sib12.serial_num_r9.to_number();
  alert.data_coding_scheme = sib12.data_coding_scheme_r9_present ? sib12.data_coding_scheme_r9.data()[0] : 0;
  alert.label               = pws_alert_label(alert.msg_id);

  const uint8_t* data = sib12.warning_msg_segment_r9.data();
  uint32_t       len   = sib12.warning_msg_segment_r9.size();
  if (alert.data_coding_scheme == kDcsUcs2) {
    alert.text = decode_ucs2(data, len);
  } else {
    alert.text = decode_gsm7(data, len);
  }
  return alert;
}

/* mbms_rom_info_list_r16 has the same field layout wherever SIB13 is reached
 * from (sib1.sib_type13_r14 vs sib.sib13_v920()), but the two paths' asn1
 * container types differ, hence the template rather than a fixed type name. */
template <typename RomInfoList>
std::vector<Phy::Sib13RomInfo> decode_rom_info(const RomInfoList& list) {
  std::vector<Phy::Sib13RomInfo> out;
  for (const auto& ri : list) {
    Phy::Sib13RomInfo r;
    r.earfcn  = ri.rom_freq_r16;
    r.bw_prb  = ri.bw_r16.to_number();
    r.has_scs = ri.subcarrier_spacing_r16_present;
    if (r.has_scs) {
      r.scs_khz = ri.subcarrier_spacing_r16.to_number();
    }
    out.push_back(r);
  }
  return out;
}

} // namespace

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
  _phy.set_mcch_received_at(std::chrono::duration_cast<std::chrono::milliseconds>(
                                 std::chrono::system_clock::now().time_since_epoch())
                                 .count());
  _state = STREAMING;
}

void Rrc::write_pdu_bcch_dlsch(srsran::unique_byte_buffer_t pdu) {
  // Stop BCCH search after successful reception of 1 BCCH block
  // mac->bcch_stop_rx();

  if (getenv("BCCH_HEXDUMP")) {
    char hex[1024] = {0};
    uint32_t n = pdu->N_bytes < 340 ? pdu->N_bytes : 340;
    for (uint32_t i = 0; i < n; i++) {
      snprintf(hex + i * 2, 3, "%02x", pdu->msg[i]);
    }
    fprintf(stderr, "BCCH_HEXDUMP N_bytes=%u bytes=%s\n", pdu->N_bytes, hex);
  }

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

  uint64_t now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::system_clock::now().time_since_epoch())
                        .count();

  if (dlsch_msg.msg.c1().type() == bcch_dl_sch_msg_type_mbms_r14_c::c1_c_::types::sib_type1_mbms_r14) {
    spdlog::debug("Processing SIB1-MBMS (1/1)");
    handle_sib1(dlsch_msg.msg.c1().sib_type1_mbms_r14(), now_ms);
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
          _phy.set_sib13_received_at(now_ms);
          if (!_rlc.has_bearer_mrb(0, 0)) {
            _rlc.add_bearer_mrb(0, 0);
          }
          _phy.set_decode_mcch(true);
          _state = ACQUIRE_AREA_CONFIG;
          break;
        }
        case sib_info_item_c::types::sib15_v1130: {
          const auto& sib15 = sib.sib15_v1130();
          Phy::Sib15Info sib15_info;
          if (sib15.mbms_sai_intra_freq_r11_present) {
            for (const auto& sai : sib15.mbms_sai_intra_freq_r11) {
              spdlog::info("SIB15: intra-freq MBMS-SAI={}", sai);
              sib15_info.intra_freq_sai.push_back(sai);
            }
          }
          if (sib15.mbms_sai_inter_freq_list_r11_present) {
            for (const auto& entry : sib15.mbms_sai_inter_freq_list_r11) {
              Phy::Sib15InterFreqSai inter;
              inter.earfcn = entry.dl_carrier_freq_r11;
              for (const auto& sai : entry.mbms_sai_list_r11) {
                spdlog::info("SIB15: inter-freq EARFCN={} MBMS-SAI={}", entry.dl_carrier_freq_r11, sai);
                inter.sai_list.push_back(sai);
              }
              sib15_info.inter_freq_sai.push_back(std::move(inter));
            }
          }
          sib15_info.last_received_at = now_ms;
          _phy.set_sib15_info(std::move(sib15_info));
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
              Phy::Sib16Info sib16_info;
              sib16_info.has_time_info = true;
              sib16_info.gps_time_10ms = gps_10ms;
              if (ti.leap_seconds_r11_present) {
                spdlog::info("SIB16: UTC-GPS leap seconds = {}", ti.leap_seconds_r11);
                sib16_info.has_leap_seconds = true;
                sib16_info.leap_seconds     = ti.leap_seconds_r11;
              }
              if (ti.local_time_offset_r11_present) {
                spdlog::info("SIB16: local time offset = {} * 15min", ti.local_time_offset_r11);
                sib16_info.has_local_time_offset   = true;
                sib16_info.local_time_offset_15min = ti.local_time_offset_r11;
              }
              sib16_info.last_received_at = now_ms;
              _phy.set_sib16_info(std::move(sib16_info));
            }
          }
          break;
        }
        case sib_info_item_c::types::sib12_v920: {
          Phy::PwsAlert alert = decode_pws_alert(sib.sib12_v920());
          spdlog::info("SIB12 (CMAS/PWS): msg_id=0x{:04x} serial=0x{:04x} {} \"{}\"", alert.msg_id,
                       alert.serial_number, alert.label.empty() ? "(unknown type)" : alert.label, alert.text);
          _phy.add_pws_alert(std::move(alert), now_ms);
          break;
        }
        default:
          spdlog::debug("SIB{} is not supported\n", sib.type().to_number());
          _phy.note_unhandled_sib(static_cast<uint8_t>(sib.type().to_number()), now_ms);
      }
    }
  }
}

void Rrc::handle_sib1(const sib_type1_mbms_r14_s& sib1, uint64_t now_ms) {
  spdlog::debug("SIB1-MBMS received, si_window={}",
                sib1.si_win_len_r14.to_number());

  Phy::Sib1Info sib1_info;

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
    Phy::Sib1Plmn p;
    if (plmn.mcc_present) {
      spdlog::info("SIB1-MBMS: PLMN MCC={}{}{} MNC={}{}{}", plmn.mcc[0], plmn.mcc[1], plmn.mcc[2],
                   plmn.mnc[0], plmn.mnc[1], plmn.mnc.size() > 2 ? std::to_string(plmn.mnc[2]) : "");
      p.mcc = fmt::format("{}{}{}", plmn.mcc[0], plmn.mcc[1], plmn.mcc[2]);
    } else {
      spdlog::info("SIB1-MBMS: PLMN MNC={}{}{}", plmn.mnc[0], plmn.mnc[1],
                   plmn.mnc.size() > 2 ? std::to_string(plmn.mnc[2]) : "");
    }
    p.mnc = fmt::format("{}{}{}", plmn.mnc[0], plmn.mnc[1], plmn.mnc.size() > 2 ? std::to_string(plmn.mnc[2]) : "");
    sib1_info.plmns.push_back(std::move(p));
  }
  spdlog::info("SIB1-MBMS: TAC=0x{:04x} CellID=0x{:07x}", cai.tac_r14.to_number(),
               cai.cell_id_r14.to_number());
  sib1_info.tac         = cai.tac_r14.to_number();
  sib1_info.cell_id     = cai.cell_id_r14.to_number();
  sib1_info.si_win_len_ms      = sib1.si_win_len_r14.to_number();
  sib1_info.sys_info_value_tag = current_tag;

  // Print SIB scheduling info
  for (auto& i : sib1.sched_info_list_mbms_r14) {
    sched_info_mbms_r14_s::si_periodicity_r14_e_ p = i.si_periodicity_r14;
    Phy::Sib1SchedInfoEntry entry;
    entry.si_periodicity_rf = p.to_number();
    for (auto t : i.sib_map_info_r14) {
      spdlog::info("SIB scheduling info, sib_type={}, si_periodicity={}",
                   t.to_number(), p.to_number());
      entry.sib_types.push_back(t.to_number());
    }
    sib1_info.sched_info.push_back(std::move(entry));
  }

  if (sib1.non_crit_ext_present && sib1.non_crit_ext.cas_muting_cfg_r19_present) {
    const auto& cas = sib1.non_crit_ext.cas_muting_cfg_r19;
    _phy.set_cas_muting(true, cas.k_cas_r19, cas.n_cas_r19.to_number());
    spdlog::info("CAS muting configured: KCAS={} NCAS={}", cas.k_cas_r19, cas.n_cas_r19.to_number());
    sib1_info.cas_muting_enabled = true;
    sib1_info.k_cas              = cas.k_cas_r19;
    sib1_info.n_cas              = cas.n_cas_r19.to_number();
  }

  sib1_info.last_received_at = now_ms;
  _phy.set_sib1_info(std::move(sib1_info));

  const auto& sib13 = sib1.sib_type13_r14;
  _phy.set_mch_scheduling_info(srsran::make_sib13(sib13));
  _phy.set_sib13_received_at(now_ms);
  if (!_rlc.has_bearer_mrb(0, 0)) {
    _rlc.add_bearer_mrb(0, 0);
  }

  _phy.set_decode_mcch(true);
  _state = ACQUIRE_AREA_CONFIG;
}

