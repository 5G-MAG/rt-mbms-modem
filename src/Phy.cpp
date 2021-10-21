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

#include "Phy.h"

#include <utility>
#include <iomanip>

#include "spdlog/spdlog.h"
#include "srslte/asn1/rrc_asn1_utils.h"

static auto receive_callback(void* obj, cf_t* data[SRSLTE_MAX_CHANNELS],         // NOLINT
                             uint32_t nsamples, srslte_timestamp_t* rx_time)
    -> int {
  return (static_cast<Phy*>(obj))->_sample_cb(data[0], nsamples, rx_time);       // NOLINT
}

const uint32_t kMaxBufferSamples = 2 * 15360;
const uint32_t kMaxSfn = 1024;
const uint32_t kSfnOffset = 4;
const uint32_t kSubframesPerFrame = 10;

const uint32_t kMaxCellsToDiscover = 3;

Phy::Phy(const libconfig::Config& cfg, get_samples_t cb, uint8_t cs_nof_prb,
         int8_t override_nof_prb)
    : _cfg(cfg),
      _sample_cb(std::move(std::move(cb))),
      _cs_nof_prb(cs_nof_prb),
      _override_nof_prb(override_nof_prb) {
  _buffer_max_samples = kMaxBufferSamples;
  _mib_buffer[0] = static_cast<cf_t*>(malloc(_buffer_max_samples * sizeof(cf_t)));  // NOLINT
}

Phy::~Phy() {
  srslte_ue_sync_free(&_ue_sync);
  free(_mib_buffer[0]);  // NOLINT
}

auto Phy::synchronize_subframe() -> bool {

  int ret = srslte_ue_sync_zerocopy(&_ue_sync, _mib_buffer, _buffer_max_samples);  // NOLINT
  if (ret < 0) {
    spdlog::error("SYNC:  Error calling ue_sync_get_buffer.\n");
    return false;
  }

  if (ret == 1) {
    std::array<uint8_t, SRSLTE_BCH_PAYLOAD_LEN> bch_payload = {};
    if (srslte_ue_sync_get_sfidx(&_ue_sync) == 0) {
      int sfn_offset = 0;
      int n =
          srslte_ue_mib_decode(&_mib, bch_payload.data(), nullptr, &sfn_offset);
      if (n == 1) {
        uint32_t sfn = 0;
        srslte_pbch_mib_mbms_unpack(bch_payload.data(), &_cell, &sfn, nullptr,
                                    _override_nof_prb);
        sfn = (sfn + sfn_offset * kSfnOffset) % kMaxSfn;
        _tti = sfn * kSubframesPerFrame;
        return true;
      }
    }
  }
  return false;
}

auto Phy::cell_search() -> bool {
  std::array<srslte_ue_cellsearch_result_t, kMaxCellsToDiscover> found_cells = {0};

  uint32_t max_peak_cell = 0;
  int ret = srslte_ue_cellsearch_scan(&_cell_search, found_cells.data(), &max_peak_cell);
  if (ret < 0) {
    spdlog::error("Phy: Error decoding MIB: Error searching PSS\n");
    return false;
  }
  if (ret == 0) {
    spdlog::error("Phy: Could not find any cell in this frequency\n");
    return false;
  }

  srslte_cell_t new_cell = {};
  new_cell.id         = found_cells.at(max_peak_cell).cell_id;
  new_cell.cp         = found_cells.at(max_peak_cell).cp;
  new_cell.frame_type = found_cells.at(max_peak_cell).frame_type;
  new_cell.mbms_dedicated = true;
  float cfo           = found_cells.at(max_peak_cell).cfo;

  spdlog::info("Phy: PSS/SSS detected: Mode {}, PCI {}, CFO {} KHz, CP {}",
               new_cell.frame_type != 0U ? "TDD" : "FDD", new_cell.id,
               cfo / 1000, srslte_cp_string(new_cell.cp));

  if (srslte_ue_mib_sync_set_cell_prb(&_mib_sync, new_cell, _cs_nof_prb) != 0) {
    spdlog::error("Phy: Error setting UE MIB sync cell");
    return false;
  }

  srslte_ue_sync_reset(&_mib_sync.ue_sync);

  std::array<uint8_t, SRSLTE_BCH_PAYLOAD_LEN> bch_payload = {};
  /* Find and decode MIB */
  int sfn_offset = 0;
  ret = srslte_ue_mib_sync_decode_prb(&_mib_sync, 40, bch_payload.data(), &new_cell.nof_ports, &sfn_offset, _cs_nof_prb);
  if (ret == 1) {
    uint32_t sfn = 0;
    srslte_pbch_mib_mbms_unpack(bch_payload.data(), &new_cell, &sfn, nullptr,
                                _override_nof_prb);

    spdlog::info(
        "Phy: MIB Decoded. Mode {}, PCI {}, PRB {}, Ports {}, CFO {} KHz, SFN "
        "{}\n",
        new_cell.frame_type != 0u ? "TDD" : "FDD", new_cell.id,
        new_cell.nof_prb, new_cell.nof_ports, cfo / 1000, sfn);

    if (!srslte_cell_isvalid(&new_cell)) {
      spdlog::error("SYNC:  Detected invalid cell.\n");
      return false;
    }

    _cell = new_cell;
    _cell.mbsfn_prb = _cell.nof_prb;

    if (srslte_ue_sync_set_cell(&_ue_sync, cell()) != 0) {
      spdlog::error("Phy: failed to set cell.\n");
      return false;
    }
    if (srslte_ue_mib_set_cell(&_mib, cell()) != 0) {
      spdlog::error("Phy: Error setting UE MIB cell");
      return false;
    }

    return true;
  }

  spdlog::error("Phy: failed to receive MIB\n");
  return false;
}

auto Phy::set_cell() -> void {
    if (srslte_ue_sync_set_cell(&_ue_sync, cell()) != 0) {
      spdlog::error("Phy: failed to set cell.\n");
    }
    if (srslte_ue_mib_set_cell(&_mib, cell()) != 0) {
      spdlog::error("Phy: Error setting UE MIB cell");
    }
}

auto Phy::init() -> bool {
  if (srslte_ue_cellsearch_init_multi_prb_cp(&_cell_search, 8, receive_callback, 1,
                                      this, _cs_nof_prb, true) != 0) {
    spdlog::error("Phy: error while initiating UE cell search\n");
    return false;
  }
  srslte_ue_cellsearch_set_nof_valid_frames(&_cell_search, 4);

  if (srslte_ue_sync_init_multi(&_ue_sync, MAX_PRB, false, receive_callback, 1,
                                this) != 0) {
    spdlog::error("Cannot init ue_sync");
    return false;
  }

  if (srslte_ue_mib_sync_init_multi_prb(&_mib_sync, receive_callback, 1, this,
                                    _cs_nof_prb) != 0) {
    spdlog::error("Cannot init ue_mib_sync");
    return false;
  }

  if (srslte_ue_mib_init(&_mib, _mib_buffer[0], 50) != 0) {
    spdlog::error("Cannot init ue_mib");
    return false;
  }

  return true;
}

auto Phy::get_next_frame(cf_t** buffer, uint32_t size) -> bool {
  return 1 == srslte_ue_sync_zerocopy(&_ue_sync, buffer, size);
}

void Phy::set_mch_scheduling_info(const srslte::sib13_t& sib13) {
  if (sib13.nof_mbsfn_area_info > 1) {
    spdlog::warn("SIB13 has {} MBSFN area info elements - only 1 supported", sib13.nof_mbsfn_area_info);
  }

  if (sib13.mbsfn_area_info_list[0].pmch_bandwidth != 0) {
    _cell.mbsfn_prb = sib13.mbsfn_area_info_list[0].pmch_bandwidth;
  }

  if (sib13.nof_mbsfn_area_info > 0) {
    _sib13 = sib13;

    bzero(&_mcch_table[0], sizeof(uint8_t) * 10);
    if (sib13.mbsfn_area_info_list[0].mcch_cfg.sf_alloc_info_is_r16) {
      generate_mcch_table_r16(
          &_mcch_table[0],
          static_cast<uint32_t>(
            sib13.mbsfn_area_info_list[0].mcch_cfg.sf_alloc_info));
    } else {
      generate_mcch_table(
          &_mcch_table[0],
          static_cast<uint32_t>(
            sib13.mbsfn_area_info_list[0].mcch_cfg.sf_alloc_info));
    }

    std::stringstream ss;
    ss << "|";
    for (unsigned char j : _mcch_table) {
      ss << static_cast<int>(j) << "|";
    }
    spdlog::debug("MCCH table: {}", ss.str());

    _mcch_configured = true;
  }
}

void Phy::set_mbsfn_config(const srslte::mcch_msg_t& mcch) {
  _mcch = mcch;
  _mch_configured = true;

  _mch_info.clear();
  for (uint32_t i = 0; i < _mcch.nof_pmch_info; i++) {
    mch_info_t mch_info;
    mch_info.mcs = _mcch.pmch_info_list[i].data_mcs;

    for (uint32_t j = 0; j < _mcch.pmch_info_list[i].nof_mbms_session_info; j++) {
      mtch_info_t mtch_info;
      mtch_info.lcid = _mcch.pmch_info_list[i].mbms_session_info_list[j].lc_ch_id;
      char tmgi[20]; // NOLINT
      /* acc to  TS24.008 10.5.6.13:
       * MCC 1,2,3: 901 ->   9, 0, 1
       * MNC 3,1,2:  56 -> (F), 5, 6
       * HEX 0x09F165
       *
       * -------------+-------------+---------
       * MCC digit 2  | MCC digit 1 | Octet 6*
       * -------------+-------------+---------
       * MNC digit 3  | MCC digit 3 | Octet 7*
       * -------------+-------------+---------
       * MNC digit 2  | MNC digit 1 | Octet 8*
       * -------------+-------------+---------
       */
      sprintf (tmgi, "%06x%02x%02x%02x",
         _mcch.pmch_info_list[i].mbms_session_info_list[j].tmgi.serviced_id[2] |
         _mcch.pmch_info_list[i].mbms_session_info_list[j].tmgi.serviced_id[1] << 8 |
         _mcch.pmch_info_list[i].mbms_session_info_list[j].tmgi.serviced_id[0] << 16 , 
         _mcch.pmch_info_list[i].mbms_session_info_list[j].tmgi.plmn_id.explicit_value.mcc[1] << 4 | mcch.pmch_info_list[i].mbms_session_info_list[j].tmgi.plmn_id.explicit_value.mcc[0],
         ( _mcch.pmch_info_list[i].mbms_session_info_list[j].tmgi.plmn_id.explicit_value.nof_mnc_digits == 2 ? 0xF : _mcch.pmch_info_list[i].mbms_session_info_list[j].tmgi.plmn_id.explicit_value.mnc[2] ) << 4 | _mcch.pmch_info_list[i].mbms_session_info_list[j].tmgi.plmn_id.explicit_value.mcc[2] ,
         _mcch.pmch_info_list[i].mbms_session_info_list[j].tmgi.plmn_id.explicit_value.mnc[1] << 4 | _mcch.pmch_info_list[i].mbms_session_info_list[j].tmgi.plmn_id.explicit_value.mnc[0]
         );
      mtch_info.tmgi = tmgi;
      mtch_info.dest = _dests[mch_info.mcs][mtch_info.lcid];
      mch_info.mtchs.push_back(mtch_info);
    }

    _mch_info.push_back(mch_info);
  }
}

auto Phy::mbsfn_config_for_tti(uint32_t tti, unsigned& area)
    -> srslte_mbsfn_cfg_t {
  srslte_mbsfn_cfg_t cfg;
  cfg.enable                  = false;
  cfg.is_mcch                 = false;

  if (!_mcch_configured) {
    {
      return cfg;
    }
  }

  uint32_t sfn = tti / 10;
  uint8_t sf = tti % 10;

  srslte::mbsfn_area_info_t& area_info = _sib13.mbsfn_area_info_list[0];

  cfg.mbsfn_area_id = area_info.mbsfn_area_id;
  cfg.non_mbsfn_region_length = enum_to_number(area_info.non_mbsfn_region_len);

  if (sfn % enum_to_number(area_info.mcch_cfg.mcch_repeat_period) == area_info.mcch_cfg.mcch_offset &&
      _mcch_table[sf] == 1) {
    // MCCH
    if (_decode_mcch) {
      cfg.mbsfn_mcs               = enum_to_number(area_info.mcch_cfg.sig_mcs);
      cfg.enable                  = true;
      cfg.is_mcch                 = true;
    }
  } else {
    if (_mch_configured) {
      cfg.mbsfn_area_id = area_info.mbsfn_area_id;

      for (uint32_t i = 0; i < _mcch.nof_pmch_info; i++) {
        unsigned fn_in_scheduling_period =  sfn % enum_to_number(_mcch.pmch_info_list[i].mch_sched_period);
        unsigned sf_idx = fn_in_scheduling_period * 10 + sf - (fn_in_scheduling_period / 4) - 1;

        spdlog::debug("tti {}, fn_in_ {}, sf_idx {}", tti, fn_in_scheduling_period,  sf_idx);

        if (sf_idx <= _mcch.pmch_info_list[i].sf_alloc_end) {
          area = i;
          if ((i == 0 && fn_in_scheduling_period == 0 && sf == 1) ||
              (i > 0 && _mcch.pmch_info_list[i-1].sf_alloc_end + 1 == sf_idx)) {
            spdlog::debug("assigning sig_mcs {}",  area_info.mcch_cfg.sig_mcs);
            cfg.mbsfn_mcs = enum_to_number(area_info.mcch_cfg.sig_mcs);
          } else {
            spdlog::debug("assigning pmch_mcs {}",  area_info.mcch_cfg.sig_mcs);
            cfg.mbsfn_mcs = _mcch.pmch_info_list[i].data_mcs;
          }
          cfg.enable = true;
          break;
        }
      }
    }
  }
  return cfg;
}
