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

#include "srsran/interfaces/rrc_interface_types.h"
#include "srsran/asn1/rrc_utils.h"
#include "spdlog/spdlog.h"

static auto receive_callback(void* obj, cf_t* data[SRSRAN_MAX_CHANNELS],         // NOLINT
                             uint32_t nsamples, srsran_timestamp_t* rx_time)
    -> int {
  return (static_cast<Phy*>(obj))->_sample_cb(data, nsamples, rx_time);       // NOLINT
}

const uint32_t kMaxBufferSamples = 2 * 15360;
const uint32_t kMaxSfn = 1024;
const uint32_t kSfnOffset = 4;
const uint32_t kSubframesPerFrame = 10;

const uint32_t kMaxCellsToDiscover = 3;

const uint32_t kMaxScannedFrames = 10;
const uint32_t kMaxValidFrames = 4;

const uint32_t kMaxFramesTimeout = 80;

Phy::Phy(const libconfig::Config& cfg, get_samples_t cb, uint8_t cs_nof_prb,
         int8_t override_nof_prb, uint8_t rx_channels)
    : _cfg(cfg),
      _sample_cb(std::move(std::move(cb))),
      _cs_nof_prb(cs_nof_prb),
      _override_nof_prb(override_nof_prb),
      _rx_channels(rx_channels) {
  _buffer_max_samples = kMaxBufferSamples;
  _mib_buffer[0] = static_cast<cf_t*>(malloc(_buffer_max_samples * sizeof(cf_t)));  // NOLINT
  _mib_buffer[1] = static_cast<cf_t*>(malloc(_buffer_max_samples * sizeof(cf_t)));  // NOLINT
}

Phy::~Phy() {
  srsran_ue_sync_free(&_ue_sync);
  free(_mib_buffer[0]);  // NOLINT
}

auto Phy::synchronize_subframe() -> bool {

  int ret = srsran_ue_sync_zerocopy(&_ue_sync, _mib_buffer, _buffer_max_samples);  // NOLINT
  if (ret < 0) {
    spdlog::error("SYNC:  Error calling ue_sync_get_buffer.\n");
    return false;
  }

  if (ret == 1) {
    std::array<uint8_t, SRSRAN_BCH_PAYLOAD_LEN> bch_payload = {};
    if (srsran_ue_sync_get_sfidx(&_ue_sync) == 0) {
      int sfn_offset = 0;
      int n =
          srsran_ue_mib_decode(&_mib, bch_payload.data(), nullptr, &sfn_offset);
      if (n == 1) {
        uint32_t sfn = 0;
        if (_cell.mbms_dedicated) {
          srsran_pbch_mib_mbms_unpack(bch_payload.data(), &_cell, &sfn, nullptr,
              _override_nof_prb);
          sfn = (sfn + sfn_offset * kSfnOffset) % kMaxSfn;
        } else {
          srsran_pbch_mib_unpack(bch_payload.data(), &_cell, &sfn);
          sfn = (sfn + sfn_offset) % kMaxSfn;
        }
        _tti =  sfn * kSubframesPerFrame;
        return true;
      }
    }
  }
  return false;
}

auto Phy::cell_search() -> bool {
  std::array<srsran_ue_cellsearch_result_t, kMaxCellsToDiscover> found_cells = {0};

  uint32_t max_peak_cell = 0;
  int ret = srsran_ue_cellsearch_scan(&_cell_search, found_cells.data(), &max_peak_cell);
  if (ret < 0) {
    spdlog::error("Phy: Error decoding MIB: Error searching PSS\n");
    return false;
  }
  if (ret == 0) {
    spdlog::error("Phy: Could not find any cell in this frequency\n");
    return false;
  }

  srsran_cell_t new_cell = {};
  new_cell.id         = found_cells.at(max_peak_cell).cell_id;
  new_cell.cp         = found_cells.at(max_peak_cell).cp;
  new_cell.frame_type = found_cells.at(max_peak_cell).frame_type;
  float cfo           = found_cells.at(max_peak_cell).cfo;

  spdlog::info("Phy: PSS/SSS detected: Mode {}, PCI {}, CFO {} KHz, CP {}",
               new_cell.frame_type != 0U ? "TDD" : "FDD", new_cell.id,
               cfo / 1000, srsran_cp_string(new_cell.cp));



  std::array<uint8_t, SRSRAN_BCH_PAYLOAD_LEN> bch_payload = {};
  /* Find and decode MIB */
  int sfn_offset = 0;

  // Try to decode MIB-MBMS
  new_cell.mbms_dedicated = true;
  if (srsran_ue_mib_sync_set_cell_prb(&_mib_sync, new_cell, _cs_nof_prb) != 0) {
    spdlog::error("Phy: Error setting UE MIB sync cell");
    return false;
  }
  srsran_ue_sync_reset(&_mib_sync.ue_sync);
  ret = srsran_ue_mib_sync_decode_prb(&_mib_sync, kMaxFramesTimeout, bch_payload.data(), &new_cell.nof_ports, &sfn_offset, _cs_nof_prb);

  if (!ret) { // MIB-MBMS failed, try to decode regular MIB
    init();
    new_cell.mbms_dedicated = false;
    if (srsran_ue_mib_sync_set_cell_prb(&_mib_sync, new_cell, _cs_nof_prb) != 0) {
      spdlog::error("Phy: Error setting UE MIB sync cell");
      return false;
    }
    srsran_ue_sync_reset(&_mib_sync.ue_sync);
    ret = srsran_ue_mib_sync_decode_prb(&_mib_sync, kMaxFramesTimeout, bch_payload.data(), &new_cell.nof_ports, &sfn_offset, _cs_nof_prb);
  }

  if (ret == 1) {
    uint32_t sfn = 0;

    if (new_cell.mbms_dedicated) {
      srsran_pbch_mib_mbms_unpack(bch_payload.data(), &new_cell, &sfn, nullptr,
          _override_nof_prb);
    } else {
      srsran_pbch_mib_unpack(bch_payload.data(), &new_cell, &sfn);
    }
    sfn = (sfn + sfn_offset) % 1024;

    spdlog::info(
        "Phy: MIB Decoded. {} cell, Mode {}, PCI {}, PRB {}, Ports {}, CFO {} KHz, SFN "
        "{}, sfn_offset {}\n",
        new_cell.mbms_dedicated ? "MBMS dedicated" : "MBMS/Unicast mixed",
        new_cell.frame_type != 0u ? "TDD" : "FDD", new_cell.id,
        new_cell.nof_prb, new_cell.nof_ports, cfo / 1000, sfn, sfn_offset);

    if (!srsran_cell_isvalid(&new_cell)) {
      spdlog::error("SYNC:  Detected invalid cell.\n");
      return false;
    }

    _cell = new_cell;
    _cell.mbsfn_prb = _cell.nof_prb;

    if (srsran_ue_sync_set_cell(&_ue_sync, cell()) != 0) {
      spdlog::error("Phy: failed to set cell.\n");
      return false;
    }
    if (srsran_ue_mib_set_cell(&_mib, cell()) != 0) {
      spdlog::error("Phy: Error setting UE MIB cell");
      return false;
    }

    return true;
  }

  spdlog::error("Phy: failed to receive MIB\n");
  return false;
}

auto Phy::set_cell() -> void {
    if (srsran_ue_sync_set_cell(&_ue_sync, cell()) != 0) {
      spdlog::error("Phy: failed to set cell.\n");
    }
    if (srsran_ue_mib_set_cell(&_mib, cell()) != 0) {
      spdlog::error("Phy: Error setting UE MIB cell");
    }
}

auto Phy::init() -> bool {
  if (srsran_ue_cellsearch_init_multi_prb_cp(&_cell_search, kMaxScannedFrames, receive_callback, _rx_channels,
                                      this, _cs_nof_prb, _search_extended_cp) != 0) {
    spdlog::error("Phy: error while initiating UE cell search\n");
    return false;
  }
  srsran_ue_cellsearch_set_nof_valid_frames(&_cell_search, kMaxValidFrames);

  if (srsran_ue_sync_init_multi(&_ue_sync, MAX_PRB, false, receive_callback, _rx_channels,
                                this) != 0) {
    spdlog::error("Cannot init ue_sync");
    return false;
  }

  if (srsran_ue_mib_sync_init_multi_prb(&_mib_sync, receive_callback, _rx_channels, this,
                                    _cs_nof_prb) != 0) {
    spdlog::error("Cannot init ue_mib_sync");
    return false;
  }

  if (srsran_ue_mib_init(&_mib, _mib_buffer[0], MAX_PRB) != 0) {
    spdlog::error("Cannot init ue_mib");
    return false;
  }

  return true;
}

auto Phy::get_next_frame(cf_t** buffer, uint32_t size) -> bool {
  return 1 == srsran_ue_sync_zerocopy(&_ue_sync, buffer, size);
}

void Phy::set_mch_scheduling_info(const srsran::sib13_t& sib13) {
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

void Phy::set_mbsfn_config(const srsran::mcch_msg_t& mcch) {
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
      mtch_info.dest = _dests[i][mtch_info.lcid];
      mch_info.mtchs.push_back(mtch_info);
    }

    _mch_info.push_back(mch_info);
  }
}

auto Phy::is_cas_subframe(unsigned tti) -> bool
{
  if (_cell.mbms_dedicated) {
    // This is subframe 0 in a radio frame divisible by 4, and hence a CAS frame. 
    return tti%40 == 0;
  } else {
    unsigned sfn = tti / 10;
    return (tti%10 == 0 || tti%10 == 5); 
  }
}

auto Phy::is_mbsfn_subframe(unsigned tti) -> bool
{
  if (_cell.mbms_dedicated) {
    // This is subframe 0 in a radio frame divisible by 4, and hence a CAS frame. 
    return !is_cas_subframe(tti);
  } else {
    return !is_cas_subframe(tti) &&
      (tti%10 == 1 || tti%10 == 2 || tti%10 == 3 || tti%10 == 6 || tti%10 == 7 || tti%10 == 8);
  }
}
auto Phy::mbsfn_config_for_tti(uint32_t tti, unsigned& area)
    -> srsran_mbsfn_cfg_t {
  srsran_mbsfn_cfg_t cfg;
  cfg.enable                  = false;
  cfg.is_mcch                 = false;

  if (!_mcch_configured) {
    {
      return cfg;
    }
  }

  uint32_t sfn = tti / 10;
  uint8_t sf = tti % 10;

  srsran::mbsfn_area_info_t& area_info = _sib13.mbsfn_area_info_list[0];

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
  } else if (sfn % enum_to_number(area_info.mcch_cfg.mcch_repeat_period) == area_info.mcch_cfg.mcch_offset &&
      sf == 1) {
      cfg.mbsfn_mcs               = enum_to_number(area_info.mcch_cfg.sig_mcs);
      cfg.enable                  = true;
      cfg.is_mcch                 = false;
  } else {
    if (_mch_configured) {
      cfg.mbsfn_area_id = area_info.mbsfn_area_id;

      for (uint32_t i = 0; i < _mcch.nof_pmch_info; i++) {
        unsigned fn_in_scheduling_period =  sfn % enum_to_number(_mcch.pmch_info_list[i].mch_sched_period);
        unsigned sf_idx = fn_in_scheduling_period * 10 + sf 
          - (fn_in_scheduling_period / 4) // minus 1 CAS SF per 4 SFNs 
          - 1; // minus 1 MCCH SF per scheduling period;

        spdlog::debug("i {}, tti {}, fn_in_ {}, sf_idx {}", i, tti, fn_in_scheduling_period,  sf_idx);

        if (sf_idx <= _mcch.pmch_info_list[i].sf_alloc_end) {
          area = i;
          if ((i == 0 && fn_in_scheduling_period == 0 && sf == 1) ||
              (i > 0 && _mcch.pmch_info_list[i-1].sf_alloc_end + 1 == sf_idx)) {
            spdlog::debug("assigning sig_mcs {}, mch_idx is {}",  area_info.mcch_cfg.sig_mcs, area);
            cfg.mbsfn_mcs = enum_to_number(area_info.mcch_cfg.sig_mcs);
          } else {
            spdlog::debug("assigning pmch_mcs {}, mch_idx is {}", _mcch.pmch_info_list[i].data_mcs, area);
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
