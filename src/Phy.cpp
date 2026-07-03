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
          uint32_t add_non_mbsfn = 0;
          srsran_pbch_mib_mbms_unpack(bch_payload.data(), &_cell, &sfn, &add_non_mbsfn,
              _override_nof_prb);
          _cell.additional_non_mbms_frames = (uint8_t)add_non_mbsfn;
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
    // NOTE: this used to call init(), which re-runs srsran_ue_cellsearch_init_multi_prb_cp(),
    // srsran_ue_sync_init_multi(), srsran_ue_mib_sync_init_multi_prb() and srsran_ue_mib_init()
    // on _cell_search/_ue_sync/_mib_sync/_mib a second time without ever freeing what the first
    // init() (called once from main() before the search loop starts) had already allocated -
    // every failed MIB-MBMS attempt leaked and re-initialized the same srsran objects on top of
    // their still-live state. That corrupts the heap over repeated cell_search() calls (each
    // failed-MBMS candidate hits this path) and eventually crashes in an unrelated later
    // allocation or FFTW plan run, especially at higher nof_prb where MBMS-first decode attempts
    // fail more often before the fallback + more frames/PRB means more memory touched per leak.
    // The srsran_ue_sync_reset() below (mirroring the MBMS attempt above) already re-arms sync;
    // what was actually still needed for the regular-MIB retry is resetting the MIB decoder's own
    // frame counter/PBCH state, which srsran_ue_mib_reset() does with no allocation involved.
    srsran_ue_mib_reset(&_mib_sync.ue_mib);
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
      uint32_t add_non_mbsfn = 0;
      srsran_pbch_mib_mbms_unpack(bch_payload.data(), &new_cell, &sfn, &add_non_mbsfn,
          _override_nof_prb);
      new_cell.additional_non_mbms_frames = (uint8_t)add_non_mbsfn;
    } else {
      srsran_pbch_mib_unpack(bch_payload.data(), &new_cell, &sfn);
    }
    /* MIB-MBMS encodes 6 SFN bits (<<4); each sfn_offset unit = kSfnOffset radio frames.
     * Standard MIB encodes 8 SFN bits (<<2); sfn_offset unit = 1 radio frame. */
    sfn = (sfn + (new_cell.mbms_dedicated ? sfn_offset * kSfnOffset : sfn_offset)) % kMaxSfn;

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
    /* TS 36.211 §6.6.4.1: CAS frame period depends on carrier width.
     * nof_prb >= 25: nf mod 4 == 0  →  tti mod 40 == 0.
     * 6 < nof_prb < 25: nf mod 8 == 4  →  tti mod 80 == 40. */
    unsigned cas_period = (_cell.nof_prb >= 25) ? 40u : 80u;
    unsigned cas_offset = (_cell.nof_prb >= 25) ? 0u  : 40u;
    if (tti % cas_period != cas_offset) {
      return false;
    }
    if (_cell.cas_muting) {
      /* Active (not muted) when sfn % (16*NCAS) < 4*KCAS (TS 36.211 CR 0577). */
      unsigned sfn = tti / 10;
      return sfn % (16u * (unsigned)_cell.n_cas) < 4u * (unsigned)_cell.k_cas;
    }
    return true;
  } else {
    return (tti%10 == 0 || tti%10 == 5);
  }
}

auto Phy::is_mbsfn_subframe(unsigned tti) -> bool
{
  if (_cell.mbms_dedicated) {
    if (is_cas_subframe(tti)) return false;
    /* additionalNonMBSFNSubframes-r14: SF1..SF(N) of an active CAS frame are not MBSFN -
     * except MCCH's own subframe, which must always be checked regardless (mirrors TX's
     * phy_common::is_mch_subframe, fixed for the same reason: MCCH's position is
     * SIB13-configured independently of additionalNonMBSFNSubframes, an unrelated
     * MIB-MBMS field, and nothing stops an operator picking values that collide - which
     * would otherwise make this function exclude MCCH's own subframe outright, so it's
     * never even attempted here, on every occasion, not just occasionally). */
    unsigned sf = tti % 10;
    bool is_mcch_sf = false;
    if (_mcch_configured) {
      unsigned sfn = tti / 10;
      const auto& mcch_cfg = _sib13.mbsfn_area_info_list[0].mcch_cfg;
      if (sfn % enum_to_number(mcch_cfg.mcch_repeat_period) == mcch_cfg.mcch_offset && _mcch_table[sf]) {
        is_mcch_sf = true;
      }
    }
    if (!is_mcch_sf && _cell.additional_non_mbms_frames > 0) {
      if (sf >= 1 && sf <= _cell.additional_non_mbms_frames && is_cas_subframe((tti / 10) * 10)) {
        return false;
      }
    }
    return true;
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
  /* srsran_mbsfn_cfg_t is a plain C struct - the declaration above does not
   * zero-initialize it, so every field must get an explicit default here or
   * it reads as garbage stack memory. The MCCH branch below (is_mcch=true)
   * only ever sets subcarrier_spacing/mbsfn_mcs/enable/is_mcch - it never
   * touches use_mcs_table2, cyclic_shift(_alpha), freq_interleaving,
   * time_interleaving_n/m, mch_subframe_idx or pmch_idx (mirroring TX's
   * phy_common::is_mcch_subframe, which likewise doesn't set them, relying
   * on the shared defaults its caller is_mch_subframe sets first). Without
   * these defaults here, MCCH decode read uninitialized use_mcs_table2 -
   * srsran_pmch_fill_ra_mcs() would then pick the wrong MCS/TBS table for
   * MCCH on whatever runs happened to leave a truthy value on the stack,
   * producing a TBS that never matched what the eNB actually encoded (and
   * varied run-to-run) - the immediate cause of MCCH's CRC always failing
   * even after the eNB was fixed to actually transmit real MCCH content
   * (see rt-mbms-tx's configure_mbsfn() mcch_table fix). */
  cfg.use_mcs_table2          = false;
  cfg.cyclic_shift            = 0;
  cfg.cyclic_shift_alpha      = 0;
  cfg.freq_interleaving       = false;
  cfg.time_interleaving_n     = 1;
  cfg.time_interleaving_m     = 1;
  cfg.mch_subframe_idx        = 0;
  cfg.pmch_idx                = 0;
  /* Default data SCS — overridden per-branch below for MCCH subframes. */
  srsran_scs_t data_scs;
  switch (mbsfn_subcarrier_spacing()) {
    case SubcarrierSpacing::df_7kHz5:     data_scs = SRSRAN_SCS_7KHZ5;     break;
    case SubcarrierSpacing::df_2kHz5:     data_scs = SRSRAN_SCS_2KHZ5;     break;
    case SubcarrierSpacing::df_1kHz25:    data_scs = SRSRAN_SCS_1KHZ25;    break;
    case SubcarrierSpacing::df_370Hz:     data_scs = SRSRAN_SCS_370HZ;     break;
    case SubcarrierSpacing::df_370Hz_sl4: data_scs = SRSRAN_SCS_370HZ_SL4; break;
    case SubcarrierSpacing::df_370Hz_sl2: data_scs = SRSRAN_SCS_370HZ_SL2; break;
    default:                              data_scs = SRSRAN_SCS_15KHZ;     break;
  }
  cfg.subcarrier_spacing = data_scs;

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
  /* FeMBMS SCS types have no PDCCH control region on MBSFN subframes (TS 36.211).
   * The ASN.1 non-MBSFNregionLength is mandatory (s1/s2 only, no s0), so the decoded
   * SIB13 value is always >= 1. Override to 0 to match the TX which forces it to 0
   * for all FeMBMS SCS types; without this the OFDM FFT guard boundary is misplaced. */
  using SCS_t = srsran::mbsfn_area_info_t::subcarrier_spacing_t;
  switch (area_info.subcarrier_spacing) {
    case SCS_t::khz_1dot25:
    case SCS_t::khz_2dot5:
    case SCS_t::khz_7dot5:
    case SCS_t::khz_0dot37:
      cfg.non_mbsfn_region_length = 0;
      break;
    default: break;
  }

  /* Re-enable MCCH decode at each modification period boundary (TS 36.331 §5.8.1.3).
   * This lets the modem detect MCCH content changes without a separate notification. */
  if (_mch_configured && !_decode_mcch.load(std::memory_order_acquire) && sf == 0) {
    uint32_t mod_period = (uint32_t)enum_to_number(area_info.mcch_cfg.mcch_mod_period);
    if (sfn % mod_period == 0) {
      spdlog::debug("MCCH modification period boundary at SFN {} — scheduling MCCH re-read", sfn);
      _decode_mcch.store(true, std::memory_order_release);
    }
  }

  if (sfn % enum_to_number(area_info.mcch_cfg.mcch_repeat_period) == area_info.mcch_cfg.mcch_offset &&
      _mcch_table[sf] == 1) {
    /* MCCH SCS mirrors TX phy_common::is_mcch_subframe: 7.5kHz for 7.5kHz areas,
     * 1.25kHz for all others (including 0.37kHz, where MCCH uses the control SCS). */
    cfg.subcarrier_spacing = (area_info.subcarrier_spacing ==
        srsran::mbsfn_area_info_t::subcarrier_spacing_t::khz_7dot5)
        ? SRSRAN_SCS_7KHZ5 : SRSRAN_SCS_1KHZ25;
    if (_decode_mcch.load(std::memory_order_acquire)) {
      cfg.mbsfn_mcs               = enum_to_number(area_info.mcch_cfg.sig_mcs);
      cfg.enable                  = true;
      cfg.is_mcch                 = true;
    }
  } else {
    if (_mch_configured) {
      cfg.mbsfn_area_id = area_info.mbsfn_area_id;

      for (uint32_t i = 0; i < _mcch.nof_pmch_info; i++) {
        uint32_t fn_in_scheduling_period = sfn % enum_to_number(_mcch.pmch_info_list[i].mch_sched_period);
        /* Count true CAS frames elapsed before fn_in (spec CR 0577: k_cas active
         * CAS per 16*n_cas-frame period).  Without muting: fn_in/4. */
        uint32_t nof_true_cas;
        if (_cell.cas_muting) {
          uint32_t n_cas  = (uint32_t)_cell.n_cas;
          uint32_t k_cas  = (uint32_t)_cell.k_cas;
          uint32_t period = 16u * n_cas;
          uint32_t rem    = fn_in_scheduling_period % period;
          uint32_t cap    = 4u * k_cas;
          nof_true_cas = (fn_in_scheduling_period / period) * k_cas + (rem < cap ? rem : cap) / 4u;
        } else {
          nof_true_cas = fn_in_scheduling_period / 4u;
        }
        /* Count MCCH subframes that have passed before (fn_in, sf) within this scheduling
         * period.  Mirrors the TX phy_common::is_mch_subframe logic (without CAS muting). */
        uint32_t sched_period = enum_to_number(_mcch.pmch_info_list[i].mch_sched_period);
        uint32_t mcch_rp      = enum_to_number(area_info.mcch_cfg.mcch_repeat_period);
        uint32_t mcch_off     = area_info.mcch_cfg.mcch_offset;
        uint32_t sfn_base     = sfn - fn_in_scheduling_period;
        uint32_t sfn_base_mod = sfn_base % mcch_rp;
        uint32_t first_mcch_m = (mcch_off + mcch_rp - sfn_base_mod) % mcch_rp;
        uint8_t  mcch_sf_in_frame = 1u;
        for (uint8_t s = 0; s < 10u; s++) { if (_mcch_table[s]) { mcch_sf_in_frame = s; break; } }
        uint32_t nof_mcch_passed = 0;
        for (uint32_t m = first_mcch_m; m < sched_period; m += mcch_rp) {
          if (m < fn_in_scheduling_period ||
              (m == fn_in_scheduling_period && sf > mcch_sf_in_frame)) {
            nof_mcch_passed++;
          }
        }
        /* additionalNonMBSFNSubframes-r14: subtract excluded SFs from the sf_idx count.
         * Mirrors the TX phy_common sf_idx formula: (nof_true_cas + anchor_active) * N,
         * where anchor_active = whether fn_in=0 of this scheduling period is active CAS. */
        uint32_t nof_additional_passed = 0u;
        uint8_t  add_non = _cell.additional_non_mbms_frames;
        if (add_non > 0u) {
          bool anchor_act = true;
          if (_cell.cas_muting) {
            uint32_t n_cas = (uint32_t)_cell.n_cas;
            uint32_t k_cas = (uint32_t)_cell.k_cas;
            anchor_act = sfn_base % (16u * n_cas) < 4u * k_cas;
          }
          nof_additional_passed = (nof_true_cas + (anchor_act ? 1u : 0u)) * (uint32_t)add_non;
        }
        int sf_idx = (int)(fn_in_scheduling_period * 10u + sf) - (int)nof_true_cas - (int)nof_mcch_passed - (int)nof_additional_passed;

        spdlog::debug("i {}, tti {}, fn_in_ {}, sf_idx {}, nof_mcch_passed {}", i, tti, fn_in_scheduling_period, sf_idx, nof_mcch_passed);

        /* 0-based MCH subframe index within this PMCH's data allocation.
         * For i=0 the MCCH occupies sf_idx=0; the first data sf has sf_idx=1.
         * pmch_start=1 maps sf_idx=1 → mch_subframe_idx=0 (TS 36.211 §6.5.3).
         * Guard: sf_idx must be >= pmch_start to avoid uint32_t wraparound on subtraction. */
        uint32_t pmch_start = (i == 0) ? 1u : (uint32_t)(_mcch.pmch_info_list[i - 1].sf_alloc_end + 1);
        if (sf_idx >= (int)pmch_start && (uint32_t)sf_idx <= _mcch.pmch_info_list[i].sf_alloc_end) {
          area = i;
          cfg.mbsfn_mcs           = _mcch.pmch_info_list[i].data_mcs;
          cfg.use_mcs_table2      = _mcch.pmch_info_list[i].use_mcs_table2;
          cfg.time_interleaving_n = _mcch.pmch_info_list[i].time_interleaving_n;
          cfg.time_interleaving_m = _mcch.pmch_info_list[i].time_interleaving_m;
          cfg.cyclic_shift        = _mcch.pmch_info_list[i].cyclic_shift;
          cfg.cyclic_shift_alpha  = _mcch.pmch_info_list[i].cyclic_shift_alpha;
          cfg.freq_interleaving   = _mcch.pmch_info_list[i].freq_interleaving;
          cfg.mch_subframe_idx = (uint32_t)sf_idx - pmch_start;
          cfg.pmch_idx         = (uint8_t)i;
          cfg.enable = true;
          spdlog::debug("PMCH {}: mch_subframe_idx {}, mcs {}", i, cfg.mch_subframe_idx, cfg.mbsfn_mcs);
          break;
        }
      }
    }
  }
  return cfg;
}
