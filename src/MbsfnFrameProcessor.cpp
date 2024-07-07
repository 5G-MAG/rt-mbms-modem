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

#include "MbsfnFrameProcessor.h"
#include "spdlog/spdlog.h"

std::map<uint8_t, uint16_t> MbsfnFrameProcessor::_sched_stops;

std::mutex MbsfnFrameProcessor::_sched_stop_mutex;
std::mutex MbsfnFrameProcessor::_rlc_mutex;

auto MbsfnFrameProcessor::init() -> bool {
  _signal_buffer_max_samples = 3 * SRSRAN_SF_LEN_PRB(MAX_PRB);

  for (auto ch = 0; ch < _rx_channels; ch++) {
    _signal_buffer_rx[ch] = srsran_vec_cf_malloc(_signal_buffer_max_samples);
    if (!_signal_buffer_rx[ch]) {
      spdlog::error("Could not allocate regular DL signal buffer\n");
      return false;
    }
  }

  if (srsran_ue_dl_init(&_ue_dl, _signal_buffer_rx, MAX_PRB, _rx_channels) != 0) {
    spdlog::error("Could not init ue_dl\n");
    return false;;
  }

  srsran_softbuffer_rx_init(&_softbuffer, 100);

  _ue_dl_cfg.snr_to_cqi_offset = 0;

  srsran_chest_dl_cfg_t* chest_cfg = &_ue_dl_cfg.chest_cfg;
  bzero(chest_cfg, sizeof(srsran_chest_dl_cfg_t));
  chest_cfg->filter_coef[0] = 0.1;
  chest_cfg->filter_type = SRSRAN_CHEST_FILTER_NONE;
  chest_cfg->noise_alg = SRSRAN_NOISE_ALG_EMPTY;
  chest_cfg->rsrp_neighbour       = false;
  chest_cfg->sync_error_enable    = true;
  chest_cfg->estimator_alg = SRSRAN_ESTIMATOR_ALG_INTERPOLATE;
  chest_cfg->cfo_estimate_enable  = false;

  _ue_dl_cfg.cfg.pdsch.csi_enable         = true;
  _ue_dl_cfg.cfg.pdsch.max_nof_iterations = 8;
  _ue_dl_cfg.cfg.pdsch.meas_evm_en        = false;
  _ue_dl_cfg.cfg.pdsch.decoder_type       = SRSRAN_MIMO_DECODER_MMSE;
  _ue_dl_cfg.cfg.pdsch.softbuffers.rx[0] = &_softbuffer;

  _pmch_cfg.pdsch_cfg.csi_enable         = true;
  _pmch_cfg.pdsch_cfg.max_nof_iterations = 8;
  _pmch_cfg.pdsch_cfg.meas_evm_en        = false;
  _pmch_cfg.pdsch_cfg.decoder_type       = SRSRAN_MIMO_DECODER_MMSE;

  _sf_cfg.sf_type = SRSRAN_SF_MBSFN;
  return true;
}

MbsfnFrameProcessor::~MbsfnFrameProcessor() {
  srsran_softbuffer_rx_free(&_softbuffer);
  srsran_ue_dl_free(&_ue_dl);
}

void MbsfnFrameProcessor::set_cell(srsran_cell_t cell) {
  _cell = cell;
  srsran_ue_dl_set_cell(&_ue_dl, cell);
}

auto MbsfnFrameProcessor::process(uint32_t tti) -> int {
  spdlog::trace("Processing MBSFN TTI {}", tti);

  uint32_t sfn = tti / 10;
  uint8_t sf = tti % 10;

  unsigned mch_idx = 0;
  _sf_cfg.tti = tti;
  _pmch_cfg.area_id = _area_id;
  srsran_mbsfn_cfg_t mbsfn_cfg = _phy.mbsfn_config_for_tti(tti, mch_idx);
  _ue_dl_cfg.chest_cfg.mbsfn_area_id = _area_id;
  //srsran_ue_dl_set_mbsfn_area_id(&_ue_dl, mbsfn_cfg.mbsfn_area_id);

  if (!_cell.mbms_dedicated) {
    srsran_ue_dl_set_non_mbsfn_region(&_ue_dl, mbsfn_cfg.non_mbsfn_region_length);
  }

  if (!mbsfn_cfg.enable) {
    spdlog::trace("PMCH: tti {}: neither MCCH nor MCH enabled. Skipping subframe");
    _mutex.unlock();
    return -1;
  }

  if (mbsfn_cfg.is_mcch) {
    _rest._mcch.total++;
  } else {
    _rest._mch[mch_idx].total++;
  }

  if (srsran_ue_dl_decode_fft_estimate(&_ue_dl, &_sf_cfg, &_ue_dl_cfg) < 0) {
    if (mbsfn_cfg.is_mcch) {
      _rest._mcch.errors++;
    } else {
      _rest._mch[mch_idx].errors++;
    }
    spdlog::error("Getting PDCCH FFT estimate");
    _mutex.unlock();
    return -1;
  }

  srsran_configure_pmch(&_pmch_cfg, &_cell, &mbsfn_cfg);
  srsran_ra_dl_compute_nof_re(&_cell, &_sf_cfg, &_pmch_cfg.pdsch_cfg.grant);

  _pmch_cfg.area_id = _area_id;

  srsran_softbuffer_rx_reset_cb(&_softbuffer, 1);

  srsran_pdsch_res_t pmch_dec = {};
  _pmch_cfg.pdsch_cfg.softbuffers.rx[0] = &_softbuffer;
  pmch_dec.payload = _payload_buffer;
  srsran_softbuffer_rx_reset_tbs(_pmch_cfg.pdsch_cfg.softbuffers.rx[0], _pmch_cfg.pdsch_cfg.grant.tb[0].tbs);

  if (srsran_ue_dl_decode_pmch(&_ue_dl, &_sf_cfg, &_pmch_cfg, &pmch_dec) != 0) {
    if (mbsfn_cfg.is_mcch) {
      _rest._mcch.errors++;
    } else {
      _rest._mch[mch_idx].errors++;
    }
    spdlog::warn("Error decoding PMCH");
    _mutex.unlock();
    return -1;
  }

  spdlog::trace("PMCH: tti: {}, l_crb={}, tbs={}, mcs={}, crc={}, snr={} dB, n_iter={}\n",
      tti,
         _pmch_cfg.pdsch_cfg.grant.nof_prb,
         _pmch_cfg.pdsch_cfg.grant.tb[0].tbs / 8,
         _pmch_cfg.pdsch_cfg.grant.tb[0].mcs_idx,
         pmch_dec.crc ? "OK" : "KO",
         _ue_dl.chest_res.snr_db,
         pmch_dec.avg_iterations_block);

  if (mbsfn_cfg.is_mcch) {
    _rest._mcch.SetData(mch_data());
    _rest._mcch.mcs = _pmch_cfg.pdsch_cfg.grant.tb[0].mcs_idx;
  } else {
    _rest._mch[mch_idx].SetData(mch_data());
    _rest._mch[mch_idx].mcs = _pmch_cfg.pdsch_cfg.grant.tb[0].mcs_idx;
    _rest._mch[mch_idx].present = true;
  }

  if (pmch_dec.crc) {
    mch_mac_msg.init_rx(
        static_cast<uint32_t>(_pmch_cfg.pdsch_cfg.grant.tb[0].tbs) / 8);
    mch_mac_msg.parse_packet(_payload_buffer);

    while (mch_mac_msg.next()) {
      if (srsran::mch_lcid::MCH_SCHED_INFO == mch_mac_msg.get()->mch_ce_type()) {
        uint16_t stop = 0;
        uint8_t lcid = 0;
        while (mch_mac_msg.get()->get_next_mch_sched_info(&lcid, &stop)) {
          const std::lock_guard<std::mutex> lock(_sched_stop_mutex);
          spdlog::debug("Scheduling stop for LCID {} in sf {}", lcid, stop);
          _sched_stops[ lcid ] = stop;
        }
      } else if (mch_mac_msg.get()->is_sdu()) {
        uint32_t lcid = mch_mac_msg.get()->get_sdu_lcid();
        spdlog::trace("Processing MAC MCH PDU entered, lcid {}", lcid);

        if (lcid >= SRSRAN_N_MCH_LCIDS) {
          spdlog::warn("Radio bearer id must be in [0:%d] - %d", SRSRAN_N_MCH_LCIDS, lcid);
          if (mbsfn_cfg.is_mcch) {
            _rest._mcch.errors++;
          } else {
            _rest._mch[mch_idx].errors++;
          }
          _mutex.unlock();
          return -1;
        }

        {
          _phy._mcs = mbsfn_cfg.mbsfn_mcs;
          const std::lock_guard<std::mutex> lock(_rlc_mutex);
          _rlc.write_pdu_mch(mch_idx, lcid, mch_mac_msg.get()->get_sdu_ptr(), mch_mac_msg.get()->get_payload_size());
        }
      }
    }
  } else {
    if (mbsfn_cfg.is_mcch) {
      _rest._mcch.errors++;
    } else {
      _rest._mch[mch_idx].errors++;
    }

    spdlog::trace("PMCH in TTI {} failed with CRC error", tti);
    _mutex.unlock();
    return -1;
  }

  if (!mbsfn_cfg.is_mcch) {
    for (uint32_t i = 0; i < _phy.mcch().nof_pmch_info; i++) {
      unsigned fn_in_scheduling_period =  sfn % srsran::enum_to_number(_phy.mcch().pmch_info_list[i].mch_sched_period);
      unsigned sf_idx;
      if (_cell.mbms_dedicated) {
        sf_idx = fn_in_scheduling_period * 10 + sf - (fn_in_scheduling_period / 4) - 1;
      } else {
        sf_idx = fn_in_scheduling_period * 6 + (sf < 6 ? sf - 1 : sf - 3);
      }
          spdlog::debug("tti{}, sfn {}, sf {}, fn_in_scheduling_period {}, sf_idf {}", tti, sfn, sf, fn_in_scheduling_period, sf_idx);

      const std::lock_guard<std::mutex> lock(_sched_stop_mutex);
      for (auto itr = _sched_stops.cbegin() ; itr != _sched_stops.cend() ;) {
        if ( sf_idx >= itr->second ) {
          spdlog::debug("Stopping LCID {} in tti {} (idx in rf {})", itr->first, tti, sf_idx);
          const std::lock_guard<std::mutex> lock(_rlc_mutex);
          _rlc.stop_mch(i, itr->first);
          itr = _sched_stops.erase(itr);
        } else {
          itr = std::next(itr);
        }
      }
    }
  } else {
    const std::lock_guard<std::mutex> lock(_rlc_mutex);
    _rlc.stop_mch(0, 0);
    _rest._mcch.present = true;
  }
  _mutex.unlock();
  return mbsfn_cfg.is_mcch ? 0 : 1;
}

void MbsfnFrameProcessor::configure_mbsfn(uint8_t area_id, srsran_scs_t subcarrier_spacing) {
  _sf_cfg.subcarrier_spacing = subcarrier_spacing;
  srsran_ue_dl_set_mbsfn_subcarrier_spacing(&_ue_dl, subcarrier_spacing);


  srsran_ue_dl_set_mbsfn_area_id(&_ue_dl, area_id);
  _area_id = area_id;
  _mbsfn_configured = true;
}

auto MbsfnFrameProcessor::mch_data() const -> std::vector<uint8_t> const {
  const uint8_t* data = reinterpret_cast<uint8_t*>(_ue_dl.pmch.d);
  return std::move(std::vector<uint8_t>( data, data + _pmch_cfg.pdsch_cfg.grant.nof_re * sizeof(cf_t)));
}
