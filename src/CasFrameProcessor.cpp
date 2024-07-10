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

#include "CasFrameProcessor.h"
#include "spdlog/spdlog.h"


auto CasFrameProcessor::init() -> bool {
  _signal_buffer_max_samples = 3 * SRSRAN_SF_LEN_PRB(MAX_PRB);

  for (auto ch = 0; ch < _rx_channels; ch++) {
    _signal_buffer_rx[ch] = srsran_vec_cf_malloc(_signal_buffer_max_samples);
    if (!_signal_buffer_rx[ch]) {
      spdlog::error("Could not allocate regular DL signal buffer\n");
      return false;
    }
  }

  if (srsran_ue_dl_init(&_ue_dl, _signal_buffer_rx, MAX_PRB, _rx_channels)) {
    spdlog::error("Could not init ue_dl\n");
    return false;;
  }

  srsran_softbuffer_rx_init(&_softbuffer, 100);

  _ue_dl_cfg.snr_to_cqi_offset = 0;

  for (auto & i : _data) {
    i = srsran_vec_u8_malloc(2000 * 8);
    if (!i) {
      spdlog::error("Allocating data");
      return false;
    }
  }

  srsran_chest_dl_cfg_t* chest_cfg = &_ue_dl_cfg.chest_cfg;
  bzero(chest_cfg, sizeof(srsran_chest_dl_cfg_t));
  chest_cfg->filter_coef[0] = 4;
  chest_cfg->filter_coef[1] = 0.2f;
  chest_cfg->filter_type = SRSRAN_CHEST_FILTER_GAUSS;
  chest_cfg->noise_alg = SRSRAN_NOISE_ALG_EMPTY;
  chest_cfg->rsrp_neighbour       = false;
  chest_cfg->sync_error_enable    = true;
  chest_cfg->estimator_alg = SRSRAN_ESTIMATOR_ALG_AVERAGE;
  chest_cfg->cfo_estimate_enable  = true;
  chest_cfg->cfo_estimate_sf_mask = 1023;

  _ue_dl_cfg.cfg.pdsch.csi_enable         = true;
  _ue_dl_cfg.cfg.pdsch.max_nof_iterations = 8;
  _ue_dl_cfg.cfg.pdsch.meas_evm_en        = false;
  _ue_dl_cfg.cfg.pdsch.decoder_type       = SRSRAN_MIMO_DECODER_MMSE;
  _ue_dl_cfg.cfg.pdsch.softbuffers.rx[0] = &_softbuffer;

  _sf_cfg.sf_type = SRSRAN_SF_NORM;
  return true;
}

CasFrameProcessor::~CasFrameProcessor() {
  for (auto & i : _data) {
    if (i) {
      free(i);
    }
  }
  srsran_softbuffer_rx_free(&_softbuffer);
  srsran_ue_dl_free(&_ue_dl);
}

void CasFrameProcessor::set_cell(srsran_cell_t cell) {
  _cell = cell;
  spdlog::debug("CAS processor setting cell ({} PRB / {} MBSFN PRB).", cell.nof_prb, cell.mbsfn_prb);
  srsran_ue_dl_set_cell(&_ue_dl, cell);
  _started = true;
}

auto CasFrameProcessor::process(uint32_t tti) -> bool {
  _sf_cfg.tti = tti;

  _rest._pdsch.total++;

  // Run the FFT and do channel estimation
  if (srsran_ue_dl_decode_fft_estimate(&_ue_dl, &_sf_cfg, &_ue_dl_cfg) < 0) {
    _rest._pdsch.errors++;
    spdlog::error("Getting PDCCH FFT estimate\n");
    _mutex.unlock();
    return false;
  }

  // Feedback the CFO from CE to the Phy
  _phy.set_cfo_from_channel_estimation(_ue_dl.chest_res.cfo);

  // Try to decode DCIs from PDCCH
  srsran_dci_dl_t dci[SRSRAN_MAX_CARRIERS] = {};    // NOLINT
  int nof_grants = srsran_ue_dl_find_dl_dci(&_ue_dl, &_sf_cfg, &_ue_dl_cfg, _cell.mbms_dedicated ? SRSRAN_SIRNTI_MBMS_DEDICATED : SRSRAN_SIRNTI, dci);
  for (int k = 0; k < nof_grants; k++) {
    char str[512];  // NOLINT
    srsran_dci_dl_info(&dci[k], str, 512);
    _rest._pdsch.mcs =  dci[k].tb[0].mcs_idx;
    spdlog::debug("Decoded PDCCH: {}, snr={} dB\n", str, _ue_dl.chest_res.snr_db);

    if (srsran_ue_dl_dci_to_pdsch_grant(&_ue_dl, &_sf_cfg, &_ue_dl_cfg, &dci[k], &_ue_dl_cfg.cfg.pdsch.grant)) {
      spdlog::error("Converting DCI message to DL dci\n");
    _mutex.unlock();
      return false;
    }

    // We can construct a DL grant
    _ue_dl_cfg.cfg.pdsch.rnti = dci[k].rnti;
    srsran_pdsch_cfg_t* pdsch_cfg = &_ue_dl_cfg.cfg.pdsch;

    srsran_pdsch_res_t pdsch_res[SRSRAN_MAX_CODEWORDS] = {};  // NOLINT
    for (int i = 0; i < SRSRAN_MAX_CODEWORDS; i++) {
      if (pdsch_cfg->grant.tb[i].enabled) {
        if (pdsch_cfg->grant.tb[i].rv < 0) {
          uint32_t sfn              = tti / 10;
          uint32_t k                = (sfn / 2) % 4;
          pdsch_cfg->grant.tb[i].rv = ((int32_t)ceilf(static_cast<float>(1.5) * k)) % 4;
        }
        pdsch_res[i].payload = _data[i];
        pdsch_res[i].crc     = false;
        srsran_softbuffer_rx_reset_tbs(pdsch_cfg->softbuffers.rx[i], (uint32_t)pdsch_cfg->grant.tb[i].tbs);
      }
    }

    _rest._pdsch.SetData(pdsch_data());
    _rest._ce_values    = std::move(ce_values());

    // Decode PDSCH..
    auto ret = srsran_ue_dl_decode_pdsch(&_ue_dl, &_sf_cfg, &_ue_dl_cfg.cfg.pdsch, pdsch_res);
    if (ret) {
      spdlog::error("Error decoding PDSCH\n");
      _rest._pdsch.errors++;
    } else {
      spdlog::debug("Decoded PDSCH");
      _rest._pdsch.evm_rms = pdsch_res[0].evm; // evm of the first codeword
      for (int i = 0; i < SRSRAN_MAX_CODEWORDS; i++) {
        // .. and pass received PDUs to RLC for further processing
        if (pdsch_cfg->grant.tb[i].enabled && pdsch_res[i].crc) {
          _rlc.write_pdu_bcch_dlsch(_data[i], (uint32_t)pdsch_cfg->grant.tb[i].tbs);
        }
      }
    }
  }
    _mutex.unlock();
  return true;
}

auto CasFrameProcessor::ce_values() -> std::vector<uint8_t> {
  auto sz = (uint32_t)srsran_symbol_sz(_cell.nof_prb);
  std::vector<float> ce_abs;
  ce_abs.resize(sz, 0);
  uint32_t g = (sz - 12 * _cell.nof_prb) / 2;
  srsran_vec_abs_dB_cf(_ue_dl.chest_res.ce[0][0], -80, &ce_abs[g], SRSRAN_NRE * _cell.nof_prb);
  const uint8_t* data = reinterpret_cast<uint8_t*>(ce_abs.data());
  return std::vector<uint8_t>( data, data + sz * sizeof(float));
}

auto CasFrameProcessor::pdsch_data() -> std::vector<uint8_t> {
  const uint8_t* data = reinterpret_cast<uint8_t*>(_ue_dl.pdsch.d[0]);
  return std::vector<uint8_t>( data, data + _ue_dl_cfg.cfg.pdsch.grant.nof_re * sizeof(cf_t));
}
