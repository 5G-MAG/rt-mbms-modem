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
#include <cstring>
#include <cstdlib>
#include <cstdio>


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
  _ue_dl_cfg.cfg.pdsch.meas_evm_en        = true;
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
  if (_cir_plan_ready) {
    srsran_dft_plan_free(&_cir_plan);
  }
}

void CasFrameProcessor::set_cell(srsran_cell_t cell) {
  _cell = cell;
  spdlog::debug("CAS processor setting cell ({} PRB / {} MBSFN PRB).", cell.nof_prb, cell.mbsfn_prb);
  srsran_ue_dl_set_cell(&_ue_dl, cell);
  _started = true;

  /* (Re)plan the CIR IFFT and size its scratch buffers here -- called only
   * from the single main thread on cell (re)configuration, never from
   * process()'s worker-pool thread, so there's no risk of concurrent
   * fftwf_plan_ / fftwf_destroy_plan calls racing across processor instances. */
  auto sz = (uint32_t)srsran_symbol_sz(_cell.nof_prb);
  if (!_cir_plan_ready || _cir_plan_size != sz) {
    if (_cir_plan_ready) {
      srsran_dft_plan_free(&_cir_plan);
    }
    srsran_dft_plan_c(&_cir_plan, (int)sz, SRSRAN_DFT_BACKWARD);
    srsran_dft_plan_set_norm(&_cir_plan, true);
    _cir_plan_size  = sz;
    _cir_plan_ready = true;
  }
  _cir_scratch_freq.resize(sz);
  _cir_scratch_time.resize(sz);
  _cir_scratch_shifted.resize(sz);
  _cir_scratch_db.resize(sz);
}

auto CasFrameProcessor::process(uint32_t tti) -> bool {
  _sf_cfg.tti = tti;

  // _pdsch.total/errors are advanced per actual SI decode below (in the CRC
  // handling) so BLER = CRC failures / decode attempts is a genuine block-error
  // rate. _pdcch keeps its per-occasion "not found" semantics.
  _rest._pdcch.total++;

  // Run the FFT and do channel estimation
  if (srsran_ue_dl_decode_fft_estimate(&_ue_dl, &_sf_cfg, &_ue_dl_cfg) < 0) {
    _rest._pdcch.errors++;
    spdlog::error("Getting PDCCH FFT estimate\n");
    _rest.record_subframe_event(tti, RestHandler::SF_EVENT_CAS, RestHandler::SF_STATUS_FAIL);
    _mutex.unlock();
    return false;
  }

  // Feedback the CFO from CE to the Phy
  _phy.set_cfo_from_channel_estimation(_ue_dl.chest_res.cfo);

  // Try to decode DCIs from PDCCH.
  // TS 36.321 §7.1 Table 7.1-1 NOTE 2: "SI-RNTI value FFFF may be used for
  // MBMS-dedicated carrier. SI-RNTI value FFF9 is only used for MBMS-dedicated
  // carrier." -- i.e. on an MBMS-dedicated cell the network may legally use
  // EITHER value; there is no SIB/MIB field telling the UE which one is in use.
  // This project's own rt-mbms-tx always transmits with FFF9 (srsenb/src/stack/
  // mac/sched_phy_ch/sched_dci.cc), so try that first (cheap, matches the common
  // case here), then fall back to the standard FFFF if nothing is found -- a
  // third-party MBMS-dedicated-cell sender may legitimately use the standard
  // value instead. Non-dedicated cells are unaffected: only ever SRSRAN_SIRNTI.
  srsran_dci_dl_t dci[SRSRAN_MAX_CARRIERS] = {};    // NOLINT
  int nof_grants = srsran_ue_dl_find_dl_dci(&_ue_dl, &_sf_cfg, &_ue_dl_cfg, _cell.mbms_dedicated ? SRSRAN_SIRNTI_MBMS_DEDICATED : SRSRAN_SIRNTI, dci);
  if (nof_grants == 0 && _cell.mbms_dedicated) {
    nof_grants = srsran_ue_dl_find_dl_dci(&_ue_dl, &_sf_cfg, &_ue_dl_cfg, SRSRAN_SIRNTI, dci);
  }
  for (int k = 0; k < nof_grants; k++) {
    char str[512];  // NOLINT
    srsran_dci_dl_info(&dci[k], str, 512);
    _rest._pdsch.mcs =  dci[k].tb[0].mcs_idx;
    spdlog::debug("Decoded PDCCH: {}, snr={} dB\n", str, _ue_dl.chest_res.snr_db);

    // TS 36.211 §6.8.1: 1 CCE = 9 REGs = 36 REs. location.L is the aggregation
    // level as a log2 value, so nof_cce = 2^L.
    _last_pdcch_nof_re = (1u << dci[k].location.L) * 36u;
    _rest._pdcch.SetData(pdcch_data());

    if (srsran_ue_dl_dci_to_pdsch_grant(&_ue_dl, &_sf_cfg, &_ue_dl_cfg, &dci[k], &_ue_dl_cfg.cfg.pdsch.grant)) {
      spdlog::error("Converting DCI message to DL dci\n");
      _rest.record_subframe_event(tti, RestHandler::SF_EVENT_CAS, RestHandler::SF_STATUS_FAIL);
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

    _rest._ce_values    = std::move(ce_values());
    _rest._cir_values   = std::move(cir_values());
    _rest._cas_grid     = std::move(cas_grid());

    // Decode PDSCH..
    auto ret = srsran_ue_dl_decode_pdsch(&_ue_dl, &_sf_cfg, &_ue_dl_cfg.cfg.pdsch, pdsch_res);
    /* pdsch_data() reads _ue_dl.pdsch.d[0], which srsran_ue_dl_decode_pdsch()
     * itself populates (equalized soft symbols, computed internally during
     * decode) -- must be captured AFTER decoding, not before, or the
     * constellation shows the PREVIOUS CAS occasion's symbols. Invisible
     * before the round-robin fix (every occasion carried the same SI message
     * with the same shape), but a visible one-cycle-stale glitch now that
     * different SI messages (different MCS/TBS index) rotate through here. */
    _rest._pdsch.SetData(pdsch_data());
    if (ret) {
      // Processing error (bad grant/buffer) before a CRC could even be computed.
      spdlog::error("Error decoding PDSCH\n");
      _rest._pdsch.total++;
      _rest._pdsch.errors++;
      _rest.record_subframe_event(tti, RestHandler::SF_EVENT_CAS, RestHandler::SF_STATUS_FAIL);
    } else {
      _rest._pdsch.evm_rms = pdsch_res[0].evm; // evm of the first codeword
      // srsran_ue_dl_decode_pdsch() returns SUCCESS even when the CRC fails; the
      // real per-TB result is in pdsch_res[i].crc. Count each decoded TB as an
      // attempt and a failed CRC as a real error, so BLER is a true block-error
      // rate rather than only catching hard processing errors.
      bool any_crc_fail = false;
      for (int i = 0; i < SRSRAN_MAX_CODEWORDS; i++) {
        if (pdsch_cfg->grant.tb[i].enabled) {
          _rest._pdsch.total++;
          if (pdsch_res[i].crc) {
            // .. and pass received PDUs to RLC for further processing
            _rlc.write_pdu_bcch_dlsch(_data[i], (uint32_t)pdsch_cfg->grant.tb[i].tbs);
          } else {
            _rest._pdsch.errors++;
            any_crc_fail = true;
            // Temporary diagnostic (CAS_RV_BRUTEFORCE=1, off by default): DCI format 1C
            // carries no RV field, so RV is blindly derived (see the rv<0 branch above);
            // if that formula doesn't match how the sender actually cycled RV, decode
            // will fail every time regardless of signal quality. Brute-force the other
            // 3 RV values (fresh softbuffer each try, since rate-matching combines with
            // buffer state) to see whether any of them actually yields a CRC pass.
            if (getenv("CAS_RV_BRUTEFORCE")) {
              int      original_rv = pdsch_cfg->grant.tb[i].rv;
              for (int try_rv = 0; try_rv < 4; try_rv++) {
                if (try_rv == original_rv) {
                  continue;
                }
                pdsch_cfg->grant.tb[i].rv = try_rv;
                srsran_softbuffer_rx_reset_tbs(pdsch_cfg->softbuffers.rx[i], (uint32_t)pdsch_cfg->grant.tb[i].tbs);
                srsran_pdsch_res_t retry_res = {};
                retry_res.payload = _data[i];
                retry_res.crc     = false;
                int retry_ret = srsran_ue_dl_decode_pdsch(&_ue_dl, &_sf_cfg, &_ue_dl_cfg.cfg.pdsch, &retry_res);
                fprintf(stderr, "RVBRUTE tti=%u original_rv=%d try_rv=%d ret=%d crc=%d\n",
                        tti, original_rv, try_rv, retry_ret, retry_res.crc ? 1 : 0);
              }
              // Restore original rv/softbuffer state so downstream logic is unaffected.
              pdsch_cfg->grant.tb[i].rv = original_rv;
              srsran_softbuffer_rx_reset_tbs(pdsch_cfg->softbuffers.rx[i], (uint32_t)pdsch_cfg->grant.tb[i].tbs);
            }
          }
        }
      }
      spdlog::debug("Decoded PDSCH (crc_fail={})", any_crc_fail);
      // Env-gated per-CAS decode diagnostic (CAS_PDSCH_DIAG=1, off by default):
      // logs MCS/EVM/CINR/CRC per SI decode so a CRC failure can be correlated
      // with the periodic EVM/CINR peaks.
      if (getenv("CAS_PDSCH_DIAG")) {
        fprintf(stderr,
                "CASDIAG tti=%u mcs=%d evm=%.4f snr=%.2f crc=%d tbs=%d format=%d rnti=0x%x nof_prb_alloc=%d L=%d ncce=%d "
                "nof_re=%d nof_bits_E=%d mod=%d\n",
                tti, dci[k].tb[0].mcs_idx, (double)pdsch_res[0].evm,
                (double)_ue_dl.chest_res.snr_db, pdsch_res[0].crc ? 1 : 0,
                (int)pdsch_cfg->grant.tb[0].tbs, (int)dci[k].format, dci[k].rnti,
                (int)pdsch_cfg->grant.nof_prb, (int)dci[k].location.L, (int)dci[k].location.ncce,
                (int)pdsch_cfg->grant.nof_re, (int)pdsch_cfg->grant.tb[0].nof_bits, (int)pdsch_cfg->grant.tb[0].mod);
      }
      _rest.record_subframe_event(tti, RestHandler::SF_EVENT_CAS,
                                  any_crc_fail ? RestHandler::SF_STATUS_FAIL : RestHandler::SF_STATUS_OK);
    }
  }
  if (nof_grants == 0) {
    // CAS occasion processed fine, but no SI message/paging was pending this cycle.
    _rest.record_subframe_event(tti, RestHandler::SF_EVENT_CAS, RestHandler::SF_STATUS_IDLE);
    _rest._pdcch.errors++;
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

auto CasFrameProcessor::cir_values() -> std::vector<uint8_t> {
  auto sz = _cir_plan_size;
  if (!_cir_plan_ready || sz == 0) {
    return {}; // set_cell() hasn't run yet - nothing to compute.
  }

  // Zero-pad the frequency-domain channel estimate into the full symbol
  // width, at the same subcarrier offset ce_values() uses, then IFFT it to
  // the time domain to get the channel impulse response. All scratch buffers
  // are pre-sized in set_cell() - no allocation on this hot path.
  cf_t* ce_freq = _cir_scratch_freq.data();
  srsran_vec_cf_zero(ce_freq, sz);
  uint32_t g = (sz - 12 * _cell.nof_prb) / 2;
  memcpy(&ce_freq[g], _ue_dl.chest_res.ce[0][0], SRSRAN_NRE * _cell.nof_prb * sizeof(cf_t));

  cf_t* cir_time = _cir_scratch_time.data();
  srsran_dft_run_c(&_cir_plan, ce_freq, cir_time);

  // fftshift so lag 0 (the main tap) is centered, then convert to dB
  // magnitude for display, matching ce_values()'s convention.
  cf_t* cir_shifted = _cir_scratch_shifted.data();
  for (uint32_t i = 0; i < sz; i++) {
    cir_shifted[i] = cir_time[(i + sz / 2) % sz];
  }
  float* cir_db = _cir_scratch_db.data();
  srsran_vec_abs_dB_cf(cir_shifted, -80, cir_db, sz);

  const uint8_t* data = reinterpret_cast<const uint8_t*>(cir_db);
  return std::vector<uint8_t>(data, data + sz * sizeof(float));
}

auto CasFrameProcessor::pdsch_data() -> std::vector<uint8_t> {
  const uint8_t* data = reinterpret_cast<uint8_t*>(_ue_dl.pdsch.d[0]);
  return std::vector<uint8_t>( data, data + _ue_dl_cfg.cfg.pdsch.grant.nof_re * sizeof(cf_t));
}

auto CasFrameProcessor::cas_grid() -> std::vector<uint8_t> {
  // Full received resource grid for this CAS subframe: nof_prb*12 subcarriers x
  // (2 slots * CP_NSYMB) OFDM symbols. sf_symbols[0] is allocated at max PRB, so
  // reading SRSRAN_SF_LEN_RE(nof_prb) is always in bounds. Magnitude in dB
  // (floor -80), same convention as ce_values(); returned as raw float bytes.
  uint32_t nof_re = SRSRAN_SF_LEN_RE(_cell.nof_prb, _cell.cp);
  std::vector<float> mag(nof_re, 0);
  srsran_vec_abs_dB_cf(_ue_dl.sf_symbols[0], -80, mag.data(), nof_re);
  const uint8_t* data = reinterpret_cast<const uint8_t*>(mag.data());
  return std::vector<uint8_t>(data, data + nof_re * sizeof(float));
}

auto CasFrameProcessor::pdcch_data() -> std::vector<uint8_t> {
  const uint8_t* data = reinterpret_cast<uint8_t*>(_ue_dl.pdcch.d);
  return std::vector<uint8_t>(data, data + _last_pdcch_nof_re * sizeof(cf_t));
}
