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
#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <cmath>
#include <complex.h>
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

void CasFrameProcessor::set_cell(srsran_cell_t cell, srsran_scs_t mbsfn_scs) {
  /* See MbsfnFrameProcessor::set_cell()'s identical guard for the full reasoning --
   * _ue_dl's buffers here are likewise fixed-allocated from MAX_PRB, and
   * srsran_ue_dl_set_cell_scs() has no awareness of that outer sizing. */
  if (cell.mbsfn_prb > MAX_PRB || cell.nof_prb > MAX_PRB) {
    spdlog::error("CasFrameProcessor::set_cell: requested nof_prb={} mbsfn_prb={} exceeds MAX_PRB={} -- "
                  "clamping both to avoid overflowing buffers sized for MAX_PRB",
                  cell.nof_prb, cell.mbsfn_prb, MAX_PRB);
    cell.nof_prb   = std::min<uint32_t>(cell.nof_prb, MAX_PRB);
    cell.mbsfn_prb = std::min<uint32_t>(cell.mbsfn_prb, MAX_PRB);
  }
  _cell = cell;
  spdlog::debug("CAS processor setting cell ({} PRB / {} MBSFN PRB).", cell.nof_prb, cell.mbsfn_prb);
  srsran_ue_dl_set_cell_scs(&_ue_dl, cell, mbsfn_scs);
  _started = true;

  /* (Re)plan the CIR IFFT and size its scratch buffers here -- called only
   * from the single main thread on cell (re)configuration, never from
   * process()'s worker-pool thread, so there's no risk of concurrent
   * fftwf_plan_ / fftwf_destroy_plan calls racing across processor instances. */
  /* Sized from nof_prb alone: _ue_dl's fft[port] (CAS/PBCH/PSS/SSS) now stays
   * permanently at the carrier's own native, narrow symbol_sz regardless of
   * mbsfn_prb (decimated samples are bridged in via cas_decimator in
   * ue_dl.c, not by widening fft[port] itself - see SIB13_MBSFN_TEST_RESULTS.md).
   * An earlier version of this sizing used max(nof_prb, mbsfn_prb) to match
   * fft[port]'s own (then-widened) size; using that stale max() now would
   * reintroduce the exact "diagnostic out of sync with the real decode path"
   * mismatch this comment used to warn against, just inverted. */
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
  // TEMPORARY DIAGNOSTIC (CAS_TIMING_DIAG=1, 2026-07-22): measures this call's own wall-clock
  // duration regardless of which of process()'s several early-return points is hit, to test
  // whether a single CAS/PDSCH decode occasionally takes close to or longer than the 5ms
  // budget an MBMS/Unicast-mixed cell's CAS period (subframe 0 and 5 of every radio frame)
  // allows before the next occasion's get_rx_buffer_and_lock() would block on this call's
  // still-held _mutex -- a candidate explanation for the periodic "Synchronization lost while
  // processing" seen only in that mode, never in MBMS-dedicated mode's much longer (40/80ms)
  // CAS period, and only when this call still holds the lock into the next occasion.
  struct ScopeTimer {
    uint32_t tti_;
    bool enabled = getenv("CAS_TIMING_DIAG") != nullptr;
    std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
    ~ScopeTimer() {
      if (!enabled) return;
      auto us = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - start).count();
      if (us > 2000) {
        fprintf(stderr, "CAS_TIMING_DIAG tti=%u process()_us=%lld\n", tti_, (long long)us);
      }
    }
  } scope_timer{tti};
  _sf_cfg.tti = tti;
  _last_pdcch_locations.clear();

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
  // DIAGNOSTIC KILL-SWITCH (kept as a regression-check tool): env-gated skip, so a
  // future investigation into CFO-loop-related drift can isolate whether this
  // feedback path (chest_res.cfo -> Phy's/ue_sync's srsran_cfo_correct(), applied to
  // raw samples before the next occasion's FFT) is a contributing factor, without
  // needing to re-derive the wiring from scratch.
  if (!getenv("CFO_FEEDBACK_DISABLE")) {
    _phy.set_cfo_from_channel_estimation(_ue_dl.chest_res.cfo);
  }

  // Per-occasion channel-estimate/sync diagnostic - unconditional (every occasion,
  // not just successful decodes) so failing occasions show up too. General-purpose;
  // reused across multiple investigations, not tied to any single one.
  if (getenv("CAS_CE_DIAG")) {
    fprintf(stderr,
            "CAS_CE_DIAG tti=%u snr_db=%.2f noise_est=%.4f noise_est_dbm=%.2f rsrp_dbm=%.2f "
            "cfo=%.4f sync_error=%.4f nof_prb=%u mbsfn_prb=%u ue_sync_mean_off=%.6f "
            "ue_sync_next_rf_off=%d ue_sync_peak=%.4f\n",
            tti, _ue_dl.chest_res.snr_db, _ue_dl.chest_res.noise_estimate,
            _ue_dl.chest_res.noise_estimate_dbm, _ue_dl.chest_res.rsrp_dbm,
            _ue_dl.chest_res.cfo, _ue_dl.chest_res.sync_error, _cell.nof_prb, _cell.mbsfn_prb,
            _phy.ue_sync_mean_sample_offset(), _phy.ue_sync_next_rf_sample_offset(),
            _phy.ue_sync_track_peak_value());
  }

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
    _last_pdcch_locations.emplace_back(dci[k].location.ncce, dci[k].location.L);
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

    _rest._ce_values      = std::move(ce_values());
    _rest._cir_values     = std::move(cir_values());
    _rest._cas_grid       = std::move(cas_grid());
    _rest._cas_composition = std::move(composition_grid());

    // Decode PDSCH..
    auto ret = srsran_ue_dl_decode_pdsch(&_ue_dl, &_sf_cfg, &_ue_dl_cfg.cfg.pdsch, pdsch_res);
    /* pdsch_data() reads _ue_dl.pdsch.d[0], which srsran_ue_dl_decode_pdsch()
     * itself populates (equalized soft symbols, computed internally during
     * decode) -- must be captured AFTER decoding, not before, or the
     * constellation shows the PREVIOUS CAS occasion's symbols. Invisible
     * before the round-robin fix (every occasion carried the same SI message
     * with the same shape), but a visible one-cycle-stale glitch now that
     * different SI messages (different MCS/TBS index) rotate through here. */

    /* TS 36.211 §6.6.4.1: on an MBMS-dedicated wideband CAS (N_RB^DL > 6) the PBCH
     * is repeated, and the repeated-PBCH symbols do not fill every RE, so the unused
     * REs carry SI-PDSCH. Whether a given cell transmits this repetition is not
     * signalled to the UE before PBCH decode, so we determine it by decode success:
     * if the legacy decode failed CRC, retry once with the repeated-PBCH RE recovery
     * enabled (ra_dl.c/pdsch.c, gated by cell.is_mbms_r16). Keep it enabled only if
     * the SI TB's 24-bit CRC then passes; otherwise revert. A wrong RE set cannot
     * pass the CRC, so this never mis-triggers on a non-repetition cell (which decodes
     * on the first, legacy attempt and never reaches here). Once confirmed, the flag
     * persists on the ue_dl/pdsch cell copies, so later CAS occasions use the recovery
     * directly. */
    if (ret == 0 && !pdsch_res[0].crc && _cell.mbms_dedicated && _cell.nof_prb > 6 &&
        !_ue_dl.cell.is_mbms_r16 && _ue_dl_cfg.cfg.pdsch.grant.tb[0].enabled) {
      _ue_dl.cell.is_mbms_r16       = true;
      _ue_dl.pdsch.cell.is_mbms_r16 = true;
      srsran_ue_dl_dci_to_pdsch_grant(&_ue_dl, &_sf_cfg, &_ue_dl_cfg, &dci[k], &_ue_dl_cfg.cfg.pdsch.grant);
      srsran_softbuffer_rx_reset_tbs(_ue_dl_cfg.cfg.pdsch.softbuffers.rx[0],
                                     (uint32_t)_ue_dl_cfg.cfg.pdsch.grant.tb[0].tbs);
      pdsch_res[0].crc = false;
      int rret = srsran_ue_dl_decode_pdsch(&_ue_dl, &_sf_cfg, &_ue_dl_cfg.cfg.pdsch, pdsch_res);
      if (rret != 0 || !pdsch_res[0].crc) {
        _ue_dl.cell.is_mbms_r16       = false;  // not a repetition cell: revert
        _ue_dl.pdsch.cell.is_mbms_r16 = false;
      } else {
        _cell.is_mbms_r16 = true;
        spdlog::info("Confirmed Rel-16 CAS PBCH repetition (TS 36.211 6.6.4.1): SI-PDSCH decoded via repeated-PBCH RE recovery");
        if (getenv("CAS_PDSCH_DIAG")) {
          fprintf(stderr, "R16REP_CONFIRMED tti=%u: SI-PDSCH decoded via TS36.211-6.6.4.1 repeated-PBCH RE recovery "
                          "(legacy nof_re failed CRC, recovered nof_re=%d passed)\n",
                  tti, (int)_ue_dl_cfg.cfg.pdsch.grant.nof_re);
        }
      }
    }

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
            // Temporary diagnostic (CAS_CFI_BRUTEFORCE=1, off by default): the PDSCH
            // RE mapping/rate-matching depends on sf->cfi (control-region size). The
            // DCI was found at the real cfi (validated by its PDCCH CRC), but if the
            // SI-PDSCH data region uses a different effective cfi, the LLRs are
            // misaligned and turbo fails on every RV with clean symbols. Recompute the
            // grant at cfi=1/2/3 (keeping the already-decoded DCI) and retry the PDSCH
            // decode to see whether any cfi actually yields a CRC pass.
            if (getenv("CAS_CFI_BRUTEFORCE")) {
              uint32_t orig_cfi = _sf_cfg.cfi;
              for (uint32_t try_cfi = 1; try_cfi <= 3; try_cfi++) {
                if (try_cfi == orig_cfi) {
                  continue;
                }
                _sf_cfg.cfi = try_cfi;
                srsran_ue_dl_dci_to_pdsch_grant(&_ue_dl, &_sf_cfg, &_ue_dl_cfg, &dci[k], &_ue_dl_cfg.cfg.pdsch.grant);
                srsran_softbuffer_rx_reset_tbs(pdsch_cfg->softbuffers.rx[i], (uint32_t)pdsch_cfg->grant.tb[i].tbs);
                srsran_pdsch_res_t cfi_res = {};
                cfi_res.payload = _data[i];
                cfi_res.crc     = false;
                int cret = srsran_ue_dl_decode_pdsch(&_ue_dl, &_sf_cfg, &_ue_dl_cfg.cfg.pdsch, &cfi_res);
                fprintf(stderr, "CFIBRUTE tti=%u orig_cfi=%u try_cfi=%u nof_re=%d tbs=%d ret=%d crc=%d\n",
                        tti, orig_cfi, try_cfi, (int)pdsch_cfg->grant.nof_re,
                        (int)pdsch_cfg->grant.tb[i].tbs, cret, cfi_res.crc ? 1 : 0);
              }
              // Restore the real cfi + grant + softbuffer so downstream logic is unaffected.
              _sf_cfg.cfi = orig_cfi;
              srsran_ue_dl_dci_to_pdsch_grant(&_ue_dl, &_sf_cfg, &_ue_dl_cfg, &dci[k], &_ue_dl_cfg.cfg.pdsch.grant);
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
                "CASDIAG tti=%u cfi=%d mcs=%d evm=%.4f snr=%.2f crc=%d tbs=%d format=%d rnti=0x%x nof_prb_alloc=%d L=%d ncce=%d "
                "nof_re=%d nof_bits_E=%d mod=%d\n",
                tti, (int)_sf_cfg.cfi, dci[k].tb[0].mcs_idx, (double)pdsch_res[0].evm,
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
    // Still refresh ce/cir/grid/composition here - no grant means no PDCCH to
    // mark, but CRS/PSS/SSS/PBCH positions are independent of whether a
    // grant was found, so the UI shouldn't be left showing a stale frame.
    _rest._ce_values      = std::move(ce_values());
    _rest._cir_values     = std::move(cir_values());
    _rest._cas_grid       = std::move(cas_grid());
    _rest._cas_composition = std::move(composition_grid());
    _rest.record_subframe_event(tti, RestHandler::SF_EVENT_CAS, RestHandler::SF_STATUS_IDLE);
    _rest._pdcch.errors++;
  }
    _mutex.unlock();
  return true;
}

auto CasFrameProcessor::ce_values() -> std::vector<uint8_t> {
  // Same nof_prb-alone sizing as cir_values() above -- fft[port]'s FFT stays
  // permanently narrow regardless of mbsfn_prb, see that function's comment.
  auto sz = (uint32_t)srsran_symbol_sz(_cell.nof_prb);
  // Floor with -80, not a raw 0-fill: this padding region never goes through
  // srsran_vec_abs_dB_cf()'s own floor below, so a bare 0.0f reads as a
  // strong, wrong signal (waterfall_color()'s -20..25dB ramp maps db=0 to
  // solid green) instead of background/no-signal. Same bug and fix as
  // MbsfnFrameProcessor::ce_values() (2026-07-18).
  std::vector<float> ce_abs(sz, -80.0f);
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

auto CasFrameProcessor::composition_grid() -> std::vector<uint8_t> {
  const uint32_t nsymb      = SRSRAN_CP_NSYMB(_cell.cp); // 6 (ECP) or 7 (NCP) per slot
  // NOT max(nof_prb, mbsfn_prb) - unlike cir_values()/ce_values() (which do need the
  // wider canvas, since they represent the actual MBSFN-adjacent sample stream), every
  // element this function marks (PBCH/PSS/SSS/CRS/PCFICH/PDCCH) is CAS-domain content
  // that only ever occupies the carrier's own nof_prb width, never the wider PMCH
  // allocation. Widening this canvas to mbsfn_prb left PBCH/PSS/SSS (computed centred
  // in `subcarriers`) misaligned against CRS/PCFICH/PDCCH (computed from narrow,
  // uncentred nof_prb-relative indices below) - confirmed live, 2026-07-18, this is
  // exactly the "CAS composition looks wrong"/"CAS is misplaced" symptom re-appearing
  // whenever mbsfn_prb > nof_prb, even though CAS's own content never changes.
  const uint32_t subcarriers = _cell.nof_prb * SRSRAN_NRE;
  const uint32_t symbols     = 2 * nsymb;
  std::vector<uint8_t> comp(subcarriers * symbols, COMP_OTHER);
  auto mark = [&](uint32_t l, uint32_t k, uint8_t v) {
    if (l < symbols && k < subcarriers) comp[l * subcarriers + k] = v;
  };

  // --- PBCH: standard base (slot 1, symbols 0..3) --------------------------
  // Mirrors srsran_pbch_cp()'s layout (pbch.c): 72 contiguous-ish subcarriers
  // centred on DC, across the first 4 symbols of slot 1. The exact per-RE
  // interleave with CRS on symbols 0/1/(3 for ECP) is handled by CRS marking
  // them over this afterwards, below - those REs really are CRS, not PBCH.
  {
    const uint32_t pbch_k0 = subcarriers / 2 - 36;
    const uint32_t nof_pbch_symbols = SRSRAN_CP_ISNORM(_cell.cp) ? 4 : 4;
    for (uint32_t i = 0; i < nof_pbch_symbols; i++) {
      for (uint32_t k = 0; k < 72; k++) mark(nsymb + i, pbch_k0 + k, COMP_PBCH);
    }
  }

  // --- PBCH: this fork's CAS repetition (TS 103 720, see pbch.c's
  // PBCH_CAS_MAP_NCP/ECP) - extra copies of PBCH content at fixed (slot,
  // symbol) positions beyond the standard subframe-0-only placement, so a
  // CAS occasion on an MBMS-dedicated (no regular scheduling) carrier can
  // recover MIB without waiting for subframe 0 specifically. Same 72-SC
  // centred range as the standard case.
  {
    struct Map { uint32_t dst_ns, dst_l; };
    static const Map MAP_NCP[] = {{0, 4}, {1, 4}, {1, 5}, {0, 3}, {1, 6}};
    static const Map MAP_ECP[] = {{0, 3}, {1, 4}, {1, 5}};
    const uint32_t pbch_k0 = subcarriers / 2 - 36;
    const Map* map = SRSRAN_CP_ISNORM(_cell.cp) ? MAP_NCP : MAP_ECP;
    size_t map_n = SRSRAN_CP_ISNORM(_cell.cp) ? (sizeof(MAP_NCP) / sizeof(Map)) : (sizeof(MAP_ECP) / sizeof(Map));
    for (size_t i = 0; i < map_n; i++) {
      uint32_t l = map[i].dst_ns * nsymb + map[i].dst_l;
      for (uint32_t k = 0; k < 72; k++) mark(l, pbch_k0 + k, COMP_PBCH);
    }
  }

  // --- PSS / SSS: last slot of subframes 0 and 5 only (standard LTE, no
  // CAS-specific repetition exists for these in this fork). Symbol/subcarrier
  // formula mirrors srsran_pss_put_slot()/srsran_sss_put_slot() exactly
  // (pss.c/sss.c): last symbol of the slot for PSS, second-to-last for SSS,
  // both 62 contiguous subcarriers centred on DC.
  {
    uint32_t sf_idx = _sf_cfg.tti % 10;
    if (sf_idx == 0 || sf_idx == 5) {
      const uint32_t k0 = subcarriers / 2 - 31;
      const uint32_t pss_l = 2 * nsymb - 1;
      const uint32_t sss_l = 2 * nsymb - 2;
      for (uint32_t k = 0; k < 62; k++) {
        mark(pss_l, k0 + k, COMP_PSS);
        mark(sss_l, k0 + k, COMP_SSS);
      }
    }
  }

  // --- CRS: srsran_refsignal_cs_nsymbol()/cs_fidx()/cs_v() exactly, for
  // every configured port - the same functions the receiver's own channel
  // estimator uses (refsignal_dl.c), so this can never disagree with what
  // chest_dl actually read as reference symbols.
  for (uint32_t port = 0; port < _cell.nof_ports; port++) {
    uint32_t n_l = srsran_refsignal_cs_nof_symbols(nullptr, &_sf_cfg, port);
    for (uint32_t l = 0; l < n_l; l++) {
      uint32_t nsymbol = srsran_refsignal_cs_nsymbol(l, _cell.cp, port);
      // Matches srsran_refsignal_cs_fidx() exactly (refsignal_dl.c): starting
      // offset is (v + cell.id % 6) % 6, NOT just v - missing the cell-ID
      // shift here would silently produce wrong positions for any cell.id
      // where cell.id % 6 != 0.
      uint32_t fidx = (srsran_refsignal_cs_v(port, l) + (_cell.id % 6)) % 6;
      for (uint32_t i = 0; i < 2 * _cell.nof_prb; i++) {
        mark(nsymbol, fidx, COMP_CRS);
        fidx += SRSRAN_NRE / 2;
      }
    }
  }

  // --- PCFICH: REGs already computed by srsRAN internally (from cell.id/
  // cell.nof_prb, at set_cell() time) for the current CFI hypothesis -
  // PCFICH's own REG set doesn't depend on CFI, so index [0] always holds it.
  {
    srsran_regs_ch_t& pcfich = _ue_dl.regs[0].pcfich;
    for (uint32_t i = 0; i < pcfich.nof_regs; i++) {
      srsran_regs_reg_t* reg = pcfich.regs[i];
      for (uint32_t j = 0; j < 4; j++) mark(reg->l, reg->k[j], COMP_PCFICH);
    }
  }

  // --- PDCCH: the exact REGs backing the candidate(s) actually decoded this
  // occasion (from _last_pdcch_locations, set in process() right after
  // srsran_ue_dl_find_dl_dci() returns) - ground truth from this frame's own
  // blind decode, not a guessed position. REGs for CFI=cfi are ordered by
  // increasing CCE (9 REGs/CCE) in _ue_dl.regs[cfi-1].pdcch.regs[].
  {
    uint32_t cfi = _sf_cfg.cfi > 0 ? _sf_cfg.cfi : 1;
    if (cfi >= 1 && cfi <= 3) {
      srsran_regs_ch_t& pdcch = _ue_dl.regs[0].pdcch[cfi - 1];
      for (const auto& loc : _last_pdcch_locations) {
        uint32_t ncce = loc.first, L = loc.second;
        uint32_t first_reg = ncce * 9, nof_regs = (1u << L) * 9u;
        for (uint32_t r = first_reg; r < first_reg + nof_regs && r < pdcch.nof_regs; r++) {
          srsran_regs_reg_t* reg = pdcch.regs[r];
          for (uint32_t j = 0; j < 4; j++) mark(reg->l, reg->k[j], COMP_PDCCH);
        }
      }
    }
  }

  return comp;
}

auto CasFrameProcessor::pdcch_data() -> std::vector<uint8_t> {
  const uint8_t* data = reinterpret_cast<uint8_t*>(_ue_dl.pdcch.d);
  return std::vector<uint8_t>(data, data + _last_pdcch_nof_re * sizeof(cf_t));
}
