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
#include <cstring>

std::map<std::pair<uint8_t,uint8_t>, uint16_t> MbsfnFrameProcessor::_sched_stops;

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

  /* Slot 0 is the only slot used when time interleaving isn't configured
   * (by far the common case) - init it eagerly so behavior/cost for that
   * case is unchanged. Slots 1..M-1 (time-interleaving pipelining) are
   * lazily init'd in process() the first time they're actually used. */
  srsran_softbuffer_rx_init(&_softbuffer[0], 100);
  _softbuffer_init[0] = true;

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
  _ue_dl_cfg.cfg.pdsch.softbuffers.rx[0] = &_softbuffer[0];

  _pmch_cfg.pdsch_cfg.csi_enable         = true;
  _pmch_cfg.pdsch_cfg.max_nof_iterations = 8;
  _pmch_cfg.pdsch_cfg.meas_evm_en        = true;
  _pmch_cfg.pdsch_cfg.decoder_type       = SRSRAN_MIMO_DECODER_MMSE;

  _sf_cfg.sf_type = SRSRAN_SF_MBSFN;
  return true;
}

MbsfnFrameProcessor::~MbsfnFrameProcessor() {
  for (uint32_t i = 0; i < SRSRAN_PMCH_MAX_TI_M; i++) {
    if (_softbuffer_init[i]) {
      srsran_softbuffer_rx_free(&_softbuffer[i]);
    }
  }
  srsran_ue_dl_free(&_ue_dl);
  if (_cir_plan_ready) {
    srsran_dft_plan_free(&_cir_plan);
  }
}

void MbsfnFrameProcessor::set_cell(srsran_cell_t cell) {
  _cell = cell;
  srsran_ue_dl_set_cell(&_ue_dl, cell);

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
  _ce_scratch_db.resize(sz);
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
    _rest.record_subframe_event(tti, RestHandler::SF_EVENT_GAP, RestHandler::SF_STATUS_IDLE);
    _mutex.unlock();
    return -1;
  }

  /* MCCH is never time-interleaved (a time-interleaved MCH must not carry
   * MCCH, per TS 36.300 §15.3.3) -- count it immediately, once per subframe,
   * same as always. MCH's total++ is deferred below: when time-interleaving
   * is active, a logical TB spans N subframes, and counting "total" once per
   * subframe here while "errors" (further below) only fires once per TB
   * would silently skew any BLER computed from these two counters -- see the
   * matching comment where MCH's total/errors are actually counted. */
  if (mbsfn_cfg.is_mcch) {
    _rest._mcch.total++;
  }

  /* Switch FFT SCS and chest refs for MCCH subframes when MCCH SCS differs from data SCS.
   * For 0.37 kHz and 2.5 kHz data carriers the MCCH is transmitted at 1.25 kHz;
   * for 7.5 kHz the MCCH SCS matches the data SCS (no switch needed).
   * Mirror TX cc_worker which calls srsran_enb_dl_set_mbsfn_subcarrier_spacing() per-TTI. */
  srsran_scs_t saved_scs = _sf_cfg.subcarrier_spacing;
  bool scs_switched = mbsfn_cfg.subcarrier_spacing != saved_scs;
  if (scs_switched) {
    srsran_ue_dl_set_mbsfn_subcarrier_spacing(&_ue_dl, mbsfn_cfg.subcarrier_spacing);
    srsran_ue_dl_set_mbsfn_area_id(&_ue_dl, _area_id);
    _sf_cfg.subcarrier_spacing = mbsfn_cfg.subcarrier_spacing;
  }

  auto restore_scs = [&]() {
    srsran_ue_dl_set_mbsfn_subcarrier_spacing(&_ue_dl, saved_scs);
    srsran_ue_dl_set_mbsfn_area_id(&_ue_dl, _area_id);
    _sf_cfg.subcarrier_spacing = saved_scs;
  };

  if (srsran_ue_dl_decode_fft_estimate(&_ue_dl, &_sf_cfg, &_ue_dl_cfg) < 0) {
    /* A hard failure independent of time-interleaving's soft-combining state
     * (this subframe never even reached the decode attempt) -- count it as
     * its own standalone total+error immediately, same for MCH whether or
     * not time-interleaving is active. */
    if (mbsfn_cfg.is_mcch) {
      _rest._mcch.errors++;
    } else {
      _rest._mch[mch_idx].total++;
      _rest._mch[mch_idx].errors++;
    }
    spdlog::error("Getting PDCCH FFT estimate");
    _rest.record_subframe_event(
        tti, mbsfn_cfg.is_mcch ? RestHandler::SF_EVENT_MCCH : RestHandler::SF_EVENT_MCH, RestHandler::SF_STATUS_FAIL);
    if (scs_switched) { restore_scs(); }
    _mutex.unlock();
    return -1;
  }

  srsran_configure_pmch(&_pmch_cfg, &_cell, &mbsfn_cfg);
  srsran_ra_dl_compute_nof_re(&_cell, &_sf_cfg, &_pmch_cfg.pdsch_cfg.grant);

  _pmch_cfg.area_id             = _area_id;
  _pmch_cfg.cyclic_shift        = mbsfn_cfg.cyclic_shift;
  _pmch_cfg.cyclic_shift_alpha  = mbsfn_cfg.cyclic_shift_alpha;
  _pmch_cfg.freq_interleaving   = mbsfn_cfg.freq_interleaving;
  _pmch_cfg.use_mcs_table2      = mbsfn_cfg.use_mcs_table2;
  _pmch_cfg.time_interleaving_n = mbsfn_cfg.time_interleaving_n;
  _pmch_cfg.time_interleaving_m = mbsfn_cfg.time_interleaving_m;
  _pmch_cfg.subframe_idx        = mbsfn_cfg.mch_subframe_idx;

  if (!mbsfn_cfg.is_mcch && getenv("PMCH_TI_DIAG")) {
    fprintf(stderr,
            "TI_DIAG_MFP this=%p tti=%u is_mcch=%d ti_n=%u ti_m=%u mch_subframe_idx=%u\n",
            (void*)this, tti, (int)mbsfn_cfg.is_mcch, mbsfn_cfg.time_interleaving_n,
            mbsfn_cfg.time_interleaving_m, mbsfn_cfg.mch_subframe_idx);
  }

  /* TS 36.213 §11.1 (see srsran_pmch_decode's comment in pmch.c for the full
   * derivation): subframe s=mch_subframe_idx belongs to slot m=s%M with
   * redundancy version n=(s%(N*M))/M. srsran_pmch_decode's per-subframe
   * rate-matching relies on slot m's OWN softbuffer LLR/CRC state persisting
   * across the N subframes of ITS span (that's how the soft-combining
   * happens - each subframe's rv_idx-specific rate-matching pass adds its
   * LLRs to that slot's buffer). Resetting every subframe (the previous,
   * unconditional behavior) would wipe that progress before it can
   * accumulate; resetting one shared buffer regardless of slot would
   * corrupt one slot's progress with another's. So: reset slot m's buffer
   * only when slot m starts a new TB (n==0 for that slot), or every
   * subframe when time interleaving isn't configured (N<=1, slot m is
   * always 0), matching the previous behavior exactly for that case. */
  bool     ti_active = mbsfn_cfg.time_interleaving_n > 1;
  /* Clamp defensively: time_interleaving_m is decoded from broadcast MCCH
   * data (not locally-trusted state), and ti_slot_m below indexes fixed-size
   * SRSRAN_PMCH_MAX_TI_M arrays - a value in this uint8_t field above that
   * bound (only reachable via a malformed/unexpected broadcast, valid RRC
   * values are 4/8/16/32) must not turn into an out-of-bounds access. */
  uint8_t ti_m_cfg = mbsfn_cfg.time_interleaving_m;
  if (ti_m_cfg == 0) {
    ti_m_cfg = 1;
  } else if (ti_m_cfg > SRSRAN_PMCH_MAX_TI_M) {
    ti_m_cfg = SRSRAN_PMCH_MAX_TI_M;
  }
  uint32_t ti_block_len = ti_active ? ((uint32_t)mbsfn_cfg.time_interleaving_n * (uint32_t)ti_m_cfg) : 1;
  uint32_t ti_s_mod   = ti_active ? (mbsfn_cfg.mch_subframe_idx % ti_block_len) : 0;
  uint32_t ti_slot_m  = ti_active ? (ti_s_mod % ti_m_cfg) : 0;
  uint32_t ti_slot_n  = ti_active ? (ti_s_mod / ti_m_cfg) : 0;

  if (!_softbuffer_init[ti_slot_m]) {
    srsran_softbuffer_rx_init(&_softbuffer[ti_slot_m], 100);
    _softbuffer_init[ti_slot_m] = true;
  }
  if (!ti_active || ti_slot_n == 0) {
    srsran_softbuffer_rx_reset_cb(&_softbuffer[ti_slot_m], 1);
    _ti_reported[ti_slot_m] = false;
  }

  srsran_pdsch_res_t pmch_dec = {};
  _pmch_cfg.pdsch_cfg.softbuffers.rx[0] = &_softbuffer[ti_slot_m];
  pmch_dec.payload = _payload_buffer;
  if (!ti_active || ti_slot_n == 0) {
    srsran_softbuffer_rx_reset_tbs(_pmch_cfg.pdsch_cfg.softbuffers.rx[0], _pmch_cfg.pdsch_cfg.grant.tb[0].tbs);
  }

  if (srsran_ue_dl_decode_pmch(&_ue_dl, &_sf_cfg, &_pmch_cfg, &pmch_dec) != 0) {
    /* Genuine execution error (srsran_ue_dl_decode_pmch's return value is
     * reserved for that, never for a plain CRC miss -- see srsran_pmch_decode's
     * own convention). Count immediately and mark this slot's TB as reported
     * so a later subframe in the same span (if ti_active) doesn't also count
     * a second, redundant outcome for what is really the same logical TB. */
    if (mbsfn_cfg.is_mcch) {
      _rest._mcch.errors++;
    } else {
      _rest._mch[mch_idx].total++;
      _rest._mch[mch_idx].errors++;
      _ti_reported[ti_slot_m] = true;
    }
    spdlog::warn("Error decoding PMCH");
    _rest.record_subframe_event(
        tti, mbsfn_cfg.is_mcch ? RestHandler::SF_EVENT_MCCH : RestHandler::SF_EVENT_MCH, RestHandler::SF_STATUS_FAIL);
    if (scs_switched) { restore_scs(); }
    _mutex.unlock();
    return -1;
  }

  if (scs_switched) { restore_scs(); }

  /* PMCH DTX detection. The eNB legitimately transmits NO PMCH on MCH data
   * subframes where MAC has nothing scheduled (cc_worker::encode_pmch returns
   * early when dci.rnti==0 - idle MTCH window), so what arrives here is an
   * RS-only subframe with empty data REs. Decoding it anyway is meaningless:
   * the all-zero LLR input usually "succeeds" by converging to the valid
   * all-zero codeword, but a +/-1-sample timing-dither at a CAS re-track
   * breaks FFT circularity just enough (~-60dB RS leakage into the empty data
   * bins, confirmed by raw-capture cross-correlation: failing subframes show
   * the RX window at lag=1 vs the TX waveform, passing ones at lag=0) to
   * occasionally flip that into a CRC failure - both outcomes are artifacts
   * of decoding a non-transmitted TB, and the failures polluted the MCH BLER
   * (~0.15%, always sf5, in 4-frame bursts = one CAS re-track period).
   * Standard receiver practice (cf. PDCCH DTX detection): gate on received
   * energy. Equalized data symbols of a real QPSK TB have ~unit average
   * power; an absent PMCH leaves ~0 (exactly 0 on a clean channel, noise-level
   * otherwise). Threshold 1e-3 sits ~30dB below real signal and well above
   * the leakage artifact. MCCH is never DTX (always transmitted). */
  if (!mbsfn_cfg.is_mcch) {
    float data_pw = srsran_vec_avg_power_cf(_ue_dl.pmch.d, _pmch_cfg.pdsch_cfg.grant.nof_re);
    if (data_pw < 1e-3f) {
      _rest.record_subframe_event(tti, RestHandler::SF_EVENT_MCH, RestHandler::SF_STATUS_IDLE);
      spdlog::trace("PMCH DTX in TTI {} (data-RE power {}), skipping decode accounting", tti, data_pw);
      _mutex.unlock();
      return 0;
    }
  }

  /* Temporary, env-gated (MCH_SF5_DIAG) per-subframe diagnostic for the
   * occasional subframe-5 PMCH CRC failures. Captures BOTH pass and fail so the
   * failing rows can be compared against the passing ones: snr_db distinguishes
   * an RF/estimate cause (low snr at failure) from a decode-config mismatch
   * (high snr but crc=0 => wrong mch_subframe_idx/mcs/tbs/grant). Silent unless
   * the env var is set, so it costs nothing in normal operation. */
  if (!mbsfn_cfg.is_mcch && getenv("MCH_SF5_DIAG")) {
    // Extra discriminators: evm (symbol-domain quality) and n_iter (turbo
    // iterations). Low evm + few iters but crc=0 => clean symbols, wrong bits
    // => RX decoding a TB the TX didn't send there. High evm/max iters => the
    // symbols themselves are corrupt (RE-extraction / channel). rx_pwr gauges
    // whether the TX transmitted anything at all on this subframe.
    float rx_pwr = srsran_vec_avg_power_cf(_ue_dl.sf_symbols[0], _ue_dl.cell.nof_prb * SRSRAN_NRE);
    fprintf(stderr,
            "MCHDIAG sfn=%u sf=%u mch_sf_idx=%u pmch_idx=%u mcs=%d nof_re=%d tbs=%d crc=%d evm=%.4f n_iter=%.2f rxpwr=%.3e\n",
            sfn, (unsigned)sf, mbsfn_cfg.mch_subframe_idx, mch_idx,
            _pmch_cfg.pdsch_cfg.grant.tb[0].mcs_idx,
            _pmch_cfg.pdsch_cfg.grant.nof_re, (int)_pmch_cfg.pdsch_cfg.grant.tb[0].tbs,
            (int)pmch_dec.crc, pmch_dec.evm, pmch_dec.avg_iterations_block, rx_pwr);
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
    _rest._mcch.evm_rms = pmch_dec.evm;
  } else {
    _rest._mch[mch_idx].SetData(mch_data());
    _rest._mch[mch_idx].mcs = _pmch_cfg.pdsch_cfg.grant.tb[0].mcs_idx;
    _rest._mch[mch_idx].evm_rms = pmch_dec.evm;
    _rest._mch[mch_idx].present = true;
  }
  if (++_ce_cir_update_counter >= CE_CIR_UPDATE_STRIDE) {
    _ce_cir_update_counter  = 0;
    // Fill persistent member buffers (reusing their capacity), then publish by
    // O(1) swap - the previously-published buffer comes back into the member for
    // reuse next cycle, so this is allocation-free in steady state. Same
    // pointer-swap publish semantics as the previous std::move assignment.
    ce_values(_ce_out_bytes);
    cir_values(_cir_out_bytes);
    _rest._ce_values_mbsfn.swap(_ce_out_bytes);
    _rest._cir_values_mbsfn.swap(_cir_out_bytes);
  }

  if (pmch_dec.crc) {
    /* srsran_pmch_decode() reports crc=true at most once per slot's N-span
     * (it forces crc=false on every subsequent subframe once that slot's
     * ti_decoded flag is set, to avoid re-delivering the same TB) -- so this
     * is always a fresh outcome for MCH, never a repeat. Count total++ here,
     * matched 1:1 with the errors++ below (deferred to the same per-TB
     * granularity, not per-subframe) so a BLER computed from these two
     * counters is meaningful whether or not time-interleaving is active. */
    if (!mbsfn_cfg.is_mcch) {
      _rest._mch[mch_idx].total++;
      if (ti_active) {
        _ti_reported[ti_slot_m] = true;
      }
    }
    mch_mac_msg.init_rx(
        static_cast<uint32_t>(_pmch_cfg.pdsch_cfg.grant.tb[0].tbs) / 8);
    mch_mac_msg.parse_packet(_payload_buffer);

    while (mch_mac_msg.next()) {
      if (getenv("PMCH_TI_DIAG")) {
        auto* s = mch_mac_msg.get();
        bool sdu = s->is_sdu();
        fprintf(stderr, "TI_DIAG_SUBH sfidx=%u is_mcch=%d is_sdu=%d ce_type=%d lcid=%d\n",
                mbsfn_cfg.mch_subframe_idx, (int)mbsfn_cfg.is_mcch, (int)sdu,
                (int)s->mch_ce_type(), sdu ? (int)s->get_sdu_lcid() : -1);
      }
      if (srsran::mch_lcid::MCH_SCHED_INFO == mch_mac_msg.get()->mch_ce_type()) {
        uint16_t stop = 0;
        uint8_t lcid = 0;
        /* pmch-TimeInterleavingN/M-LastMTCH-r19 (TS 36.331 CR5168r3): track the two
         * highest stop values decoded from THIS period's own fresh MSI content (not
         * the persistent _sched_stops map below, which can carry stale entries from
         * earlier periods with a different session composition). TX encodes these
         * CEs in schedule order (mac.cc's mtch_sched[] order, cumulative stop values),
         * so the highest stop is the overall period boundary (mtch_stop) and the
         * second-highest is where the last session's own window starts -- exactly
         * mirroring TX's mtch_sched[num_mtch_sched-2].stop. */
        uint16_t highest_stop = 0, second_highest_stop = 0;
        /* TS 36.321 §6.1.3.7a: for a time-interleaved MCH the MSI is an Extended MSI,
         * whose scheduling entries (LCID+StopMTCH, one per MTCH, identical 2-octet
         * format to the regular MSI in §6.1.3.7) are FOLLOWED by optional MTCH-suspend
         * (LCID+S, 1 octet) sub-elements. Those suspend octets must not be misread as
         * further 2-octet scheduling entries, so cap the scheduling read at the number
         * of MTCHs configured for this PMCH (from the MCCH). Regular (non-TI) MSI has
         * only scheduling entries and is left uncapped (unchanged behaviour). The suspend
         * sub-elements are a UE power-saving hint (which MTCHs to stop decoding); a
         * forwarding receiver has no consumer for them, so they are parsed-past, not
         * acted upon. */
        const srsran::mcch_msg_t& mcch_msi = _phy.current_mcch();
        uint32_t sched_cap = UINT32_MAX;
        if (mch_idx < mcch_msi.nof_pmch_info && mcch_msi.pmch_info_list[mch_idx].time_interleaving_n > 1) {
          sched_cap = mcch_msi.pmch_info_list[mch_idx].nof_mbms_session_info;
        }
        uint32_t sched_cnt = 0;
        while (sched_cnt < sched_cap && mch_mac_msg.get()->get_next_mch_sched_info(&lcid, &stop)) {
          sched_cnt++;
          const std::lock_guard<std::mutex> lock(_sched_stop_mutex);
          spdlog::debug("Scheduling stop for PMCH {} LCID {} in sf {}", mch_idx, lcid, stop);
          _sched_stops[ {(uint8_t)mch_idx, lcid} ] = stop;
          if (stop > highest_stop) {
            second_highest_stop = highest_stop;
            highest_stop         = stop;
          } else if (stop > second_highest_stop) {
            second_highest_stop = stop;
          }
        }
        /* second_highest_stop stays 0 for a single-session period (only one CE
         * decoded), which set_last_mtch_start()/mbsfn_config_for_tti() already
         * treat as "no distinct last-session window" -- the common case is
         * unaffected. */
        _phy.set_last_mtch_start((uint8_t)mch_idx, second_highest_stop);
      } else if (mch_mac_msg.get()->is_sdu()) {
        uint32_t lcid = mch_mac_msg.get()->get_sdu_lcid();
        spdlog::trace("Processing MAC MCH PDU entered, lcid {}", lcid);

        /* TS 36.321 Table 6.2.1-4: LCID 0 within an MCH MAC PDU is reserved
         * for MCCH specifically - a regular MTCH data subframe must never
         * legitimately carry it. Without this check, a data subframe with no
         * real MAC content queued (TX has nothing to send, e.g. no MBMS
         * traffic source configured) still transmits a well-formed all-zero
         * PMCH TB (see rt-mbms-tx's encode_pmch), which decodes successfully
         * as a degenerate case; its first MAC subheader byte (0x00: E-bit=0,
         * LCID=0) then gets misread as an MCCH SDU and misdelivered to the
         * RRC/MCCH handler, corrupting Phy::_mcch (nof_pmch_info reset to 0)
         * until the next real MCCH occasion overwrites it. */
        if (lcid == (uint32_t)srsran::mch_lcid::MCCH && !mbsfn_cfg.is_mcch) {
          /* warn (not debug) would be appropriate severity-wise, but this fires on
           * EVERY MCH subframe whenever no real MBMS traffic is queued (the normal
           * idle state, not an error) - at up to ~1kHz that's synchronous log I/O
           * flooding the real-time decode thread, which was actually causing
           * periodic CINR/sync glitches (confirmed via a live capture correlating
           * dips with this exact log line's timestamps). debug so it's silent at
           * this project's default -l 2 (info) but still available via -l 0/1. */
          spdlog::debug("Dropping spurious MCCH-LCID SDU decoded from a non-MCCH subframe (mch_idx {})", mch_idx);
          continue;
        }

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
          if (!mbsfn_cfg.is_mcch && getenv("PMCH_TI_DIAG")) {
            uint8_t* p = mch_mac_msg.get()->get_sdu_ptr();
            uint32_t sz = mch_mac_msg.get()->get_payload_size();
            char hex[97] = {0};
            for (uint32_t k = 0; k < sz && k < 48; k++) {
              snprintf(hex + k * 2, 3, "%02x", p[k]);
            }
            fprintf(stderr, "TI_DIAG_MACSDU sfidx=%u lcid=%u sz=%u first48=%s\n", mbsfn_cfg.mch_subframe_idx, lcid, sz, hex);
          }
          _phy._mcs = mbsfn_cfg.mbsfn_mcs;
          const std::lock_guard<std::mutex> lock(_rlc_mutex);
          _rlc.write_pdu_mch(mch_idx, lcid, mch_mac_msg.get()->get_sdu_ptr(), mch_mac_msg.get()->get_payload_size());
        }
      }
    }
  } else {
    /* This slot's current TB already succeeded earlier in its own N-span
     * (crc=true branch above already ran and counted it) -- this call is
     * just one of the redundant trailing "already decoded" subframes
     * srsran_pmch_decode() reports as crc=false by design. Not a new
     * outcome; nothing to count. Without this check, MCH's total/errors
     * would double-count the same logical TB and this trailing subframe
     * would be misreported as a fresh failure. */
    if (!mbsfn_cfg.is_mcch && ti_active && _ti_reported[ti_slot_m]) {
      _rest.record_subframe_event(tti, RestHandler::SF_EVENT_MCH, RestHandler::SF_STATUS_OK);
      _mutex.unlock();
      return 0;
    }
    /* Rel-19 §6.5.3: partial accumulation — not a real failure, waiting for
     * remaining subframes of this slot's own N-span (ti_slot_n/ti_active
     * computed earlier from the same (m,n) split used for the softbuffer). */
    if (ti_active && ti_slot_n < (uint32_t)(mbsfn_cfg.time_interleaving_n - 1u)) {
      _rest.record_subframe_event(tti, RestHandler::SF_EVENT_MCH, RestHandler::SF_STATUS_IDLE);
      _mutex.unlock();
      return 0;
    }
    if (mbsfn_cfg.is_mcch) {
      _rest._mcch.errors++;
    } else {
      /* Reached the last subframe of this TB's span and it never decoded --
       * a genuine failure. Count total++ here (matching errors++'s
       * granularity, once per TB, not once per subframe -- see the crc=true
       * branch's matching comment) and mark reported so this same outcome
       * can't be double-counted if somehow called again before the next
       * slot-m reset. */
      _rest._mch[mch_idx].total++;
      _rest._mch[mch_idx].errors++;
      if (ti_active) {
        _ti_reported[ti_slot_m] = true;
      }
    }
    _rest.record_subframe_event(
        tti, mbsfn_cfg.is_mcch ? RestHandler::SF_EVENT_MCCH : RestHandler::SF_EVENT_MCH, RestHandler::SF_STATUS_FAIL);

    /* Failure-triggered raw-sample dump: the sporadic sf5 failures drift in
     * tti even within a run, so only a capture taken AT the failure itself is
     * race-free. This processor still owns _signal_buffer_rx (mutex held), so
     * dump the exact time-domain input whose decode just failed CRC, for
     * offline FFT/window-offset comparison against the TX's (verified
     * bit-identical) postifft reference. Env-gated; ~1 failure per 1-3s and
     * 120kB each, bounded by tti-name wraparound. */
    if (!mbsfn_cfg.is_mcch && getenv("PMCH_RE_DUMP")) {
      char fn[128];
      snprintf(fn, sizeof(fn), "/tmp/pmch_rx_rawFAIL_tti%u.bin", tti);
      FILE* fr = fopen(fn, "wb");
      if (fr) {
        fwrite(_signal_buffer_rx[0], sizeof(cf_t), 15360, fr);
        fclose(fr);
      }
      fprintf(stderr, "[PMCH_RE_DUMP] RX RAWFAIL tti=%u sfn=%u sf=%u mch_sf_idx=%u\n",
              tti, sfn, (unsigned)sf, mbsfn_cfg.mch_subframe_idx);
    }

    spdlog::trace("PMCH in TTI {} failed with CRC error", tti);
    _mutex.unlock();
    return -1;
  }

  if (!mbsfn_cfg.is_mcch) {
    /* Use _pmch_cfg.subframe_idx (= mch_subframe_idx from Phy::mbsfn_config_for_tti) as the
     * allocation index for sched_stop comparison.  This is the 0-based per-PMCH index
     * used by the TX when it sets the MCH stop values in the MCCH scheduling info.
     * Only check stops keyed to the current PMCH (mch_idx). */
    unsigned sf_idx = _pmch_cfg.subframe_idx;
    spdlog::debug("tti{}, sfn {}, sf {}, mch_idx {}, sf_idx (mch_subframe_idx) {}", tti, sfn, sf, mch_idx, sf_idx);

    const std::lock_guard<std::mutex> lock(_sched_stop_mutex);
    for (auto itr = _sched_stops.cbegin(); itr != _sched_stops.cend();) {
      if (itr->first.first != (uint8_t)mch_idx) {
        itr = std::next(itr);
        continue;
      }
      if (sf_idx >= itr->second) {
        uint8_t lcid = itr->first.second;
        spdlog::debug("Stopping PMCH {} LCID {} in tti {} (idx in rf {})", mch_idx, lcid, tti, sf_idx);
        const std::lock_guard<std::mutex> lock(_rlc_mutex);
        if (!_allow_rrc_sn_across_periods) {
          _rlc.stop_mch(mch_idx, lcid);
        }
        itr = _sched_stops.erase(itr);
      } else {
        itr = std::next(itr);
      }
    }
  } else {
    {
      const std::lock_guard<std::mutex> lock(_sched_stop_mutex);
      _sched_stops.clear();
    }
    const std::lock_guard<std::mutex> lock(_rlc_mutex);
    _rlc.stop_mch(0, 0);
    _rest._mcch.present = true;
  }
  _rest.record_subframe_event(
      tti, mbsfn_cfg.is_mcch ? RestHandler::SF_EVENT_MCCH : RestHandler::SF_EVENT_MCH, RestHandler::SF_STATUS_OK);
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

void MbsfnFrameProcessor::ce_values(std::vector<uint8_t>& out) {
  auto sz = _cir_plan_size; // set once in set_cell(), shared sizing for ce/cir scratch
  if (sz == 0) {
    out.clear(); // set_cell() hasn't run yet - nothing to compute.
    return;
  }
  float* ce_abs = _ce_scratch_db.data();
  memset(ce_abs, 0, sz * sizeof(float));
  uint32_t g = (sz - 12 * _cell.nof_prb) / 2;
  srsran_vec_abs_dB_cf(_ue_dl.chest_res.ce[0][0], -80, &ce_abs[g], SRSRAN_NRE * _cell.nof_prb);
  // assign() reuses out's existing capacity - no heap allocation after the
  // first call (out is a persistent member, swapped into _rest below).
  const uint8_t* data = reinterpret_cast<const uint8_t*>(ce_abs);
  out.assign(data, data + sz * sizeof(float));
}

void MbsfnFrameProcessor::cir_values(std::vector<uint8_t>& out) {
  auto sz = _cir_plan_size;
  if (!_cir_plan_ready || sz == 0) {
    out.clear(); // set_cell() hasn't run yet - nothing to compute.
    return;
  }

  // Same construction as CasFrameProcessor::cir_values(): zero-pad the
  // frequency-domain channel estimate to the full symbol width, IFFT it,
  // fftshift so lag 0 is centered, then take the magnitude in dB. All
  // scratch buffers are pre-sized in set_cell() - no allocation here.
  cf_t* ce_freq = _cir_scratch_freq.data();
  srsran_vec_cf_zero(ce_freq, sz);
  uint32_t g = (sz - 12 * _cell.nof_prb) / 2;
  memcpy(&ce_freq[g], _ue_dl.chest_res.ce[0][0], SRSRAN_NRE * _cell.nof_prb * sizeof(cf_t));

  cf_t* cir_time = _cir_scratch_time.data();
  srsran_dft_run_c(&_cir_plan, ce_freq, cir_time);

  cf_t* cir_shifted = _cir_scratch_shifted.data();
  for (uint32_t i = 0; i < sz; i++) {
    cir_shifted[i] = cir_time[(i + sz / 2) % sz];
  }
  float* cir_db = _cir_scratch_db.data();
  srsran_vec_abs_dB_cf(cir_shifted, -80, cir_db, sz);

  // assign() reuses out's existing capacity - no heap allocation after the
  // first call (out is a persistent member, swapped into _rest below).
  const uint8_t* data = reinterpret_cast<const uint8_t*>(cir_db);
  out.assign(data, data + sz * sizeof(float));
}
