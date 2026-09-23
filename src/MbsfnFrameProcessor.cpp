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
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <sched.h>

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
   * lazily init'd in process() the first time they're actually used.
   *
   * PMCH_MAX_CB_TI (83), not the plain srsran_softbuffer_rx_init(q, 100)
   * sizing: that sizes max_cb from a single subframe's max TBS at 100 PRB
   * (21 CBs), but a TI'd MCH's per-TB code-block count is inflated by the TI
   * factor before srsran_ra_tbs_round_pmch_ti() rounds it against
   * pmch_ti_tbs.h's fixed tables, which saturate at 502624 bits regardless
   * of mbsfn_prb - a hard worst case of 502624/(SRSRAN_TCOD_MAX_LEN_CB-24)+1
   * = 83 CBs, about 4x the untuned sizing. sch.c hard-rejects any decode
   * where cb_segm->C > softbuffer->max_cb, so an undersized buffer here
   * silently blocks exactly the TI+wideband-MTCH combination this whole
   * investigation has been trying to get working. A fixed constant, not
   * dynamic per-mbsfn_prb sizing: the saturation behavior makes 83 a provable
   * worst case regardless of configured bandwidth/TI factor. */
  srsran_softbuffer_rx_init_guru(&_softbuffer[0], PMCH_MAX_CB_TI, SOFTBUFFER_SIZE);
  _softbuffer_init[0] = true;

  _ue_dl_cfg.snr_to_cqi_offset = 0;

  srsran_chest_dl_cfg_t* chest_cfg = &_ue_dl_cfg.chest_cfg;
  bzero(chest_cfg, sizeof(srsran_chest_dl_cfg_t));
  chest_cfg->filter_coef[0] = 0.1;
  chest_cfg->filter_type = SRSRAN_CHEST_FILTER_NONE;
  chest_cfg->noise_alg = SRSRAN_NOISE_ALG_EMPTY;
  chest_cfg->rsrp_neighbour       = false;
  /* RE-ENABLED 2026-07-19 (was force-disabled 2026-07-14 - see git history for
   * the original comment/reasoning). That decision was correct for its time:
   * the sync-error estimator's measured values ran into the hundreds (e.g.
   * -427, 311, -348) and its "correction" corrupted data/pilot REs alike,
   * root-caused as the primary cause of a ~99.6-99.75% CRC failure back then.
   * Confirmed live today that the underlying conditions have changed (this
   * session's enb_dl.c root-cause fix plus the CAS/PMCH FFT decoupling
   * redesign): SYNC_ERR_DIAG now shows a rock-stable, physically sane
   * measurement (~-14.00, std-dev <0.02 across many subframes) at the current
   * wideband pmch_bandwidth config, not the wild pre-fix values. Enabling the
   * correction moves the MBSFN CIR's main tap to exactly lag 0 (previously
   * off-center by ~14 samples) and substantially reduces - though does not
   * fully eliminate - a periodic ripple visible on the CE/CIR dashboard
   * panels. Does not change MCH BLER either way (confirmed separately still
   * 1.0 with this on or off) - that remaining decode failure has a different,
   * not-yet-found cause. See SIB13_MBSFN_TEST_RESULTS.md. */
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

void MbsfnFrameProcessor::set_cell(srsran_cell_t cell, srsran_scs_t mbsfn_scs) {
  /* _ue_dl's buffers (signal_buffer_rx et al.) are allocated once, in this class's own
   * init(), sized from the fixed MAX_PRB -- srsran_ue_dl_set_cell_scs() below has no
   * awareness of that outer allocation and will happily reconfigure the PHY chain for
   * anything cell.nof_prb/mbsfn_prb claims, so a wider request here would silently
   * write past those buffers rather than fail cleanly. Nothing today can actually
   * request more than 40 PRB (pmch-Bandwidth-r17's real ASN.1 range, TS 36.331 clause
   * 6.3.7, checked directly), comfortably under MAX_PRB=100 -- but
   * modem_zmqtest.conf's mbsfn_prb_test_override is an operator-set debug value with
   * no such ceiling, so this is a real boundary, not a hypothetical one. */
  if (cell.mbsfn_prb > MAX_PRB || cell.nof_prb > MAX_PRB) {
    spdlog::error("MbsfnFrameProcessor::set_cell: requested nof_prb={} mbsfn_prb={} exceeds MAX_PRB={} -- "
                  "clamping both to avoid overflowing buffers sized for MAX_PRB",
                  cell.nof_prb, cell.mbsfn_prb, MAX_PRB);
    cell.nof_prb   = std::min<uint32_t>(cell.nof_prb, MAX_PRB);
    cell.mbsfn_prb = std::min<uint32_t>(cell.mbsfn_prb, MAX_PRB);
  }
  _cell = cell;
  srsran_ue_dl_set_cell_scs(&_ue_dl, cell, mbsfn_scs);

  /* (Re)plan the CIR IFFT and size its scratch buffers here -- called only
   * from the single main thread on cell (re)configuration, never from
   * process()'s worker-pool thread, so there's no risk of concurrent
   * fftwf_plan_ / fftwf_destroy_plan calls racing across processor instances.
   *
   * FIXED (2026-07-19): this used srsran_symbol_sz(_cell.nof_prb) - the plain
   * 15kHz-numerology table - even though MBSFN here actually runs at a reduced
   * SCS (e.g. 1.25kHz, symbol_sz=12288 for 40 PRB, confirmed live via
   * fft_mbsfn's own cfg). That mismatch (a 12x-undersized canvas for 1.25kHz)
   * meant ce_values()/cir_values() below read only the first ~1/12th of the
   * real chest_res.ce[] array and IFFT'd it at the wrong transform size -
   * exactly the "PMCH only acquired at 25 instead of 40 PRB" / periodic
   * frequency-notch appearance seen on the CE waterfall and CIR dashboard
   * panels. This is a display-only bug (actual PMCH decode uses
   * _pmch_cfg.pdsch_cfg.grant.nof_re against the full-size ce[] array
   * directly, unaffected), but the wrong canvas size and RE-per-PRB stride
   * corrupt every value shown on those two panels. */
  auto sz = (uint32_t)srsran_symbol_sz_scs(_cell.nof_prb, mbsfn_scs);
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

  /* PROC_TIMING_DIAG: direct measurement of (a) how long this specific call
   * takes to reach the MCCHDIAG print, and (b) the wall-clock gap since the
   * previous call to this function - to test whether occasions with
   * elevated MCCH EVM correlate with unusually long processing or an
   * unusually late/gapped call, rather than continuing to guess at external
   * causes (bridge logging, CIR/CE stride, coarse CPU-frequency variance -
   * all tested and ruled out, see the comments on those). Static, so this is
   * genuinely the previous call's own entry time, not per-tti state. */
  auto t_entry = std::chrono::steady_clock::now();
  static std::chrono::steady_clock::time_point t_last_entry{};
  static bool have_last_entry = false;
  double interval_us = 0.0;
  if (have_last_entry) {
    interval_us = std::chrono::duration<double, std::micro>(t_entry - t_last_entry).count();
  }
  t_last_entry = t_entry;
  have_last_entry = true;

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
    // TEMPORARY (SCS_TIMING_DIAG): measure the actual wall-clock cost of
    // switching MBSFN SCS -- srsran_ue_dl_set_mbsfn_subcarrier_spacing()
    // replans an FFTW plan (FFTW_MEASURE mode) whenever the FFT size
    // changes, under a mutex shared with every other PHY worker thread.
    // Investigating whether this is expensive enough on the real-time
    // decode path to explain the n_prb=25 PMCH BLER / periodic sync loss.
    auto t0 = std::chrono::steady_clock::now();
    srsran_ue_dl_set_mbsfn_subcarrier_spacing(&_ue_dl, mbsfn_cfg.subcarrier_spacing);
    srsran_ue_dl_set_mbsfn_area_id(&_ue_dl, _area_id);
    _sf_cfg.subcarrier_spacing = mbsfn_cfg.subcarrier_spacing;
    if (getenv("SCS_TIMING_DIAG")) {
      auto us = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - t0).count();
      fprintf(stderr, "SCS_TIMING switch_to=%d us=%lld\n", (int)mbsfn_cfg.subcarrier_spacing, (long long)us);
    }
  }

  auto restore_scs = [&]() {
    auto t0 = std::chrono::steady_clock::now();
    srsran_ue_dl_set_mbsfn_subcarrier_spacing(&_ue_dl, saved_scs);
    srsran_ue_dl_set_mbsfn_area_id(&_ue_dl, _area_id);
    _sf_cfg.subcarrier_spacing = saved_scs;
    if (getenv("SCS_TIMING_DIAG")) {
      auto us = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - t0).count();
      fprintf(stderr, "SCS_TIMING restore_to=%d us=%lld\n", (int)saved_scs, (long long)us);
    }
  };

  if (SRSRAN_SCS_IS_370HZ(_sf_cfg.subcarrier_spacing)) {
    /* This state lives on Phy, not on this instance: mb_idx round-robins across
     * thread_cnt separate MbsfnFrameProcessor instances on every dispatched
     * subframe whenever there's no active, time-interleaved MCH content to pin
     * it (confirmed live -- MCCH-only occasions, this repo's own test files
     * included, hit exactly this case) -- per-instance accumulation state
     * would never survive across the 3 consecutive subframes a 370 kHz slot
     * spans. Phy is the one object guaranteed not to be rotated away from. */
    uint32_t nof_prb370 = _cell.mbsfn_prb ? _cell.mbsfn_prb : _cell.nof_prb;
    if (!_phy.scs370_accumulate_and_prepare(tti, _signal_buffer_rx, _rx_channels, nof_prb370,
                                             _sf_cfg.subcarrier_spacing)) {
      // Still gathering this slot's remaining subframes (or resyncing after a
      // missed one) -- nothing to decode yet, not a failure. Same "nothing to
      // do this subframe" handling as the !mbsfn_cfg.enable case above, minus
      // the total/errors counters, since no decode was ever attempted.
      //
      // Undo the is_mcch total++ above: that counter's whole design (see its
      // own comment) assumes 1 subframe = 1 real MCCH attempt, true for every
      // other SCS but not 370 kHz, where mbsfn_config_for_tti() now correctly
      // marks all 3 constituent subframes of a slot is_mcch=true even though
      // only the last one is a real decode attempt. Without this, BLER =
      // errors/total undercounts by ~3x here specifically -- confirmed live:
      // 90 real decode attempts, all crc=0, yet total/errors initially read
      // as if only 1 in 3 subframes were a real attempt (BLER 0.32, not the
      // true ~1.0), which is exactly the misleading-aggregate-metric trap the
      // "don't take this at face value" caution was about.
      if (mbsfn_cfg.is_mcch) {
        _rest._mcch.total--;
      }
      _rest.record_subframe_event(tti, mbsfn_cfg.is_mcch ? RestHandler::SF_EVENT_MCCH : RestHandler::SF_EVENT_MCH,
                                   RestHandler::SF_STATUS_IDLE, mch_idx);
      if (scs_switched) { restore_scs(); }
      _mutex.unlock();
      return -1;
    }
  }

  if (getenv("WIDE_FFT_DIAG") && SRSRAN_SCS_IS_370HZ(_sf_cfg.subcarrier_spacing)) {
    srsran_ofdm_t* fft = &_ue_dl.fft_mbsfn[0];
    uint32_t symbol_sz = fft->cfg.symbol_sz;
    uint32_t third = symbol_sz / 3;
    float p0 = srsran_vec_avg_power_cf(fft->cfg.in_buffer, third);
    float p1 = srsran_vec_avg_power_cf(fft->cfg.in_buffer + third, third);
    float p2 = srsran_vec_avg_power_cf(fft->cfg.in_buffer + 2 * third, third);
    static cf_t last_seen_p1_sample = 0;
    cf_t cur_p1_sample = fft->cfg.in_buffer[third];
    fprintf(stderr,
            "WIDE_FFT_DIAG tti=%u symbol_sz=%u sf_sz=%u ue_sync_sf_len=%u pwr_third0=%.4e pwr_third1=%.4e "
            "pwr_third2=%.4e p1_sample_unchanged_since_last_call=%d\n",
            tti, symbol_sz, fft->sf_sz, _phy.ue_sync_sf_len(), p0, p1, p2,
            (cur_p1_sample == last_seen_p1_sample) ? 1 : 0);
    last_seen_p1_sample = cur_p1_sample;
  }
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
        tti, mbsfn_cfg.is_mcch ? RestHandler::SF_EVENT_MCCH : RestHandler::SF_EVENT_MCH, RestHandler::SF_STATUS_FAIL,
        mch_idx);
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

  /* The eNB's control socket allows changing embms.pmch1.time_interleaving_n/m while running
   * (rrc::reconfigure_embms()) -- confirmed live, this can happen mid-session. ti_block_len
   * above is a function of N and (clamped) M, so a live change redefines what subframe index
   * maps to which slot m: the SAME absolute subframe that used to be slot m's n==3 (mid-span,
   * soft-combining in progress) can suddenly become a DIFFERENT slot's n==0 (start of a brand
   * new TB) under the new N/M, or vice versa. Every _softbuffer[]/_softbuffer_init[]/
   * _ti_reported[] entry was populated under the OLD mapping's assumptions, so leaving them in
   * place doesn't just risk stale data being reported once -- decode staying keyed off _last_ti_n
   * showed this reproduces as a hard BLER-collapsing corruption within a few live changes (not
   * just a cosmetic staleness), confirmed live 2026-07-31 on this exact rig. A slot's own
   * srsran_softbuffer_rx_t allocation doesn't need to change size (PMCH_MAX_CB_TI/
   * SOFTBUFFER_SIZE are fixed regardless of N/M), so this only needs a content reset
   * (srsran_softbuffer_rx_reset(), safe to call on a never-initialized struct too -- it no-ops
   * internally when buffer_f is still null), not a re-init/reallocation. */
  if (mbsfn_cfg.time_interleaving_n != _last_ti_n || ti_m_cfg != _last_ti_m) {
    for (uint32_t m = 0; m < SRSRAN_PMCH_MAX_TI_M; m++) {
      srsran_softbuffer_rx_reset(&_softbuffer[m]);
      _ti_reported[m] = false;
    }
    _last_ti_n = mbsfn_cfg.time_interleaving_n;
    _last_ti_m = ti_m_cfg;
  }

  if (!_softbuffer_init[ti_slot_m]) {
    // PMCH_MAX_CB_TI, not plain nof_prb sizing - see the matching comment on
    // slot 0's eager init above.
    srsran_softbuffer_rx_init_guru(&_softbuffer[ti_slot_m], PMCH_MAX_CB_TI, SOFTBUFFER_SIZE);
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
        tti, mbsfn_cfg.is_mcch ? RestHandler::SF_EVENT_MCCH : RestHandler::SF_EVENT_MCH, RestHandler::SF_STATUS_FAIL,
        mch_idx);
    if (scs_switched) { restore_scs(); }
    _mutex.unlock();
    return -1;
  }

  if (scs_switched) { restore_scs(); }

  /* PMCH DTX / validity detection. The eNB legitimately transmits NO PMCH on
   * MCH data subframes where MAC has nothing scheduled (cc_worker::encode_pmch
   * returns early when dci.rnti==0 - idle MTCH window), so what arrives here
   * is an RS-only subframe with empty data REs. Decoding it anyway is
   * meaningless. Standard receiver practice (cf. PDCCH DTX detection): gate on
   * received energy. Equalized data symbols of a real QPSK/16QAM TB have
   * ~unit average power (confirmed live: 0.9997-1.0002 on every genuine
   * decode).
   *
   * Lower bound WIDENED 2026-07-19 (was 1e-3): cc_worker::encode_pmch()'s idle
   * path now explicitly re-writes the MBSFN reference signals after zeroing
   * the RE grid (needed so the receiver's own channel estimate/CIR stays
   * valid on idle occasions - see SIB13_MBSFN_TEST_RESULTS.md), which
   * introduces a small, genuine amount of pilot-to-data leakage into the
   * equalized "empty" data REs - confirmed live: 0.0010-0.0012 on idle
   * occasions now (was 0.000000-0.000003 before that fix, when the whole grid
   * including pilots was zeroed). That's barely above the old 1e-3 threshold
   * and was being misclassified as real content, driving MCH BLER back to
   * ~1.0 on the same handful of subframes every time. 1e-2 keeps 2 full
   * orders of magnitude of margin below the ~1.0 real-content floor while
   * safely covering this leakage.
   * - Below 1e-2 (2 orders of magnitude under real signal): nothing was
   *   transmitted, IDLE.
   * - Above 10 (order of magnitude above the ~1.0 real-content ceiling):
   *   confirmed live (2026-07-17, CAS-muting sf=0 investigation) this only
   *   happens on subframes the eNB also transmitted nothing on (same ~130
   *   raw pre-equalization power as genuine idle subframes at every other
   *   position - not a real, elevated signal) - the huge equalized value
   *   (thousands, seen live) can only come from a degenerate/near-zero
   *   denominator in the equalizer, not a real received TB. Treating it as a
   *   genuine CRC failure wrongly inflated MCH BLER by counting an invalid
   *   measurement as a real, failed decode attempt (confirmed: this fully
   *   explains the CAS-muting sf=0 BLER anomaly - ~14% of otherwise-idle
   *   occasions were misclassified this way, always on the same subframe
   *   position, at a uniform rate regardless of absolute frame number -
   *   consistent with a timing/estimation artifact, not real content).
   *   Treat the same as IDLE: nothing reliable to decode either way. MCCH is
   *   never DTX (always transmitted), so this whole gate does not apply to
   *   it, upper bound included. */
  float data_pw = srsran_vec_avg_power_cf(_ue_dl.pmch.d, _pmch_cfg.pdsch_cfg.grant.nof_re);
  if (!mbsfn_cfg.is_mcch) {
    if (data_pw < 1e-2f || data_pw > 10.0f) {
      if (getenv("MCH_DIAG")) {
        float rx_pwr_idle = srsran_vec_avg_power_cf(_ue_dl.sf_symbols[0], _ue_dl.cell.nof_prb * SRSRAN_NRE);
        fprintf(stderr, "MCHIDLE sfn=%u sf=%u datapw=%.6f rxpwr=%.3e worker=%p\n",
                sfn, (unsigned)sf, data_pw, rx_pwr_idle, (void*)this);
      }
      _rest.record_subframe_event(tti, RestHandler::SF_EVENT_MCH, RestHandler::SF_STATUS_IDLE, mch_idx);
      spdlog::trace("PMCH DTX/invalid in TTI {} (data-RE power {}), skipping decode accounting", tti, data_pw);
      _mutex.unlock();
      return 0;
    }
  }

  /* Temporary, env-gated (MCH_DIAG) per-subframe diagnostic, generalized from
   * an earlier subframe-5-specific investigation (that one is fixed - see the
   * DTX-detection block above). Captures BOTH pass and fail so the failing
   * rows can be compared against the passing ones: snr_db distinguishes an
   * RF/estimate cause (low snr at failure) from a decode-config mismatch
   * (high snr but crc=0 => wrong mch_subframe_idx/mcs/tbs/grant); data_pw is
   * the equalized, post-channel-estimate data power the DTX gate above already
   * computes (should be ~unit for a healthy QPSK-equivalent TB - low here
   * despite passing the DTX gate points at a channel-estimate/equalization
   * problem, not a decode-config mismatch). Silent unless the env var is set,
   * so it costs nothing in normal operation. */
  if (!mbsfn_cfg.is_mcch && getenv("MCH_DIAG")) {
    // Extra discriminators: snr_db (channel-estimate quality) and n_iter
    // (turbo iterations). Low snr + few iters but crc=0 => genuine RF/
    // channel-estimate cause. High snr/max iters despite crc=0 => clean
    // channel but wrong bits => RX decoding a TB the TX didn't send there
    // (wrong mch_subframe_idx/mcs/tbs/grant). rx_pwr gauges whether the TX
    // transmitted anything at all on this subframe (raw, pre-equalization,
    // first OFDM symbol only - nof_prb*SRSRAN_NRE is one symbol's worth of
    // REs, not the whole subframe).
    float rx_pwr = srsran_vec_avg_power_cf(_ue_dl.sf_symbols[0], _ue_dl.cell.nof_prb * SRSRAN_NRE);
    // Direct channel-estimate health check: if ce is near-zero at this
    // subframe, equalization (division by ce) would blow up datapw exactly
    // as observed - this distinguishes "bad channel estimate" from "bad raw
    // signal" (rx_pwr already covers the latter).
    float ce_pw = _ue_dl.chest_res.ce[0][0]
        ? srsran_vec_avg_power_cf(_ue_dl.chest_res.ce[0][0], _pmch_cfg.pdsch_cfg.grant.nof_re)
        : -1.0f;
    // pmch_dec.evm was claimed dead above (always zero-initialized) as of
    // whenever that comment was written; srsran_evm_run_s() was since added
    // in pmch.c (see its call site's own comment there) - print the actual
    // current value instead of trusting the stale claim, to settle it either
    // way against the dashboard's own mcs=0/evm=0.00 reading for a
    // no-real-content PMCH (2026-07-21).
    fprintf(stderr,
            "MCHDIAG sfn=%u sf=%u mch_sf_idx=%u pmch_idx=%u mcs=%d nof_re=%d tbs=%d crc=%d snr_db=%.2f n_iter=%.2f rxpwr=%.3e datapw=%.4f cepw=%.6e rsrp=%.4e syncerr=%.4f cfo=%.6f evm=%.4f worker=%p\n",
            sfn, (unsigned)sf, mbsfn_cfg.mch_subframe_idx, mch_idx,
            _pmch_cfg.pdsch_cfg.grant.tb[0].mcs_idx,
            _pmch_cfg.pdsch_cfg.grant.nof_re, (int)_pmch_cfg.pdsch_cfg.grant.tb[0].tbs,
            (int)pmch_dec.crc, _ue_dl.chest_res.snr_db, pmch_dec.avg_iterations_block, rx_pwr, data_pw,
            ce_pw, _ue_dl.chest_res.rsrp, _ue_dl.chest_res.sync_error, _phy.cfo(), pmch_dec.evm, (void*)this);
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
    /* CPU_MIGRATION_DIAG: hypothesis 3 (coarse CPU-frequency variance) from
     * the original MCCH EVM ripple investigation was "ruled out at this
     * granularity" - it sampled per-core frequency on a SEPARATE thread every
     * ~300ms and correlated against timestamped MCCHDIAG lines, three orders
     * of magnitude too coarse to catch a migration/frequency-ramp event that
     * only matters for the few hundred microseconds this decode actually
     * takes. This measures INLINE, at the exact moment of THIS occasion's
     * decode, on the SAME thread doing the decode: which core it's running
     * on (sched_getcpu(), a cheap syscall) and that core's current scaling
     * frequency (a small sysfs read - cheap enough for a ~once-per-MCCH-
     * occasion rate, unlike the ~1kHz per-subframe rate that ruled out
     * similar diagnostics elsewhere this campaign). thread_local so each
     * worker thread tracks its own last-seen core independently; a mismatch
     * from the previous occasion means this thread migrated cores since
     * then - a real mechanism for the kind of cache-cold-start/TLB-flush
     * jitter that a non-RT-priority thread floating across 32 cores could
     * plausibly hit, and one the original investigation never actually
     * tested (only frequency was checked, not migration itself). */
    if (getenv("CPU_MIGRATION_DIAG")) {
      thread_local int last_cpu = -1;
      int cur_cpu = sched_getcpu();
      bool migrated = (last_cpu != -1) && (cur_cpu != last_cpu);
      long freq_khz = -1;
      char path[64];
      snprintf(path, sizeof(path), "/sys/devices/system/cpu/cpu%d/cpufreq/scaling_cur_freq", cur_cpu);
      FILE* ff = fopen(path, "r");
      if (ff) {
        if (fscanf(ff, "%ld", &freq_khz) != 1) {
          freq_khz = -1;
        }
        fclose(ff);
      }
      fprintf(stderr, "CPU_MIGRATION_DIAG sfn=%u sf=%u cpu=%d last_cpu=%d migrated=%d freq_khz=%ld evm=%.4f\n",
              sfn, (unsigned)sf, cur_cpu, last_cpu, (int)migrated, freq_khz, pmch_dec.evm);
      last_cpu = cur_cpu;
    }
    /* ALIGN_DUMP: a different angle than timing (bridge logging, CIR/CE
     * stride, coarse CPU-frequency variance, and this occasion's own
     * processing duration were all tested and ruled out - see the comments
     * on those). Tests whether a BUMP occasion's raw post-FFT RE grid and
     * channel estimate show a detectable sample-alignment shift (a phase
     * ramp across RE index, the classic signature of a residual timing/CFO
     * error) that a NORMAL occasion doesn't - same raw[i]/ce[i] technique
     * already validated for Finding 7 earlier this campaign. Self-triggers
     * on evm alone (no tti-matching needed): dumps this occasion whenever
     * evm crosses the threshold, then automatically dumps the VERY NEXT
     * occasion too (regardless of its own evm) as the paired "normal"
     * comparison sample. Hard-capped at 6 pairs so this can't grow
     * unbounded; each dump is small (nof_re complex floats, a few KB). */
    if (getenv("ALIGN_DUMP")) {
      static int  pairs_dumped   = 0;
      static bool want_normal    = false;
      constexpr int kMaxPairs    = 6;
      constexpr float kEvmThresh = 0.055f;
      auto dump_occasion = [&](const char* tag) {
        char path[256];
        snprintf(path, sizeof(path), "/home/jordijoan/align_dump_%02d_%s_sfn%u_sf%u.bin",
                 pairs_dumped, tag, sfn, (unsigned)sf);
        FILE* f = fopen(path, "wb");
        if (f) {
          uint32_t nre = _pmch_cfg.pdsch_cfg.grant.nof_re;
          fwrite(&nre, sizeof(nre), 1, f);
          float evm_val = pmch_dec.evm;
          fwrite(&evm_val, sizeof(evm_val), 1, f);
          fwrite(_ue_dl.sf_symbols[0], sizeof(cf_t), nre, f);
          fwrite(_ue_dl.chest_res.ce[0][0], sizeof(cf_t), nre, f);
          fclose(f);
        }
      };
      if (want_normal) {
        dump_occasion("normal");
        want_normal = false;
        pairs_dumped++;
      } else if (pmch_dec.evm > kEvmThresh && pairs_dumped < kMaxPairs) {
        dump_occasion("bump");
        want_normal = true;
      }
    }
    if (getenv("MCH_DIAG")) {
      float ce_pw_mcch = _ue_dl.chest_res.ce[0][0]
          ? srsran_vec_avg_power_cf(_ue_dl.chest_res.ce[0][0], _pmch_cfg.pdsch_cfg.grant.nof_re)
          : -1.0f;
      double duration_us = std::chrono::duration<double, std::micro>(
          std::chrono::steady_clock::now() - t_entry).count();
      fprintf(stderr,
              "MCCHDIAG sfn=%u sf=%u mcs=%d nof_re=%d tbs=%d crc=%d snr_db=%.2f n_iter=%.2f "
              "rxpwr=%.3e datapw=%.4f cepw=%.6e rsrp=%.4e syncerr=%.4f nofprb=%u mbsfnprb=%u cfo=%.6f evm=%.4f "
              "durationus=%.1f intervalus=%.1f worker=%p\n",
              sfn, (unsigned)sf, _pmch_cfg.pdsch_cfg.grant.tb[0].mcs_idx,
              _pmch_cfg.pdsch_cfg.grant.nof_re, (int)_pmch_cfg.pdsch_cfg.grant.tb[0].tbs,
              (int)pmch_dec.crc, _ue_dl.chest_res.snr_db, pmch_dec.avg_iterations_block,
              srsran_vec_avg_power_cf(_ue_dl.sf_symbols[0], _ue_dl.cell.nof_prb * SRSRAN_NRE),
              data_pw, ce_pw_mcch, _ue_dl.chest_res.rsrp, _ue_dl.chest_res.sync_error,
              _ue_dl.cell.nof_prb, _ue_dl.cell.mbsfn_prb, _phy.cfo(), pmch_dec.evm,
              duration_us, interval_us, (void*)this);
    }
  } else {
    _rest._mch[mch_idx].SetData(mch_data());
    _rest._mch[mch_idx].mcs = _pmch_cfg.pdsch_cfg.grant.tb[0].mcs_idx;
    _rest._mch[mch_idx].evm_rms = pmch_dec.evm;
    _rest._mch[mch_idx].present = true;
  }
  if (++_ce_cir_update_counter >= CE_CIR_UPDATE_STRIDE) {
    _ce_cir_update_counter  = 0;
    // CIRSTRIDE_DIAG: added to test whether this inline, same-thread IDFT+dB
    // conversion (running right after decoding this occasion, before the
    // worker is free for the next one) explains the MCCH EVM bumps chased
    // across this file and ZmqRxDevice.cpp's logThreadLoop. NOT CONFIRMED:
    // a live capture correlating this print's own sfn/sf against MCCHDIAG
    // showed only ~2/10 CIRSTRIDE events immediately followed by an elevated
    // MCCH reading, and several clear bumps had no CIRSTRIDE event anywhere
    // near them. Ruled out as the (sole) cause, same as the bridge-logging
    // hypothesis before it. Left in place - harmless when unset - in case a
    // future pass wants to re-check it under different conditions.
    if (getenv("CIRSTRIDE_DIAG")) {
      fprintf(stderr, "CIRSTRIDE sfn=%u sf=%u is_mcch=%d\n", sfn, (unsigned)sf, (int)mbsfn_cfg.is_mcch);
    }
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

    if (mbsfn_cfg.is_mcch && getenv("MCCH_RAW_DIAG")) {
      uint32_t tbs_bytes = static_cast<uint32_t>(_pmch_cfg.pdsch_cfg.grant.tb[0].tbs) / 8;
      char hex[97] = {0};
      for (uint32_t k = 0; k < tbs_bytes && k < 48; k++) {
        snprintf(hex + k * 2, 3, "%02x", _payload_buffer[k]);
      }
      fprintf(stderr, "MCCH_RAW_DIAG sfidx=%u tbs_bytes=%u first48=%s\n",
              mbsfn_cfg.mch_subframe_idx, tbs_bytes, hex);
    }

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
            fprintf(stderr, "TI_DIAG_MACSDU mch_idx=%u sfidx=%u lcid=%u sz=%u first48=%s\n", mch_idx, mbsfn_cfg.mch_subframe_idx, lcid, sz, hex);
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
      _rest.record_subframe_event(tti, RestHandler::SF_EVENT_MCH, RestHandler::SF_STATUS_OK, mch_idx);
      _mutex.unlock();
      return 0;
    }
    /* Rel-19 §6.5.3: partial accumulation — not a real failure, waiting for
     * remaining subframes of this slot's own N-span (ti_slot_n/ti_active
     * computed earlier from the same (m,n) split used for the softbuffer). */
    if (ti_active && ti_slot_n < (uint32_t)(mbsfn_cfg.time_interleaving_n - 1u)) {
      _rest.record_subframe_event(tti, RestHandler::SF_EVENT_MCH, RestHandler::SF_STATUS_IDLE, mch_idx);
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
        tti, mbsfn_cfg.is_mcch ? RestHandler::SF_EVENT_MCCH : RestHandler::SF_EVENT_MCH, RestHandler::SF_STATUS_FAIL,
        mch_idx);

    /* Failure-triggered raw-sample dump: failures can drift in tti even
     * within a run, so only a capture taken AT the failure itself is
     * race-free. This processor still owns _signal_buffer_rx (mutex held), so
     * dump the exact time-domain input whose decode just failed CRC, for
     * offline FFT/window-offset comparison against a matching TX-side
     * reference. Env-gated; bounded by rarity and tti-name wraparound. Not
     * specific to any one subframe position - see the DTX/validity gate
     * above for the 2026-07 CAS-muting sf=0 finding this helped confirm. */
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
      tti, mbsfn_cfg.is_mcch ? RestHandler::SF_EVENT_MCCH : RestHandler::SF_EVENT_MCH, RestHandler::SF_STATUS_OK,
      mch_idx);
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
  // Floor with -80 (the same floor srsran_vec_abs_dB_cf() below uses for its
  // own real content), NOT memset-0: this region outside the active
  // SRSRAN_NRE*nof_prb window never goes through that dB conversion, so a
  // raw 0.0f here is NOT "no signal" - waterfall_color()'s -20..25dB color
  // ramp maps db=0 to a strong, solid green, not black/background. Confirmed
  // live, 2026-07-18: this was exactly the solid green blocks bookending the
  // real (narrower) content on the MBSFN CE waterfall dashboard chart.
  for (uint32_t i = 0; i < sz; i++) {
    ce_abs[i] = -80.0f;
  }
  /* SRSRAN_NRE_SCS(_ue_dl.subcarrier_spacing), not the plain SRSRAN_NRE (12):
   * the real ce[] array is laid out at the ACTUAL numerology's RE-per-PRB
   * density (144 for 1.25kHz, matching fft_mbsfn's own nof_re=5760 for 40
   * PRB) - using 12 here read only the first ~1/12th of it. See set_cell()'s
   * matching fix/comment above. */
  uint32_t nre_per_prb = SRSRAN_NRE_SCS(_ue_dl.subcarrier_spacing);
  uint32_t g = (sz - nre_per_prb * _cell.nof_prb) / 2;
  srsran_vec_abs_dB_cf(_ue_dl.chest_res.ce[0][0], -80, &ce_abs[g], nre_per_prb * _cell.nof_prb);
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
  // See ce_values()'s matching fix/comment: SCS-aware RE-per-PRB density, not the plain 12.
  uint32_t nre_per_prb = SRSRAN_NRE_SCS(_ue_dl.subcarrier_spacing);
  uint32_t g = (sz - nre_per_prb * _cell.nof_prb) / 2;
  memcpy(&ce_freq[g], _ue_dl.chest_res.ce[0][0], nre_per_prb * _cell.nof_prb * sizeof(cf_t));

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
