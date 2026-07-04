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
  _pmch_cfg.pdsch_cfg.meas_evm_en        = false;
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
    if (mbsfn_cfg.is_mcch) {
      _rest._mcch.errors++;
    } else {
      _rest._mch[mch_idx].errors++;
    }
    spdlog::error("Getting PDCCH FFT estimate");
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
  }

  srsran_pdsch_res_t pmch_dec = {};
  _pmch_cfg.pdsch_cfg.softbuffers.rx[0] = &_softbuffer[ti_slot_m];
  pmch_dec.payload = _payload_buffer;
  if (!ti_active || ti_slot_n == 0) {
    srsran_softbuffer_rx_reset_tbs(_pmch_cfg.pdsch_cfg.softbuffers.rx[0], _pmch_cfg.pdsch_cfg.grant.tb[0].tbs);
  }

  if (srsran_ue_dl_decode_pmch(&_ue_dl, &_sf_cfg, &_pmch_cfg, &pmch_dec) != 0) {
    if (mbsfn_cfg.is_mcch) {
      _rest._mcch.errors++;
    } else {
      _rest._mch[mch_idx].errors++;
    }
    spdlog::warn("Error decoding PMCH");
    if (scs_switched) { restore_scs(); }
    _mutex.unlock();
    return -1;
  }

  if (scs_switched) { restore_scs(); }

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
          spdlog::debug("Scheduling stop for PMCH {} LCID {} in sf {}", mch_idx, lcid, stop);
          _sched_stops[ {(uint8_t)mch_idx, lcid} ] = stop;
        }
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
          spdlog::warn("Dropping spurious MCCH-LCID SDU decoded from a non-MCCH subframe (mch_idx {})", mch_idx);
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
            fprintf(stderr, "TI_DIAG_MACSDU lcid=%u sz=%u first48=%s\n", lcid, sz, hex);
          }
          _phy._mcs = mbsfn_cfg.mbsfn_mcs;
          const std::lock_guard<std::mutex> lock(_rlc_mutex);
          _rlc.write_pdu_mch(mch_idx, lcid, mch_mac_msg.get()->get_sdu_ptr(), mch_mac_msg.get()->get_payload_size());
        }
      }
    }
  } else {
    /* Rel-19 §6.5.3: partial accumulation — not a real failure, waiting for
     * remaining subframes of this slot's own N-span (ti_slot_n/ti_active
     * computed earlier from the same (m,n) split used for the softbuffer). */
    if (ti_active && ti_slot_n < (uint32_t)(mbsfn_cfg.time_interleaving_n - 1u)) {
      _mutex.unlock();
      return 0;
    }
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
