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

#include <chrono>
#include <thread>
#include <utility>
#include <iomanip>

#include "srsran/interfaces/rrc_interface_types.h"
#include "srsran/asn1/rrc_utils.h"
#include "spdlog/spdlog.h"

namespace {
uint64_t now_ms() {
  return std::chrono::duration_cast<std::chrono::milliseconds>(
             std::chrono::system_clock::now().time_since_epoch())
      .count();
}
} // namespace

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
  if (_scs370_resamplers_for_nof_prb != 0) {
    for (unsigned ch = 0; ch < _scs370_resamplers_for_rx_channels; ch++) {
      srsran_resampler_fft_free(&_scs370_interp[ch]);
      srsran_resampler_fft_free(&_scs370_decim[ch]);
    }
  }
}

bool Phy::scs370_accumulate_and_prepare(uint32_t tti, cf_t* signal_buffer[SRSRAN_MAX_PORTS], unsigned rx_channels,
                                         uint32_t nof_prb, srsran_scs_t scs) {
  std::lock_guard<std::mutex> lock(_scs370_mutex);
  uint32_t sf_len = SRSRAN_SF_LEN_PRB(nof_prb);

  if (_scs370_resamplers_for_nof_prb != nof_prb || _scs370_resamplers_for_rx_channels != rx_channels) {
    /* Reduce target:source to lowest terms (e.g. 11.52MHz:7.68MHz -> 3:2) via gcd,
     * mirroring srsran_ue_dl_set_cell_scs()'s existing wide<->narrow ratio pattern
     * but in the opposite (interpolate-up) direction, since that path explicitly
     * declines to support 370Hz (see its own "known, separately-scoped
     * architectural gap" comment) - this is the missing other half. */
    int source_hz = srsran_sampling_freq_hz(nof_prb);
    int target_hz = srsran_sampling_freq_hz_scs(nof_prb, scs);
    if (source_hz <= 0 || target_hz <= 0) {
      spdlog::error("scs370_accumulate_and_prepare: invalid sampling rate for nof_prb={}", nof_prb);
      return false;
    }
    uint32_t a = (uint32_t)target_hz;
    uint32_t b = (uint32_t)source_hz;
    while (b != 0) {
      uint32_t t = b;
      b = a % b;
      a = t;
    }
    uint32_t gcd = a;
    _scs370_interp_ratio = (uint32_t)target_hz / gcd;
    _scs370_decim_ratio  = (uint32_t)source_hz / gcd;

    for (unsigned ch = 0; ch < rx_channels; ch++) {
      if (_scs370_resamplers_for_nof_prb != 0) {
        srsran_resampler_fft_free(&_scs370_interp[ch]);
        srsran_resampler_fft_free(&_scs370_decim[ch]);
      }
      if (_scs370_interp_ratio > 1) {
        srsran_resampler_fft_init(&_scs370_interp[ch], SRSRAN_RESAMPLER_MODE_INTERPOLATE, _scs370_interp_ratio);
      }
      if (_scs370_decim_ratio > 1) {
        srsran_resampler_fft_init(&_scs370_decim[ch], SRSRAN_RESAMPLER_MODE_DECIMATE, _scs370_decim_ratio);
      }
    }

    uint32_t max_source_samples = 3u * sf_len;  // a 370Hz slot is always exactly 3 real subframes
    uint32_t max_interp_samples = max_source_samples * SRSRAN_MAX(_scs370_interp_ratio, 1u);
    for (unsigned ch = 0; ch < rx_channels; ch++) {
      _scs370_accum[ch].assign(max_source_samples, cf_t{});
      _scs370_interp_scratch[ch].assign(max_interp_samples, cf_t{});
      _scs370_resampled[ch].assign(max_interp_samples, cf_t{});
    }
    spdlog::info("scs370: configured for nof_prb={} rx_channels={} source_hz={} target_hz={} interp_ratio={} decim_ratio={}",
                 nof_prb, rx_channels, source_hz, target_hz, _scs370_interp_ratio, _scs370_decim_ratio);
    _scs370_resamplers_for_nof_prb      = nof_prb;
    _scs370_resamplers_for_rx_channels  = rx_channels;
    _scs370_expected_pos = -1;
  }

  std::pair<uint32_t, uint32_t> slot_pos = scs370_slot_position(tti);
  uint32_t pos_in_slot = slot_pos.first;
  uint32_t slot_len    = slot_pos.second;
  if (slot_len == 0) {
    // CAS's own subframe (see scs370_slot_position()'s doc comment) -- main.cpp's
    // dispatch already never calls process() for this tti, so this should be
    // unreachable; guarded defensively rather than dividing/indexing by it.
    _scs370_expected_pos = -1;
    return false;
  }

  if ((int)pos_in_slot != _scs370_expected_pos) {
    // Either the start of a fresh slot, or we lost/skipped a subframe (which can
    // legitimately include a different MbsfnFrameProcessor instance's mb_idx
    // rotation calling in with an unrelated tti in between) -- either way,
    // restart the resamplers' internal filter state rather than let a
    // discontinuous window feed through as if it were continuous.
    for (unsigned ch = 0; ch < rx_channels; ch++) {
      if (_scs370_interp_ratio > 1) srsran_resampler_fft_reset_state(&_scs370_interp[ch]);
      if (_scs370_decim_ratio > 1) srsran_resampler_fft_reset_state(&_scs370_decim[ch]);
    }
    if (pos_in_slot != 0) {
      // Missed this slot's start -- nothing usable accumulated yet.
      _scs370_expected_pos = -1;
      if (getenv("WIDE_FFT_DIAG")) {
        fprintf(stderr, "WIDE_FFT_DIAG scs370 tti=%u pos_in_slot=%u slot_len=%u -- missed slot start, dropping\n",
                tti, pos_in_slot, slot_len);
      }
      return false;
    }
  }

  for (unsigned ch = 0; ch < rx_channels; ch++) {
    memcpy(&_scs370_accum[ch][pos_in_slot * sf_len], signal_buffer[ch], sf_len * sizeof(cf_t));
  }
  _scs370_expected_pos = (int)pos_in_slot + 1;

  if (pos_in_slot + 1 < slot_len) {
    if (getenv("WIDE_FFT_DIAG")) {
      fprintf(stderr, "WIDE_FFT_DIAG scs370 tti=%u pos_in_slot=%u/%u -- accumulating, not yet decoding\n",
              tti, pos_in_slot, slot_len);
    }
    return false;
  }

  // Full slot assembled at the source rate -- resample up to this numerology's
  // own rate (chained interpolate-then-decimate, since the ratio is rational,
  // not a plain integer in either direction for most bandwidths), then land the
  // result in the caller's signal_buffer, where fft_mbsfn's cfg.in_buffer
  // already points, so the existing, completely unmodified FFT/decode path
  // takes over from here.
  _scs370_expected_pos = -1;
  uint32_t accumulated_samples = slot_len * sf_len;
  for (unsigned ch = 0; ch < rx_channels; ch++) {
    const cf_t* interp_in  = _scs370_accum[ch].data();
    cf_t*       interp_out = _scs370_interp_scratch[ch].data();
    uint32_t    interp_len;
    if (_scs370_interp_ratio > 1) {
      srsran_resampler_fft_run(&_scs370_interp[ch], interp_in, interp_out, accumulated_samples);
      interp_len = accumulated_samples * _scs370_interp_ratio;
    } else {
      memcpy(interp_out, interp_in, accumulated_samples * sizeof(cf_t));
      interp_len = accumulated_samples;
    }
    cf_t*    final_out = _scs370_resampled[ch].data();
    uint32_t final_len;
    if (_scs370_decim_ratio > 1) {
      srsran_resampler_fft_run(&_scs370_decim[ch], interp_out, final_out, interp_len);
      final_len = interp_len / _scs370_decim_ratio;
    } else {
      memcpy(final_out, interp_out, interp_len * sizeof(cf_t));
      final_len = interp_len;
    }
    memcpy(signal_buffer[ch], final_out, final_len * sizeof(cf_t));
    if (getenv("WIDE_FFT_DIAG") && ch == 0) {
      fprintf(stderr,
              "WIDE_FFT_DIAG scs370 tti=%u slot_len=%u accumulated=%u interp_len=%u final_len=%u -- DECODING NOW\n",
              tti, slot_len, accumulated_samples, interp_len, final_len);
    }
  }
  return true;
}

auto Phy::synchronize_subframe() -> bool {

  // See get_next_frame()/set_cfo_from_channel_estimation()'s comments: a CAS worker task
  // dispatched just before a resync (state -> syncing) can still be finishing up (and calling
  // set_cfo_from_channel_estimation()) while this runs, so this needs the same lock.
  std::lock_guard<std::mutex> lock(_ue_sync_mutex);
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
        /* Unlike cell_search() (which decodes into a scratch new_cell and only
         * commits after validating it), this resync path used to unpack
         * straight into the live _cell with no validity check at all - a
         * corrupted/out-of-range MIB decode (e.g. a reserved dl-Bandwidth
         * codepoint) would silently overwrite the live cell with garbage
         * (observed: nof_prb=125 from a reserved bw_idx=6), crashing whatever
         * downstream PHY component next tried to use it. Decode into a copy
         * first and validate before committing.
         *
         * Deliberately check only nof_prb (via srsran_nofprb_isvalid()), NOT
         * the broader srsran_cell_isvalid() - the latter also checks
         * mbsfn_prb<=nof_prb and the CAS-muting n_cas set, neither of which
         * this MIB unpack touches. mbsfn_prb in particular is legitimately
         * allowed to exceed the MIB-derived nof_prb in file-source mode
         * (main.cpp's "decode a narrow CAS from a wider channel" case, where
         * mbsfn_prb is sized from the file's native capture bandwidth, not
         * the transmitted cell's own PRB count) - re-validating it here would
         * wrongly reject that legitimate configuration on every resync. */
        srsran_cell_t candidate_cell = _cell;
        if (candidate_cell.mbms_dedicated) {
          uint32_t add_non_mbsfn = 0;
          srsran_pbch_mib_mbms_unpack(bch_payload.data(), &candidate_cell, &sfn, &add_non_mbsfn,
              _override_nof_prb);
          candidate_cell.additional_non_mbms_frames = (uint8_t)add_non_mbsfn;
          sfn = (sfn + sfn_offset * kSfnOffset) % kMaxSfn;
        } else {
          srsran_pbch_mib_unpack(bch_payload.data(), &candidate_cell, &sfn);
          sfn = (sfn + sfn_offset) % kMaxSfn;
        }
        if (!srsran_nofprb_isvalid(candidate_cell.nof_prb)) {
          spdlog::error("Phy: resync MIB decode produced an invalid nof_prb={} - discarding, keeping previous cell state",
                        candidate_cell.nof_prb);
          return false;
        }
        _cell = candidate_cell;
        _tti =  sfn * kSubframesPerFrame;
        _last_mib_decoded_at = now_ms();
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

  // Which hypothesis to try first is configurable (modem.phy.expect_mixed_cell, default
  // false) rather than hardcoded, so existing MBMS-dedicated deployments see zero behaviour
  // change by default. Trying the wrong hypothesis first is not free: srsran_pbch_decode()'s
  // Rel-16 CAS-repetition combining fallback (pbch.c, gated on q->cell.mbms_dedicated &&
  // nof_prb > 6) blends in symbols from resource elements that only actually carry repeated-
  // PBCH content on a genuine FeMBMS-dedicated CAS-repetition cell. Tried against a real
  // MBMS/Unicast-mixed cell (TS 36.300 §15.2.2), those REs hold unrelated real signal content
  // (not noise), and blending it into the LLR buffer made a wrong-hypothesis decode noticeably
  // MORE likely to produce a spurious CRC pass than a clean failure would -- confirmed live:
  // MBMS-first search against a genuinely mixed cell never once landed on the true nof_prb,
  // always some other plausible-but-wrong value. An MBMS/Unicast-mixed cell deployment should
  // therefore set expect_mixed_cell=true, which also skips a full wasted first-attempt
  // (kMaxFramesTimeout frames) against a real MBMS-dedicated cell -- confirmed live:
  // unconditionally trying the mixed-cell hypothesis first roughly doubled the time needed to
  // lock a genuinely MBMS-dedicated cell, since every search cycle burned a full
  // guaranteed-to-fail mixed-cell attempt first.
  bool expect_mixed_cell = false;
  _cfg.lookupValue("modem.phy.expect_mixed_cell", expect_mixed_cell);

  new_cell.mbms_dedicated = !expect_mixed_cell;
  if (srsran_ue_mib_sync_set_cell_prb(&_mib_sync, new_cell, _cs_nof_prb) != 0) {
    spdlog::error("Phy: Error setting UE MIB sync cell");
    return false;
  }
  srsran_ue_sync_reset(&_mib_sync.ue_sync);
  ret = srsran_ue_mib_sync_decode_prb(&_mib_sync, kMaxFramesTimeout, bch_payload.data(), &new_cell.nof_ports, &sfn_offset, _cs_nof_prb);

  if (!ret) { // first hypothesis failed, try the other one
    // NOTE: this used to call init(), which re-runs srsran_ue_cellsearch_init_multi_prb_cp(),
    // srsran_ue_sync_init_multi(), srsran_ue_mib_sync_init_multi_prb() and srsran_ue_mib_init()
    // on _cell_search/_ue_sync/_mib_sync/_mib a second time without ever freeing what the first
    // init() (called once from main() before the search loop starts) had already allocated -
    // every failed first-attempt leaked and re-initialized the same srsran objects on top of
    // their still-live state. That corrupts the heap over repeated cell_search() calls (each
    // failed candidate hits this path) and eventually crashes in an unrelated later allocation
    // or FFTW plan run, especially at higher nof_prb where a mismatched first attempt fails
    // more often before the fallback + more frames/PRB means more memory touched per leak.
    // The srsran_ue_sync_reset() below (mirroring the first attempt above) already re-arms
    // sync; what was actually still needed for the retry is resetting the MIB decoder's own
    // frame counter/PBCH state, which srsran_ue_mib_reset() does with no allocation involved.
    srsran_ue_mib_reset(&_mib_sync.ue_mib);
    new_cell.mbms_dedicated = expect_mixed_cell;
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
    _mib_decode_count++;
    _last_mib_decoded_at = now_ms();

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
    /* 2026-07-18: tried making this SCS-aware (a dedicated srsran_ue_sync_set_cell_scs(),
     * using the MBSFN area's actual reduced SCS instead of assuming 15kHz for the
     * fft_size). Reverted: _ue_sync tracks PSS/SSS, which is always standard 15kHz
     * numerology regardless of MBSFN SCS (CAS is never reduced-SCS) - the "acquisition
     * bandwidth widens to the next PRB tier" behavior already correctly stays 15kHz-based
     * here. Confirmed live: the SCS-aware fft_size (e.g. 12288 for pmch_bandwidth=30's
     * 50-PRB tier at 1.25kHz) exceeds sync.c's hardcoded SRSRAN_SYNC_FFT_SZ_MAX=2048,
     * a standard-LTE-only ceiling the PSS/SSS engine was never designed to exceed.
     * The real fix belongs in the MBSFN-specific channel-estimation code
     * (chest_dl.c), which needs mbsfn_prb-awareness independent of this engine. */
    if (srsran_ue_sync_set_cell(&_ue_sync, cell()) != 0) {
      spdlog::error("Phy: failed to set cell.\n");
    }
    /* srsran_ue_sync_set_cell() resizes fft_size/sf_len for the new (wider)
     * geometry but leaves the tracking state machine (state/frame_ok_cnt/
     * mean_sample_offset/next_rf_sample_offset) untouched. _ue_sync was in
     * SF_TRACK before this retune (already synced at the old, narrower
     * geometry) and would otherwise STAY in SF_TRACK, applying tracking
     * corrections computed under the old sf_len against samples now aligned
     * to the new, wider one - a stale-state timing/position desync, not a
     * content/decode problem. main.cpp's caller already sets state=syncing
     * right after this call, expecting a fresh acquisition; that only
     * actually happens if the sync engine is told to re-find, hence this
     * reset. cell_search() already does the equivalent reset correctly for
     * its own (different) sync object (_mib_sync.ue_sync) - this call site
     * was just missing it for the shared _ue_sync instance. Confirmed live,
     * 2026-07-18: CAS's own PSS/SSS/PBCH dashboard positions went missing
     * shortly after a wideband-PMCH retune, consistent with this gap. */
    srsran_ue_sync_reset(&_ue_sync);
    /* 2026-07-26: bw_ref scaling (below) was tried first and confirmed, by direct live
     * measurement, NOT to change the sf_idx=5-specific sync-loss rate at all -- see
     * project-mixed-mode-sync-loss-2026-07-22 memory. Widening CasFrameProcessor's
     * subframe coverage to {0,4,5,9} (Phy::is_cas_subframe()) was tried next and gave
     * the sharpest data point of the investigation: subframes 4/9 (which NEVER run a
     * PSS/SSS tracking check at all) had ZERO failures across ~80,000 combined attempts,
     * while 0/5 (the ONLY subframes where srsran_ue_sync_run_track_pss_mode() actually
     * runs) kept failing at the exact same rate as always. That points at track_peak_ok()
     * itself (lib/srsran/lib/src/phy/ue/ue_sync.c, ~line 637): its OWN self-correction
     * uses bw_pss (the first argument here), hardcoded to the library default of 1 --
     * full weight, UNDAMPED, unlike bw_ref which was already reduced. bw_pss's update
     * only ever fires when a tracking check runs, i.e. only at sf_idx 0/5 in mixed mode
     * (every 5ms) vs sf_idx 0 only, every 40-80ms, in dedicated mode (the same 8-16x
     * frequency gap already established for bw_ref). Scaling bw_pss down for mixed mode
     * by the same conservative factor targets this second, previously-overlooked
     * accumulation path. NOT YET LIVE-VERIFIED -- revert this specific change if it
     * doesn't measurably reduce sf_idx=0/5 sync loss. */
    float bw_pss = _cell.mbms_dedicated ? 1.0F : 1.0F / 8.0F;
    float bw_ref = _cell.mbms_dedicated ? 0.1F : 0.1F / 8.0F;
    srsran_ue_sync_set_cfo_loop_bw(&_ue_sync, bw_pss, bw_ref, 750, 0, 750, 160);
    if (srsran_ue_mib_set_cell(&_mib, cell()) != 0) {
      spdlog::error("Phy: Error setting UE MIB cell");
    }
    if (getenv("CAS_CE_DIAG")) {
      fprintf(stderr,
              "UE_SYNC_TRACE nof_prb=%u mbsfn_prb=%u ue_sync.fft_size=%u ue_sync.sf_len=%u\n",
              _cell.nof_prb, _cell.mbsfn_prb, _ue_sync.fft_size, _ue_sync.sf_len);
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
  /* Lower the REF-CFO loop's bandwidth (gain) from the library default of 1
   * (unity - the ENTIRE new per-subframe estimate is applied every time, with
   * no smoothing) to 0.1. _ue_sync is shared between CasFrameProcessor and
   * every MbsfnFrameProcessor (both read raw samples via the same
   * Phy::get_next_frame() -> srsran_ue_sync_zerocopy()), and the CFO
   * correction applied to those raw samples (ue_sync.c's
   * srsran_vec_apply_cfo, using q->cfo_current_value) comes ENTIRELY from
   * CAS's own per-subframe estimate (CasFrameProcessor.cpp's
   * set_cfo_from_channel_estimation() -> srsran_ue_sync_set_cfo_ref()) - MBSFN
   * does no CFO estimation of its own (cfo_estimate_enable=false, and
   * chest_dl.c's estimator is unconditionally excluded for SRSRAN_SF_MBSFN
   * regardless of that flag). At unity gain, cfo_current_value random-walks
   * by roughly one subframe's worth of CAS's own single-subframe measurement
   * noise on every CAS occasion - the exact value an MBSFN subframe gets
   * corrected with is whatever CAS most recently, asynchronously produced.
   * Confirmed live, 2026-07-18, as the likely cause of PMCH decode being
   * non-deterministic (same code/config, ~78% CRC pass one run, 100% failure
   * another) even after CAS's own timing/sync stability was separately fixed.
   * All other loop parameters kept at their existing defaults (only bw_ref
   * changes) via the same public API this codebase already exposes for
   * exactly this purpose (srsran_ue_sync_set_cfo_loop_bw(), ue_sync.h). */
  srsran_ue_sync_set_cfo_loop_bw(&_ue_sync, 1, 0.1F, 750, 0, 750, 160);

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
  // See set_cfo_from_channel_estimation()'s comment (Phy.h): serializes this call against
  // that one, since srsran_ue_sync_zerocopy() reads/updates the same _ue_sync.cfo_current_value
  // that call writes from a different (CAS worker) thread.
  std::lock_guard<std::mutex> lock(_ue_sync_mutex);
  if (getenv("WIDE_READ_DIAG")) {
    fprintf(stderr, "WIDE_READ_DIAG requested_size=%u ue_sync.sf_len=%u ue_sync.file_mode=%d\n",
            size, _ue_sync.sf_len, (int)_ue_sync.file_mode);
  }
  int ret = srsran_ue_sync_zerocopy(&_ue_sync, buffer, size);
  // TEMPORARY DIAGNOSTIC (SYNC_FAIL_DIAG=1, 2026-07-22): root-causing a continuous
  // sync-loss/reacquire loop seen in mixed-cell (mbms_dedicated=false) mode at
  // n_prb=25 -- repeated "Synchronization lost" every ~40-50ms despite clean
  // SNR/CFO in CAS_CE_DIAG right up to each loss. Logs _ue_sync's own internal
  // state whenever this call does NOT return 1, to see whether it's stuck
  // re-entering SF_FIND, losing PSS stability, or something else -- ruling
  // in/out mechanisms rather than guessing further from static code reading.
  if (getenv("SYNC_FAIL_DIAG") && ret != 1) {
    fprintf(stderr,
            "SYNC_FAIL_DIAG ret=%d state=%d sf_idx=%u frame_ok_cnt=%llu frame_no_cnt=%u "
            "frame_total_cnt=%u pss_is_stable=%d pss_stable_cnt=%u peak_idx=%u "
            "next_rf_sample_offset=%d mean_sample_offset=%f\n",
            ret, (int)_ue_sync.state, _ue_sync.sf_idx,
            (unsigned long long)_ue_sync.frame_ok_cnt, _ue_sync.frame_no_cnt,
            _ue_sync.frame_total_cnt, (int)_ue_sync.pss_is_stable, _ue_sync.pss_stable_cnt,
            _ue_sync.peak_idx, _ue_sync.next_rf_sample_offset, _ue_sync.mean_sample_offset);
  }
  return 1 == ret;
}

void Phy::set_mch_scheduling_info(const srsran::sib13_t& sib13) {
  if (sib13.nof_mbsfn_area_info > 1) {
    spdlog::warn("SIB13 has {} MBSFN area info elements - only 1 supported", sib13.nof_mbsfn_area_info);
  }

  /* pmch-Bandwidth-r17 is OPTIONAL (TS 36.331 clause 6.3.7): its absence
   * (encoded here as 0) means "no wideband extension, use the base cell
   * width", same as mbsfn_prb's own zero-means-nof_prb convention elsewhere
   * (SRSRAN_MAX(nof_prb, mbsfn_prb) throughout pmch.c/chest_dl.c). The old
   * `!= 0` guard here only ever moved mbsfn_prb UP and never reset it back
   * down when a later SIB13 legitimately reverts to pmch_bandwidth=0 -
   * confirmed live (2026-07-17): a fresh receive that caught pmch_bandwidth=30
   * on its very first SIB13 decode (a stale broadcast mid-transition) stayed
   * stuck at mbsfn_prb=30 indefinitely, even after subsequent SIB13 updates
   * correctly showed 0, corrupting every PDSCH/PMCH buffer/stride
   * computation downstream. Track the broadcast value unconditionally. */
  _cell.mbsfn_prb = sib13.mbsfn_area_info_list[0].pmch_bandwidth;

  if (getenv("SIB13_DIAG")) {
    fprintf(stderr, "SIB13_DIAG(Phy) nof_mbsfn_area_info=%u pmch_bandwidth=%u\n",
            sib13.nof_mbsfn_area_info, sib13.mbsfn_area_info_list[0].pmch_bandwidth);
  }
  if (sib13.nof_mbsfn_area_info > 0) {
    {
      std::lock_guard<std::mutex> lock(_sib13_mutex);
      _sib13 = sib13;
    }

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
    if (getenv("MCCH_TABLE_DIAG")) {
      fprintf(stderr, "MCCH_TABLE_DIAG is_r16=%d raw_alloc=0x%x repeat_period=%u offset=%u table=%s\n",
              (int)sib13.mbsfn_area_info_list[0].mcch_cfg.sf_alloc_info_is_r16,
              (unsigned)sib13.mbsfn_area_info_list[0].mcch_cfg.sf_alloc_info,
              (unsigned)enum_to_number(sib13.mbsfn_area_info_list[0].mcch_cfg.mcch_repeat_period),
              (unsigned)sib13.mbsfn_area_info_list[0].mcch_cfg.mcch_offset,
              ss.str().c_str());
    }

    _mcch_configured = true;
    if (getenv("SIB13_DIAG")) {
      fprintf(stderr, "SIB13_DIAG(Phy) wrote _mcch_configured=true, readback=%d\n", (int)_mcch_configured.load());
    }
  }
}

void Phy::set_mbsfn_config(const srsran::mcch_msg_t& mcch) {
  if (getenv("RACE_DIAG")) {
    auto now = std::chrono::high_resolution_clock::now().time_since_epoch();
    fprintf(stderr, "RACEDIAG_WRITE_BEGIN thread=%zu ns=%lld\n",
            std::hash<std::thread::id>{}(std::this_thread::get_id()),
            (long long)std::chrono::duration_cast<std::chrono::nanoseconds>(now).count());
  }
  std::lock_guard<std::mutex> lock(_mcch_mutex);
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
  if (getenv("RACE_DIAG")) {
    auto now = std::chrono::high_resolution_clock::now().time_since_epoch();
    fprintf(stderr, "RACEDIAG_WRITE_END thread=%zu ns=%lld\n",
            std::hash<std::thread::id>{}(std::this_thread::get_id()),
            (long long)std::chrono::duration_cast<std::chrono::nanoseconds>(now).count());
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
    /* TEMPORARY (2026-07-26): widened from {0,5} to the full broadcast-ineligible set.
     * Traced this eNB's own MAC scheduler (rt-mbms-tx/srsenb/src/stack/mac/sched_carrier.cc,
     * phy_common::build_mch_table()/gen_mch_tables.c): the real non-MBSFN subframe set in
     * mixed mode is {0,4,5,9} -- the MCH allocation table only ever writes {1,2,3,6,7,8},
     * matching is_mbsfn_subframe()'s own mixed-mode branch just below, which already
     * correctly excludes all four. This receiver was previously only ever attempting
     * CAS (PDCCH+PDSCH) decode at 0 and 5, silently never even looking at 4 or 9, even
     * though the eNB may legitimately schedule SI/paging grants on any of the four (its
     * own round-robin SI-window counter is not synchronized to which of the four it
     * lands on). Suspected (not yet confirmed) contributor to the observed sf_idx=5-
     * specific sync-loss skew: a scheduler rotating across 4 real positions, previously
     * sampled at only 2 of them. Does NOT touch PSS/SSS tracking (ue_sync.c's own
     * find_peak gating stays at sf_idx==0/5 only -- PSS/SSS physically only exist there,
     * regardless of cell type; this change is purely about which subframes this
     * receiver's own CasFrameProcessor attempts to decode SI/paging content from). */
    return (tti%10 == 0 || tti%10 == 4 || tti%10 == 5 || tti%10 == 9);
  }
}

auto Phy::is_mbsfn_subframe(unsigned tti) -> bool
{
  if (getenv("MCCH_SCHED_DIAG")) {
    static uint64_t call_count = 0;
    call_count++;
    if (call_count % 1000 == 1) {
      fprintf(stderr, "MCCH_SCHED_DIAG is_mbsfn_subframe called tti=%u mbms_dedicated=%d mcch_configured=%d call_count=%llu\n",
              tti, (int)_cell.mbms_dedicated, (int)_mcch_configured.load(), (unsigned long long)call_count);
    }
  }
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
      // _sib13 is guarded by _sib13_mutex (see its own declaration comment) -- this read was
      // missing that lock, racing against set_sib13_info()'s write from whichever thread
      // decodes a fresh SIB13. Snapshot under lock rather than holding it across the whole
      // function, matching the _mcch_mutex pattern already used in mbsfn_config_for_tti().
      srsran::mbsfn_area_info_t::mcch_cfg_t mcch_cfg;
      srsran::mbsfn_area_info_t::subcarrier_spacing_t area_scs;
      {
        std::lock_guard<std::mutex> lock(_sib13_mutex);
        mcch_cfg = _sib13.mbsfn_area_info_list[0].mcch_cfg;
        area_scs = _sib13.mbsfn_area_info_list[0].subcarrier_spacing;
      }
      if (getenv("MCCH_SCHED_DIAG")) {
        static unsigned last_printed_sfn = 0xffffffff;
        if (sfn != last_printed_sfn) {
          last_printed_sfn = sfn;
          fprintf(stderr,
                  "MCCH_SCHED_DIAG sfn=%u sf=%u repeat_period=%u offset=%u sfn_mod=%u table[sf]=%u\n",
                  sfn, sf, enum_to_number(mcch_cfg.mcch_repeat_period), mcch_cfg.mcch_offset,
                  sfn % enum_to_number(mcch_cfg.mcch_repeat_period), (unsigned)_mcch_table[sf]);
        }
      }
      /* Same widening as mbsfn_config_for_tti(): 370 kHz MCCH occupies a whole
       * 3-4 subframe slot, so a repeat_period/offset/_mcch_table match on ANY
       * subframe of the slot containing this tti means the WHOLE slot is
       * MCCH's own subframe (and must be exempted from the exclusion checks
       * below, same as any other SCS's exact match already is). Every other
       * SCS keeps the original, single-subframe check unchanged. */
      if (area_scs == srsran::mbsfn_area_info_t::subcarrier_spacing_t::khz_0dot37) {
        std::pair<uint32_t, uint32_t> slot_pos = scs370_slot_position(tti);
        uint32_t slot_start_tti = tti - slot_pos.first;
        for (uint32_t k = 0; k < slot_pos.second && !is_mcch_sf; k++) {
          uint32_t t = slot_start_tti + k;
          if ((t / 10) % enum_to_number(mcch_cfg.mcch_repeat_period) == mcch_cfg.mcch_offset &&
              _mcch_table[t % 10]) {
            is_mcch_sf = true;
          }
        }
      } else if (sfn % enum_to_number(mcch_cfg.mcch_repeat_period) == mcch_cfg.mcch_offset && _mcch_table[sf]) {
        is_mcch_sf = true;
      }
    }
    if (!is_mcch_sf && _cell.additional_non_mbms_frames > 0) {
      if (sf >= 1 && sf <= _cell.additional_non_mbms_frames && is_cas_subframe((tti / 10) * 10)) {
        return false;
      }
    }
    /* commonSF-Alloc-v1610: when the network has actually signalled it, only
     * treat sf#0/sf#5 as MBSFN-common capacity if declared (see mcch_msg_t's
     * comment for the bit-order caveat). Absent - before MCCH is decoded, or
     * for a third-party cell that doesn't send this extension - preserves the
     * prior always-eligible assumption, so this is purely additive: it cannot
     * regress a deployment (like this project's own TX, which always declares
     * both) that never exercises the "not declared" case. MCCH's own subframe
     * is exempted, same as the additionalNonMBSFNSubframes check above. */
    if (!is_mcch_sf && _mcch_configured) {
      std::lock_guard<std::mutex> lock(_mcch_mutex);
      if (_mcch.common_sf_alloc_v1610_present) {
        if (sf == 0 && !_mcch.common_sf_alloc_v1610_sf0) return false;
        if (sf == 5 && !_mcch.common_sf_alloc_v1610_sf5) return false;
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
  cfg.n_soft_ref_category     = 0;
  cfg.scaling_factor_beta_num = 0;
  cfg.scaling_factor_beta_den = 0;
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

  if (getenv("MCCH_SCHED_DIAG")) {
    static uint64_t call_count2 = 0;
    call_count2++;
    fprintf(stderr, "MCCH_SCHED_DIAG2 tti=%u mcch_configured=%d call_count=%llu\n",
            tti, (int)_mcch_configured.load(), (unsigned long long)call_count2);
  }
  if (!_mcch_configured) {
    {
      return cfg;
    }
  }

  /* Snapshot _mcch/_mch_configured once, under the lock, instead of reading
   * them field-by-field across this whole function -- see _mcch_mutex's doc
   * comment for the confirmed cross-thread race this closes (this function
   * runs on the MTCH-decoding thread pool, concurrently with
   * set_mbsfn_config() writing a fresh MCCH decode from a different thread).
   * Everything below uses this local copy, not the live members. */
  srsran::mcch_msg_t mcch_snapshot;
  bool mch_configured_snapshot;
  {
    std::lock_guard<std::mutex> lock(_mcch_mutex);
    mcch_snapshot          = _mcch;
    mch_configured_snapshot = _mch_configured;
  }

  uint32_t sfn = tti / 10;
  uint8_t sf = tti % 10;

  // _sib13 is guarded by _sib13_mutex (see its own declaration comment) -- this was a bare
  // reference into live, concurrently-writable state (set_sib13_info() writes a fresh decode
  // from whichever thread receives it), missing the lock entirely. Snapshot by value under
  // the lock instead, matching the mcch_snapshot pattern just above; every area_info.* use
  // below now reads this local copy, not the live member.
  srsran::mbsfn_area_info_t area_info;
  {
    std::lock_guard<std::mutex> lock(_sib13_mutex);
    area_info = _sib13.mbsfn_area_info_list[0];
  }

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
  if (mch_configured_snapshot && !_decode_mcch.load(std::memory_order_acquire) && sf == 0) {
    uint32_t mod_period = (uint32_t)enum_to_number(area_info.mcch_cfg.mcch_mod_period);
    if (sfn % mod_period == 0) {
      spdlog::debug("MCCH modification period boundary at SFN {} — scheduling MCCH re-read", sfn);
      _decode_mcch.store(true, std::memory_order_release);
    }
  }

  /* 370 kHz MCCH occupies a whole 3-4 subframe slot, not the single subframe the
   * standard mcch-RepetitionPeriod/mcch-Offset/sf-AllocInfo model was built for -
   * per TS 36.331's own sf-AllocInfo-r16 field description: "When subcarrierSpacingMBMS
   * indicates 0.37 kHz subcarrier spacing, a valid MBMS slot can carry MCCH if ANY
   * subframe corresponding to the slot is configured to carry MCCH." So for 370 kHz,
   * a match on any one subframe of the slot containing tti means the WHOLE slot
   * (every subframe in it) is MCCH-enabled - every other SCS keeps the original,
   * single-subframe check unchanged. */
  bool mcch_subframe_match;
  if (area_info.subcarrier_spacing == SCS_t::khz_0dot37) {
    std::pair<uint32_t, uint32_t> slot_pos = scs370_slot_position(tti);
    uint32_t slot_start_tti = tti - slot_pos.first;
    mcch_subframe_match = false;
    for (uint32_t k = 0; k < slot_pos.second; k++) {
      uint32_t t = slot_start_tti + k;
      if ((t / 10) % enum_to_number(area_info.mcch_cfg.mcch_repeat_period) == area_info.mcch_cfg.mcch_offset &&
          _mcch_table[t % 10] == 1) {
        mcch_subframe_match = true;
        break;
      }
    }
  } else {
    mcch_subframe_match = sfn % enum_to_number(area_info.mcch_cfg.mcch_repeat_period) == area_info.mcch_cfg.mcch_offset &&
                           _mcch_table[sf] == 1;
  }

  if (getenv("MCCH_SCHED_DIAG") && mcch_subframe_match) {
    fprintf(stderr, "MCCH_SCHED_DIAG3 MATCH sfn=%u sf=%u decode_mcch=%d\n",
            sfn, sf, (int)_decode_mcch.load(std::memory_order_acquire));
  }
  if (mcch_subframe_match) {
    /* MCCH and MTCH are both carried over the same PMCH, so they share the same
     * subcarrier spacing - SCS is a property of the PMCH transmission itself, not
     * of the logical channel mapped onto it. cfg.subcarrier_spacing is already set
     * to data_scs above from the same area_info.subcarrier_spacing; no separate,
     * narrower mapping for MCCH is correct here. (Previously this branch redundantly
     * re-derived a SCS using only 3 of the 5 real cases, silently defaulting 2.5kHz
     * and 0.37kHz areas' MCCH to 1.25kHz - wrong per spec, though only externally
     * visible as a decode failure for 7.5kHz, where the corresponding TX-side branch
     * has an explicit correct case that this incomplete one didn't mirror.) */
    if (_decode_mcch.load(std::memory_order_acquire)) {
      cfg.mbsfn_mcs               = enum_to_number(area_info.mcch_cfg.sig_mcs);
      cfg.enable                  = true;
      cfg.is_mcch                 = true;
    }
  } else {
    if (mch_configured_snapshot) {
      cfg.mbsfn_area_id = area_info.mbsfn_area_id;

      for (uint32_t i = 0; i < mcch_snapshot.nof_pmch_info; i++) {
        uint32_t fn_in_scheduling_period = sfn % enum_to_number(mcch_snapshot.pmch_info_list[i].mch_sched_period);
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
        uint32_t sched_period = enum_to_number(mcch_snapshot.pmch_info_list[i].mch_sched_period);
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
        if (getenv("CAS_MUTE_DIAG")) {
          fprintf(stderr, "CAS_MUTE_DIAG_RX tti=%u sfn=%u sf=%u fn_in=%u nof_true_cas=%u sf_idx=%d "
                          "nof_mcch_passed=%u nof_additional_passed=%u cas_muting=%d k_cas=%u n_cas=%u\n",
                  tti, sfn, sf, fn_in_scheduling_period, nof_true_cas, sf_idx, nof_mcch_passed,
                  nof_additional_passed, (int)_cell.cas_muting, (unsigned)_cell.k_cas, (unsigned)_cell.n_cas);
        }

        /* 0-based MCH subframe index within this PMCH's data allocation.
         * For i=0 the MCCH occupies sf_idx=0; the first data sf has sf_idx=1.
         * pmch_start=1 maps sf_idx=1 → mch_subframe_idx=0 (TS 36.211 §6.5.3).
         * Guard: sf_idx must be >= pmch_start to avoid uint32_t wraparound on subtraction. */
        uint32_t pmch_start = (i == 0) ? 1u : (uint32_t)(mcch_snapshot.pmch_info_list[i - 1].sf_alloc_end + 1);
        if (sf_idx >= (int)pmch_start && (uint32_t)sf_idx <= mcch_snapshot.pmch_info_list[i].sf_alloc_end) {
          area = i;
          cfg.mbsfn_mcs           = mcch_snapshot.pmch_info_list[i].data_mcs;
          cfg.use_mcs_table2      = mcch_snapshot.pmch_info_list[i].use_mcs_table2;
          cfg.time_interleaving_n = mcch_snapshot.pmch_info_list[i].time_interleaving_n;
          cfg.time_interleaving_m = mcch_snapshot.pmch_info_list[i].time_interleaving_m;
          cfg.n_soft_ref_category     = mcch_snapshot.pmch_info_list[i].n_soft_ref_category;
          cfg.scaling_factor_beta_num = mcch_snapshot.pmch_info_list[i].scaling_factor_beta_num;
          cfg.scaling_factor_beta_den = mcch_snapshot.pmch_info_list[i].scaling_factor_beta_den;
          cfg.cyclic_shift        = mcch_snapshot.pmch_info_list[i].cyclic_shift;
          cfg.cyclic_shift_alpha  = mcch_snapshot.pmch_info_list[i].cyclic_shift_alpha;
          cfg.freq_interleaving   = mcch_snapshot.pmch_info_list[i].freq_interleaving;
          cfg.mch_subframe_idx = (uint32_t)sf_idx - pmch_start;
          cfg.pmch_idx         = (uint8_t)i;
          cfg.enable = true;
          if (cfg.mch_subframe_idx == 0 && getenv("RACE_DIAG")) {
            auto now = std::chrono::high_resolution_clock::now().time_since_epoch();
            fprintf(stderr, "RACEDIAG_READ tti=%u thread=%zu ns=%lld\n", tti,
                    std::hash<std::thread::id>{}(std::this_thread::get_id()),
                    (long long)std::chrono::duration_cast<std::chrono::nanoseconds>(now).count());
          }
          /* pmch-TimeInterleavingN/M-LastMTCH-r19 (TS 36.331 CR5168r3): mirrors TX's
           * identical is_mch_subframe() logic exactly -- MbsfnFrameProcessor pushes
           * the last session's window start via set_last_mtch_start() once per
           * period, right after decoding that period's MSI. 0 (no override
           * configured, or single-session) means this comparison is always false,
           * so the common case is unaffected. */
          uint32_t last_mtch_start_sf = get_last_mtch_start((uint8_t)i);
          if (last_mtch_start_sf > 0 && cfg.mch_subframe_idx >= last_mtch_start_sf) {
            cfg.mch_subframe_idx -= last_mtch_start_sf;
            uint8_t n_last = mcch_snapshot.pmch_info_list[i].time_interleaving_n_last_mtch;
            uint8_t m_last = mcch_snapshot.pmch_info_list[i].time_interleaving_m_last_mtch;
            if (n_last > 0) {
              cfg.time_interleaving_n = n_last;
              cfg.time_interleaving_m = (m_last > 0) ? m_last : cfg.time_interleaving_m;
            } else if (m_last > 0) {
              cfg.time_interleaving_m = m_last;
            }
          }
          spdlog::debug("PMCH {}: mch_subframe_idx {}, mcs {}", i, cfg.mch_subframe_idx, cfg.mbsfn_mcs);
          break;
        }
      }
    }
  }
  return cfg;
}
