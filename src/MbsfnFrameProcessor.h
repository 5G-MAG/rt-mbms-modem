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

#pragma once

#include <cstdint>
#include <string>
#include <thread>
#include <vector>
#include <map>
#include <utility>
#include "srsran/srsran.h"
#include "srsran/rlc/rlc.h"
#include "srsran/upper/pdcp.h"
#include "srsran/mac/pdu.h"
#include <libconfig.h++>
#include "Phy.h"
#include "RestHandler.h"

/**
 *  Frame processor for MBSFN subframes. Handles the complete processing chain for
 *  a CAS subframe: calls FFT and channel estimation, decodes PDSCH and passes received PDUs to RLC.
 */
class MbsfnFrameProcessor {
  public:
    /**
     *  Default constructor.
     *
     *  @param cfg Config singleton reference
     *  @param phy PHY reference
     *  @param rlc RLC reference
     *  @param log_h srsLTE log handle for the MCH MAC msg decoder
     *  @param rest RESTful API handler reference
     */
    MbsfnFrameProcessor(const libconfig::Config& cfg, srsran::rlc& rlc, Phy& phy, srslog::basic_logger& log_h, RestHandler& rest, unsigned rx_channels )
      : _cfg(cfg)
      , _rlc(rlc)
      , _phy(phy)
      , _rest(rest)
      , mch_mac_msg(20, log_h)
      , _rx_channels(rx_channels)
      {
        _allow_rrc_sn_across_periods = false;
        cfg.lookupValue("modem.phy.allow_rrc_sn_across_periods", _allow_rrc_sn_across_periods); 
      }

    /**
     *  Default destructor.
     */
    virtual ~MbsfnFrameProcessor();

    /**
     *  Initialize signal- and softbuffers, init all underlying components. 
     *  Must be called once before the first call to process().
     */
    bool init();

    /**
     *  Process the sample data in the signal buffer. Data must already be present in the buffer
     *  obtained through the handle returnd by rx_buffer()
     *
     *  @param tti TTI of the subframe the data belongs to
     */
    int process(uint32_t tti);

    /**
     *  Set the parameters for the cell (Nof PRB, etc).
     *
     *  @param cell The cell we're camping on
     *  @param mbsfn_scs Real MBSFN subcarrier spacing, for correctly sizing the FFT
     *         when cell.mbsfn_prb != cell.nof_prb - see srsran_ue_dl_set_cell_scs().
     */
    void set_cell(srsran_cell_t cell, srsran_scs_t mbsfn_scs = SRSRAN_SCS_15KHZ);

    /**
     *  Get a handle of the signal buffer to store samples for processing in, 
     *  and lock this processor.
     *
     *  The processor unlocks itself after (failed or successful) frame processing in process().
     *  If process() is not called by the application after calling this method, it must unlock the 
     *  processor itself by calling unlock()
     */
    cf_t** get_rx_buffer_and_lock() { _mutex.lock(); return _signal_buffer_rx; }

    /**
     *  Size of the signal buffer
     */
    uint32_t rx_buffer_size() { return _signal_buffer_max_samples; }

    /**
     *  Set MBSFN parameters: area ID and subcarrier spacing
     */
    void configure_mbsfn(uint8_t area_id, srsran_scs_t subcarrier_spacing);

    /**
     *  Returns tru if MBSFN params have already been configured
     */
    bool mbsfn_configured() { return _mbsfn_configured; }

    /**
     *  The grid width (nof_prb, already resolved to max(cell nof_prb,
     *  mbsfn_prb) by the caller) this instance was last configured with -
     *  see set_cell()'s doc comment for why the caller must reconfigure
     *  (not just check mbsfn_configured()) when this changes: pmch_bandwidth
     *  can legitimately change value mid-session (e.g. a live SET), and each
     *  of the thread_cnt worker instances only ever configures once without
     *  this check, silently keeping a stale FFT/buffer width indefinitely
     *  and corrupting every subsequent decode (confirmed live 2026-07-17,
     *  NaN post-equalization power once the actual width no longer matched
     *  what this instance was still using).
     */
    uint32_t configured_nof_prb() { return _cell.nof_prb; }

    /**
     *  Unlock the processor
     *
     *  @see get_rx_buffer_and_lock() 
     */
    void unlock() { _mutex.unlock(); }

    /**
     *  Lock the processor
     *
     *  Used when getting the BLER values.
     */
    void lock() { _mutex.lock(); }

    /**
     *  Get the constellation diagram data (I/Q data of the subcarriers after CE)
     */
    const std::vector<uint8_t> mch_data() const;

    /**
     *  Fill `out` with the CE values (frequency domain) for displaying the
     *  spectrum, same convention as CasFrameProcessor::ce_values(). Reuses
     *  out's capacity (no allocation once out is sized).
     */
    void ce_values(std::vector<uint8_t>& out);

    /**
     *  Fill `out` with the channel impulse response (IFFT of the
     *  frequency-domain channel estimate), magnitude in dB, fftshifted so lag 0
     *  is centered. Reuses out's capacity (no allocation once out is sized).
     */
    void cir_values(std::vector<uint8_t>& out);

    /**
     *  Get the CINR estimate (in dB)
     */
    float cinr_db() { return _ue_dl.chest_res.snr_db; }

  private:
    const libconfig::Config& _cfg;
    srsran::rlc& _rlc;
    Phy& _phy;

    srsran_cell_t _cell;

    cf_t*    _signal_buffer_rx[SRSRAN_MAX_PORTS] = {};
    uint32_t _signal_buffer_max_samples          = 0;

    static const uint32_t  _payload_buffer_sz = SRSRAN_MAX_BUFFER_SIZE_BYTES;
    uint8_t                _payload_buffer[_payload_buffer_sz];
    /* Non-time-interleaved PMCH/MCCH decode uses slot 0 only (see process()).
     * Time-interleaved PMCH needs one softbuffer PER SLOT m (m=0..M-1) so
     * each of the M pipelined transport blocks accumulates its own LLR/CRC
     * state independently - see srsran_pmch_decode's comment in pmch.c for
     * the (m,n) derivation. Lazily initialized (_softbuffer_init[m] tracks
     * which slots have actually been srsran_softbuffer_rx_init'd) since a
     * configured M is typically far below the spec max of 32. */
    srsran_softbuffer_rx_t _softbuffer[SRSRAN_PMCH_MAX_TI_M];
    bool                   _softbuffer_init[SRSRAN_PMCH_MAX_TI_M] = {};
    /* Tracks whether slot m's CURRENT time-interleaved TB has already been
     * counted (as a success or a final failure) in _rest._mch stats. Needed
     * because srsran_pmch_decode() reports crc=false on every subframe of a
     * span after the one where it first actually succeeds (by design, to
     * avoid re-delivering the same TB's content) - without this, those
     * trailing "already decoded" subframes would be miscounted as fresh
     * failures. Reset at slot m's own n==0 (new TB starting), same moment
     * the softbuffer itself resets. */
    bool                   _ti_reported[SRSRAN_PMCH_MAX_TI_M] = {};
    /* Last (N, M) this processor actually saw, so process() can detect a LIVE change (the eNB's
     * control socket allows changing embms.pmch1.time_interleaving_n/m while running -- see
     * rrc::reconfigure_embms()) and flush the slot-indexed softbuffer state above before it goes
     * stale. Sentinel 0xFF (not a legal N or M) so the very first call after construction always
     * takes the "changed" branch too -- harmless there since every slot starts uninitialized
     * anyway, but it keeps the check itself branch-free (no separate "is this the first call"
     * flag). See process()'s own comment on why a block-length change makes existing softbuffer
     * state actively wrong, not just stale. */
    uint8_t                _last_ti_n = 0xFF;
    uint8_t                _last_ti_m = 0xFF;

    srsran_ue_dl_t     _ue_dl     = {};
    srsran_ue_dl_cfg_t _ue_dl_cfg = {};
    srsran_dl_sf_cfg_t _sf_cfg = {};
    srsran_pmch_cfg_t  _pmch_cfg  = {};

    uint8_t _area_id = 1;
    bool _mbsfn_configured = false;

    srsran::mch_pdu mch_mac_msg;
    std::mutex _mutex;

    RestHandler& _rest;

    unsigned _rx_channels;

    /* IFFT plan and scratch buffers for cir_values()/ce_values(), (re)created
     * in set_cell() -- called only from the single main thread, never from
     * process()'s worker-pool thread -- and sized once so these never
     * allocate on their hot per-subframe path. */
    srsran_dft_plan_t  _cir_plan       = {};
    bool               _cir_plan_ready = false;
    uint32_t           _cir_plan_size  = 0;
    std::vector<cf_t>  _cir_scratch_freq;
    std::vector<cf_t>  _cir_scratch_time;
    std::vector<cf_t>  _cir_scratch_shifted;
    std::vector<float> _cir_scratch_db;
    std::vector<float> _ce_scratch_db;
    /* Persistent output byte-buffers for ce_values()/cir_values(), swapped into
     * the _rest members for publishing so neither the fill nor the publish
     * allocates in steady state (the swapped-back buffer is reused next cycle). */
    std::vector<uint8_t> _ce_out_bytes;
    std::vector<uint8_t> _cir_out_bytes;

    /* ce_values()/cir_values() are only ever polled by the UI at ~10Hz, but
     * this process() runs on every MBSFN subframe (up to ~1kHz across all
     * mbsfn_processors) - recomputing the IFFT and re-deriving the CE
     * snapshot on every single one is wasted work and, worse, wasted
     * allocation/compute pressure on the actual decode hot path for no
     * visible benefit. Update only every Nth subframe. */
    static constexpr uint32_t CE_CIR_UPDATE_STRIDE = 10;
    uint32_t _ce_cir_update_counter = 0;

    /* Worst-case code-block count for a time-interleaved MCH TB: a TI'd
     * TBS is inflated by the TI factor then rounded against
     * pmch_ti_tbs.h's fixed tables, which saturate at 502624 bits
     * regardless of mbsfn_prb. 502624/(SRSRAN_TCOD_MAX_LEN_CB-24)+1 = 83,
     * about 4x srsran_softbuffer_rx_init(q, 100)'s untuned single-subframe
     * sizing (21 CBs) - see the softbuffer init sites in the .cpp. */
    static constexpr uint32_t PMCH_MAX_CB_TI = 83;

    bool _allow_rrc_sn_across_periods = false;
    static std::mutex _sched_stop_mutex;
    static std::map<std::pair<uint8_t,uint8_t>, uint16_t> _sched_stops;

    static std::mutex _rlc_mutex;
    static int _current_mcs;
};
