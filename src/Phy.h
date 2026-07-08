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

#include <functional>
#include <cstdint>
#include <string>
#include <map>
#include <vector>
#include <atomic>
#include <array>
#include <mutex>
#include <utility>
#include <thread>
#include <libconfig.h++>

#include "srsran/srsran.h"
#include "srsran/interfaces/rrc_interface_types.h"
#include "srsran/common/gen_mch_tables.h"
#include "srsran/phy/common/phy_common.h"

constexpr unsigned int MAX_PRB = 100;

/**
 *  The PHY component. Handles synchronisation and is the central hub for
 *  lower level processing
 *
 */
class Phy {
 public:
    /**
     *  Definition of the callback function used to fetch samples from the SDR
     */
    typedef std::function<int(cf_t* data[SRSRAN_MAX_CHANNELS], uint32_t nsamples, srsran_timestamp_t* rx_time)> get_samples_t;

    /**
     *  Default constructor.
     *
     *  @param cfg Config singleton reference
     *  @param cb  Sample recv callback
     *  @param cs_nof_prb  Nr of PRBs to use during cell search
     *  @param override_nof_prb  If set, overrides the nof PRB received in the MIB
     */
    Phy(const libconfig::Config& cfg, get_samples_t cb, uint8_t cs_nof_prb, int8_t override_nof_prb, uint8_t rx_channels);
    
    /**
     *  Default destructor.
     */
    virtual ~Phy();
    
    /**
     *  Initialize the underlying components.
     */
    bool init();

    /**
     *  Search for a cell
     *
     *  Returns true if a cell has been found and the MIB could be decoded, false otherwise.
     */
    bool cell_search();

    /**
     *  Synchronizes PSS/SSS and tries to deocode the MIB.
     *
     *  Returns true on success, false otherwise.
     */
    bool synchronize_subframe();

    /**
     * Get the sample data for the next subframe.
     */
    bool get_next_frame(cf_t** buffer, uint32_t size);

    /**
     * Get the current cell (with params adjusted for MBSFN)
     */
    srsran_cell_t cell() { 
      return _cell;
    }

    /**
     * Get the current number of PRB.
     */
    unsigned nr_prb() { return _cell.nof_prb; }

    /**
     * Get the current subframe TTI
     */
    uint32_t tti() { return _tti; }
    
    /**
     * Get the current CFO value
     */
    float cfo() { return srsran_ue_sync_get_cfo(&_ue_sync);}

    /**
     * Set the CFO value from channel estimation
     */
    void set_cfo_from_channel_estimation(float cfo) { srsran_ue_sync_set_cfo_ref(&_ue_sync, cfo); }

    /**
     *  Number of times the MIB has been successfully decoded (once at initial
     *  acquisition, and again on every resync). MIB isn't re-decoded every
     *  subframe like PDSCH/PMCH, so this - not a BLER - is the honest signal
     *  of MIB health: it stops incrementing if the receiver can no longer
     *  reacquire after a sync loss.
     */
    uint32_t mib_decode_count() { return _mib_decode_count; }

    /**
     *  Number of antenna ports from the most recently decoded MIB.
     */
    uint32_t mib_nof_ports() { return _cell.nof_ports; }

    /**
     *  When the MIB was last successfully decoded (ms since epoch), from
     *  either the initial cell_search() acquisition or a synchronize_subframe()
     *  resync. Unprotected, matching _cell's own existing no-mutex precedent
     *  (same writer thread; REST reads a torn-read-tolerant snapshot).
     */
    uint64_t last_mib_decoded_at() { return _last_mib_decoded_at; }
    void set_mib_decoded_at(uint64_t now_ms) { _last_mib_decoded_at = now_ms; }

    /**
     *  Current absolute SFN (System Frame Number), derived from the tti()
     *  counter (which is only ever set to sfn * kSubframesPerFrame - see
     *  synchronize_subframe()). Same 10-subframes-per-frame constant already
     *  hardcoded in RestHandler::record_subframe_event().
     */
    uint32_t sfn() { return _tti / 10; }

    /**
     * PSS correlation peak value from the tracking-stage synchronizer (higher
     * is a stronger/cleaner PSS detection).
     */
    float pss_peak_value() { return _ue_sync.strack.peak_value; }

    /**
     * SSS correlation value from the tracking-stage synchronizer.
     */
    float sss_corr() { return _ue_sync.strack.sss_corr; }

    /**
     * Whether SSS was successfully detected on the most recent tracking attempt.
     */
    bool sss_detected() { return _ue_sync.strack.sss_detected; }

    /**
     * Set the values received in SIB13
     */
    void set_mch_scheduling_info(const srsran::sib13_t& sib13);

    /**
     * Set MBSFN configuration values
     */
    void set_mbsfn_config(const srsran::mcch_msg_t& mcch);

    /**
     * Clear configuration values
     */
    void reset() { _mcch_configured = _mch_configured = false; }

    /**
     * Return true if MCCH has been configured
     */
    bool mcch_configured() { return _mcch_configured; }

    /**
     * Returns the current MBSFN area ID
     */
    uint8_t mbsfn_area_id() { return _sib13.mbsfn_area_info_list[0].mbsfn_area_id; }

    /**
     * Returns the MBSFN configuration (MCS, etc) for the subframe with the passed TTI
     */
    srsran_mbsfn_cfg_t mbsfn_config_for_tti(uint32_t tti, unsigned& area);

    /**
     * Enable MCCH decoding
     */
    void set_decode_mcch(bool d) { _decode_mcch.store(d, std::memory_order_release); }

    /**
     * pmch-TimeInterleavingN/M-LastMTCH-r19 (TS 36.331 CR5168r3) cross-layer channel:
     * MbsfnFrameProcessor pushes this once per scheduling period, right after decoding
     * that period's MSI, to tell mbsfn_config_for_tti() where (relative to this PMCH's
     * own data region) the last of several MTCH sessions' window starts. 0 = no
     * distinct last-session window this period. Mirrors TX's
     * phy_common::set_last_mtch_start() exactly -- same convention, same "plain
     * non-blocking write" design (not a blocking wait: the boundary genuinely isn't
     * knowable before the first MSI of a newly-active window decodes, and blocking
     * PHY on that would risk stalling it if the boundary never arrives for a cell
     * that never uses this feature).
     */
    void set_last_mtch_start(uint8_t pmch_idx, uint32_t start_sf) {
      if (pmch_idx < _last_mtch_start.size()) {
        std::lock_guard<std::mutex> lock(_last_mtch_start_mutex);
        _last_mtch_start[pmch_idx] = start_sf;
      }
    }

    /**
     * Return the most recently decoded MCCH message (for change detection)
     */
    const srsran::mcch_msg_t& current_mcch() const { return _mcch; }

    /**
     * Get number of PRB in MBSFN/PMCH
     */
    uint8_t nof_mbsfn_prb() { return _cell.mbsfn_prb; }

    /**
     * Override number of PRB in MBSFN/PMCH
     */
    void set_nof_mbsfn_prb(uint8_t prb) { _cell.mbsfn_prb = prb; }

    void set_cas_muting(bool enabled, uint8_t k, uint8_t n) {
      _cell.cas_muting = enabled;
      _cell.k_cas      = k;
      _cell.n_cas      = n;
      /* _ue_sync was already handed a copy of _cell via srsran_ue_sync_set_cell()
       * during initial cell acquisition, before SIB1 (and this CAS-muting config)
       * was ever decoded - poke the live sync object's copy directly, same pattern
       * as the other runtime _ue_sync.* setters above/below, so its PSS-tracking
       * loop (ue_sync.c) knows which CAS occasions are genuinely muted instead of
       * treating every missed one as a sync loss. */
      _ue_sync.cell.cas_muting = enabled;
      _ue_sync.cell.k_cas      = k;
      _ue_sync.cell.n_cas      = n;
    }

    /**
     * Set a pending ROM cross-carrier redirect (TS 36.331 Rel-16 mbms_rom_info_list_r16).
     * Called by RRC when SIB13 carries redirect info; consumed by the main loop via
     * consume_rom_redirect(). Thread-safe: written from the CAS processor thread,
     * read from the main thread.
     * EARFCN and PRB are packed into a single 64-bit atomic so they are always a
     * consistent pair. EARFCN 0 is the "no pending" sentinel and cannot be redirected to.
     */
    void set_rom_redirect(uint32_t earfcn, uint32_t nof_prb) {
      _rom_redirect.store(((uint64_t)earfcn << 32) | nof_prb, std::memory_order_release);
    }
    bool rom_redirect_pending() const {
      return (_rom_redirect.load(std::memory_order_acquire) >> 32) != 0;
    }
    // Returns {earfcn, nof_prb} as an atomic pair; earfcn=0 means nothing was pending.
    std::pair<uint32_t, uint32_t> consume_rom_redirect() {
      uint64_t v = _rom_redirect.exchange(0, std::memory_order_acq_rel);
      return {static_cast<uint32_t>(v >> 32), static_cast<uint32_t>(v & 0xFFFFFFFFu)};
    }

    void set_cell();

    bool is_cas_subframe(unsigned tti);
    bool is_mbsfn_subframe(unsigned tti);

    /**
     * SIB12 (SystemInformationBlockType12-r9, TS 36.331 §6.3.1) CMAS/PWS
     * warning notification, decoded to displayable text. msg_id/serial_number
     * together identify one distinct alert - the network re-broadcasts the
     * same alert repeatedly while it's active, so a client (like this one's
     * REST consumer) must dedup on that pair rather than treat every
     * reception as a new alert, mirroring how Android's CellBroadcastReceiver
     * dedups CMAS/ETWS pages. Single-segment only for now (matches the
     * eNB side's current limitation - see rt-mbms-tx rrc.cc's
     * SINGLE_SEGMENT_WARN_THRESHOLD TODO); a genuinely multi-segment message
     * would need reassembly by warning_msg_segment_num before this point.
     */
    struct PwsAlert {
      uint32_t msg_id             = 0;
      uint32_t serial_number      = 0;
      uint8_t  data_coding_scheme = 0;
      std::string text;      /* decoded per data_coding_scheme (GSM 7-bit or UCS-2) */
      std::string label;     /* human-readable alert type from msg_id, e.g. "CMAS: Severe" -
                               * mirrors mbms-control-portal's lib/cap.js ALERT_TYPES table,
                               * the CBE that actually originates these message identifiers
                               * in this project; empty if msg_id isn't one of its known values. */
      uint64_t first_received_at = 0; /* ms since epoch, set by the caller */
      uint64_t last_received_at  = 0;
      uint32_t repeat_count      = 1; /* how many times this exact msg_id/serial has been seen */
    };

    /**
     * Record a decoded SIB12 alert. Dedups against the most recently stored
     * alert with the same msg_id/serial_number (just bumps repeat_count and
     * last_received_at); anything else is a genuinely new alert, prepended to
     * the history. History is capped at kMaxPwsAlertHistory entries, oldest
     * dropped first - this is receive-only telemetry, not something a client
     * ever needs unbounded.
     */
    void add_pws_alert(PwsAlert alert, uint64_t now_ms) {
      std::lock_guard<std::mutex> lock(_pws_alerts_mutex);
      if (!_pws_alerts.empty() && _pws_alerts.front().msg_id == alert.msg_id &&
          _pws_alerts.front().serial_number == alert.serial_number) {
        _pws_alerts.front().repeat_count++;
        _pws_alerts.front().last_received_at = now_ms;
        return;
      }
      alert.first_received_at = now_ms;
      alert.last_received_at  = now_ms;
      _pws_alerts.insert(_pws_alerts.begin(), std::move(alert));
      if (_pws_alerts.size() > kMaxPwsAlertHistory) {
        _pws_alerts.resize(kMaxPwsAlertHistory);
      }
    }
    std::vector<PwsAlert> pws_alerts() const {
      std::lock_guard<std::mutex> lock(_pws_alerts_mutex);
      return _pws_alerts;
    }

    /**
     * SIB10 (SystemInformationBlockType10, TS 36.331 §6.3.1) ETWS Primary
     * Notification - a compact, text-free alert (unlike SIB11/SIB12) that
     * just signals a hazard classification and UE alerting instructions. Same
     * msg_id/serial_number dedup and repeating-broadcast semantics as
     * PwsAlert above (TS 23.041 §5.4.4: the network keeps rebroadcasting the
     * same primary notification while active).
     */
    struct EtwsPrimaryAlert {
      uint32_t msg_id             = 0;
      uint32_t serial_number      = 0;
      uint8_t  warning_type_value = 0;     /* TS 23.041 §9.3.24: 0=EQ,1=tsunami,2=EQ+tsunami,3=test,4=other */
      bool     emergency_user_alert = false;
      bool     popup                = false;
      std::string label;     /* human-readable, e.g. "ETWS: Earthquake" - see pws_alert_label()/
                               * etws_warning_type_label() in Rrc.cpp */
      uint64_t first_received_at = 0;
      uint64_t last_received_at  = 0;
      uint32_t repeat_count      = 1;
    };
    void add_etws_primary_alert(EtwsPrimaryAlert alert, uint64_t now_ms) {
      std::lock_guard<std::mutex> lock(_etws_primary_alerts_mutex);
      if (!_etws_primary_alerts.empty() && _etws_primary_alerts.front().msg_id == alert.msg_id &&
          _etws_primary_alerts.front().serial_number == alert.serial_number) {
        _etws_primary_alerts.front().repeat_count++;
        _etws_primary_alerts.front().last_received_at = now_ms;
        return;
      }
      alert.first_received_at = now_ms;
      alert.last_received_at  = now_ms;
      _etws_primary_alerts.insert(_etws_primary_alerts.begin(), std::move(alert));
      if (_etws_primary_alerts.size() > kMaxPwsAlertHistory) {
        _etws_primary_alerts.resize(kMaxPwsAlertHistory);
      }
    }
    std::vector<EtwsPrimaryAlert> etws_primary_alerts() const {
      std::lock_guard<std::mutex> lock(_etws_primary_alerts_mutex);
      return _etws_primary_alerts;
    }

    /**
     * SIB11 (SystemInformationBlockType11, TS 36.331 §6.3.1) ETWS Secondary
     * Notification - carries the actual warning text (same GSM-7/UCS-2
     * decode as SIB12/CMAS). Per TS 23.041 §5.4.4, if a primary and secondary
     * notification are both signalled for the same event they share the same
     * msg_id/serial_number - so this history and SIB10's above use the same
     * dedup key shape but are tracked independently (a UE may receive either
     * without the other). Single-segment only for now, same limitation as
     * PwsAlert above.
     */
    struct EtwsSecondaryAlert {
      uint32_t msg_id             = 0;
      uint32_t serial_number      = 0;
      uint8_t  data_coding_scheme = 0;
      std::string text;
      std::string label;
      uint64_t first_received_at = 0;
      uint64_t last_received_at  = 0;
      uint32_t repeat_count      = 1;
    };
    void add_etws_secondary_alert(EtwsSecondaryAlert alert, uint64_t now_ms) {
      std::lock_guard<std::mutex> lock(_etws_secondary_alerts_mutex);
      if (!_etws_secondary_alerts.empty() && _etws_secondary_alerts.front().msg_id == alert.msg_id &&
          _etws_secondary_alerts.front().serial_number == alert.serial_number) {
        _etws_secondary_alerts.front().repeat_count++;
        _etws_secondary_alerts.front().last_received_at = now_ms;
        return;
      }
      alert.first_received_at = now_ms;
      alert.last_received_at  = now_ms;
      _etws_secondary_alerts.insert(_etws_secondary_alerts.begin(), std::move(alert));
      if (_etws_secondary_alerts.size() > kMaxPwsAlertHistory) {
        _etws_secondary_alerts.resize(kMaxPwsAlertHistory);
      }
    }
    std::vector<EtwsSecondaryAlert> etws_secondary_alerts() const {
      std::lock_guard<std::mutex> lock(_etws_secondary_alerts_mutex);
      return _etws_secondary_alerts;
    }

    /**
     * ETSI TS 103 720 clause 5.10 / ETSI TS 124 117 (OMA-DM MO
     * urn:oma:mo:ext-3gpp-tv-config:1.0): the standardized "TV Service
     * Configuration MO" an application is expected to push to the MBMS Client
     * (this process) via the MBMS-API, rather than the receiver sourcing its
     * frequency/service info from ad hoc local config. Distilled to the two
     * leaf groups this PHY/RRC-layer receiver actually needs -
     * <X>/PLMNList/<X>/RANInfo/<X>/EARFCN and .../TMGIConfiguration's TMGI
     * lists (USD kept as an opaque TS 26.346 string - parsing it is a
     * middleware-layer concern, out of scope here).
     */
    struct TvConfigTmgi {
      std::string tmgi;
      std::string usd; /* TS 26.346 User Service Description, opaque here */
    };
    struct TvConfigPlmn {
      std::string plmn_id;
      std::vector<uint32_t> earfcns;             /* RANInfo/<X>/EARFCN */
      std::vector<TvConfigTmgi> tmgis_for_sa;      /* TMGIConfiguration/TMGIListForSA */
      std::vector<TvConfigTmgi> tmgis_for_service; /* TMGIConfiguration/TMGIListForService */
    };

    /**
     * Replace the whole TV Service Configuration MO (all PLMNs at once,
     * matching the MO's own Replace access type on PLMNList) - called by
     * RestHandler's PUT /tv_config, and once at startup from modem.conf's
     * [tv_config] section if present.
     */
    void set_tv_config(std::vector<TvConfigPlmn> plmns) {
      std::lock_guard<std::mutex> lock(_tv_config_mutex);
      _tv_config = std::move(plmns);
    }
    std::vector<TvConfigPlmn> tv_config() const {
      std::lock_guard<std::mutex> lock(_tv_config_mutex);
      return _tv_config;
    }
    /**
     * All EARFCNs provisioned for a given PLMN (empty if that PLMN isn't
     * configured, or plmn_id is empty and there's more than one PLMN entry -
     * a specific PLMN must be named in that case). Used to cross-check a
     * live ROM redirect (TS 36.331 mbms_rom_info_list_r16) against what was
     * actually provisioned, and to pick the initial search frequency.
     */
    std::vector<uint32_t> tv_config_earfcns(const std::string& plmn_id = "") const {
      std::lock_guard<std::mutex> lock(_tv_config_mutex);
      for (const auto& plmn : _tv_config) {
        if (plmn_id.empty() || plmn.plmn_id == plmn_id) {
          return plmn.earfcns;
        }
      }
      return {};
    }

    /**
     * SIB1-MBMS (SystemInformationBlockType1-MBMS-r14, TS 36.331 §6.3.11a) fields
     * not already captured elsewhere - decoded in Rrc::handle_sib1 but previously
     * only spdlog'd. PLMN/TAC/cell identity, SI scheduling table, and the Rel-19
     * CAS muting config, kept as the latest-received snapshot for a REST consumer.
     */
    struct Sib1Plmn {
      std::string mcc; /* formatted 3-digit string, empty if mcc_present was false */
      std::string mnc; /* formatted 2- or 3-digit string */
    };
    struct Sib1SchedInfoEntry {
      uint16_t si_periodicity_rf = 0;   /* enum_to_number() of si_periodicity_r14 */
      std::vector<uint8_t> sib_types;   /* sib_map_info_r14 entries' to_number() */
    };
    struct Sib1Info {
      std::vector<Sib1Plmn> plmns;
      uint32_t tac                = 0;
      uint32_t cell_id            = 0;
      uint16_t si_win_len_ms      = 0;
      uint8_t  sys_info_value_tag = 0;
      std::vector<Sib1SchedInfoEntry> sched_info;
      bool     cas_muting_enabled = false;
      uint8_t  k_cas              = 0;
      uint8_t  n_cas              = 0;
      uint64_t last_received_at   = 0; /* ms since epoch */
    };

    void set_sib1_info(Sib1Info info) {
      std::lock_guard<std::mutex> lock(_sib1_mutex);
      _sib1         = std::move(info);
      _sib1_present = true;
    }
    bool sib1_present() const {
      std::lock_guard<std::mutex> lock(_sib1_mutex);
      return _sib1_present;
    }
    Sib1Info sib1_info() const {
      std::lock_guard<std::mutex> lock(_sib1_mutex);
      return _sib1;
    }

    /**
     * SIB13's Rel-16 ROM redirect list (mbms_rom_info_list_r16) in full - the
     * operational redirect (set_rom_redirect(), below) only ever acts on the
     * first entry; this keeps every entry for display/audit purposes.
     */
    struct Sib13RomInfo {
      uint32_t earfcn  = 0;
      uint8_t  bw_prb  = 0;   /* bw_r16.to_number() */
      bool     has_scs = false;
      float    scs_khz = 0;   /* subcarrier_spacing_r16.to_number(), if has_scs */
    };
    void set_sib13_rom_info(std::vector<Sib13RomInfo> rom_info) {
      std::lock_guard<std::mutex> lock(_sib13_mutex);
      _sib13_rom_info = std::move(rom_info);
    }
    std::vector<Sib13RomInfo> sib13_rom_info() const {
      std::lock_guard<std::mutex> lock(_sib13_mutex);
      return _sib13_rom_info;
    }
    void set_sib13_received_at(uint64_t now_ms) {
      std::lock_guard<std::mutex> lock(_sib13_mutex);
      _sib13_last_received_at = now_ms;
    }
    uint64_t sib13_last_received_at() const {
      std::lock_guard<std::mutex> lock(_sib13_mutex);
      return _sib13_last_received_at;
    }
    /* Copy-out getter for the raw SIB13 struct, for the sib_info REST endpoint.
     * The three pre-existing internal readers (mbsfn_area_id(),
     * mbsfn_subcarrier_spacing(), mbsfn_subcarrier_spacing_khz()) stay unlocked
     * as before - only the new write path (set_mch_scheduling_info) and this
     * getter take _sib13_mutex, since the REST thread is a new, third reader. */
    srsran::sib13_t sib13() const {
      std::lock_guard<std::mutex> lock(_sib13_mutex);
      return _sib13;
    }

    /**
     * SIB15 (SystemInformationBlockType15-r11, TS 36.331 §6.3.13) MBMS Service
     * Area Identities, intra- and inter-frequency.
     */
    struct Sib15InterFreqSai {
      uint32_t earfcn = 0;
      std::vector<uint32_t> sai_list;
    };
    struct Sib15Info {
      std::vector<uint32_t> intra_freq_sai;
      std::vector<Sib15InterFreqSai> inter_freq_sai;
      uint64_t last_received_at = 0;
    };
    void set_sib15_info(Sib15Info info) {
      std::lock_guard<std::mutex> lock(_sib15_mutex);
      _sib15         = std::move(info);
      _sib15_present = true;
    }
    bool sib15_present() const {
      std::lock_guard<std::mutex> lock(_sib15_mutex);
      return _sib15_present;
    }
    Sib15Info sib15_info() const {
      std::lock_guard<std::mutex> lock(_sib15_mutex);
      return _sib15;
    }

    /**
     * SIB16 (SystemInformationBlockType16-r11, TS 36.331 §6.3.14) GPS time.
     */
    struct Sib16Info {
      bool     has_time_info           = false;
      uint64_t gps_time_10ms           = 0;  /* raw 48-bit counter, TS 36.331 units */
      bool     has_leap_seconds        = false;
      uint16_t leap_seconds            = 0;
      bool     has_local_time_offset   = false;
      int8_t   local_time_offset_15min = 0;
      uint64_t last_received_at        = 0;
    };
    void set_sib16_info(Sib16Info info) {
      std::lock_guard<std::mutex> lock(_sib16_mutex);
      _sib16         = std::move(info);
      _sib16_present = true;
    }
    bool sib16_present() const {
      std::lock_guard<std::mutex> lock(_sib16_mutex);
      return _sib16_present;
    }
    Sib16Info sib16_info() const {
      std::lock_guard<std::mutex> lock(_sib16_mutex);
      return _sib16;
    }

    /**
     * Any SIB type this receiver saw in a SystemInformation-MBMS message but
     * doesn't decode (everything besides SIB1-MBMS/10/11/12/13/15/16 - see
     * Rrc::write_pdu_bcch_dlsch's default: case). This receiver is an
     * MBMS-dedicated-cell client, not a full LTE UE, so most of TS 36.331's
     * SIB catalogue (reselection, inter-RAT, EAB, SC-PTM, sidelink, V2X, ...)
     * is expected to never appear here - this is an audit trail confirming
     * that expectation rather than scaffolding for future decoders. ETWS
     * (SIB10/11) is legitimately in-scope per SIB-Type-MBMS-r14's own enum
     * and is now decoded above, not just audited.
     */
    struct UnhandledSibInfo {
      uint32_t count            = 0;
      uint64_t last_received_at = 0; // ms since epoch
    };
    void note_unhandled_sib(uint8_t sib_type, uint64_t now_ms) {
      std::lock_guard<std::mutex> lock(_unhandled_sibs_mutex);
      auto& entry = _unhandled_sibs[sib_type];
      entry.count++;
      entry.last_received_at = now_ms;
    }
    std::map<uint8_t, UnhandledSibInfo> unhandled_sibs() const {
      std::lock_guard<std::mutex> lock(_unhandled_sibs_mutex);
      return _unhandled_sibs;
    }

    /**************** Getters and setters for phy params of _ue_sync **************/

    /**
     * Set the factor for the PSS loopback. When the CFO is estimated, _ue_sync takes the previous value and ads it to the current estimated CFO, this parameters is the weight of the previous value, from 0 (nothing) to 1 (all). 
     */
    void inline set_ue_sync_cfo_loop_bw_pss(float bw) { _ue_sync.cfo_loop_bw_pss = bw; }; 
    float inline get_ue_sync_cfo_loop_bw_pss() { return _ue_sync.cfo_loop_bw_pss; }; 

    /**
     * Enables or disables the CFO estimation from the PSS in the finding stage.
     */
    void inline set_ue_sync_find_cfo_pss_enable(bool enable) { srsran_sync_set_cfo_pss_enable(&_ue_sync.sfind, enable); }; 
    bool inline get_ue_sync_find_cfo_pss_enable() { return _ue_sync.sfind.cfo_pss_enable; }; 

    /**
     * Enables or disables the CFO estimation from the PSS in the tracking stage.
     */
    void inline set_ue_sync_track_cfo_pss_enable(bool enable) { srsran_sync_set_cfo_pss_enable(&_ue_sync.strack, enable); }; 
    bool inline get_ue_sync_track_cfo_pss_enable() { return _ue_sync.strack.cfo_pss_enable; }; 

    /**
     * Enables the CFO correction while finding the synchronization from previous estimations.
     */
    void inline set_ue_sync_find_cfo_correct_enable(bool enable) {
      _ue_sync.cfo_correct_enable_find = enable;
    } 
    bool inline get_ue_sync_find_cfo_correct_enable() { return _ue_sync.cfo_correct_enable_find; } 

    /**
     * Enables the CFO correction while tracking the synchronization from previous estimations.
     */
    void inline set_ue_sync_track_cfo_correct_enable(bool enable) { _ue_sync.cfo_correct_enable_track = enable; } 
    bool inline get_ue_sync_track_cfo_correct_enable() { return _ue_sync.cfo_correct_enable_track; } 

    /**
     * Sets the ema alpha value used for the tracking of the CFO in the trackin sync object, both in CP and PSS CFO estimation
     */
    void inline set_ue_sync_track_cfo_ema(float ema) { srsran_ue_sync_set_cfo_ema(&_ue_sync, ema); }; 
    float inline get_ue_sync_track_cfo_ema() { return _ue_sync.strack.cfo_ema_alpha; }; 

    /**
     * Sets the ema alpha value used for the tracking of the CFO in the trackin sync object, both in CP and PSS CFO estimation.
     */
    void inline set_ue_sync_find_cfo_ema(float ema) { srsran_sync_set_cfo_ema_alpha(&_ue_sync.sfind, ema); }; 
    float inline get_ue_sync_find_cfo_ema() { return _ue_sync.sfind.cfo_ema_alpha; }; 

    /**
     * Sets the ema alpha value used for the tracking of the SFO in ue_sync in the function track_peak_ok.
     */
    void inline set_ue_sync_track_sfo_ema(float ema) { srsran_ue_sync_set_sfo_ema(&_ue_sync, ema); }; 
    float inline get_ue_sync_track_sfo_ema() { return _ue_sync.sfo_ema; }; 

    /**
     * Sets the weight factor alpha for the exponential moving average of the PSS correlation output  
     */
    void inline set_ue_sync_pss_cfo_ema_find(float ema) { srsran_sync_set_em_alpha(&_ue_sync.sfind, ema); }; 
    float inline get_ue_sync_pss_cfo_ema_find() { return _ue_sync.sfind.pss.ema_alpha; }; 

    /**
     * Sets the weight factor alpha for the exponential moving average of the PSS correlation output    
     */
    void inline set_ue_sync_pss_cfo_ema_track(float ema) { srsran_sync_set_em_alpha(&_ue_sync.strack, ema); }; 
    float inline get_ue_sync_pss_cfo_ema_track() { return _ue_sync.strack.pss.ema_alpha; }; 

    /**
     * Sets the threshold of the peak found while tracking for synchronization.     
     */
    void inline set_ue_sync_threshold_track(float threshold) { srsran_sync_set_threshold(&_ue_sync.strack, threshold); }; 
    float inline get_ue_sync_threshold_track() { return _ue_sync.strack.threshold; }; 

    /**
     * Sets the threshold of the peak found while searching for synchronization.      
     */
    void inline set_ue_sync_threshold_find(float threshold) { srsran_sync_set_threshold(&_ue_sync.sfind, threshold); }; 
    float inline get_ue_sync_threshold_find() { return _ue_sync.sfind.threshold; }; 

    typedef struct {
      std::string tmgi;
      std::string dest;
      int lcid;
    } mtch_info_t;
    typedef struct {
      int mcs;
      std::vector< mtch_info_t > mtchs;
    } mch_info_t;

    const std::vector< mch_info_t>& mch_info() { return _mch_info;  }

    void set_dest_for_lcid(uint32_t mch_idx, int lcid, std::string dest) { _dests[mch_idx][lcid] = dest; }

    enum class SubcarrierSpacing {
      df_15kHz,
      df_7kHz5,
      df_2kHz5,
      df_1kHz25,
      df_370Hz,       /* 0.37 kHz, RS pattern not yet determined */
      df_370Hz_sl4,  /* 0.37 kHz + RS type 1 (timeSeparation=sl4) */
      df_370Hz_sl2   /* 0.37 kHz + RS type 2 (timeSeparation=sl2) */
    };

    SubcarrierSpacing mbsfn_subcarrier_spacing() {
      if (_cell.mbms_dedicated) {
        const auto& info = _sib13.mbsfn_area_info_list[0];
        if (info.subcarrier_spacing == srsran::mbsfn_area_info_t::subcarrier_spacing_t::khz_0dot37) {
          if (info.time_separation == srsran::mbsfn_area_info_t::time_separation_t::sl2) return SubcarrierSpacing::df_370Hz_sl2;
          return SubcarrierSpacing::df_370Hz_sl4;  /* SL4 is default when absent (TS 36.211 §4.1) */
        }
        switch (info.subcarrier_spacing) {
          case srsran::mbsfn_area_info_t::subcarrier_spacing_t::khz_1dot25: return SubcarrierSpacing::df_1kHz25;
          case srsran::mbsfn_area_info_t::subcarrier_spacing_t::khz_2dot5:  return SubcarrierSpacing::df_2kHz5;
          case srsran::mbsfn_area_info_t::subcarrier_spacing_t::khz_7dot5:  return SubcarrierSpacing::df_7kHz5;
          default: return SubcarrierSpacing::df_15kHz;
        }
      } else {
        return SubcarrierSpacing::df_15kHz;
      }
    }

    float mbsfn_subcarrier_spacing_khz() {
      if (_cell.mbms_dedicated) {
        switch (_sib13.mbsfn_area_info_list[0].subcarrier_spacing) {
          case srsran::mbsfn_area_info_t::subcarrier_spacing_t::khz_1dot25: return 1.25;
          case srsran::mbsfn_area_info_t::subcarrier_spacing_t::khz_2dot5:  return 2.5;
          case srsran::mbsfn_area_info_t::subcarrier_spacing_t::khz_7dot5:  return 7.5;
          case srsran::mbsfn_area_info_t::subcarrier_spacing_t::khz_0dot37: return 0.37f;
          default: return 15;
        }
      } else {
        return 15;
      }
    }

    srsran::mcch_msg_t& mcch() { return _mcch; }

    /* Companion timestamp for _mcch, unprotected to match _mcch/_cell's own
     * existing no-mutex precedent (single writer thread; REST reads a
     * torn-read-tolerant snapshot). */
    void set_mcch_received_at(uint64_t now_ms) { _mcch_last_received_at = now_ms; }
    uint64_t mcch_last_received_at() { return _mcch_last_received_at; }

    int _mcs = 0;
    get_samples_t _sample_cb;

 private:
    const libconfig::Config& _cfg;
    srsran_ue_sync_t _ue_sync = {};
    srsran_ue_cellsearch_t _cell_search = {};
    srsran_ue_mib_sync_t  _mib_sync = {};
    srsran_ue_mib_t  _mib = {};
    srsran_cell_t _cell = {};

    std::atomic<bool> _decode_mcch{false};

    /* pmch-TimeInterleavingN/M-LastMTCH-r19 state, indexed by pmch_idx (sized to
     * match mcch_msg_t::pmch_info_list's own capacity -- see set_last_mtch_start()). */
    std::array<uint32_t, 15> _last_mtch_start = {};
    mutable std::mutex       _last_mtch_start_mutex;
    uint32_t get_last_mtch_start(uint8_t pmch_idx) const {
      if (pmch_idx >= _last_mtch_start.size()) {
        return 0;
      }
      std::lock_guard<std::mutex> lock(_last_mtch_start_mutex);
      return _last_mtch_start[pmch_idx];
    }

    cf_t* _mib_buffer[SRSRAN_MAX_CHANNELS] = {};
    uint32_t _buffer_max_samples = 0;
    uint32_t _tti = 0;
    uint32_t _mib_decode_count = 0;
    uint64_t _last_mib_decoded_at = 0;

    uint8_t  _mcch_table[10] = {};
    bool _mcch_configured = false;
    srsran::sib13_t _sib13 = {};
    srsran::mcch_msg_t _mcch = {};
    uint64_t _mcch_last_received_at = 0;

    bool _mch_configured = false;

    uint8_t _cs_nof_prb;

    std::vector< mch_info_t > _mch_info;

    std::map< uint32_t, std::map< int, std::string >> _dests;

    int8_t _override_nof_prb;
    uint8_t _rx_channels;
    bool _search_extended_cp = true;

    std::atomic<uint64_t> _rom_redirect{0}; // upper 32 bits = EARFCN, lower 32 bits = nof_prb

    std::vector<TvConfigPlmn> _tv_config;
    mutable std::mutex        _tv_config_mutex;

    static constexpr size_t kMaxPwsAlertHistory = 50;
    std::vector<PwsAlert> _pws_alerts;
    mutable std::mutex    _pws_alerts_mutex;
    std::vector<EtwsPrimaryAlert>   _etws_primary_alerts;
    mutable std::mutex              _etws_primary_alerts_mutex;
    std::vector<EtwsSecondaryAlert> _etws_secondary_alerts;
    mutable std::mutex              _etws_secondary_alerts_mutex;

    mutable std::mutex _sib13_mutex; /* guards _sib13, _sib13_rom_info, _sib13_last_received_at */
    std::vector<Sib13RomInfo> _sib13_rom_info;
    uint64_t _sib13_last_received_at = 0;

    Sib1Info  _sib1;
    bool      _sib1_present = false;
    mutable std::mutex _sib1_mutex;

    Sib15Info _sib15;
    bool      _sib15_present = false;
    mutable std::mutex _sib15_mutex;

    Sib16Info _sib16;
    bool      _sib16_present = false;
    mutable std::mutex _sib16_mutex;

    std::map<uint8_t, UnhandledSibInfo> _unhandled_sibs;
    mutable std::mutex                  _unhandled_sibs_mutex;
};
