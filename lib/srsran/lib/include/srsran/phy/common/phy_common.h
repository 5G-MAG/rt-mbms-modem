/**
 * Copyright 2013-2021 Software Radio Systems Limited
 *
 * This file is part of srsRAN.
 *
 * srsRAN is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as
 * published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 *
 * srsRAN is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * A copy of the GNU Affero General Public License can be found in
 * the LICENSE file in the top-level directory of this distribution
 * and at http://www.gnu.org/licenses/.
 *
 */

/**********************************************************************************************
 *  File:         phy_common.h
 *
 *  Description:  Common parameters and lookup functions for PHY
 *
 *  Reference:    3GPP TS 36.211 version 10.0.0 Release 10
 *********************************************************************************************/

#ifndef SRSRAN_PHY_COMMON_H
#define SRSRAN_PHY_COMMON_H

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

#include "srsran/config.h"

#define SRSRAN_NOF_SF_X_FRAME 10
#define SRSRAN_NOF_SLOTS_PER_SF 2
#define SRSRAN_NSLOTS_X_FRAME (SRSRAN_NOF_SLOTS_PER_SF * SRSRAN_NOF_SF_X_FRAME)

#define SRSRAN_NSOFT_BITS 250368 // Soft buffer size for Category 1 UE

#define SRSRAN_PC_MAX 23 // Maximum TX power for Category 1 UE (in dBm)

#define SRSRAN_NOF_NID_1 (168)
#define SRSRAN_NOF_NID_2 (3)
#define SRSRAN_NUM_PCI (SRSRAN_NOF_NID_1 * SRSRAN_NOF_NID_2)

#define SRSRAN_MAX_CARRIERS 5 // Maximum number of supported simultaneous carriers
#define SRSRAN_MAX_PORTS 4
#define SRSRAN_MAX_CHANNELS (SRSRAN_MAX_CARRIERS * SRSRAN_MAX_PORTS)
#define SRSRAN_MAX_LAYERS 4
#define SRSRAN_MAX_CODEWORDS 2
#define SRSRAN_MAX_TB SRSRAN_MAX_CODEWORDS
#define SRSRAN_MAX_QM 8

#define SRSRAN_MAX_CODEBLOCKS 32

#define SRSRAN_MAX_CODEBOOKS 4

#define SRSRAN_NOF_CFI 3
#define SRSRAN_CFI_ISVALID(x) ((x >= 1 && x <= 3))
#define SRSRAN_CFI_IDX(x) ((x - 1) % SRSRAN_NOF_CFI)

#define SRSRAN_LTE_CRC24A 0x1864CFB
#define SRSRAN_LTE_CRC24B 0X1800063
#define SRSRAN_LTE_CRC24C 0X1B2B117
#define SRSRAN_LTE_CRC16 0x11021
#define SRSRAN_LTE_CRC11 0xE21
#define SRSRAN_LTE_CRC8 0x19B
#define SRSRAN_LTE_CRC6 0x61

#define SRSRAN_MAX_MBSFN_AREA_IDS 256
#define SRSRAN_PMCH_RV 0

typedef enum { SRSRAN_CP_NORM = 0, SRSRAN_CP_EXT } srsran_cp_t;
typedef enum { SRSRAN_SF_NORM = 0, SRSRAN_SF_MBSFN } srsran_sf_t;
typedef enum {
  SRSRAN_SCS_15KHZ  = 0,
  SRSRAN_SCS_7KHZ5,
  SRSRAN_SCS_1KHZ25,
  SRSRAN_SCS_2KHZ5,
  SRSRAN_SCS_370HZ,       /* 0.37 kHz OFDM baseline; CR 0548: Nu=82944 at Fs=30.72 MHz (75 PRB) */
  SRSRAN_SCS_370HZ_SL4,  /* 0.37 kHz + RS type 1: k=12m'+stagger, stagger=3*(ns mod 4) */
  SRSRAN_SCS_370HZ_SL2   /* 0.37 kHz + RS type 2: k=6m'+stagger,  stagger=3*(ns mod 2) */
} srsran_scs_t;

/* True for any of the three 0.37 kHz SCS variants */
#define SRSRAN_SCS_IS_370HZ(scs) ((scs) == SRSRAN_SCS_370HZ || (scs) == SRSRAN_SCS_370HZ_SL4 || (scs) == SRSRAN_SCS_370HZ_SL2)

#define SRSRAN_INVALID_RNTI 0x0 // TS 36.321 - Table 7.1-1 RNTI 0x0 isn't a valid DL RNTI
#define SRSRAN_CRNTI_START 0x000B
#define SRSRAN_CRNTI_END 0xFFF3
#define SRSRAN_RARNTI_START 0x0001
#define SRSRAN_RARNTI_END 0x000A
#define SRSRAN_SIRNTI 0xFFFF
#define SRSRAN_SIRNTI_MBMS_DEDICATED 0xFFF9
#define SRSRAN_PRNTI 0xFFFE
#define SRSRAN_MRNTI 0xFFFD

#define SRSRAN_RNTI_ISRAR(rnti) (rnti >= SRSRAN_RARNTI_START && rnti <= SRSRAN_RARNTI_END)
#define SRSRAN_RNTI_ISUSER(rnti) (rnti >= SRSRAN_CRNTI_START && rnti <= SRSRAN_CRNTI_END)
#define SRSRAN_RNTI_ISSI(rnti) (rnti == SRSRAN_SIRNTI)
#define SRSRAN_RNTI_ISPA(rnti) (rnti == SRSRAN_PRNTI)
#define SRSRAN_RNTI_ISMBSFN(rnti) (rnti == SRSRAN_MRNTI)
#define SRSRAN_RNTI_ISSIRAPA(rnti) (SRSRAN_RNTI_ISSI(rnti) || SRSRAN_RNTI_ISRAR(rnti) || SRSRAN_RNTI_ISPA(rnti))

#define SRSRAN_CELL_ID_UNKNOWN 1000

#define SRSRAN_MAX_NSYMB 7

#define SRSRAN_MAX_PRB 110
#define SRSRAN_NRE 12
#define SRSRAN_NRE_SCS_7KHZ5 24
#define SRSRAN_NRE_SCS_1KHZ25 144
#define SRSRAN_NRE_SCS_2KHZ5 72
#define SRSRAN_NRE_SCS_370HZ 486
#define SRSRAN_NRE_SCS(scs) (scs == SRSRAN_SCS_15KHZ ? SRSRAN_NRE : \
                              (scs == SRSRAN_SCS_7KHZ5 ? SRSRAN_NRE_SCS_7KHZ5 : \
                              (scs == SRSRAN_SCS_2KHZ5 ? SRSRAN_NRE_SCS_2KHZ5 : \
                              (SRSRAN_SCS_IS_370HZ(scs) ? SRSRAN_NRE_SCS_370HZ : SRSRAN_NRE_SCS_1KHZ25))))


#define SRSRAN_SYMBOL_SZ_MAX 2048

#define SRSRAN_CP_NORM_NSYMB 7
#define SRSRAN_CP_NORM_SF_NSYMB (2 * SRSRAN_CP_NORM_NSYMB)
#define SRSRAN_CP_NORM_0_LEN 160
#define SRSRAN_CP_NORM_LEN 144

#define SRSRAN_CP_EXT_NSYMB 6
#define SRSRAN_CP_EXT_SF_NSYMB (2 * SRSRAN_CP_EXT_NSYMB)
#define SRSRAN_CP_EXT_LEN 512
#define SRSRAN_CP_EXT_7_5_LEN 1024

#define SRSRAN_CP_SCS_7KHZ5_NSYMB  3
#define SRSRAN_CP_SCS_1KHZ25_NSYMB 1
#define SRSRAN_CP_SCS_2KHZ5_NSYMB  2
/* TS 36.211 Table 6.12-1: CP lengths in Ts units */
#define SRSRAN_CP_MBSFN_LEN(scs) (scs == SRSRAN_SCS_1KHZ25 ? 6144 : \
                                  (scs == SRSRAN_SCS_2KHZ5  ? 3072 : \
                                  (SRSRAN_SCS_IS_370HZ(scs)  ? 9216 : \
                                  (scs == SRSRAN_SCS_7KHZ5  ? 1024 : SRSRAN_CP_EXT_LEN))))

/* Number of logical slot iterations for PMCH RE mapping per 1 ms LTE subframe.
 * 0.37 kHz: one 3 ms symbol spans 3 subframes; process 1 per subframe call.
 * 1.25 kHz: 1 symbol per subframe → 1 slot.
 * 2.5 kHz: SRSRAN_MBSFN_NOF_SYMBOLS(2kHz5)=2 is the TOTAL per-subframe count;
 *   must use nof_slots=1 to avoid the loop visiting symbol indices 2/3 (buffer overflow).
 * 7.5 kHz: 3 symbols per 0.5 ms slot, 2 slots → nof_slots=2. */
#define SRSRAN_MBSFN_NOF_SLOTS(scs) ((scs == SRSRAN_SCS_1KHZ25 || \
                                      SRSRAN_SCS_IS_370HZ(scs)  || \
                                      scs == SRSRAN_SCS_2KHZ5) ? 1 : 2)
/* Number of OFDM symbols per logical slot (see MBSFN_NOF_SLOTS). For 2.5 kHz with
 * nof_slots=1 this is the total per-subframe symbol count (2). */
#define SRSRAN_MBSFN_NOF_SYMBOLS(scs) (scs == SRSRAN_SCS_1KHZ25 ? SRSRAN_CP_SCS_1KHZ25_NSYMB : \
                                       (scs == SRSRAN_SCS_2KHZ5  ? SRSRAN_CP_SCS_2KHZ5_NSYMB  : \
                                       (scs == SRSRAN_SCS_7KHZ5  ? SRSRAN_CP_SCS_7KHZ5_NSYMB  : \
                                       (SRSRAN_SCS_IS_370HZ(scs) ? 1 : SRSRAN_CP_EXT_NSYMB))))


#define SRSRAN_CP_ISNORM(cp) (cp == SRSRAN_CP_NORM)
#define SRSRAN_CP_ISEXT(cp) (cp == SRSRAN_CP_EXT)
#define SRSRAN_CP_NSYMB(cp) (SRSRAN_CP_ISNORM(cp) ? SRSRAN_CP_NORM_NSYMB : SRSRAN_CP_EXT_NSYMB)

#define SRSRAN_CP_LEN(symbol_sz, c) ((int)ceilf((((float)(c) * (symbol_sz)) / 2048.0f)))
#define SRSRAN_CP_LEN_NORM(symbol, symbol_sz)                                                                          \
  (((symbol) == 0) ? SRSRAN_CP_LEN((symbol_sz), SRSRAN_CP_NORM_0_LEN) : SRSRAN_CP_LEN((symbol_sz), SRSRAN_CP_NORM_LEN))
#define SRSRAN_CP_LEN_EXT(symbol_sz) (SRSRAN_CP_LEN((symbol_sz), SRSRAN_CP_EXT_LEN))
#define SRSRAN_CP_LEN_MBSFN_SCS(symbol_sz, scs) (SRSRAN_CP_LEN((symbol_sz), SRSRAN_CP_MBSFN_LEN(scs)))


#define SRSRAN_CP_SZ(symbol_sz, cp)                                                                                    \
  (SRSRAN_CP_LEN(symbol_sz, (SRSRAN_CP_ISNORM(cp) ? SRSRAN_CP_NORM_LEN : SRSRAN_CP_EXT_LEN)))
#define SRSRAN_SYMBOL_SZ(symbol_sz, cp) (symbol_sz + SRSRAN_CP_SZ(symbol_sz, cp))
#define SRSRAN_SLOT_LEN(symbol_sz) (symbol_sz * 15 / 2)
#define SRSRAN_SF_LEN(symbol_sz) (symbol_sz * 15)
#define SRSRAN_SF_LEN_MAX (SRSRAN_SF_LEN(SRSRAN_SYMBOL_SZ_MAX))

#define SRSRAN_SLOT_LEN_PRB(nof_prb) (SRSRAN_SLOT_LEN(srsran_symbol_sz(nof_prb)))
#define SRSRAN_SF_LEN_PRB(nof_prb) ((uint32_t)SRSRAN_SF_LEN(srsran_symbol_sz(nof_prb)))

#define SRSRAN_SLOT_LEN_RE(nof_prb, cp) (nof_prb * SRSRAN_NRE * SRSRAN_CP_NSYMB(cp))
#define SRSRAN_SF_LEN_RE(nof_prb, cp) (2 * SRSRAN_SLOT_LEN_RE(nof_prb, cp))
#define SRSRAN_NOF_RE(cell) (2 * SRSRAN_SLOT_LEN_RE(cell.nof_prb, cell.cp))

#define SRSRAN_TA_OFFSET (10e-6)

#define SRSRAN_LTE_TS (1.0f / (15000.0f * 2048.0f))

#define SRSRAN_SLOT_IDX_CPNORM(symbol_idx, symbol_sz)                                                                  \
  (symbol_idx == 0 ? 0                                                                                                 \
                   : (symbol_sz + SRSRAN_CP_LEN(symbol_sz, SRSRAN_CP_NORM_0_LEN) +                                     \
                      (symbol_idx - 1) * (symbol_sz + SRSRAN_CP_LEN(symbol_sz, SRSRAN_CP_NORM_LEN))))
#define SRSRAN_SLOT_IDX_CPEXT(idx, symbol_sz) (idx * (symbol_sz + SRSRAN_CP(symbol_sz, SRSRAN_CP_EXT_LEN)))

#define SRSRAN_RE_IDX(nof_prb, symbol_idx, sample_idx) ((symbol_idx) * (nof_prb) * (SRSRAN_NRE) + sample_idx)
#define SRSRAN_RE_IDX_MBSFN(nof_prb, symbol_idx, sample_idx, scs) ((symbol_idx) * (nof_prb) * (SRSRAN_NRE_SCS(scs)) + sample_idx)

#define SRSRAN_RS_VSHIFT(cell_id) (cell_id % 6)

#define SRSRAN_GUARD_RE(nof_prb) ((srsran_symbol_sz(nof_prb) - nof_prb * SRSRAN_NRE) / 2)

#define SRSRAN_SYMBOL_HAS_REF(l, cp, nof_ports) ((l == 1 && nof_ports == 4) || l == 0 || l == SRSRAN_CP_NSYMB(cp) - 3)

#define SRSRAN_NOF_CTRL_SYMBOLS(cell, cfi) (cfi + (cell.nof_prb < 10 ? 1 : 0))

#define SRSRAN_SYMBOL_HAS_REF_MBSFN(l, s) ((l == 2 && s == 0) || (l == 0 && s == 1) || (l == 4 && s == 1))
#define SRSRAN_SYMBOL_HAS_REF_MBSFN_7KHZ5(l, s) ((l == 1 && s == 0) || (l == 0 && s == 1) || (l == 2 && s == 1))
#define SRSRAN_SYMBOL_HAS_REF_MBSFN_1KHZ25(l, s) (true)
#define SRSRAN_SYMBOL_HAS_REF_MBSFN_2KHZ5(l, s) (true)
/* TS 36.211 §6.10.2.2.4: SL4 type-1 RS occupy only l=0 per slot (pilot spacing 12 does not fit
 * evenly in 6 symbols). SL2 type-2 and base 0.37 kHz have RS in every OFDM symbol. Reconciled
 * from rt-mbms-tx's phy_common.h, which had this SL2/SL4 split and this copy didn't - this
 * generic "always true" version silently over-counted SL4 RS-bearing symbols wherever
 * SRSRAN_SYMBOL_HAS_REF_MBSFN_SCS is used (ra_dl.c, pmch.c), corrupting RE counts for SL4. */
#define SRSRAN_SYMBOL_HAS_REF_MBSFN_370HZ(l, s) (true)
#define SRSRAN_SYMBOL_HAS_REF_MBSFN_370HZ_SL2(l, s) (true)
#define SRSRAN_SYMBOL_HAS_REF_MBSFN_370HZ_SL4(l, s) ((l) == 0)
#define SRSRAN_SYMBOL_HAS_REF_MBSFN_SCS(l, s, scs) (scs == SRSRAN_SCS_15KHZ ? SRSRAN_SYMBOL_HAS_REF_MBSFN(l, s) : \
    (scs == SRSRAN_SCS_7KHZ5 ? SRSRAN_SYMBOL_HAS_REF_MBSFN_7KHZ5(l, s) : \
    (scs == SRSRAN_SCS_2KHZ5 ? SRSRAN_SYMBOL_HAS_REF_MBSFN_2KHZ5(l, s) : \
    (scs == SRSRAN_SCS_370HZ_SL4 ? SRSRAN_SYMBOL_HAS_REF_MBSFN_370HZ_SL4(l, s) : \
    (scs == SRSRAN_SCS_370HZ_SL2 ? SRSRAN_SYMBOL_HAS_REF_MBSFN_370HZ_SL2(l, s) : \
    (SRSRAN_SCS_IS_370HZ(scs) ? SRSRAN_SYMBOL_HAS_REF_MBSFN_370HZ(l, s) : SRSRAN_SYMBOL_HAS_REF_MBSFN_1KHZ25(l, s)))))))

#define SRSRAN_SYMBOL_REF_OFFSET_MBSFN(l, s) ((l == 2 && s == 0) || (l == 0 && s == 1) || (l == 4 && s == 1))

#define SRSRAN_NON_MBSFN_REGION_GUARD_LENGTH(non_mbsfn_region, symbol_sz)                                              \
  ((non_mbsfn_region == 1)                                                                                             \
       ? (SRSRAN_CP_LEN_EXT(symbol_sz) - SRSRAN_CP_LEN_NORM(0, symbol_sz))                                             \
       : (2 * SRSRAN_CP_LEN_EXT(symbol_sz) - SRSRAN_CP_LEN_NORM(0, symbol_sz) - SRSRAN_CP_LEN_NORM(1, symbol_sz)))

#define SRSRAN_FDD_NOF_HARQ (FDD_HARQ_DELAY_DL_MS + FDD_HARQ_DELAY_UL_MS)
#define SRSRAN_MAX_HARQ_PROC 15

#define SRSRAN_NOF_LTE_BANDS 64

#define SRSRAN_DEFAULT_MAX_FRAMES_PBCH 500
#define SRSRAN_DEFAULT_MAX_FRAMES_PSS 10
#define SRSRAN_DEFAULT_NOF_VALID_PSS_FRAMES 10

#define ZERO_OBJECT(x) memset(&(x), 0x0, sizeof((x)))

typedef enum SRSRAN_API { SRSRAN_PHICH_NORM = 0, SRSRAN_PHICH_EXT } srsran_phich_length_t;

typedef enum SRSRAN_API {
  SRSRAN_PHICH_R_1_6 = 0,
  SRSRAN_PHICH_R_1_2,
  SRSRAN_PHICH_R_1,
  SRSRAN_PHICH_R_2
} srsran_phich_r_t;

/// LTE duplex modes.
typedef enum SRSRAN_API {
  /// FDD uses frame structure type 1.
  SRSRAN_FDD = 0,
  /// TDD uses frame structure type 2.
  SRSRAN_TDD = 1
} srsran_frame_type_t;

/// Maximum number of TDD special subframe configurations.
#define SRSRAN_MAX_TDD_SS_CONFIGS (10u)

/// Maximum number of TDD uplink-downlink subframe configurations.
#define SRSRAN_MAX_TDD_SF_CONFIGS (7u)

/// Configuration fields for operating in TDD mode.
typedef struct SRSRAN_API {
  /// Uplink-downlink configuration, valid range is [0,SRSRAN_MAX_TDD_SF_CONFIGS[.
  /// TS 36.211 v8.9.0 Table 4.2-2.
  uint32_t sf_config;
  /// Special subframe symbol length configuration, valid range is [0,SRSRAN_MAX_TDD_SS_CONFIGS[.
  /// TS 36.211 v13.13.0 Table 4.2-1.
  uint32_t ss_config;
  /// Set to true when the fields have been configured, otherwise false.
  bool configured;
} srsran_tdd_config_t;

/// TDD uplink-downlink subframe types.
typedef enum SRSRAN_API {
  /// Subframe is reserved for downlink transmissions.
  SRSRAN_TDD_SF_D = 0,
  /// Subframe is reserved for uplink transmissions.
  SRSRAN_TDD_SF_U = 1,
  /// Special subframe.
  SRSRAN_TDD_SF_S = 2,
} srsran_tdd_sf_t;

typedef struct {
  uint8_t mbsfn_area_id;
  uint8_t non_mbsfn_region_length;
  uint8_t mbsfn_mcs;
  bool    enable;
  bool    is_mcch;
  srsran_scs_t subcarrier_spacing;
  /* Rel-19 LTE_terr_bcast_Ph2 */
  bool     use_mcs_table2;      /* use TS 36.213 Table 11.1-2 (256QAM) instead of Table 11.1-1 */
  uint8_t  time_interleaving_n; /* NTimePMCH: TB spans this many subframes; 0/1 = disabled */
  uint8_t  time_interleaving_m; /* MTimePMCH: scheduling period length in subframes; 4/8/16/32 */
  uint32_t mch_subframe_idx;    /* 0-based index of this MCH subframe within the scheduling period */
  bool     cyclic_shift;        /* pmch-CyclicShiftAlpha-r19 present (TS 36.211 §6.5.1) */
  uint8_t  cyclic_shift_alpha;  /* α: 1, 2, or 3 */
  bool     freq_interleaving;   /* pmch-FreqInterleaving-r19 (TS 36.211 §6.5.2) */
  uint8_t  pmch_idx;            /* index into mcch.pmch_info_list[] for this subframe */
  /* PMCH-SoftBufferSizeParameters-r19 (TS 36.212 §5.1.4.1.2 N_cb capping) --
   * see the matching fields' doc comment in pmch.h's srsran_pmch_cfg_t. */
  uint8_t  n_soft_ref_category;
  uint8_t  scaling_factor_beta_num;
  uint8_t  scaling_factor_beta_den;
} srsran_mbsfn_cfg_t;

// Common cell constant properties that require object reconfiguration
typedef struct SRSRAN_API {
  uint32_t              nof_prb;
  uint32_t              nof_ports;
  uint32_t              id;
  srsran_cp_t           cp;
  srsran_phich_length_t phich_length;
  srsran_phich_r_t      phich_resources;
  srsran_frame_type_t   frame_type;
  bool                  mbms_dedicated;
  uint8_t               additional_non_mbms_frames;
  uint8_t               mbsfn_prb;
  /* Rel-19 CAS muting (TS 36.211 §6.6.4, §6.11.1.2, §6.11.2.2): PSS/SSS/PBCH only in
   * the first 4·k_cas frames of each 16·n_cas-frame period starting at nf mod 16·n_cas = 0 */
  bool                  cas_muting;
  uint8_t               k_cas;   /* KCAS: 4..63 */
  uint8_t               n_cas;   /* NCAS: 2, 4, 8, or 16 */
  /* Rel-16: semiStaticCFI-MBMS-r16 (TS 36.331/36.213 §9.1.3, MIB-MBMS bits [11-12]):
   * INTEGER(0..3). 0 = derive CFI from PCFICH; 1/2/3 directly ARE the CFI value. */
  uint8_t               semi_static_cfi;   /* 0..3; 0 also doubles as "not yet decoded" on RX */
} srsran_cell_t;

/* Rel-16 CAS (MBMS) detection. On a Rel-16 MBMS-dedicated CAS the PBCH is repeated
 * (TS 36.211 §6.6.4.1); the repetition does not fill its whole OFDM symbol, so the
 * unused REs carry PDSCH. The SI-PDSCH RE mapping (ra_dl.c/pdsch.c) must therefore
 * account for those extra PDSCH REs, or the rate-matched LLRs desync and the SI
 * turbo block fails CRC at every cfi/RV (clean symbols, wrong bits).
 * Proxy detector: a cell signalling semiStaticCFI-MBMS-r16 (semi_static_cfi != 0)
 * is a Rel-16 CAS. This cleanly separates the Rel-16 captures from legacy ones and
 * cannot affect a legacy cell (semi_static_cfi == 0). A fully general detector would
 * correlation-check the PBCH repetition itself (cf. jsroldan srsRAN
 * srsran_rel16_pbch_mrc) — upgrade here if a Rel-16 CAS with semiStaticCFI disabled
 * but PBCH repetition present is ever encountered. */
static inline bool srsran_cell_is_mbms_r16(const srsran_cell_t* cell)
{
  return cell->mbms_dedicated && cell->nof_prb > 6 && cell->semi_static_cfi != 0;
}

// Common downlink properties that may change every subframe
typedef struct SRSRAN_API {
  srsran_tdd_config_t tdd_config;
  uint32_t            tti;
  uint32_t            cfi;
  srsran_sf_t         sf_type;
  uint32_t            non_mbsfn_region;
  srsran_scs_t        subcarrier_spacing;
} srsran_dl_sf_cfg_t;

typedef struct SRSRAN_API {
  srsran_tdd_config_t tdd_config;
  uint32_t            tti;
  bool                shortened;
} srsran_ul_sf_cfg_t;

typedef enum SRSRAN_API {
  SRSRAN_TM1 = 0,
  SRSRAN_TM2,
  SRSRAN_TM3,
  SRSRAN_TM4,
  SRSRAN_TM5,
  SRSRAN_TM6,
  SRSRAN_TM7,
  SRSRAN_TM8,
  SRSRAN_TMINV // Invalid Transmission Mode
} srsran_tm_t;

typedef enum SRSRAN_API {
  SRSRAN_TXSCHEME_PORT0,
  SRSRAN_TXSCHEME_DIVERSITY,
  SRSRAN_TXSCHEME_SPATIALMUX,
  SRSRAN_TXSCHEME_CDD
} srsran_tx_scheme_t;

typedef enum SRSRAN_API { SRSRAN_MIMO_DECODER_ZF, SRSRAN_MIMO_DECODER_MMSE } srsran_mimo_decoder_t;

/*!
 * \brief Types of modulations and associated modulation order.
 */
typedef enum SRSRAN_API {
  SRSRAN_MOD_BPSK = 0, /*!< \brief BPSK. */
  SRSRAN_MOD_QPSK,     /*!< \brief QPSK. */
  SRSRAN_MOD_16QAM,    /*!< \brief QAM16. */
  SRSRAN_MOD_64QAM,    /*!< \brief QAM64. */
  SRSRAN_MOD_256QAM,   /*!< \brief QAM256. */
  SRSRAN_MOD_NITEMS
} srsran_mod_t;

typedef enum {
  SRSRAN_DCI_FORMAT0 = 0,
  SRSRAN_DCI_FORMAT1,
  SRSRAN_DCI_FORMAT1A,
  SRSRAN_DCI_FORMAT1B,
  SRSRAN_DCI_FORMAT1C,
  SRSRAN_DCI_FORMAT1D,
  SRSRAN_DCI_FORMAT2,
  SRSRAN_DCI_FORMAT2A,
  SRSRAN_DCI_FORMAT2B,
  // SRSRAN_DCI_FORMAT3,
  // SRSRAN_DCI_FORMAT3A,
  SRSRAN_DCI_FORMATN0,
  SRSRAN_DCI_FORMATN1,
  SRSRAN_DCI_FORMATN2,
  SRSRAN_DCI_FORMAT_RAR, // Not a real LTE format. Used internally to indicate RAR grant
  SRSRAN_DCI_NOF_FORMATS
} srsran_dci_format_t;

typedef enum {
  SRSRAN_PUCCH_ACK_NACK_FEEDBACK_MODE_NORMAL = 0, /* No cell selection no pucch3 */
  SRSRAN_PUCCH_ACK_NACK_FEEDBACK_MODE_CS,
  SRSRAN_PUCCH_ACK_NACK_FEEDBACK_MODE_PUCCH3,
  SRSRAN_PUCCH_ACK_NACK_FEEDBACK_MODE_ERROR,
} srsran_ack_nack_feedback_mode_t;

typedef struct SRSRAN_API {
  int   id;
  float fd;
} srsran_earfcn_t;

enum band_geographical_area {
  SRSRAN_BAND_GEO_AREA_ALL,
  SRSRAN_BAND_GEO_AREA_NAR,
  SRSRAN_BAND_GEO_AREA_APAC,
  SRSRAN_BAND_GEO_AREA_EMEA,
  SRSRAN_BAND_GEO_AREA_JAPAN,
  SRSRAN_BAND_GEO_AREA_CALA,
  SRSRAN_BAND_GEO_AREA_NA
};

///< NB-IoT specific structs
typedef enum {
  SRSRAN_NBIOT_MODE_INBAND_SAME_PCI = 0,
  SRSRAN_NBIOT_MODE_INBAND_DIFFERENT_PCI,
  SRSRAN_NBIOT_MODE_GUARDBAND,
  SRSRAN_NBIOT_MODE_STANDALONE,
  SRSRAN_NBIOT_MODE_N_ITEMS,
} srsran_nbiot_mode_t;

typedef struct SRSRAN_API {
  srsran_cell_t       base;      // the umbrella or super cell
  uint32_t            nbiot_prb; // the index of the NB-IoT PRB within the cell
  uint32_t            n_id_ncell;
  uint32_t            nof_ports; // The number of antenna ports for NB-IoT
  bool                is_r14;    // Whether the cell is a R14 cell
  srsran_nbiot_mode_t mode;
} srsran_nbiot_cell_t;

#define SRSRAN_NBIOT_MAX_PORTS 2
#define SRSRAN_NBIOT_MAX_CODEWORDS SRSRAN_MAX_CODEWORDS

#define SRSRAN_SF_LEN_PRB_NBIOT (SRSRAN_SF_LEN_PRB(1))

#define SRSRAN_SF_LEN_RE_NBIOT (SRSRAN_SF_LEN_RE(1, SRSRAN_CP_NORM))

#define SRSRAN_NBIOT_FFT_SIZE 128
#define SRSRAN_NBIOT_FREQ_SHIFT_FACTOR ((float)-0.5)
#define SRSRAN_NBIOT_NUM_RX_ANTENNAS 1
#define SRSRAN_NBIOT_MAX_PRB 1

#define SRSRAN_NBIOT_DEFAULT_NUM_PRB_BASECELL 1
#define SRSRAN_NBIOT_DEFAULT_PRB_OFFSET 0

#define SRSRAN_DEFAULT_MAX_FRAMES_NPBCH 500
#define SRSRAN_DEFAULT_MAX_FRAMES_NPSS 20
#define SRSRAN_DEFAULT_NOF_VALID_NPSS_FRAMES 20

#define SRSRAN_NBIOT_NPBCH_NOF_TOTAL_BITS (1600) ///< Number of bits for the entire NPBCH (See 36.211 Sec 10.2.4.1)
#define SRSRAN_NBIOT_NPBCH_NOF_BITS_SF                                                                                 \
  (SRSRAN_NBIOT_NPBCH_NOF_TOTAL_BITS / 8) ///< The NPBCH is transmitted in 8 blocks (See 36.211 Sec 10.2.4.4)

#define SRSRAN_MAX_DL_BITS_CAT_NB1 (680) ///< TS 36.306 v15.4.0 Table 4.1C-1

///< PHY common function declarations

SRSRAN_API bool srsran_cell_isvalid(srsran_cell_t* cell);

SRSRAN_API void srsran_cell_fprint(FILE* stream, srsran_cell_t* cell, uint32_t sfn);

SRSRAN_API bool srsran_cellid_isvalid(uint32_t cell_id);

SRSRAN_API bool srsran_nofprb_isvalid(uint32_t nof_prb);

/**
 * Returns the subframe type for a given subframe number and a TDD configuration.
 * Check TS 36.211 v8.9.0 Table 4.2-2.
 *
 * @param tdd_config TDD configuration.
 * @param sf_idx Subframe number, must be in range [0,SRSRAN_NOF_SF_X_FRAME[.
 * @return Returns the subframe type.
 */
SRSRAN_API srsran_tdd_sf_t srsran_sfidx_tdd_type(srsran_tdd_config_t tdd_config, uint32_t sf_idx);

/**
 * Returns the number of UpPTS symbols in a subframe.
 * Check TS 36.211 v13.13.0 Table 4.2-1.
 *
 * @param tdd_config TDD configuration.
 * @return Returns the number of UpPTS symbols.
 */
SRSRAN_API uint32_t srsran_sfidx_tdd_nof_up(srsran_tdd_config_t tdd_config);

/**
 * Returns the number of GP symbols in a subframe.
 * Check TS 36.211 v13.13.0 Table 4.2-1.
 *
 * @param tdd_config TDD configuration.
 * @return Returns the number of GP symbols.
 */
SRSRAN_API uint32_t srsran_sfidx_tdd_nof_gp(srsran_tdd_config_t tdd_config);

/**
 * Returns the number of DwPTS symbols in a subframe.
 * Check TS 36.211 v13.13.0 Table 4.2-1.
 *
 * @param tdd_config TDD configuration.
 * @return Returns the number of DwPTS symbols.
 */
SRSRAN_API uint32_t srsran_sfidx_tdd_nof_dw(srsran_tdd_config_t tdd_config);

SRSRAN_API uint32_t srsran_tdd_nof_harq(srsran_tdd_config_t tdd_config);

SRSRAN_API uint32_t srsran_sfidx_tdd_nof_dw_slot(srsran_tdd_config_t tdd_config, uint32_t slot, srsran_cp_t cp);

SRSRAN_API bool srsran_sfidx_isvalid(uint32_t sf_idx);

SRSRAN_API bool srsran_portid_isvalid(uint32_t port_id);

SRSRAN_API bool srsran_N_id_2_isvalid(uint32_t N_id_2);

SRSRAN_API bool srsran_N_id_1_isvalid(uint32_t N_id_1);

SRSRAN_API bool srsran_symbol_sz_isvalid(uint32_t symbol_sz);

SRSRAN_API int srsran_symbol_sz(uint32_t nof_prb);

SRSRAN_API int srsran_symbol_sz_scs(uint32_t nof_prb, srsran_scs_t subcarrier_spacing);

SRSRAN_API int srsran_symbol_sz_power2(uint32_t nof_prb);

SRSRAN_API int srsran_nof_prb(uint32_t symbol_sz);

SRSRAN_API uint32_t srsran_max_cce(uint32_t nof_prb);

SRSRAN_API int srsran_sampling_freq_hz(uint32_t nof_prb);

SRSRAN_API int srsran_sampling_freq_hz_scs(uint32_t nof_prb, srsran_scs_t scs);

SRSRAN_API void srsran_use_standard_symbol_size(bool enabled);

SRSRAN_API bool srsran_symbol_size_is_standard();

SRSRAN_API uint32_t srsran_re_x_prb(uint32_t ns, uint32_t symbol, uint32_t nof_ports, uint32_t nof_symbols);

SRSRAN_API uint32_t srsran_voffset(uint32_t symbol_id, uint32_t cell_id, uint32_t nof_ports);

SRSRAN_API int srsran_group_hopping_f_gh(uint32_t f_gh[SRSRAN_NSLOTS_X_FRAME], uint32_t cell_id);

SRSRAN_API uint32_t srsran_N_ta_new_rar(uint32_t ta);

SRSRAN_API uint32_t srsran_N_ta_new(uint32_t N_ta_old, uint32_t ta);

SRSRAN_API float srsran_coderate(uint32_t tbs, uint32_t nof_re);

SRSRAN_API char* srsran_cp_string(srsran_cp_t cp);

SRSRAN_API srsran_mod_t srsran_str2mod(const char* mod_str);

SRSRAN_API char* srsran_mod_string(srsran_mod_t mod);

SRSRAN_API uint32_t srsran_mod_bits_x_symbol(srsran_mod_t mod);

SRSRAN_API uint8_t srsran_band_get_band(uint32_t dl_earfcn);

SRSRAN_API bool srsran_band_is_tdd(uint32_t band);

SRSRAN_API double srsran_band_fd(uint32_t dl_earfcn);

SRSRAN_API double srsran_band_fu(uint32_t ul_earfcn);

SRSRAN_API uint32_t srsran_band_ul_earfcn(uint32_t dl_earfcn);

SRSRAN_API int
srsran_band_get_fd_band(uint32_t band, srsran_earfcn_t* earfcn, int earfcn_start, int earfcn_end, uint32_t max_elems);

SRSRAN_API int srsran_band_get_fd_band_all(uint32_t band, srsran_earfcn_t* earfcn, uint32_t max_nelems);

SRSRAN_API int
srsran_band_get_fd_region(enum band_geographical_area region, srsran_earfcn_t* earfcn, uint32_t max_elems);

SRSRAN_API int srsran_str2mimotype(char* mimo_type_str, srsran_tx_scheme_t* type);

SRSRAN_API char* srsran_mimotype2str(srsran_tx_scheme_t mimo_type);

/* Returns the interval tti1-tti2 mod 10240 */
SRSRAN_API uint32_t srsran_tti_interval(uint32_t tti1, uint32_t tti2);

SRSRAN_API uint32_t srsran_print_check(char* s, size_t max_len, uint32_t cur_len, const char* format, ...);

SRSRAN_API bool  srsran_nbiot_cell_isvalid(srsran_nbiot_cell_t* cell);
SRSRAN_API bool  srsran_nbiot_portid_isvalid(uint32_t port_id);
SRSRAN_API float srsran_band_fu_nbiot(uint32_t ul_earfcn, const float m_ul);

SRSRAN_API char* srsran_nbiot_mode_string(srsran_nbiot_mode_t mode);

/**
 * Returns a constant string pointer with the ACK/NACK feedback mode
 *
 * @param ack_nack_feedback_mode Mode
 * @return Returns constant pointer with the text of the mode if succesful, `error` otherwise
 */
SRSRAN_API const char* srsran_ack_nack_feedback_mode_string(srsran_ack_nack_feedback_mode_t ack_nack_feedback_mode);

/**
 * Returns a constant string pointer with the ACK/NACK feedback mode
 *
 * @param ack_nack_feedback_mode Mode
 * @return Returns constant pointer with the text of the mode if succesful, `error` otherwise
 */
SRSRAN_API srsran_ack_nack_feedback_mode_t srsran_string_ack_nack_feedback_mode(const char* str);

/**
 * Returns the number of bits for Rank Indicador reporting depending on the cell
 *
 * @param cell
 */
SRSRAN_API uint32_t srsran_ri_nof_bits(const srsran_cell_t* cell);

#ifdef __cplusplus
}
#endif

#endif // SRSRAN_PHY_COMMON_H
