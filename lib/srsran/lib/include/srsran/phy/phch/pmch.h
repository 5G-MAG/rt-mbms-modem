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

/******************************************************************************
 *  File:         pmch.h
 *
 *  Description:  Physical multicast channel
 *
 *  Reference:    3GPP TS 36.211 version 10.0.0 Release 10 Sec. 6.5
 *****************************************************************************/

#ifndef SRSRAN_PMCH_H
#define SRSRAN_PMCH_H

/* TS 36.331: pmch-TimeInterleaving-M (MTimePMCH) max legal value. Bounds the
 * per-slot state arrays below - see srsran_pmch_t. */
#define SRSRAN_PMCH_MAX_TI_M 32

#include "srsran/config.h"
#include "srsran/phy/common/phy_common.h"
#include "srsran/phy/common/sequence.h"
#include "srsran/phy/mimo/layermap.h"
#include "srsran/phy/mimo/precoding.h"
#include "srsran/phy/modem/demod_soft.h"
#include "srsran/phy/modem/evm.h"
#include "srsran/phy/modem/mod.h"
#include "srsran/phy/phch/dci.h"
#include "srsran/phy/phch/pdsch.h"
#include "srsran/phy/phch/ra_dl.h"
#include "srsran/phy/phch/regs.h"
#include "srsran/phy/phch/sch.h"
#include "srsran/phy/scrambling/scrambling.h"
typedef struct {
  srsran_sequence_t seq[SRSRAN_NOF_SF_X_FRAME];
} srsran_pmch_seq_t;

typedef struct SRSRAN_API {
  srsran_pdsch_cfg_t pdsch_cfg;
  uint16_t           area_id;
  /* Rel-19 LTE_terr_bcast_Ph2 (TS 36.211 §6.5.1): PMCH cyclic bit shift */
  bool     cyclic_shift;       /* enabled by pmch-CyclicShiftAlpha-r19 */
  uint8_t  cyclic_shift_alpha; /* α: 1, 2, or 3 (alpha1/alpha2/alpha3 from RRC) */
  uint32_t subframe_idx;       /* i: index of this subframe within the N-subframe TB span */
  /* Rel-19 LTE_terr_bcast_Ph2 (TS 36.211 §6.5.2): frequency-domain block interleaving */
  bool freq_interleaving;      /* enabled by pmch-FreqInterleaving-r19 */
  /* Rel-19 LTE_terr_bcast_Ph2 (TS 36.213 §11.1): MCS table and time interleaving */
  bool    use_mcs_table2;       /* use Table 11.1-2 (256QAM) instead of Table 11.1-1 */
  uint8_t time_interleaving_n;  /* NTimePMCH: TB spans N subframes; 0/1 = disabled */
  uint8_t time_interleaving_m;  /* MTimePMCH: scheduling period length in subframes */
  /* PMCH-SoftBufferSizeParameters-r19: N_cb = min(floor(N_IR/C), K_w) soft-buffer
   * limitation (TS 36.212 §5.1.4.1.2). n_soft_ref_category (TS 36.306 Table 4.1-1)
   * and the exact scaling_factor_beta_num/den fraction are broadcast alongside
   * time_interleaving_n/m whenever time interleaving is active. 0 category = not
   * yet configured/decoded; srsran_pmch_encode/decode falls back to the uncapped
   * K_w in that case, exactly like before this field existed. */
  uint8_t n_soft_ref_category;
  uint8_t scaling_factor_beta_num;
  uint8_t scaling_factor_beta_den;
} srsran_pmch_cfg_t;

/* PMCH object */
typedef struct SRSRAN_API {
  srsran_cell_t cell;

  uint32_t nof_rx_antennas;

  uint32_t max_re;

  /* buffers */
  // void buffers are shared for tx and rx
  cf_t* ce[SRSRAN_MAX_PORTS][SRSRAN_MAX_PORTS];
  cf_t* symbols[SRSRAN_MAX_PORTS];
  cf_t* x[SRSRAN_MAX_PORTS];
  cf_t* d;
  void* e;

  /* EVM measurement buffer (RX only) - pdsch.c's srsran_pdsch_res_t::evm was
   * always left at its zero-initialized default for PMCH, since nothing ever
   * computed it (see srsran_pmch_decode's evm_run_s call). */
  srsran_evm_buffer_t* evm_buffer;

  /* tx & rx objects */
  srsran_modem_table_t mod[5]; /* BPSK, QPSK, 16QAM, 64QAM, 256QAM */

  // This is to generate the scrambling seq for multiple MBSFN Area IDs
  srsran_pmch_seq_t** seqs;

  srsran_sch_t dl_sch;

  /* Rel-19 time interleaving (TS 36.211 §6.5.3 / TS 36.213 §11.1). A PMCH
   * configured with pmch-TimeInterleaving-N/M pipelines up to M independent
   * transport-block streams (TBm, m=0..M-1) within one N*M-subframe block;
   * subframe s (0-based, s=cfg->subframe_idx) belongs to slot m=s%M with
   * redundancy version rvidx=n=(s%(N*M))/M. Each slot needs its own cached
   * payload and de-dup state, hence the arrays below, indexed by m.
   *
   * ti_rx_buf is unused by the rate-matching path - LLR soft-combining
   * happens inside the per-codeblock softbuffer via srsran_dlsch_decode_mch
   * instead (that softbuffer is caller-owned, not part of this struct - see
   * MbsfnFrameProcessor.cpp, which must itself keep one softbuffer per slot
   * m for the same reason these arrays exist). Left allocated rather than
   * removed since nothing currently depends on its absence. */
  int16_t* ti_rx_buf;  /* accumulated RX LLRs (unused - see above); max_re * 8 int16 */
  uint32_t ti_buf_nre; /* max_re value at allocation time */

  /* Per-slot (indexed by m) cached raw TB payload for TX re-encoding.
   * srsran_pmch_encode caches data into ti_tx_buf[m] the first time slot m
   * is seen at rvidx n==0, then re-encodes fresh from that cache on every
   * subsequent n for that slot - MAC only supplies a fresh `data` pointer
   * at n==0 (see mac.cc), matching this. Lazily allocated (NULL until slot m
   * is first used) since a configured M is typically far below the spec max
   * of 32 - sized ti_buf_nre * 8 bits worth of bytes per slot, same as the
   * single buffer this replaces (worst-case-N=16 sizing, unchanged). */
  uint8_t* ti_tx_buf[SRSRAN_PMCH_MAX_TI_M];

  /* Per-slot (indexed by m) de-dup guard for the RX path: once a TB's
   * codeblocks all pass CRC (partway through its own N-subframe span), the
   * codeblock-CRC-skip machinery means every later subframe of that slot's
   * span would also report success - this flag (reset when slot m starts a
   * new TB, i.e. at n==0 for that slot) stops srsran_pmch_decode from
   * re-reporting crc=true (and its caller from re-delivering the same
   * transport block) more than once per slot per block. */
  bool ti_decoded[SRSRAN_PMCH_MAX_TI_M];

} srsran_pmch_t;

SRSRAN_API int srsran_pmch_init(srsran_pmch_t* q, uint32_t max_prb, uint32_t nof_rx_antennas);

SRSRAN_API void srsran_pmch_free(srsran_pmch_t* q);

SRSRAN_API int srsran_pmch_set_cell(srsran_pmch_t* q, srsran_cell_t cell);

SRSRAN_API int srsran_pmch_set_area_id(srsran_pmch_t* q, uint16_t area_id);

SRSRAN_API void srsran_pmch_free_area_id(srsran_pmch_t* q, uint16_t area_id);

SRSRAN_API void srsran_configure_pmch(srsran_pmch_cfg_t* pmch_cfg, srsran_cell_t* cell, srsran_mbsfn_cfg_t* mbsfn_cfg);

SRSRAN_API int srsran_pmch_encode(srsran_pmch_t*      q,
                                  srsran_dl_sf_cfg_t* sf,
                                  srsran_pmch_cfg_t*  cfg,
                                  uint8_t*            data,
                                  cf_t*               sf_symbols[SRSRAN_MAX_PORTS]);

SRSRAN_API int srsran_pmch_decode(srsran_pmch_t*         q,
                                  srsran_dl_sf_cfg_t*    sf,
                                  srsran_pmch_cfg_t*     cfg,
                                  srsran_chest_dl_res_t* channel,
                                  cf_t*                  sf_symbols[SRSRAN_MAX_PORTS],
                                  srsran_pdsch_res_t*    data);

#endif // SRSRAN_PMCH_H
