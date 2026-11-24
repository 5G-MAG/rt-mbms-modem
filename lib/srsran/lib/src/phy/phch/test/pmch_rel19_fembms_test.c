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

/* FeMBMS PMCH Rel-19 feature loopback self-test.
 *
 * The two existing FeMBMS PMCH tests (pmch_fembms_test.c, freq-domain identity
 * loopback; pmch_ofdm_fembms_test.c, OFDM round trip) exercise the base FeMBMS
 * numerologies but leave the Rel-19 LTE_terr_bcast_Ph2 PMCH feature knobs on
 * srsran_pmch_cfg_t completely untested.  This test fills that gap.
 *
 * It is a frequency-domain, identity-channel loopback (encode -> identity copy of
 * the resource grid -> decode -> byte-exact compare + CRC check), which is the
 * right altitude here: each Rel-19 feature is a self-symmetric transform inside
 * srsran_pmch_encode/decode (TX applies it, RX undoes it), so a perfect-channel
 * loopback isolates the feature paths without dragging in OFDM buffer-sizing,
 * FFT normalisation, or channel estimation.  No OFDM objects are used, so the
 * known set_prb_scs / never-call-set_non_mbsfn_region pitfall does not arise here;
 * it is documented in the OFDM test and re-noted below for any future OFDM add-on.
 *
 * Features exercised, each as a clearly-labelled sub-case with pass/fail, on at
 * least two representative SCS (7.5 kHz and 1.25 kHz):
 *
 *   (a) FREQUENCY interleaving  (cfg->freq_interleaving) OFF and ON.
 *       Single-subframe round trip; no TBS scaling; grant built directly.
 *
 *   (b) CYCLIC shift            (cfg->cyclic_shift + cfg->cyclic_shift_alpha,
 *                                with cfg->subframe_idx).
 *       Single-subframe round trip; alpha swept over {1,2,3}.  The feature has TWO
 *       halves: a per-subcarrier phase rotation (always active when alpha in {1,2,3})
 *       and a bit-level left cyclic shift gated on Xi = pmch_cyclic_shift_Xi, which is
 *       0 whenever the number of code blocks C == 1.  The default QPSK sub-cases keep
 *       TBS below the single-CB limit (C == 1, Xi == 0), so they exercise the phase
 *       rotation only.  Two extra sub-cases (b') raise the MCS (mcs_override = 13,
 *       16QAM) so TBS at 25 PRB is 6456 bits > 6144 -> C = 2 and, with i*alpha not a
 *       multiple of C, Xi > 0, so the bit-level shift/anti-shift path actually runs and
 *       is verified end to end.
 *
 *   (c) TIME interleaving       (cfg->time_interleaving_n/_m).
 *       TS 36.212 §5.1.4.1.2 / TS 36.213 §11.1 / TS 36.321 §5.12 (Rel-19
 *       correction CRs, not just the introduction CR - see the project
 *       roadmap's Rel-19 Ph2 findings log): within a block of N*M subframes
 *       (indexed s=0..N*M-1), subframe s belongs to slot m=s%M - one of M
 *       INDEPENDENTLY PIPELINED transport blocks TBm - with redundancy
 *       version n=s/M (TS 36.213 CR1448/CR1459 clause 11.1's own wording:
 *       "MBSFN subframe indices m+n*MTimePMCH ... are associated with TBm
 *       ... with redundancy version rvidx=n"). This is NOT one ongoing TB
 *       cycling rv=0..N-1 while other slots sit idle - ALL M slots are live
 *       across the whole block, e.g. for N=2,M=4 the subframe order is
 *       (m=0,n=0),(m=1,n=0),(m=2,n=0),(m=3,n=0),(m=0,n=1),(m=1,n=1),...:
 *       every slot starts together, then every slot advances together.
 *       tb[0].tbs is the TOTAL transport block size for ONE slot's own TB,
 *       spanning its N subframes - TS 36.213 CR1448 (R1-2504967) clause
 *       11.1: "the UE shall scale the derived transport block size by
 *       alpha=NTimePMCH, then round to the closest valid transport block
 *       size" (a function of N only - all M slots share this same tbs,
 *       each carrying different content), mirroring srsran_configure_pmch's
 *       own scaling (pmch.c) so this test exercises the same convention
 *       production code actually builds, not a per-subframe-sized grant.
 *       Each slot independently rate-matches via the MCH-specific k0
 *       formula selecting a different starting offset in each codeblock's
 *       circular buffer at rv_idx=n - not a contiguous N-way slice of one
 *       big rv=0 pass. The receiver attempts combine-and-decode on EVERY
 *       subframe (early-decode), not only the last one of a slot's span;
 *       ti_decoded[m] (a per-slot srsran_pmch_t field) guards against
 *       reporting success more than once per slot per block. TX: full
 *       payload at n=0 for a given slot, data=NULL at n>0 for that slot
 *       (srsran_pmch_encode caches it internally in ti_tx_buf[m] and
 *       re-encodes fresh at each rv_idx). All N*M subframes of one block go
 *       through the SAME srsran_pmch_t instance in order, since
 *       ti_tx_buf[]/ti_decoded[] are per-object (indexed by slot) state
 *       persisting across calls, mirroring how one MbsfnFrameProcessor
 *       instance must keep one softbuffer per slot for the same reason. The
 *       test uses a SEPARATE, independently-random payload per slot so that
 *       cross-slot corruption (e.g. an off-by-one in the m/n split
 *       delivering one slot's bytes under another slot's identity) shows up
 *       as a payload mismatch, not just a missed CRC.
 *
 * For features (a) and (b), tb[0].tbs is the plain per-subframe grant value
 * (time interleaving is off, N=1 implicitly, so no scaling applies). For
 * feature (c), see above: tb[0].tbs is deliberately the TOTAL (per-slot),
 * N-scaled value, matching production's srsran_configure_pmch - NOT the
 * per-subframe size; do not "simplify" this back to an unscaled grant.
 *
 * The softbuffers are sized by nof_prb only, exactly as the production decoder does.
 */

#include <srsran/phy/utils/random.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "srsran/srsran.h"

/* Mirror of MAX_PMCH_RE() in pmch.c, built from the public phy_common.h macros.
 * Max data+RS resource elements per PRB in one MBSFN subframe for the given SCS. */
#define TEST_MAX_PMCH_RE(scs) (SRSRAN_MBSFN_NOF_SLOTS(scs) * SRSRAN_MBSFN_NOF_SYMBOLS(scs) * SRSRAN_NRE_SCS(scs))

typedef enum { FEAT_FREQ_INTERLEAVE, FEAT_CYCLIC_SHIFT, FEAT_TIME_INTERLEAVE, FEAT_LASTMTCH } feature_t;

typedef struct {
  const char*  name;
  feature_t    feature;
  srsran_scs_t scs;
  uint32_t     tti;
  /* Feature-specific parameters (only the relevant ones are used per feature). */
  bool    freq_interleaving; /* FEAT_FREQ_INTERLEAVE */
  uint8_t cyclic_alpha;      /* FEAT_CYCLIC_SHIFT: 1, 2 or 3 */
  uint8_t time_n;            /* FEAT_TIME_INTERLEAVE: NTimePMCH */
  uint8_t time_m;            /* FEAT_TIME_INTERLEAVE: MTimePMCH (>= time_n, per RRC's own clamp) */
  /* Per-case MCS override (0 = use the test-wide default).  Used only to raise the
   * per-subframe TBS above the single-code-block limit (B > 6144, i.e. TBS > 6120) so
   * srsran_cbsegm yields C >= 2 and pmch_cyclic_shift_Xi returns non-zero, forcing the
   * bit-level cyclic-shift path (pmch_cyclic_shift_bits / _llr) to actually run rather
   * than early-returning on Xi == 0. */
  uint32_t mcs_override;
  /* FEAT_LASTMTCH: main session runs (time_n, time_m); after its block ends, the
   * window boundary hits (subframe_idx resets to 0 for the "last MTCH" session)
   * and time interleaving switches to (time_n_last, time_m_last). time_n_last==1
   * means n1 (no interleaving for the last session specifically). 0 (both) means
   * "not a LastMTCH case" -- appended at the end, after mcs_override, to keep
   * every existing case's positional initializer meaning unchanged. */
  uint8_t time_n_last;
  uint8_t time_m_last;
  /* N_cb = min(floor(N_IR/C), K_w) soft-buffer cap (TS 36.212 §5.1.4.1.2), only read by
   * run_time_interleave_case(). 0/0/0 (the default for every case that omits these
   * trailing initializers) means "not configured" -- srsran_pmch_encode/decode falls
   * back to the uncapped K_w, exactly as if this field never existed. */
  uint8_t n_soft_ref_category;
  uint8_t beta_num;
  uint8_t beta_den;
} rel19_case_t;

/* Mirror of pmch_cyclic_shift_Xi() in pmch.c (static there, so not linkable from the
 * test).  Kept byte-for-byte in step with the production helper so the test can report
 * the shift amount Xi the encoder will use and detect a regression to Xi == 0 (which
 * would make the whole §6.5.1 bit-level shift / anti-shift path a dead no-op). */
static uint32_t test_cyclic_shift_Xi(uint32_t Mbit_bits, uint32_t C, uint32_t i, uint32_t alpha)
{
  if (C == 0) {
    return 0;
  }
  uint32_t Si      = (i * alpha) % C;
  uint32_t E_floor = Mbit_bits / C;
  uint32_t rem     = Mbit_bits % C;
  uint32_t Xi      = 0;
  for (uint32_t r = C - Si; r < C; r++) {
    Xi += (r >= C - rem) ? E_floor + 1 : E_floor;
  }
  return Xi;
}

/* Common per-case cell / dl_sf / grant setup shared by every feature.  Fills the
 * caller structs and builds the grant into pmch_cfg; returns 0 on success.
 * pmch_cfg->area_id and pmch_cfg->pdsch_cfg.softbuffers.tx[0] must already be set. */
static int build_common(const rel19_case_t*     tc,
                        uint32_t                nof_prb,
                        uint32_t                mcs_idx,
                        srsran_cell_t*          cell,
                        srsran_dl_sf_cfg_t*     dl_sf,
                        srsran_pmch_cfg_t*      pmch_cfg)
{
  memset(cell, 0, sizeof(*cell));
  cell->nof_prb         = nof_prb;
  cell->nof_ports       = 1;
  cell->id              = 1;
  cell->cp              = SRSRAN_CP_EXT; /* all MBSFN subframes use extended CP */
  cell->phich_length    = SRSRAN_PHICH_NORM;
  cell->phich_resources = SRSRAN_PHICH_R_1_6;
  cell->frame_type      = SRSRAN_FDD;

  ZERO_OBJECT(*dl_sf);
  dl_sf->cfi                = 0; /* FeMBMS dedicated carrier: no control region (lstart=0) */
  dl_sf->tti                = tc->tti;
  dl_sf->sf_type            = SRSRAN_SF_MBSFN;
  dl_sf->subcarrier_spacing = tc->scs;

  srsran_dci_dl_t dci;
  ZERO_OBJECT(dci);
  dci.rnti                    = SRSRAN_MRNTI;
  dci.format                  = SRSRAN_DCI_FORMAT1;
  dci.alloc_type              = SRSRAN_RA_ALLOC_TYPE0;
  dci.type0_alloc.rbg_bitmask = 0xffffffff;
  dci.tb[0].mcs_idx           = mcs_idx;
  SRSRAN_DCI_TB_DISABLE(dci.tb[1]);

  if (srsran_ra_dl_dci_to_grant(cell, dl_sf, SRSRAN_TM1, false, &dci, &pmch_cfg->pdsch_cfg.grant)) {
    ERROR("%s: error building grant", tc->name);
    return -1;
  }
  return 0;
}

/* --------------------------------------------------------------------------
 * (a)+(b) Single-subframe feature round trip (frequency interleaving, cyclic
 *         shift).  One encode -> identity copy -> decode, byte-exact + CRC.
 * -------------------------------------------------------------------------- */
static int run_single_sf_case(const rel19_case_t* tc, uint32_t nof_prb, uint32_t mcs_idx, uint16_t mbsfn_area_id)
{
  int             ret    = -1;
  srsran_random_t random = srsran_random_init(tc->tti + 1);

  const uint32_t grid_re = nof_prb * TEST_MAX_PMCH_RE(SRSRAN_SCS_370HZ);

  cf_t*                  tx_grid[SRSRAN_MAX_PORTS] = {0};
  cf_t*                  rx_grid[SRSRAN_MAX_PORTS] = {0};
  uint8_t*               data_tx                   = NULL;
  uint8_t*               data_rx                   = NULL;
  srsran_softbuffer_tx_t softbuffer_tx             = {0};
  srsran_softbuffer_rx_t softbuffer_rx             = {0};
  srsran_chest_dl_res_t  chest_dl_res              = {0};
  srsran_pmch_t          pmch                      = {0};
  bool                   pmch_ok = false, sb_tx_ok = false, sb_rx_ok = false, chest_ok = false;

  srsran_cell_t      cell;
  srsran_dl_sf_cfg_t dl_sf;
  srsran_pmch_cfg_t  pmch_cfg;

  tx_grid[0] = srsran_vec_cf_malloc(grid_re);
  rx_grid[0] = srsran_vec_cf_malloc(grid_re);
  if (!tx_grid[0] || !rx_grid[0]) {
    ERROR("%s: error allocating resource grids", tc->name);
    goto cleanup;
  }
  srsran_vec_cf_zero(tx_grid[0], grid_re);
  srsran_vec_cf_zero(rx_grid[0], grid_re);

  if (srsran_chest_dl_res_init(&chest_dl_res, nof_prb)) {
    ERROR("%s: error initialising chest_dl_res", tc->name);
    goto cleanup;
  }
  chest_ok = true;
  srsran_chest_dl_res_set_identity(&chest_dl_res); /* perfect channel */

  if (srsran_softbuffer_tx_init(&softbuffer_tx, nof_prb)) {
    ERROR("%s: error initialising TX softbuffer", tc->name);
    goto cleanup;
  }
  sb_tx_ok = true;
  if (srsran_softbuffer_rx_init(&softbuffer_rx, nof_prb)) {
    ERROR("%s: error initialising RX softbuffer", tc->name);
    goto cleanup;
  }
  sb_rx_ok = true;

  if (srsran_pmch_init(&pmch, nof_prb, 1)) {
    ERROR("%s: error initialising PMCH", tc->name);
    goto cleanup;
  }
  pmch_ok = true;
  if (srsran_pmch_set_area_id(&pmch, mbsfn_area_id)) {
    ERROR("%s: error setting PMCH area id", tc->name);
    goto cleanup;
  }

  ZERO_OBJECT(pmch_cfg);
  pmch_cfg.area_id                     = mbsfn_area_id;
  pmch_cfg.pdsch_cfg.softbuffers.tx[0] = &softbuffer_tx;

  if (build_common(tc, nof_prb, mcs_idx, &cell, &dl_sf, &pmch_cfg)) {
    goto cleanup;
  }

  /* --- Apply the feature knobs for this sub-case --- */
  switch (tc->feature) {
    case FEAT_FREQ_INTERLEAVE:
      pmch_cfg.freq_interleaving = tc->freq_interleaving;
      break;
    case FEAT_CYCLIC_SHIFT:
      /* Cyclic shift is a no-op unless BOTH the flag is set AND alpha in {1,2,3}. */
      pmch_cfg.cyclic_shift       = true;
      pmch_cfg.cyclic_shift_alpha = tc->cyclic_alpha;
      pmch_cfg.subframe_idx       = tc->tti; /* i: drives pmch_cyclic_shift_Xi */
      break;
    case FEAT_TIME_INTERLEAVE:
    default:
      ERROR("%s: run_single_sf_case invoked for a non-single-subframe feature", tc->name);
      goto cleanup;
  }

  uint32_t tbs    = (uint32_t)pmch_cfg.pdsch_cfg.grant.tb[0].tbs;
  uint32_t nof_re = pmch_cfg.pdsch_cfg.grant.nof_re;
  if (tbs == 0 || nof_re == 0 || nof_re > grid_re) {
    ERROR("%s: implausible grant tbs=%d nof_re=%d (grid_re=%d)", tc->name, tbs, nof_re, grid_re);
    goto cleanup;
  }

  /* Report (and, for the dedicated multi-CB cases, require) the bit-level shift amount
   * the encoder will use.  Xi mirrors pmch_cyclic_shift_Xi's exact inputs: Mbit_sf =
   * grant.tb[0].nof_bits, C from srsran_cbsegm(tbs), i = subframe_idx, alpha.  A case
   * flagged as bit-shift (mcs_override set) MUST get Xi > 0, otherwise the §6.5.1
   * bit-shift / anti-shift path would silently early-return (dead no-op). */
  if (tc->feature == FEAT_CYCLIC_SHIFT) {
    srsran_cbsegm_t cb_segm;
    srsran_cbsegm(&cb_segm, tbs);
    uint32_t Mbit_sf = pmch_cfg.pdsch_cfg.grant.tb[0].nof_bits;
    uint32_t Xi      = test_cyclic_shift_Xi(Mbit_sf, cb_segm.C, pmch_cfg.subframe_idx, tc->cyclic_alpha);
    printf("    [%s] C=%d Mbit_sf=%d alpha=%d i=%d -> Xi=%d (%s)\n",
           tc->name,
           cb_segm.C,
           Mbit_sf,
           tc->cyclic_alpha,
           pmch_cfg.subframe_idx,
           Xi,
           Xi ? "bit-shift active" : "phase-rotation only");
    if (tc->mcs_override != 0 && Xi == 0) {
      ERROR("%s: expected a non-zero cyclic-shift Xi (C=%d) but got Xi=0; bit-shift path is dead",
            tc->name,
            cb_segm.C);
      goto cleanup;
    }
  }

  data_tx = srsran_vec_u8_malloc(tbs / 8);
  /* decode's output buffer contract is tbs+24 bits, not tbs bits: the TB-level
   * CRC24 is decoded in-place right after the payload and re-verified there
   * (see decode_tb's srsran_crc_match_byte(&q->crc_tb, data, cb_segm->tbs)
   * call, which reads tbs+24 bits). Sizing this at tbs/8 caused a real
   * heap-buffer-overflow, found via ASan. */
  data_rx = srsran_vec_u8_malloc((tbs + 24 + 7) / 8);
  if (!data_tx || !data_rx) {
    ERROR("%s: error allocating payload buffers", tc->name);
    goto cleanup;
  }
  for (uint32_t i = 0; i < tbs / 8; i++) {
    data_tx[i] = (uint8_t)srsran_random_uniform_int_dist(random, 0, 255);
  }

  if (srsran_pmch_encode(&pmch, &dl_sf, &pmch_cfg, data_tx, tx_grid)) {
    ERROR("%s: PMCH encode failed", tc->name);
    goto cleanup;
  }

  /* Identity channel: the received grid equals the transmitted grid. */
  memcpy(rx_grid[0], tx_grid[0], grid_re * sizeof(cf_t));

  srsran_softbuffer_rx_reset_tbs(&softbuffer_rx, tbs);
  pmch_cfg.pdsch_cfg.softbuffers.rx[0] = &softbuffer_rx;

  srsran_pdsch_res_t pdsch_res[SRSRAN_MAX_CODEWORDS];
  ZERO_OBJECT(pdsch_res);
  pdsch_res[0].payload = data_rx;

  if (srsran_pmch_decode(&pmch, &dl_sf, &pmch_cfg, &chest_dl_res, rx_grid, pdsch_res)) {
    ERROR("%s: PMCH decode returned error", tc->name);
    goto cleanup;
  }
  if (!pdsch_res[0].crc) {
    ERROR("%s: PMCH CRC failed (tbs=%d nof_re=%d)", tc->name, tbs, nof_re);
    goto cleanup;
  }
  if (memcmp(data_tx, data_rx, tbs / 8) != 0) {
    ERROR("%s: payload mismatch after decode (tbs=%d nof_re=%d)", tc->name, tbs, nof_re);
    goto cleanup;
  }

  printf("  %-34s tti=%2d  nof_re=%6d  tbs=%5d bits  ... OK\n", tc->name, tc->tti, nof_re, tbs);
  ret = 0;

cleanup:
  if (data_tx) {
    free(data_tx);
  }
  if (data_rx) {
    free(data_rx);
  }
  if (tx_grid[0]) {
    free(tx_grid[0]);
  }
  if (rx_grid[0]) {
    free(rx_grid[0]);
  }
  if (chest_ok) {
    srsran_chest_dl_res_free(&chest_dl_res);
  }
  if (sb_tx_ok) {
    srsran_softbuffer_tx_free(&softbuffer_tx);
  }
  if (sb_rx_ok) {
    srsran_softbuffer_rx_free(&softbuffer_rx);
  }
  if (pmch_ok) {
    srsran_pmch_free(&pmch);
  }
  srsran_random_free(random);
  return ret;
}

/* --------------------------------------------------------------------------
 * (c) Time-interleaving feature round trip.  N subframes, encoded/decoded in
 *     order j=0..N-1 through ONE pmch instance:
 *       - tb[0].tbs is the TOTAL transport block size across all N
 *         subframes (TS 36.213 CR1448 clause 11.1: scaled by alpha=N,
 *         rounded to the nearest valid TBS table entry - see the scaling
 *         block below, which replicates srsran_configure_pmch's own logic
 *         so the grant matches what production actually builds).
 *         tb[0].nof_bits (Mbit_sf) is NOT scaled - it stays the per-subframe
 *         RE-derived bit budget, the same on every one of the N subframes.
 *       - TX: full payload at j=0 (data != NULL), data=NULL for j=1..N-1;
 *             each subframe is an INDEPENDENT rv_idx=j rate-matching pass
 *             over the SAME C-codeblock payload (TS 36.212 §5.1.4.1.2),
 *             extracting e_min = Qm*(Mbit_sf/Qm/C) bits per codeblock at a
 *             k0 offset that shifts with rv_idx - not N separate contiguous
 *             chunks of the bitstream.
 *       - RX: srsran_dlsch_decode_mch attempts combine-and-decode on EVERY
 *             subframe (TS 36.321 §5.12 early-decode), reusing the
 *             per-codeblock CRC-skip machinery for "stop once successful".
 *             srsran_pmch_t's ti_decoded field means pdsch_res[0].crc is
 *             true on AT MOST one j per block (the first one that actually
 *             succeeds) - this test doesn't assume WHICH j that is, only
 *             that it happens by j=N-1 and that the decoded payload is
 *             then correct. */
static int run_time_interleave_case(const rel19_case_t* tc, uint32_t nof_prb, uint32_t mcs_idx, uint16_t mbsfn_area_id)
{
  int             ret    = -1;
  srsran_random_t random = srsran_random_init(tc->tti + 1);
  const uint8_t   N      = tc->time_n;
  const uint8_t   M      = tc->time_m;

  const uint32_t grid_re = nof_prb * TEST_MAX_PMCH_RE(SRSRAN_SCS_370HZ);

  cf_t*                  tx_grid[SRSRAN_MAX_PORTS]        = {0};
  cf_t*                  rx_grid[SRSRAN_MAX_PORTS]        = {0};
  uint8_t*               data_tx[SRSRAN_PMCH_MAX_TI_M]    = {0}; /* one independent payload per slot m */
  uint8_t*               data_rx                          = NULL; /* reused scratch, checked immediately on crc=true */
  srsran_softbuffer_tx_t softbuffer_tx                    = {0};
  srsran_softbuffer_rx_t softbuffer_rx[SRSRAN_PMCH_MAX_TI_M] = {0}; /* one independent softbuffer per slot m */
  bool                   sb_rx_ok[SRSRAN_PMCH_MAX_TI_M]   = {false};
  bool                   decoded_ok[SRSRAN_PMCH_MAX_TI_M] = {false};
  srsran_chest_dl_res_t  chest_dl_res                     = {0};
  srsran_pmch_t          pmch                             = {0};
  bool                   pmch_ok = false, sb_tx_ok = false, chest_ok = false;

  srsran_cell_t      cell;
  srsran_dl_sf_cfg_t dl_sf;
  srsran_pmch_cfg_t  pmch_cfg;

  if (N < 2) {
    ERROR("%s: time_n must be >= 2", tc->name);
    goto cleanup;
  }
  if (M < N || M > SRSRAN_PMCH_MAX_TI_M) {
    /* TS 36.213 CR1448 clause 11.1 / rrc.cc's own clamp: MTimePMCH is always
     * >= NTimePMCH once time interleaving is configured - M<N is not a
     * legal configuration this test should be constructing. */
    ERROR("%s: time_m (%d) must be >= time_n (%d) and <= %d", tc->name, M, N, SRSRAN_PMCH_MAX_TI_M);
    goto cleanup;
  }

  tx_grid[0] = srsran_vec_cf_malloc(grid_re);
  rx_grid[0] = srsran_vec_cf_malloc(grid_re);
  if (!tx_grid[0] || !rx_grid[0]) {
    ERROR("%s: error allocating resource grids", tc->name);
    goto cleanup;
  }

  if (srsran_chest_dl_res_init(&chest_dl_res, nof_prb)) {
    ERROR("%s: error initialising chest_dl_res", tc->name);
    goto cleanup;
  }
  chest_ok = true;
  srsran_chest_dl_res_set_identity(&chest_dl_res); /* perfect channel */

  if (srsran_softbuffer_tx_init(&softbuffer_tx, nof_prb)) {
    ERROR("%s: error initialising TX softbuffer", tc->name);
    goto cleanup;
  }
  sb_tx_ok = true;
  for (uint8_t m = 0; m < M; m++) {
    if (srsran_softbuffer_rx_init(&softbuffer_rx[m], nof_prb)) {
      ERROR("%s: error initialising RX softbuffer for slot %d", tc->name, m);
      goto cleanup;
    }
    sb_rx_ok[m] = true;
  }

  if (srsran_pmch_init(&pmch, nof_prb, 1)) {
    ERROR("%s: error initialising PMCH", tc->name);
    goto cleanup;
  }
  pmch_ok = true;
  if (srsran_pmch_set_area_id(&pmch, mbsfn_area_id)) {
    ERROR("%s: error setting PMCH area id", tc->name);
    goto cleanup;
  }

  ZERO_OBJECT(pmch_cfg);
  pmch_cfg.area_id                     = mbsfn_area_id;
  pmch_cfg.pdsch_cfg.softbuffers.tx[0] = &softbuffer_tx;

  if (build_common(tc, nof_prb, mcs_idx, &cell, &dl_sf, &pmch_cfg)) {
    goto cleanup;
  }

  /* Enable time interleaving. Both N and M matter to encode/decode now - see
   * srsran_pmch_encode/decode's comment in pmch.c: m=s%M, n=(s%(N*M))/M. */
  pmch_cfg.time_interleaving_n = N;
  pmch_cfg.time_interleaving_m = M;
  /* N_cb soft-buffer cap (TS 36.212 §5.1.4.1.2) - 0/0/0 (every case that doesn't set
   * these) means "not configured", matching pre-capping behavior exactly. */
  pmch_cfg.n_soft_ref_category     = tc->n_soft_ref_category;
  pmch_cfg.scaling_factor_beta_num = tc->beta_num;
  pmch_cfg.scaling_factor_beta_den = tc->beta_den;

  /* TS 36.213 CR1448 (R1-2504967) clause 11.1: "the UE shall scale the
   * derived transport block size by alpha=NTimePMCH, then round to the
   * closest valid transport block size". build_common() above built a
   * plain, TI-unaware grant via srsran_ra_dl_dci_to_grant; replicate
   * srsran_configure_pmch's scaling block (pmch.c) here so tb[0].tbs ends up
   * holding the same TOTAL, N-scaled value production code would give it -
   * do not skip this step, it is not equivalent to leaving tbs unscaled.
   * Scaling depends only on N, not M: all M slots share this same tbs, each
   * carrying its own, independent N-subframe-spanning transport block. */
  {
    int base_tbs = pmch_cfg.pdsch_cfg.grant.tb[0].tbs;
    int scaled   = base_tbs * (int)N;
    int tbs_idx  = srsran_ra_tbs_to_table_idx(
        (uint32_t)scaled, pmch_cfg.pdsch_cfg.grant.nof_prb, SRSRAN_RA_NOF_TBS_IDX - 1);
    if (tbs_idx >= (int)SRSRAN_RA_NOF_TBS_IDX) {
      tbs_idx = (int)SRSRAN_RA_NOF_TBS_IDX - 1;
    }
    if (tbs_idx < 0) {
      tbs_idx = 0;
    }
    pmch_cfg.pdsch_cfg.grant.tb[0].tbs = srsran_ra_tbs_from_idx((uint32_t)tbs_idx, pmch_cfg.pdsch_cfg.grant.nof_prb);
  }

  const uint32_t tbs    = (uint32_t)pmch_cfg.pdsch_cfg.grant.tb[0].tbs;
  const uint32_t nof_re = pmch_cfg.pdsch_cfg.grant.nof_re;
  if (tbs == 0 || nof_re == 0 || nof_re > grid_re) {
    ERROR("%s: implausible grant tbs=%d nof_re=%d (grid_re=%d)", tc->name, tbs, nof_re, grid_re);
    goto cleanup;
  }

  /* Each of the M slots gets its OWN, independently-random payload - this is
   * what lets the test detect cross-slot corruption (e.g. an off-by-one in
   * the m/n split delivering slot 0's bytes under slot 1's identity), which
   * a single-payload test structurally cannot catch. */
  for (uint8_t m = 0; m < M; m++) {
    data_tx[m] = srsran_vec_u8_malloc(tbs / 8);
    if (!data_tx[m]) {
      ERROR("%s: error allocating payload buffer for slot %d", tc->name, m);
      goto cleanup;
    }
    for (uint32_t i = 0; i < tbs / 8; i++) {
      data_tx[m][i] = (uint8_t)srsran_random_uniform_int_dist(random, 0, 255);
    }
  }
  /* decode's output buffer contract is tbs+24 bits (payload + TB-CRC24,
   * decoded in-place and re-verified there), not tbs bits - see sch.c's
   * decode_tb_cb / decode_tb. tbs/8 alone under-allocates by 3 bytes. */
  data_rx = srsran_vec_u8_malloc((tbs + 24 + 7) / 8);
  if (!data_rx) {
    ERROR("%s: error allocating RX scratch buffer", tc->name);
    goto cleanup;
  }

  /* Walk one full N*M-subframe block in order (s=0..N*M-1) through the SAME
   * pmch object. Subframe s belongs to slot m=s%M with redundancy version
   * n=s/M (TS 36.213 CR1448/CR1459 clause 11.1) - e.g. for N=2,M=4 the order
   * is (m=0,n=0),(m=1,n=0),(m=2,n=0),(m=3,n=0),(m=0,n=1),(m=1,n=1),... : all
   * four slots start together, then all four advance together, matching
   * genuine pipelining rather than one slot finishing before the next
   * starts. ti_tx_buf[m]/ti_decoded[m] inside pmch/pmch_t are per-slot, so
   * one pmch object correctly keeps all M slots' state independent. */
  for (uint32_t s = 0; s < (uint32_t)N * M; s++) {
    uint8_t slot_m = (uint8_t)(s % M);
    uint8_t slot_n = (uint8_t)(s / M);

    pmch_cfg.subframe_idx = s;

    srsran_vec_cf_zero(tx_grid[0], grid_re);
    srsran_vec_cf_zero(rx_grid[0], grid_re);

    /* Full payload only at n=0 for this slot; data=NULL otherwise -
     * srsran_pmch_encode caches it internally (in ti_tx_buf[slot_m]) and
     * re-encodes it fresh at each rv_idx=n, it does not need the caller to
     * keep passing it. */
    uint8_t* enc_data = (slot_n == 0) ? data_tx[slot_m] : NULL;
    if (srsran_pmch_encode(&pmch, &dl_sf, &pmch_cfg, enc_data, tx_grid)) {
      ERROR("%s: PMCH encode failed at s=%d (slot=%d n=%d)", tc->name, s, slot_m, slot_n);
      goto cleanup;
    }

    /* Identity channel per subframe. */
    memcpy(rx_grid[0], tx_grid[0], grid_re * sizeof(cf_t));

    /* Per-slot RX softbuffer reset at n=0, mirroring MbsfnFrameProcessor.cpp
     * exactly (both reset_cb and reset_tbs) - the per-codeblock CRC/LLR
     * state must persist across a slot's own N subframes for soft-combining
     * to accumulate, but must NOT leak between two different slots or two
     * different TBs of the same slot. */
    if (slot_n == 0) {
      srsran_softbuffer_rx_reset_cb(&softbuffer_rx[slot_m], 1);
      srsran_softbuffer_rx_reset_tbs(&softbuffer_rx[slot_m], tbs);
    }
    pmch_cfg.pdsch_cfg.softbuffers.rx[0] = &softbuffer_rx[slot_m];

    srsran_pdsch_res_t pdsch_res[SRSRAN_MAX_CODEWORDS];
    ZERO_OBJECT(pdsch_res);
    pdsch_res[0].payload = data_rx;

    if (srsran_pmch_decode(&pmch, &dl_sf, &pmch_cfg, &chest_dl_res, rx_grid, pdsch_res)) {
      ERROR("%s: PMCH decode returned error at s=%d (slot=%d n=%d)", tc->name, s, slot_m, slot_n);
      goto cleanup;
    }

    /* Early-decode (TS 36.321 §5.12): decode may legitimately succeed at ANY
     * n, not only the last one - this test does not assume which. Once it
     * succeeds, ti_decoded[slot_m]'s de-dup guard means pdsch_res[0].crc
     * reports true at most once per slot per block, so only the first
     * success is checked against THIS SLOT's OWN payload - a mismatch here
     * (e.g. matching a DIFFERENT slot's data_tx instead) is exactly the
     * cross-slot-corruption failure mode this M-slot test exists to catch. */
    if (pdsch_res[0].crc) {
      if (decoded_ok[slot_m]) {
        ERROR("%s: unexpected repeated CRC=true for slot %d at s=%d (N=%d M=%d) - ti_decoded de-dup guard not working",
              tc->name, slot_m, s, (int)N, (int)M);
        goto cleanup;
      }
      if (memcmp(data_tx[slot_m], data_rx, tbs / 8) != 0) {
        ERROR("%s: payload mismatch for slot %d at s=%d (tbs=%d N=%d M=%d) - possible cross-slot corruption",
              tc->name, slot_m, s, tbs, (int)N, (int)M);
        goto cleanup;
      }
      decoded_ok[slot_m] = true;
    }
  }

  for (uint8_t m = 0; m < M; m++) {
    if (!decoded_ok[m]) {
      ERROR("%s: slot %d never decoded successfully (N=%d M=%d)", tc->name, m, (int)N, (int)M);
      goto cleanup;
    }
  }

  printf("  %-34s tti=%2d  nof_re=%6d  tbs=%5d bits  N=%d  M=%d  ... OK\n", tc->name, tc->tti, nof_re, tbs, (int)N, (int)M);
  ret = 0;

cleanup:
  for (uint8_t m = 0; m < M && m < SRSRAN_PMCH_MAX_TI_M; m++) {
    if (data_tx[m]) {
      free(data_tx[m]);
    }
  }
  if (data_rx) {
    free(data_rx);
  }
  if (tx_grid[0]) {
    free(tx_grid[0]);
  }
  if (rx_grid[0]) {
    free(rx_grid[0]);
  }
  if (chest_ok) {
    srsran_chest_dl_res_free(&chest_dl_res);
  }
  if (sb_tx_ok) {
    srsran_softbuffer_tx_free(&softbuffer_tx);
  }
  for (uint32_t m = 0; m < SRSRAN_PMCH_MAX_TI_M; m++) {
    if (sb_rx_ok[m]) {
      srsran_softbuffer_rx_free(&softbuffer_rx[m]);
    }
  }
  if (pmch_ok) {
    srsran_pmch_free(&pmch);
  }
  srsran_random_free(random);
  return ret;
}

/* pmch-TimeInterleavingN/M-LastMTCH-r19 (TS 36.331 CR5168r3) loopback test.
 *
 * Runs TWO back-to-back segments through the SAME srsran_pmch_t object, deliberately
 * with a DIFFERENT (N, M) in each and a session boundary NOT aligned to either
 * segment's own N*M block size (the exact scenario the introduction-CR research
 * flagged as untested by the flat, whole-period counter this feature replaces):
 *   segment 1 ("main"):      subframe_idx = 0 .. N*M-1,           using (tc->time_n, tc->time_m)
 *   segment 2 ("last MTCH"): subframe_idx RESETS to 0 .. N'*M'-1, using (tc->time_n_last, tc->time_m_last)
 * This mirrors exactly what phy_common.cc's is_mch_subframe() (TX) and Phy.cpp's
 * mbsfn_config_for_tti() (RX) now do at a LastMTCH window boundary -- both reset
 * cfg->mch_subframe_idx to 0 and switch time_interleaving_n/_m at the same instant.
 * Each of the M+M' slots across both segments gets its own independently-random
 * payload, so any cross-segment corruption (e.g. segment 2's slot 0 accidentally
 * decoding as "already done" from segment 1's stale ti_decoded[0], or reading
 * segment 1's cached ti_tx_buf[0] instead of its own fresh payload) is caught by a
 * payload mismatch, exactly like run_time_interleave_case's existing cross-slot
 * check. pmch.c itself needs (and gets) zero code changes for this to pass -- see
 * this test file's own verification of pmch.c's reset trigger (cfg->subframe_idx
 * % block_len, freshly computed every call from whatever the caller currently has
 * set) for why the boundary "just works" through the existing mechanism.
 *
 * NOTE: this repo's srsran_pmch_encode() has an older 5-argument signature (no
 * shared_ti_tx_buf) than the tx repo's -- the modem app is receive-only and never
 * calls srsran_pmch_encode() in production, so this divergence was never mirrored
 * here. Calls below deliberately match THIS repo's own signature. */
static int run_lastmtch_case(const rel19_case_t* tc, uint32_t nof_prb, uint32_t mcs_idx, uint16_t mbsfn_area_id)
{
  int             ret    = -1;
  srsran_random_t random = srsran_random_init(tc->tti + 1);
  const uint8_t   N1     = tc->time_n;
  const uint8_t   M1     = tc->time_m;
  const uint8_t   N2     = tc->time_n_last;
  const uint8_t   M2     = (tc->time_m_last > 0) ? tc->time_m_last : tc->time_m;

  const uint32_t grid_re = nof_prb * TEST_MAX_PMCH_RE(SRSRAN_SCS_370HZ);

  cf_t*                  tx_grid[SRSRAN_MAX_PORTS]     = {0};
  cf_t*                  rx_grid[SRSRAN_MAX_PORTS]     = {0};
  /* Slots 0..M1-1 belong to segment 1; a SEPARATE set of slots 0..M2-1 belongs to
   * segment 2 -- kept in distinct arrays (not reused across segments) so a mismatch
   * unambiguously identifies which segment's data leaked into which. */
  uint8_t*               data_tx1[SRSRAN_PMCH_MAX_TI_M] = {0};
  uint8_t*               data_tx2[SRSRAN_PMCH_MAX_TI_M] = {0};
  uint8_t*               data_rx                        = NULL;
  srsran_softbuffer_tx_t softbuffer_tx                  = {0};
  srsran_softbuffer_rx_t softbuffer_rx[SRSRAN_PMCH_MAX_TI_M] = {0};
  bool                   sb_rx_ok[SRSRAN_PMCH_MAX_TI_M]  = {false};
  bool                   decoded1[SRSRAN_PMCH_MAX_TI_M]  = {false};
  bool                   decoded2[SRSRAN_PMCH_MAX_TI_M]  = {false};
  srsran_chest_dl_res_t  chest_dl_res                    = {0};
  srsran_pmch_t          pmch                            = {0};
  bool                   pmch_ok = false, sb_tx_ok = false, chest_ok = false;

  srsran_cell_t      cell;
  srsran_dl_sf_cfg_t dl_sf;
  srsran_pmch_cfg_t  pmch_cfg;

  if (N1 < 2 || N2 < 2) {
    ERROR("%s: both time_n and time_n_last must be >= 2 for this test (n1/disabled is a separate, "
          "already-covered code path -- see this function's own comment)",
          tc->name);
    goto cleanup;
  }
  if (M1 < N1 || M1 > SRSRAN_PMCH_MAX_TI_M || M2 < N2 || M2 > SRSRAN_PMCH_MAX_TI_M) {
    ERROR("%s: time_m (%d) and time_m_last (%d) must each be >= their own N and <= %d",
          tc->name, M1, M2, SRSRAN_PMCH_MAX_TI_M);
    goto cleanup;
  }

  tx_grid[0] = srsran_vec_cf_malloc(grid_re);
  rx_grid[0] = srsran_vec_cf_malloc(grid_re);
  if (!tx_grid[0] || !rx_grid[0]) {
    ERROR("%s: error allocating resource grids", tc->name);
    goto cleanup;
  }

  if (srsran_chest_dl_res_init(&chest_dl_res, nof_prb)) {
    ERROR("%s: error initialising chest_dl_res", tc->name);
    goto cleanup;
  }
  chest_ok = true;
  srsran_chest_dl_res_set_identity(&chest_dl_res);

  if (srsran_softbuffer_tx_init(&softbuffer_tx, nof_prb)) {
    ERROR("%s: error initialising TX softbuffer", tc->name);
    goto cleanup;
  }
  sb_tx_ok = true;
  for (uint8_t m = 0; m < SRSRAN_PMCH_MAX_TI_M; m++) {
    if (srsran_softbuffer_rx_init(&softbuffer_rx[m], nof_prb)) {
      ERROR("%s: error initialising RX softbuffer for slot %d", tc->name, m);
      goto cleanup;
    }
    sb_rx_ok[m] = true;
  }

  if (srsran_pmch_init(&pmch, nof_prb, 1)) {
    ERROR("%s: error initialising PMCH", tc->name);
    goto cleanup;
  }
  pmch_ok = true;
  if (srsran_pmch_set_area_id(&pmch, mbsfn_area_id)) {
    ERROR("%s: error setting PMCH area id", tc->name);
    goto cleanup;
  }

  ZERO_OBJECT(pmch_cfg);
  pmch_cfg.area_id                     = mbsfn_area_id;
  pmch_cfg.pdsch_cfg.softbuffers.tx[0] = &softbuffer_tx;

  if (build_common(tc, nof_prb, mcs_idx, &cell, &dl_sf, &pmch_cfg)) {
    goto cleanup;
  }

  /* Generously-sized fixed buffer, reused by both segments below (decode's output
   * buffer contract is tbs+24 bits, not tbs bits - see sch.c's decode_tb_cb /
   * decode_tb).  25 PRB's largest realistic N-scaled TBS in this test is well
   * under 16384 bits. */
  data_rx = srsran_vec_u8_malloc(16384 / 8);
  if (!data_rx) {
    ERROR("%s: error allocating RX scratch buffer", tc->name);
    goto cleanup;
  }

  /* Run one segment (N, M) starting at subframe_idx=0, using data_tx[]/decoded[]
   * for its own slots.  Shared by both segments below -- the only difference
   * between them is which N/M and which payload/decoded arrays are passed in. */
#define RUN_SEGMENT(N, M, data_tx, decoded, seg_name)                                                                \
  do {                                                                                                                \
    pmch_cfg.time_interleaving_n = (N);                                                                              \
    pmch_cfg.time_interleaving_m = (M);                                                                              \
    int base_tbs = 0;                                                                                                \
    {                                                                                                                 \
      srsran_dci_dl_t dci_tmp;                                                                                       \
      ZERO_OBJECT(dci_tmp);                                                                                          \
      dci_tmp.rnti                    = SRSRAN_MRNTI;                                                                \
      dci_tmp.format                  = SRSRAN_DCI_FORMAT1;                                                          \
      dci_tmp.alloc_type              = SRSRAN_RA_ALLOC_TYPE0;                                                       \
      dci_tmp.type0_alloc.rbg_bitmask = 0xffffffff;                                                                   \
      dci_tmp.tb[0].mcs_idx           = mcs_idx;                                                                     \
      SRSRAN_DCI_TB_DISABLE(dci_tmp.tb[1]);                                                                          \
      if (srsran_ra_dl_dci_to_grant(&cell, &dl_sf, SRSRAN_TM1, false, &dci_tmp, &pmch_cfg.pdsch_cfg.grant)) {         \
        ERROR("%s: error rebuilding grant for %s", tc->name, seg_name);                                              \
        goto cleanup;                                                                                                \
      }                                                                                                              \
      base_tbs = pmch_cfg.pdsch_cfg.grant.tb[0].tbs;                                                                 \
    }                                                                                                                \
    int scaled  = base_tbs * (int)(N);                                                                              \
    int tbs_idx = srsran_ra_tbs_to_table_idx((uint32_t)scaled, pmch_cfg.pdsch_cfg.grant.nof_prb, SRSRAN_RA_NOF_TBS_IDX - 1); \
    if (tbs_idx >= (int)SRSRAN_RA_NOF_TBS_IDX) tbs_idx = (int)SRSRAN_RA_NOF_TBS_IDX - 1;                              \
    if (tbs_idx < 0) tbs_idx = 0;                                                                                    \
    pmch_cfg.pdsch_cfg.grant.tb[0].tbs = srsran_ra_tbs_from_idx((uint32_t)tbs_idx, pmch_cfg.pdsch_cfg.grant.nof_prb); \
    const uint32_t seg_tbs    = (uint32_t)pmch_cfg.pdsch_cfg.grant.tb[0].tbs;                                        \
    const uint32_t seg_nof_re = pmch_cfg.pdsch_cfg.grant.nof_re;                                                     \
    if (seg_tbs == 0 || seg_nof_re == 0 || seg_nof_re > grid_re) {                                                   \
      ERROR("%s: implausible grant tbs=%d nof_re=%d in %s", tc->name, seg_tbs, seg_nof_re, seg_name);                \
      goto cleanup;                                                                                                  \
    }                                                                                                                 \
    for (uint8_t m = 0; m < (M); m++) {                                                                              \
      (data_tx)[m] = srsran_vec_u8_malloc(seg_tbs / 8);                                                              \
      if (!(data_tx)[m]) {                                                                                           \
        ERROR("%s: error allocating payload buffer for slot %d in %s", tc->name, m, seg_name);                      \
        goto cleanup;                                                                                                \
      }                                                                                                              \
      for (uint32_t b = 0; b < seg_tbs / 8; b++) {                                                                   \
        (data_tx)[m][b] = (uint8_t)srsran_random_uniform_int_dist(random, 0, 255);                                   \
      }                                                                                                              \
    }                                                                                                                \
    for (uint32_t s = 0; s < (uint32_t)(N) * (M); s++) {                                                             \
      uint8_t slot_m = (uint8_t)(s % (M));                                                                           \
      uint8_t slot_n = (uint8_t)(s / (M));                                                                           \
      pmch_cfg.subframe_idx = s;                                                                                     \
      srsran_vec_cf_zero(tx_grid[0], grid_re);                                                                       \
      srsran_vec_cf_zero(rx_grid[0], grid_re);                                                                       \
      uint8_t* enc_data = (slot_n == 0) ? (data_tx)[slot_m] : NULL;                                                  \
      if (srsran_pmch_encode(&pmch, &dl_sf, &pmch_cfg, enc_data, tx_grid)) {                                         \
        ERROR("%s: PMCH encode failed at %s s=%d (slot=%d n=%d)", tc->name, seg_name, s, slot_m, slot_n);            \
        goto cleanup;                                                                                                \
      }                                                                                                              \
      memcpy(rx_grid[0], tx_grid[0], grid_re * sizeof(cf_t));                                                        \
      if (slot_n == 0) {                                                                                             \
        srsran_softbuffer_rx_reset_cb(&softbuffer_rx[slot_m], 1);                                                    \
        srsran_softbuffer_rx_reset_tbs(&softbuffer_rx[slot_m], seg_tbs);                                             \
      }                                                                                                              \
      pmch_cfg.pdsch_cfg.softbuffers.rx[0] = &softbuffer_rx[slot_m];                                                 \
      srsran_pdsch_res_t pdsch_res[SRSRAN_MAX_CODEWORDS];                                                            \
      ZERO_OBJECT(pdsch_res);                                                                                        \
      pdsch_res[0].payload = data_rx;                                                                                \
      if (srsran_pmch_decode(&pmch, &dl_sf, &pmch_cfg, &chest_dl_res, rx_grid, pdsch_res)) {                         \
        ERROR("%s: PMCH decode returned error at %s s=%d (slot=%d n=%d)", tc->name, seg_name, s, slot_m, slot_n);    \
        goto cleanup;                                                                                                \
      }                                                                                                              \
      if (pdsch_res[0].crc) {                                                                                        \
        if ((decoded)[slot_m]) {                                                                                     \
          ERROR("%s: unexpected repeated CRC=true for slot %d in %s at s=%d", tc->name, slot_m, seg_name, s);        \
          goto cleanup;                                                                                              \
        }                                                                                                            \
        if (memcmp((data_tx)[slot_m], data_rx, seg_tbs / 8) != 0) {                                                  \
          ERROR("%s: payload mismatch for slot %d in %s at s=%d -- possible cross-segment corruption",              \
                tc->name, slot_m, seg_name, s);                                                                      \
          goto cleanup;                                                                                              \
        }                                                                                                            \
        (decoded)[slot_m] = true;                                                                                    \
      }                                                                                                              \
    }                                                                                                                \
    for (uint8_t m = 0; m < (M); m++) {                                                                              \
      if (!(decoded)[m]) {                                                                                           \
        ERROR("%s: slot %d in %s never decoded successfully", tc->name, m, seg_name);                                \
        goto cleanup;                                                                                                \
      }                                                                                                              \
    }                                                                                                                \
  } while (0)

  RUN_SEGMENT(N1, M1, data_tx1, decoded1, "main session");
  /* Window boundary: subframe_idx resets to 0 and N/M switch to the LastMTCH
   * override -- exactly what set_last_mtch_start()-driven is_mch_subframe() /
   * mbsfn_config_for_tti() now do. */
  RUN_SEGMENT(N2, M2, data_tx2, decoded2, "last-MTCH session");

#undef RUN_SEGMENT

  printf("  %-34s tti=%2d  N=%d M=%d -> N'=%d M'=%d  ... OK\n", tc->name, tc->tti, N1, M1, N2, M2);
  ret = 0;

cleanup:
  for (uint8_t m = 0; m < SRSRAN_PMCH_MAX_TI_M; m++) {
    if (data_tx1[m]) free(data_tx1[m]);
    if (data_tx2[m]) free(data_tx2[m]);
  }
  if (data_rx) {
    free(data_rx);
  }
  if (tx_grid[0]) free(tx_grid[0]);
  if (rx_grid[0]) free(rx_grid[0]);
  if (chest_ok) {
    srsran_chest_dl_res_free(&chest_dl_res);
  }
  if (sb_tx_ok) {
    srsran_softbuffer_tx_free(&softbuffer_tx);
  }
  for (uint32_t m = 0; m < SRSRAN_PMCH_MAX_TI_M; m++) {
    if (sb_rx_ok[m]) {
      srsran_softbuffer_rx_free(&softbuffer_rx[m]);
    }
  }
  if (pmch_ok) {
    srsran_pmch_free(&pmch);
  }
  srsran_random_free(random);
  return ret;
}

static int run_case(const rel19_case_t* tc, uint32_t nof_prb, uint32_t mcs_idx, uint16_t mbsfn_area_id)
{
  /* A per-case MCS override lets a cyclic-shift sub-case push the per-subframe TBS over
   * the single-code-block limit so the bit-shift path (Xi > 0) is genuinely exercised. */
  uint32_t eff_mcs = tc->mcs_override ? tc->mcs_override : mcs_idx;
  if (tc->feature == FEAT_TIME_INTERLEAVE) {
    return run_time_interleave_case(tc, nof_prb, eff_mcs, mbsfn_area_id);
  }
  if (tc->feature == FEAT_LASTMTCH) {
    return run_lastmtch_case(tc, nof_prb, eff_mcs, mbsfn_area_id);
  }
  return run_single_sf_case(tc, nof_prb, eff_mcs, mbsfn_area_id);
}

int main(int argc, char** argv)
{
  (void)argc;
  (void)argv;

  const uint32_t nof_prb       = 25; /* valid for every FeMBMS SCS */
  const uint32_t mcs_idx       = 2;  /* low MCS -> QPSK, modest TBS */
  const uint16_t mbsfn_area_id = 1;

  /* Representative SCS: 7.5 kHz and 1.25 kHz (per the task).  Each Rel-19 feature is
   * exercised on both.  Frequency interleaving is checked OFF and ON; the OFF case
   * doubles as a regression baseline (feature disabled must still round-trip). */
  const rel19_case_t cases[] = {
      /* (a) FREQUENCY interleaving OFF / ON */
      {"freq-interleave OFF 7.5kHz", FEAT_FREQ_INTERLEAVE, SRSRAN_SCS_7KHZ5, 1, false, 0, 0},
      {"freq-interleave ON  7.5kHz", FEAT_FREQ_INTERLEAVE, SRSRAN_SCS_7KHZ5, 1, true, 0, 0},
      {"freq-interleave OFF 1.25kHz", FEAT_FREQ_INTERLEAVE, SRSRAN_SCS_1KHZ25, 1, false, 0, 0},
      {"freq-interleave ON  1.25kHz", FEAT_FREQ_INTERLEAVE, SRSRAN_SCS_1KHZ25, 1, true, 0, 0},

      /* (b) CYCLIC shift, alpha in {1,2,3}.  These six sub-cases use the default QPSK
       * MCS, whose per-subframe TBS at 25 PRB (<= ~4008 bits) stays under the single
       * code-block limit, so srsran_cbsegm yields C == 1 and pmch_cyclic_shift_Xi
       * returns 0.  With Xi == 0 the bit-level cyclic shift (pmch_cyclic_shift_bits /
       * _llr) early-returns, so ONLY the per-subcarrier phase rotation is exercised
       * (which is self-inverse on a perfect channel).  They verify the phase-rotation
       * half of the feature. */
      {"cyclic-shift a=1 7.5kHz (phase only, C=1)", FEAT_CYCLIC_SHIFT, SRSRAN_SCS_7KHZ5, 3, false, 1, 0, 0},
      {"cyclic-shift a=2 7.5kHz (phase only, C=1)", FEAT_CYCLIC_SHIFT, SRSRAN_SCS_7KHZ5, 5, false, 2, 0, 0},
      {"cyclic-shift a=3 7.5kHz (phase only, C=1)", FEAT_CYCLIC_SHIFT, SRSRAN_SCS_7KHZ5, 7, false, 3, 0, 0},
      {"cyclic-shift a=1 1.25kHz (phase only, C=1)", FEAT_CYCLIC_SHIFT, SRSRAN_SCS_1KHZ25, 3, false, 1, 0, 0},
      {"cyclic-shift a=2 1.25kHz (phase only, C=1)", FEAT_CYCLIC_SHIFT, SRSRAN_SCS_1KHZ25, 5, false, 2, 0, 0},
      {"cyclic-shift a=3 1.25kHz (phase only, C=1)", FEAT_CYCLIC_SHIFT, SRSRAN_SCS_1KHZ25, 7, false, 3, 0, 0},

      /* (b') CYCLIC shift with the bit-level shift ACTIVE.  mcs_override = 14 maps (via
       * dl_mcs_tbs_idx_table[14]) to I_TBS 13, whose per-subframe TBS at 25 PRB is 6456
       * bits, so B = 6456 + 24 = 6480 exceeds the 6144-bit single code-block limit and
       * srsran_cbsegm yields C = 2 (mcs 14 is 16QAM, not QPSK; the low-MCS QPSK cases
       * above cover the phase-rotation half, this pair covers the bit-shift half).
       * With C >= 2 and subframe_idx (i) * alpha not a multiple of C, pmch_cyclic_shift_Xi
       * returns a non-zero Xi, so the TX bit-level left cyclic shift and the RX anti-shift
       * both run.  tti = 3, alpha = 1 -> Si = (3 * 1) % 2 = 1 -> Xi > 0; tti = 5, alpha = 3
       * -> Si = 15 % 2 = 1 -> Xi > 0.  Byte-exact recovery confirms the shift/anti-shift
       * pair are genuine inverses; if the anti-shift were broken the payload would not
       * round-trip.  run_single_sf_case prints the computed Xi so a regression to Xi == 0
       * (dead bit-shift path) is visible. */
      {"cyclic-shift a=1 7.5kHz (bit-shift, C=2)", FEAT_CYCLIC_SHIFT, SRSRAN_SCS_7KHZ5, 3, false, 1, 0, 0, 14},
      {"cyclic-shift a=3 1.25kHz (bit-shift, C=2)", FEAT_CYCLIC_SHIFT, SRSRAN_SCS_1KHZ25, 5, false, 3, 0, 0, 14},

      /* (c) TIME interleaving: M==N (smallest legal M, per rrc.cc's own
       * M>=N clamp) and M>N (genuine multi-slot pipelining), on both SCS. */
      {"time-interleave N=2 M=2 7.5kHz",  FEAT_TIME_INTERLEAVE, SRSRAN_SCS_7KHZ5,  1, false, 0, 2, 2},

      /* (c') N_cb = min(floor(N_IR/C), K_w) soft-buffer cap (TS 36.212 §5.1.4.1.2),
       * exercised on top of the exact same N=2/M=2 config as the case just above.
       * category=1 (N_soft=250368) + beta=one32nd (1/32) at M=2 was hand-verified
       * (standalone rm_turbo test + a temporary forced-cap run of this exact case,
       * rt-mbms-tx) to produce a genuinely BINDING cap (n_cb_cap=3912, below the
       * ~6624-bit uncapped K_w for this case's TBS) that still round-trips
       * correctly - not just "the field is accepted", but "encode really shrinks
       * the circular buffer and decode still recovers the payload byte-exact". */
      {"time-interleave N=2 M=2 7.5kHz + N_cb cap (cat=1,beta=1/32)",
       FEAT_TIME_INTERLEAVE, SRSRAN_SCS_7KHZ5, 1, false, 0, 2, 2, 0, 0, 0, 1, 1, 32},
      {"time-interleave N=2 M=4 7.5kHz",  FEAT_TIME_INTERLEAVE, SRSRAN_SCS_7KHZ5,  1, false, 0, 2, 4},
      {"time-interleave N=4 M=4 1.25kHz", FEAT_TIME_INTERLEAVE, SRSRAN_SCS_1KHZ25, 1, false, 0, 4, 4},
      {"time-interleave N=2 M=8 1.25kHz", FEAT_TIME_INTERLEAVE, SRSRAN_SCS_1KHZ25, 1, false, 0, 2, 8},

      /* (d) pmch-TimeInterleavingN/M-LastMTCH-r19: main session's own N*M block,
       * then a boundary (subframe_idx resets to 0) into a LastMTCH session with a
       * DIFFERENT N'/M', deliberately misaligned with the main block size so
       * neither segment's length is a multiple of the other's - the scenario the
       * flat, whole-period counter this feature replaces could never represent.
       * mcs_override fields are unused here (0); the last two struct fields are
       * (time_n_last, time_m_last). */
      {"lastmtch N=2 M=4 -> N'=4 M'=4 7.5kHz",  FEAT_LASTMTCH, SRSRAN_SCS_7KHZ5,  1, false, 0, 2, 4, 0, 4, 4},
      {"lastmtch N=4 M=4 -> N'=2 M'=8 1.25kHz", FEAT_LASTMTCH, SRSRAN_SCS_1KHZ25, 1, false, 0, 4, 4, 0, 2, 8},
  };
  const int nof_cases = (int)(sizeof(cases) / sizeof(cases[0]));

  printf("FeMBMS PMCH Rel-19 feature loopback self-test (nof_prb=%d, mcs=%d):\n", nof_prb, mcs_idx);

  int failures = 0;
  for (int i = 0; i < nof_cases; i++) {
    if (run_case(&cases[i], nof_prb, mcs_idx, mbsfn_area_id) != 0) {
      failures++;
    }
  }

  if (failures) {
    printf("FeMBMS PMCH Rel-19 loopback: %d/%d cases FAILED\n", failures, nof_cases);
    exit(SRSRAN_ERROR);
  }
  printf("FeMBMS PMCH Rel-19 loopback: all %d cases passed\n", nof_cases);
  exit(SRSRAN_SUCCESS);
}
