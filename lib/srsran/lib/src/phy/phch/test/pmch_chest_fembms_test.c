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

/* FeMBMS PMCH real (non-identity) channel-estimation round-trip self-test.
 *
 * This is the highest-value FeMBMS PMCH test: unlike pmch_fembms_test.c
 * (frequency-domain identity copy) and pmch_ofdm_fembms_test.c (OFDM round
 * trip but STILL identity channel estimate), this test does NOT call
 * srsran_chest_dl_res_set_identity.  Instead it acts as a mini eNB + UE and
 * runs the real downlink MBSFN channel estimator, exercising the code paths
 * the other two tests bypass:
 *
 *   TX (eNB):
 *     srsran_pmch_encode            -> PMCH data into the freq grid (RS holes)
 *     srsran_refsignal_mbsfn_put_sf -> inject MBSFN RS pilots into the SAME grid
 *     srsran_ofdm_tx_sf             -> IFFT -> time samples
 *   channel:
 *     passthrough copy              -> unity (perfect) channel
 *   RX (UE):
 *     srsran_ofdm_rx_sf             -> FFT -> received freq grid
 *     srsran_chest_dl_estimate_cfg  -> real estimator (estimate_port_mbsfn),
 *                                      reads pilots via refsignal_mbsfn_get_sf,
 *                                      least-squares + interpolate -> ce ~ 1.0
 *     srsran_pmch_decode            -> equalize with the estimated ce, then
 *                                      byte-exact compare + CRC check
 *
 * Why the RS injection matters: srsran_pmch_encode writes ONLY PMCH data,
 * leaving the reference-signal REs as holes.  In production the eNB places the
 * MBSFN RS separately (enb_dl.c put_refs -> srsran_refsignal_mbsfn_put_sf).  The
 * two existing tests get away without this because set_identity fabricates the
 * channel estimate directly; a real-estimator test MUST put the pilots into the
 * grid, or the estimator finds only zeros and ce is garbage.
 *
 * Estimator config (verified): the MBSFN path rejects the zero-init defaults
 * (ESTIMATOR_ALG_AVERAGE + NOISE_ALG_REFS), so this test uses
 * srsran_chest_dl_estimate_cfg with estimator_alg = SRSRAN_ESTIMATOR_ALG_INTERPOLATE,
 * filter_type = SRSRAN_CHEST_FILTER_NONE (set explicitly; the enum's zero value is
 * GAUSS, so a zero-init cfg would silently take the Gauss-smoothing branch instead of
 * the filter-free least-squares + interpolate path) and mbsfn_area_id set.  With a
 * unity channel the least-squares estimate on the unit-magnitude MBSFN pilots is ~1.0
 * across the grid, so PMCH equalization is lossless and the payload round-trips
 * byte-exact.
 *
 * Buffer sizing (nof_prb=25, CP_EXT), per the verified sizing analysis:
 *   - freq grids: nof_prb * TEST_MAX_PMCH_RE(SRSRAN_SCS_370HZ) = 25*486 = 12150 cf_t
 *                 (0.37 kHz is the largest RE/PRB grid; fits every SCS).
 *   - time grids: SRSRAN_SF_LEN(srsran_symbol_sz_scs(nof_prb, SRSRAN_SCS_370HZ))
 *                 (0.37 kHz is the largest symbol_sz; set_prb_scs never resizes the
 *                 caller buffer, so pre-size for the largest SCS once).
 *
 * The 0.37 kHz pilot table index is the SCS-aware sf_idx = (tti%40-1)/3 (13 slots
 * of 3 ms over a 40 ms period), NOT tti%10 which every other SCS uses; this test
 * derives it exactly as enb_dl.c put_refs does.
 *
 * Softbuffers are sized by nof_prb only, exactly as the production decoder does.
 *
 * CAVEAT (0.37 kHz): as in pmch_ofdm_fembms_test.c, srsRAN models one 0.37 kHz
 * symbol (3 ms in TS 36.211) as a single OFDM symbol inside the 1 ms subframe
 * window.  The IFFT->FFT round trip and the RS put/get are self-consistent for
 * this simplified 1 ms model, so a pass confirms model + estimator consistency,
 * not full TS 36.211 0.37 kHz support.  These cases are tagged "(1ms model)".
 */

#include <srsran/phy/utils/random.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "srsran/srsran.h"

/* Mirror of MAX_PMCH_RE() in pmch.c, built from the public phy_common.h macros.
 * Max data+RS resource elements per PRB in one MBSFN subframe for the given SCS. */
#define TEST_MAX_PMCH_RE(scs) (SRSRAN_MBSFN_NOF_SLOTS(scs) * SRSRAN_MBSFN_NOF_SYMBOLS(scs) * SRSRAN_NRE_SCS(scs))

/* Time-domain subframe length (in cf_t samples) for a given nof_prb and SCS.
 * SCS-aware, unlike SRSRAN_SF_LEN_PRB() which locks symbol_sz to 15 kHz. */
#define TEST_SF_LEN_SCS(nof_prb, scs) ((uint32_t)SRSRAN_SF_LEN(srsran_symbol_sz_scs((nof_prb), (scs))))

typedef struct {
  const char*  name;
  srsran_scs_t scs;
  uint32_t     tti; /* picks the 0.37 kHz pilot table index / stagger; other SCS: any */
} fembms_case_t;

/* Pilot-table index into q->pilots[port][sf_idx], SCS-aware, matching
 * enb_dl.c put_refs():
 *   0.37 kHz: 40 ms period, 13 slots of 3 ms -> sf_idx = (tti%40 - 1)/3 (0..12).
 *   all other SCS: 10 ms period -> sf_idx = tti % 10. */
static uint32_t mbsfn_pilot_sf_idx(srsran_scs_t scs, uint32_t tti)
{
  if (SRSRAN_SCS_IS_370HZ(scs)) {
    uint32_t pos40 = tti % 40u;
    return (pos40 > 0u) ? (pos40 - 1u) / 3u : 0u;
  }
  return tti % 10u;
}

/* One eNB-encode/RS-inject/IFFT -> unity channel -> FFT/real-estimate/decode round
 * trip.  Returns 0 on byte-exact recovery with CRC OK, -1 otherwise.  Allocates and
 * frees ALL of its own per-case state (fresh OFDM, refsignal and chest objects) so a
 * failure in one case cannot corrupt the next, and re-zeroes every buffer up front. */
static int run_case(const fembms_case_t* tc, uint32_t nof_prb, uint32_t mcs_idx, uint16_t mbsfn_area_id)
{
  int             ret    = -1;
  srsran_random_t random = srsran_random_init(tc->tti + 1);

  srsran_cell_t cell   = {0};
  cell.nof_prb         = nof_prb;
  cell.nof_ports       = 1;
  cell.id              = 1;
  cell.cp              = SRSRAN_CP_EXT; /* all MBSFN subframes use extended CP */
  cell.phich_length    = SRSRAN_PHICH_NORM;
  cell.phich_resources = SRSRAN_PHICH_R_1_6;
  cell.frame_type      = SRSRAN_FDD;
  /* mbsfn_prb left 0 -> RS spans all nof_prb, matching full-carrier PMCH data. */

  srsran_dl_sf_cfg_t dl_sf;
  ZERO_OBJECT(dl_sf);
  dl_sf.cfi                = 0; /* FeMBMS dedicated carrier: no control region */
  dl_sf.tti                = tc->tti;
  dl_sf.sf_type            = SRSRAN_SF_MBSFN;
  dl_sf.subcarrier_spacing = tc->scs;

  /* Freq grid: largest FeMBMS RE/PRB is 0.37 kHz (486).  One size fits every SCS. */
  const uint32_t grid_re = nof_prb * TEST_MAX_PMCH_RE(SRSRAN_SCS_370HZ);
  /* Time grid: largest FeMBMS symbol_sz is 0.37 kHz.  set_prb_scs never resizes the
   * caller buffer, so pre-size for the largest SCS once. */
  const uint32_t sf_len_max  = TEST_SF_LEN_SCS(nof_prb, SRSRAN_SCS_370HZ);
  /* Time samples actually produced/consumed for THIS case's SCS (combine bound). */
  const uint32_t sf_len_case = TEST_SF_LEN_SCS(nof_prb, tc->scs);

  cf_t*                  tx_slot_symbols = NULL; /* freq: encoder out + RS -> IFFT in */
  cf_t*                  rx_slot_symbols = NULL; /* freq: FFT out -> estimator/decoder in */
  cf_t*                  tx_sf_symbols   = NULL; /* time: IFFT out                 */
  cf_t*                  rx_sf_symbols   = NULL; /* time: FFT in                   */
  uint8_t*               data_tx         = NULL;
  uint8_t*               data_rx         = NULL;
  srsran_softbuffer_tx_t softbuffer_tx   = {0};
  srsran_softbuffer_rx_t softbuffer_rx   = {0};
  srsran_chest_dl_res_t  chest_dl_res    = {0};
  srsran_chest_dl_t      chest_dl        = {0};
  srsran_pmch_t          pmch            = {0};
  srsran_ofdm_t          ifft_mbsfn      = {0};
  srsran_ofdm_t          fft_mbsfn       = {0};
  srsran_refsignal_t     mbsfn_refs      = {0}; /* MBSFN RS generator (eNB side)   */
  srsran_refsignal_t     cs_refs         = {0}; /* CS RS; only read on 15 kHz path */
  bool pmch_ok = false, sb_tx_ok = false, sb_rx_ok = false, chest_res_ok = false, chest_ok = false;
  bool ifft_ok = false, fft_ok = false, mbsfn_refs_ok = false, cs_refs_ok = false;

  /* --- Allocate freq + time buffers and zero them --- */
  tx_slot_symbols = srsran_vec_cf_malloc(grid_re);
  rx_slot_symbols = srsran_vec_cf_malloc(grid_re);
  tx_sf_symbols   = srsran_vec_cf_malloc(sf_len_max);
  rx_sf_symbols   = srsran_vec_cf_malloc(sf_len_max);
  if (!tx_slot_symbols || !rx_slot_symbols || !tx_sf_symbols || !rx_sf_symbols) {
    ERROR("%s: error allocating resource grids", tc->name);
    goto cleanup;
  }
  srsran_vec_cf_zero(tx_slot_symbols, grid_re);
  srsran_vec_cf_zero(rx_slot_symbols, grid_re);
  srsran_vec_cf_zero(tx_sf_symbols, sf_len_max);
  srsran_vec_cf_zero(rx_sf_symbols, sf_len_max);

  /* --- Channel-estimate result buffers (NOT set_identity; filled by the estimator) --- */
  if (srsran_chest_dl_res_init(&chest_dl_res, nof_prb)) {
    ERROR("%s: error initialising chest_dl_res", tc->name);
    goto cleanup;
  }
  chest_res_ok = true;

  /* --- Real downlink channel estimator (UE side).  init sizes buffers for the
   * widest FeMBMS SCS (0.37 kHz); set_cell then set_mbsfn_area_id (order matters:
   * the latter reads q->cell.nof_prb and lazily builds mbsfn_refs[area_id]). --- */
  if (srsran_chest_dl_init(&chest_dl, nof_prb, 1)) {
    ERROR("%s: error initialising chest_dl", tc->name);
    goto cleanup;
  }
  chest_ok = true;
  if (srsran_chest_dl_set_cell(&chest_dl, cell)) {
    ERROR("%s: error setting chest_dl cell", tc->name);
    goto cleanup;
  }
  if (srsran_chest_dl_set_mbsfn_area_id(&chest_dl, mbsfn_area_id, tc->scs)) {
    ERROR("%s: error setting chest_dl MBSFN area id", tc->name);
    goto cleanup;
  }

  /* --- MBSFN reference-signal generator (eNB side).  Allocate with the densest SCS
   * (0.37 kHz SL2, 81 pilots/RB), exactly as enb_dl does, so the pilot table is big
   * enough for any SCS; then generate the sequence for the actual case SCS. --- */
  if (srsran_refsignal_mbsfn_init(&mbsfn_refs, nof_prb, SRSRAN_SCS_370HZ_SL2)) {
    ERROR("%s: error initialising MBSFN refsignal", tc->name);
    goto cleanup;
  }
  mbsfn_refs_ok = true;
  if (srsran_refsignal_mbsfn_set_cell(&mbsfn_refs, cell, mbsfn_area_id, tc->scs)) {
    ERROR("%s: error generating MBSFN refsignal sequence", tc->name);
    goto cleanup;
  }

  /* --- CS reference-signal generator.  The 4th arg (cs_pilots) to put_sf must be
   * non-NULL even for pure FeMBMS SCS (validated but only read on the 15 kHz path);
   * a properly generated CS RS buffer satisfies the check for any SCS. --- */
  if (srsran_refsignal_cs_init(&cs_refs, nof_prb)) {
    ERROR("%s: error initialising CS refsignal", tc->name);
    goto cleanup;
  }
  cs_refs_ok = true;
  if (srsran_refsignal_cs_set_cell(&cs_refs, cell)) {
    ERROR("%s: error generating CS refsignal sequence", tc->name);
    goto cleanup;
  }

  /* --- Softbuffers (sized by nof_prb only, as production does) --- */
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

  /* --- PMCH --- */
  if (srsran_pmch_init(&pmch, nof_prb, 1)) {
    ERROR("%s: error initialising PMCH", tc->name);
    goto cleanup;
  }
  pmch_ok = true;
  if (srsran_pmch_set_area_id(&pmch, mbsfn_area_id)) {
    ERROR("%s: error setting PMCH area id", tc->name);
    goto cleanup;
  }

  /* --- OFDM objects.  init_mbsfn builds a 15 kHz object bound to the caller
   * time/freq buffers; set_prb_scs re-binds the numerology to tc->scs, preserving
   * the caller buffers.  Buffers are pre-sized for the largest SCS, so switching to
   * any SCS (incl. 0.37 kHz) is safe.  KNOWN PITFALL: after set_prb_scs, never call
   * srsran_ofdm_set_non_mbsfn_region() for FeMBMS — set_prb_scs installs
   * non_mbsfn_region = -1 (the dedicated-carrier sentinel); the unconditional setter
   * would clobber it, giving wrong CP on TX and a heap OOB second-slot write on RX.
   * Only set_normalize(true). --- */
  if (srsran_ofdm_tx_init_mbsfn(&ifft_mbsfn, SRSRAN_CP_EXT, tx_slot_symbols, tx_sf_symbols, nof_prb)) {
    ERROR("%s: error creating IFFT object", tc->name);
    goto cleanup;
  }
  ifft_ok = true;
  if (srsran_ofdm_tx_set_prb_scs(&ifft_mbsfn, SRSRAN_CP_EXT, nof_prb, tc->scs)) {
    ERROR("%s: error setting IFFT SCS", tc->name);
    goto cleanup;
  }
  srsran_ofdm_set_normalize(&ifft_mbsfn, true);

  if (srsran_ofdm_rx_init_mbsfn(&fft_mbsfn, SRSRAN_CP_EXT, rx_sf_symbols, rx_slot_symbols, nof_prb)) {
    ERROR("%s: error creating FFT object", tc->name);
    goto cleanup;
  }
  fft_ok = true;
  if (srsran_ofdm_rx_set_prb_scs(&fft_mbsfn, SRSRAN_CP_EXT, nof_prb, tc->scs)) {
    ERROR("%s: error setting FFT SCS", tc->name);
    goto cleanup;
  }
  srsran_ofdm_set_normalize(&fft_mbsfn, true);

  /* --- Build the PMCH grant (directly, as the FeMBMS loopback template does) --- */
  srsran_pmch_cfg_t pmch_cfg;
  ZERO_OBJECT(pmch_cfg);
  pmch_cfg.area_id                     = mbsfn_area_id;
  pmch_cfg.pdsch_cfg.softbuffers.tx[0] = &softbuffer_tx;

  srsran_dci_dl_t dci;
  ZERO_OBJECT(dci);
  dci.rnti                    = SRSRAN_MRNTI;
  dci.format                  = SRSRAN_DCI_FORMAT1;
  dci.alloc_type              = SRSRAN_RA_ALLOC_TYPE0;
  dci.type0_alloc.rbg_bitmask = 0xffffffff;
  dci.tb[0].mcs_idx           = mcs_idx;
  SRSRAN_DCI_TB_DISABLE(dci.tb[1]);

  if (srsran_ra_dl_dci_to_grant(&cell, &dl_sf, SRSRAN_TM1, false, &dci, &pmch_cfg.pdsch_cfg.grant)) {
    ERROR("%s: error building grant", tc->name);
    goto cleanup;
  }

  uint32_t tbs    = (uint32_t)pmch_cfg.pdsch_cfg.grant.tb[0].tbs;
  uint32_t nof_re = pmch_cfg.pdsch_cfg.grant.nof_re;
  if (tbs == 0 || nof_re == 0 || nof_re > grid_re) {
    ERROR("%s: implausible grant tbs=%d nof_re=%d (grid_re=%d)", tc->name, tbs, nof_re, grid_re);
    goto cleanup;
  }
  if (sf_len_case == 0 || sf_len_case > sf_len_max) {
    ERROR("%s: implausible sf_len_case=%d (sf_len_max=%d)", tc->name, sf_len_case, sf_len_max);
    goto cleanup;
  }

  /* --- Payload --- */
  data_tx = srsran_vec_u8_malloc(tbs / 8);
  /* decode's output buffer contract is tbs+24 bits (payload + TB-CRC24,
   * decoded in-place and re-verified there), not tbs bits - see sch.c's
   * decode_tb_cb / decode_tb. tbs/8 alone under-allocates by 3 bytes. */
  data_rx = srsran_vec_u8_malloc((tbs + 24 + 7) / 8);
  if (!data_tx || !data_rx) {
    ERROR("%s: error allocating payload buffers", tc->name);
    goto cleanup;
  }
  for (uint32_t i = 0; i < tbs / 8; i++) {
    data_tx[i] = (uint8_t)srsran_random_uniform_int_dist(random, 0, 255);
  }

  /* --- Encode: PMCH data into the freq grid (leaves RS holes) --- */
  cf_t* tx_slots[SRSRAN_MAX_PORTS] = {0};
  tx_slots[0]                      = tx_slot_symbols;
  if (srsran_pmch_encode(&pmch, &dl_sf, &pmch_cfg, data_tx, tx_slots)) {
    ERROR("%s: PMCH encode failed", tc->name);
    goto cleanup;
  }

  /* --- Inject the MBSFN reference signals into the SAME grid (eNB put_refs).  PMCH
   * data and MBSFN RS occupy disjoint REs, so no separate buffer is needed.  Use the
   * SCS-aware pilot-table index; cs_pilots is passed non-NULL to satisfy the check. --- */
  uint32_t pilot_sf_idx = mbsfn_pilot_sf_idx(tc->scs, tc->tti);
  if (srsran_refsignal_mbsfn_put_sf(cell,
                                    0,
                                    cs_refs.pilots[0][tc->tti % 10u],
                                    mbsfn_refs.pilots[0][pilot_sf_idx],
                                    tx_slot_symbols,
                                    tc->scs,
                                    tc->tti)) {
    ERROR("%s: MBSFN RS put_sf failed", tc->name);
    goto cleanup;
  }

  /* --- IFFT: freq grid (data + RS) -> time samples --- */
  srsran_ofdm_tx_sf(&ifft_mbsfn);

  /* --- Unity (perfect) channel: received time samples equal transmitted --- */
  for (uint32_t k = 0; k < sf_len_case; k++) {
    rx_sf_symbols[k] = tx_sf_symbols[k];
  }

  /* --- FFT: time samples -> received freq grid --- */
  srsran_ofdm_rx_sf(&fft_mbsfn);

  /* --- Real channel estimate.  The MBSFN path rejects the zero-init cfg defaults
   * (ESTIMATOR_ALG_AVERAGE + NOISE_ALG_REFS), so use estimate_cfg with INTERPOLATE
   * and the matching mbsfn_area_id.  filter_type is set to SRSRAN_CHEST_FILTER_NONE
   * EXPLICITLY: the enum's zero value is SRSRAN_CHEST_FILTER_GAUSS, so a ZERO_OBJECT
   * cfg would take the Gauss-smoothing branch in chest_interpolate_noise_est (with a
   * degenerate width, since MBSFN never computes a REFS noise estimate) instead of the
   * filter-free interpolate_pilots path documented here.  With NONE the estimator runs
   * a direct least-squares + interpolate on the raw pilot estimates; on the unity
   * channel the unit-magnitude MBSFN pilots give ce ~1.0 across the grid, so PMCH
   * equalization is lossless.  Setting it explicitly documents intent and is robust to
   * future enum reordering. --- */
  srsran_chest_dl_cfg_t chest_cfg;
  ZERO_OBJECT(chest_cfg);
  chest_cfg.estimator_alg = SRSRAN_ESTIMATOR_ALG_INTERPOLATE;
  chest_cfg.filter_type   = SRSRAN_CHEST_FILTER_NONE;
  chest_cfg.mbsfn_area_id = mbsfn_area_id;

  cf_t* rx_slots[SRSRAN_MAX_PORTS] = {0};
  rx_slots[0]                      = rx_slot_symbols;

  if (srsran_chest_dl_estimate_cfg(&chest_dl, &dl_sf, &chest_cfg, rx_slots, &chest_dl_res)) {
    ERROR("%s: real channel estimation failed", tc->name);
    goto cleanup;
  }

  /* --- Decode using the ESTIMATED channel (not an identity) --- */
  srsran_softbuffer_rx_reset_tbs(&softbuffer_rx, tbs);
  pmch_cfg.pdsch_cfg.softbuffers.rx[0] = &softbuffer_rx;

  srsran_pdsch_res_t pdsch_res[SRSRAN_MAX_CODEWORDS];
  ZERO_OBJECT(pdsch_res);
  pdsch_res[0].payload = data_rx;

  if (srsran_pmch_decode(&pmch, &dl_sf, &pmch_cfg, &chest_dl_res, rx_slots, pdsch_res)) {
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

  printf("  %-24s tti=%2d  nof_re=%6d  tbs=%5d bits  sf_len=%7d  noise=%.2e  ... OK\n",
         tc->name,
         tc->tti,
         nof_re,
         tbs,
         sf_len_case,
         chest_dl_res.noise_estimate);
  ret = 0;

cleanup:
  if (data_tx) {
    free(data_tx);
  }
  if (data_rx) {
    free(data_rx);
  }
  if (ifft_ok) {
    srsran_ofdm_tx_free(&ifft_mbsfn);
  }
  if (fft_ok) {
    srsran_ofdm_rx_free(&fft_mbsfn);
  }
  if (tx_slot_symbols) {
    free(tx_slot_symbols);
  }
  if (rx_slot_symbols) {
    free(rx_slot_symbols);
  }
  if (tx_sf_symbols) {
    free(tx_sf_symbols);
  }
  if (rx_sf_symbols) {
    free(rx_sf_symbols);
  }
  if (mbsfn_refs_ok) {
    srsran_refsignal_free(&mbsfn_refs);
  }
  if (cs_refs_ok) {
    srsran_refsignal_free(&cs_refs);
  }
  if (chest_ok) {
    srsran_chest_dl_free(&chest_dl);
  }
  if (chest_res_ok) {
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

int main(int argc, char** argv)
{
  (void)argc;
  (void)argv;

  const uint32_t nof_prb       = 25; /* valid for every FeMBMS SCS */
  const uint32_t mcs_idx       = 2;  /* low MCS -> QPSK, modest TBS */
  const uint16_t mbsfn_area_id = 1;

  /* For 0.37 kHz SL4 the pilot-table index is sf_idx = (tti%40-1)/3 and the grid
   * stagger is 3*((tti/3)%4).  Sweep tti = 1,4,7,10 so the estimator reads four
   * distinct pilot slots / staggers; the put_sf and refsignal_mbsfn_get_sf paths
   * inside the estimator must agree for the round trip to close. */
  const fembms_case_t cases[] = {
      {"7.5 kHz", SRSRAN_SCS_7KHZ5, 1},
      {"2.5 kHz", SRSRAN_SCS_2KHZ5, 1},
      {"1.25 kHz", SRSRAN_SCS_1KHZ25, 1},
      {"0.37 SL2 (1ms model)", SRSRAN_SCS_370HZ_SL2, 1},
      {"0.37 SL4 (1ms model)", SRSRAN_SCS_370HZ_SL4, 1},
      {"0.37 SL4 (1ms model)", SRSRAN_SCS_370HZ_SL4, 4},
      {"0.37 SL4 (1ms model)", SRSRAN_SCS_370HZ_SL4, 7},
      {"0.37 SL4 (1ms model)", SRSRAN_SCS_370HZ_SL4, 10},
  };
  const int nof_cases = (int)(sizeof(cases) / sizeof(cases[0]));

  printf("FeMBMS PMCH real-channel-estimation round-trip self-test (nof_prb=%d, mcs=%d):\n", nof_prb, mcs_idx);

  int failures = 0;
  for (int i = 0; i < nof_cases; i++) {
    if (run_case(&cases[i], nof_prb, mcs_idx, mbsfn_area_id) != 0) {
      failures++;
    }
  }

  if (failures) {
    printf("FeMBMS PMCH real-CE round trip: %d/%d cases FAILED\n", failures, nof_cases);
    exit(SRSRAN_ERROR);
  }
  printf("FeMBMS PMCH real-CE round trip: all %d cases passed\n", nof_cases);
  exit(SRSRAN_SUCCESS);
}
