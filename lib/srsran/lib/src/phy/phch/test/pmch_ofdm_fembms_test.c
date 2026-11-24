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

/* FeMBMS PMCH OFDM time-domain loopback self-test.
 *
 * This is the time-domain counterpart of pmch_fembms_test.c.  Where that test
 * does a frequency-domain identity-channel copy, this one exercises the full
 * OFDM round trip per FeMBMS numerology:
 *
 *   srsran_pmch_encode  -> freq grid (tx_slot_symbols)
 *   srsran_ofdm_tx_sf   -> IFFT -> time samples (tx_sf_symbols)
 *   passthrough combine -> time samples (rx_sf_symbols)   [perfect channel]
 *   srsran_ofdm_rx_sf   -> FFT  -> freq grid (rx_slot_symbols)
 *   srsran_pmch_decode  -> payload, byte-exact compare + CRC check
 *
 * The existing pmch_test.c already does this chain but is hardcoded to 15 kHz in
 * its buffer-sizing macros (SRSRAN_NOF_RE / SRSRAN_SF_LEN_PRB) and never calls
 * the SCS setters.  This test makes every size SCS-aware and re-binds the OFDM
 * numerology with srsran_ofdm_tx/rx_set_prb_scs.
 *
 * Buffer sizing (nof_prb=25, CP_EXT), per the verified sizing analysis:
 *   - freq grids:  nof_prb * TEST_MAX_PMCH_RE(SRSRAN_SCS_370HZ) = 25*486 = 12150 cf_t
 *                  (0.37 kHz is the largest RE/PRB grid; fits every SCS).
 *   - time grids:  SRSRAN_SF_LEN(srsran_symbol_sz_scs(nof_prb, SRSRAN_SCS_370HZ))
 *                  = 31104*15 = 466560 cf_t (0.37 kHz is the largest symbol_sz;
 *                  the SCS setter does NOT resize caller buffers, so they are
 *                  pre-sized for the target/largest SCS once).
 *   - combine bound (per case, SCS-aware):
 *                  SRSRAN_SF_LEN(srsran_symbol_sz_scs(nof_prb, scs)) time samples.
 *
 * The softbuffer is sized by nof_prb only, exactly as the production decoder
 * (MbsfnFrameProcessor) does.
 *
 * CAVEAT (0.37 kHz): a true 0.37 kHz symbol spans 3 ms (92160 Ts) per TS 36.211,
 * but srsRAN models it as a single OFDM symbol inside the 1 ms subframe window
 * (1/3 of the real symbol).  The IFFT->FFT round trip is self-inverse for this
 * simplified 1 ms model, so the 0.37 kHz cases below verify model consistency, not
 * full TS 36.211 0.37 kHz support.  They are tagged "(1ms model)" in the output.
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
 * This is SCS-aware, unlike SRSRAN_SF_LEN_PRB() which locks symbol_sz to 15 kHz. */
#define TEST_SF_LEN_SCS(nof_prb, scs) ((uint32_t)SRSRAN_SF_LEN(srsran_symbol_sz_scs((nof_prb), (scs))))

typedef struct {
  const char*  name;
  srsran_scs_t scs;
  uint32_t     tti; /* picks the 0.37 kHz pilot stagger; irrelevant for other SCS */
} fembms_case_t;

/* One encode -> IFFT -> combine -> FFT -> decode round trip.  Returns 0 on
 * byte-exact recovery with CRC OK, -1 otherwise.  Allocates and frees its own
 * per-case state (including fresh OFDM objects) so a failure in one case cannot
 * corrupt the next, and re-zeroes all buffers up front. */
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

  srsran_dl_sf_cfg_t dl_sf;
  ZERO_OBJECT(dl_sf);
  dl_sf.cfi                = 0; /* FeMBMS dedicated carrier: no control region */
  dl_sf.tti                = tc->tti;
  dl_sf.sf_type            = SRSRAN_SF_MBSFN;
  dl_sf.subcarrier_spacing = tc->scs;

  /* Freq grid: largest FeMBMS RE/PRB is 0.37 kHz (486).  One size fits every SCS. */
  const uint32_t grid_re = nof_prb * TEST_MAX_PMCH_RE(SRSRAN_SCS_370HZ);
  /* Time grid: largest FeMBMS symbol_sz is 0.37 kHz (31104).  set_prb_scs never
   * resizes the caller buffer, so pre-size it for the largest SCS once. */
  const uint32_t sf_len_max = TEST_SF_LEN_SCS(nof_prb, SRSRAN_SCS_370HZ);
  /* Time samples actually produced/consumed for THIS case's SCS (combine bound). */
  const uint32_t sf_len_case = TEST_SF_LEN_SCS(nof_prb, tc->scs);

  cf_t*                  tx_slot_symbols = NULL; /* freq: encoder out / IFFT in   */
  cf_t*                  rx_slot_symbols = NULL; /* freq: FFT out / decoder in     */
  cf_t*                  tx_sf_symbols   = NULL; /* time: IFFT out                 */
  cf_t*                  rx_sf_symbols   = NULL; /* time: FFT in                   */
  uint8_t*               data_tx         = NULL;
  uint8_t*               data_rx         = NULL;
  srsran_softbuffer_tx_t softbuffer_tx   = {0};
  srsran_softbuffer_rx_t softbuffer_rx   = {0};
  srsran_chest_dl_res_t  chest_dl_res    = {0};
  srsran_pmch_t          pmch            = {0};
  srsran_ofdm_t          ifft_mbsfn      = {0};
  srsran_ofdm_t          fft_mbsfn       = {0};
  bool pmch_ok = false, sb_tx_ok = false, sb_rx_ok = false, chest_ok = false, ifft_ok = false, fft_ok = false;

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

  /* --- Perfect (identity) channel estimate --- */
  if (srsran_chest_dl_res_init(&chest_dl_res, nof_prb)) {
    ERROR("%s: error initialising chest_dl_res", tc->name);
    goto cleanup;
  }
  chest_ok = true;
  srsran_chest_dl_res_set_identity(&chest_dl_res);

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
   * time/freq buffers; set_prb_scs then re-binds the numerology to tc->scs,
   * preserving the caller in/out buffers and reallocating only internal tmp
   * if symbol_sz grew.  Buffers are already sized for the largest SCS, so the
   * switch to any SCS (including 0.37 kHz) is safe. --- */
  if (srsran_ofdm_tx_init_mbsfn(&ifft_mbsfn, SRSRAN_CP_EXT, tx_slot_symbols, tx_sf_symbols, nof_prb)) {
    ERROR("%s: error creating IFFT object", tc->name);
    goto cleanup;
  }
  ifft_ok = true;
  if (srsran_ofdm_tx_set_prb_scs(&ifft_mbsfn, SRSRAN_CP_EXT, nof_prb, tc->scs)) {
    ERROR("%s: error setting IFFT SCS", tc->name);
    goto cleanup;
  }
  /* Do NOT call srsran_ofdm_set_non_mbsfn_region() here.  For FeMBMS, set_prb_scs
   * (ofdm_init_mbsfn_) installs non_mbsfn_region=-1, the dedicated-carrier sentinel
   * that selects the SCS-specific CP for every symbol on TX and skips the 15 kHz-only
   * second-slot path on RX.  The setter is unconditional and would clobber -1 back to
   * a positive value (the setter takes uint8_t, so -1 cannot be passed through it),
   * corrupting the CP layout on TX
   * and triggering an out-of-bounds second-slot write on RX.  Unlike 15 kHz, FeMBMS SCS
   * have no normal->extended CP boundary, so the region must stay -1. */
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
  /* Same as the IFFT object above: leave non_mbsfn_region at the -1 sentinel that
   * set_prb_scs installed.  Forcing it to a positive value would make
   * srsran_ofdm_rx_sf run the 15 kHz-only ofdm_rx_slot(q, 1), an out-of-bounds heap
   * write into rx_slot_symbols for the 2.5/1.25/0.37 kHz cases. */
  srsran_ofdm_set_normalize(&fft_mbsfn, true);

  /* --- Build the PMCH grant --- */
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

  /* --- Encode (freq grid) --- */
  cf_t* tx_slots[SRSRAN_MAX_PORTS] = {0};
  tx_slots[0]                      = tx_slot_symbols;
  if (srsran_pmch_encode(&pmch, &dl_sf, &pmch_cfg, data_tx, tx_slots)) {
    ERROR("%s: PMCH encode failed", tc->name);
    goto cleanup;
  }

  /* --- IFFT: freq grid -> time samples --- */
  srsran_ofdm_tx_sf(&ifft_mbsfn);

  /* --- Passthrough combine (perfect channel) over this SCS's time samples --- */
  for (uint32_t k = 0; k < sf_len_case; k++) {
    rx_sf_symbols[k] = tx_sf_symbols[k];
  }

  /* --- FFT: time samples -> freq grid --- */
  srsran_ofdm_rx_sf(&fft_mbsfn);

  /* --- Decode --- */
  srsran_softbuffer_rx_reset_tbs(&softbuffer_rx, tbs);
  pmch_cfg.pdsch_cfg.softbuffers.rx[0] = &softbuffer_rx;

  cf_t* rx_slots[SRSRAN_MAX_PORTS] = {0};
  rx_slots[0]                      = rx_slot_symbols;

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

  printf("  %-22s tti=%2d  nof_re=%6d  tbs=%5d bits  sf_len=%7d  ... OK\n",
         tc->name,
         tc->tti,
         nof_re,
         tbs,
         sf_len_case);
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

int main(int argc, char** argv)
{
  (void)argc;
  (void)argv;

  const uint32_t nof_prb       = 25; /* valid for every FeMBMS SCS (0.37 kHz max is 75) */
  const uint32_t mcs_idx       = 2;  /* low MCS -> QPSK, modest TBS */
  const uint16_t mbsfn_area_id = 1;

  /* For 0.37 kHz SL4 the pilot stagger is g = 3*((tti/3)%4); sweep tti = 1,4,7,10
   * to exercise all four global staggers {0,3,6,9}, matching the freq-domain
   * pmch_fembms_test.c.  (The OFDM/IFFT stage itself is stagger-independent — it only
   * cares about the SCS numerology — so a single stagger would suffice for the time-
   * domain round trip; the full sweep is kept for parity with the freq-domain test.)
   *
   * NOTE on 0.37 kHz: srsRAN models one 0.37 kHz symbol (which spans 3 ms / 92160 Ts
   * in TS 36.211) as a single self-contained OFDM symbol inside the 1 ms subframe
   * window, i.e. only 1/3 of the true symbol.  The IFFT->FFT round trip self-inverts
   * for this simplified model, so a pass below confirms the model is internally
   * consistent — it does NOT prove full TS 36.211 0.37 kHz end-to-end support, which
   * requires a wider (3 ms) frame.  The "(1ms model)" tag in the output marks this. */
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

  printf("FeMBMS PMCH OFDM time-domain loopback self-test (nof_prb=%d, mcs=%d):\n", nof_prb, mcs_idx);

  int failures = 0;
  for (int i = 0; i < nof_cases; i++) {
    if (run_case(&cases[i], nof_prb, mcs_idx, mbsfn_area_id) != 0) {
      failures++;
    }
  }

  if (failures) {
    printf("FeMBMS PMCH OFDM loopback: %d/%d cases FAILED\n", failures, nof_cases);
    exit(SRSRAN_ERROR);
  }
  printf("FeMBMS PMCH OFDM loopback: all %d cases passed\n", nof_cases);
  exit(SRSRAN_SUCCESS);
}
