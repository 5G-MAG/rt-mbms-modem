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

/* FeMBMS PMCH loopback self-test.
 *
 * Exercises the FeMBMS numerologies (7.5 / 2.5 / 1.25 kHz and 0.37 kHz SL4/SL2)
 * that the existing pmch_test.c does NOT cover (it is hardcoded to 15 kHz MBSFN).
 *
 * This is a frequency-domain, identity-channel loopback: it encodes a PMCH
 * subframe, copies the resource grid unchanged (perfect channel), then decodes
 * and checks byte-exact recovery.  No OFDM modulation is involved, which keeps
 * the test free of time-domain buffer-sizing and FFT-normalisation concerns and
 * focuses on the parts where the FeMBMS fixes live:
 *   - srsran_ra_dl_grant_nof_re / ra_re_x_prb data-RE counts per SCS (incl. the
 *     7.5 kHz unequal-per-slot RS count and the SL4 per-PRB stagger formula),
 *   - pmch_cp RE put/get symmetry (RS holes, lstart=0 for all FeMBMS SCS),
 *   - scrambling/descrambling, modulation/demodulation, rate matching.
 *
 * The softbuffer is sized by nof_prb only, exactly as the production decoder
 * (MbsfnFrameProcessor) does, so any sizing problem surfaced here is real.
 */

#include <srsran/phy/utils/random.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "srsran/srsran.h"

/* Mirror of MAX_PMCH_RE() in pmch.c, built from the public phy_common.h macros.
 * Max data+RS resource elements per PRB in one MBSFN subframe for the given SCS. */
#define TEST_MAX_PMCH_RE(scs)                                                                                          \
  (SRSRAN_MBSFN_NOF_SLOTS(scs) * SRSRAN_MBSFN_NOF_SYMBOLS(scs) * SRSRAN_NRE_SCS(scs))

typedef struct {
  const char*  name;
  srsran_scs_t scs;
  uint32_t     tti; /* picks the 0.37 kHz pilot stagger; irrelevant for other SCS */
} fembms_case_t;

/* One encode -> identity-copy -> decode round trip.  Returns 0 on byte-exact
 * recovery, -1 otherwise.  Allocates and frees its own per-case state so a
 * failure in one case cannot corrupt the next. */
static int run_case(const fembms_case_t* tc, uint32_t nof_prb, uint32_t mcs_idx, uint16_t mbsfn_area_id)
{
  int             ret    = -1;
  srsran_random_t random = srsran_random_init(tc->tti + 1);

  srsran_cell_t cell = {0};
  cell.nof_prb       = nof_prb;
  cell.nof_ports     = 1;
  cell.id            = 1;
  cell.cp            = SRSRAN_CP_EXT; /* all MBSFN subframes use extended CP */
  cell.phich_length  = SRSRAN_PHICH_NORM;
  cell.phich_resources = SRSRAN_PHICH_R_1_6;
  cell.frame_type    = SRSRAN_FDD;

  srsran_dl_sf_cfg_t dl_sf;
  ZERO_OBJECT(dl_sf);
  dl_sf.cfi                = 0; /* FeMBMS dedicated carrier: no control region */
  dl_sf.tti                = tc->tti;
  dl_sf.sf_type            = SRSRAN_SF_MBSFN;
  dl_sf.subcarrier_spacing = tc->scs;

  /* Per-PRB grid: largest FeMBMS RE/PRB is 0.37 kHz (486).  Sizing for the case
   * SCS is enough, but use the maximum so the buffers fit every SCS unchanged. */
  uint32_t grid_re = nof_prb * TEST_MAX_PMCH_RE(SRSRAN_SCS_370HZ);

  cf_t*                   tx_grid[SRSRAN_MAX_PORTS] = {0};
  cf_t*                   rx_grid[SRSRAN_MAX_PORTS] = {0};
  uint8_t*                data_tx                   = NULL;
  uint8_t*                data_rx                   = NULL;
  srsran_softbuffer_tx_t  softbuffer_tx             = {0};
  srsran_softbuffer_rx_t  softbuffer_rx             = {0};
  srsran_chest_dl_res_t   chest_dl_res              = {0};
  srsran_pmch_t           pmch                      = {0};
  bool                    pmch_ok = false, sb_tx_ok = false, sb_rx_ok = false, chest_ok = false;

  tx_grid[0] = srsran_vec_cf_malloc(grid_re);
  rx_grid[0] = srsran_vec_cf_malloc(grid_re);
  if (!tx_grid[0] || !rx_grid[0]) {
    ERROR("Error allocating resource grids");
    goto cleanup;
  }
  srsran_vec_cf_zero(tx_grid[0], grid_re);
  srsran_vec_cf_zero(rx_grid[0], grid_re);

  if (srsran_chest_dl_res_init(&chest_dl_res, nof_prb)) {
    ERROR("Error initialising chest_dl_res");
    goto cleanup;
  }
  chest_ok = true;
  srsran_chest_dl_res_set_identity(&chest_dl_res); /* perfect channel */

  if (srsran_softbuffer_tx_init(&softbuffer_tx, nof_prb)) {
    ERROR("Error initialising TX softbuffer");
    goto cleanup;
  }
  sb_tx_ok = true;
  if (srsran_softbuffer_rx_init(&softbuffer_rx, nof_prb)) {
    ERROR("Error initialising RX softbuffer");
    goto cleanup;
  }
  sb_rx_ok = true;

  if (srsran_pmch_init(&pmch, nof_prb, 1)) {
    ERROR("Error initialising PMCH");
    goto cleanup;
  }
  pmch_ok = true;
  if (srsran_pmch_set_area_id(&pmch, mbsfn_area_id)) {
    ERROR("Error setting PMCH area id");
    goto cleanup;
  }

  /* Build the PMCH grant (this is where ra_re_x_prb / nof_re is computed). */
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
    ERROR("Error building grant");
    goto cleanup;
  }

  uint32_t tbs    = (uint32_t)pmch_cfg.pdsch_cfg.grant.tb[0].tbs;
  uint32_t nof_re = pmch_cfg.pdsch_cfg.grant.nof_re;
  if (tbs == 0 || nof_re == 0 || nof_re > grid_re) {
    ERROR("%s: implausible grant tbs=%d nof_re=%d (grid_re=%d)", tc->name, tbs, nof_re, grid_re);
    goto cleanup;
  }

  data_tx = srsran_vec_u8_malloc(tbs / 8);
  /* decode's output buffer contract is tbs+24 bits (payload + TB-CRC24,
   * decoded in-place and re-verified there), not tbs bits - see sch.c's
   * decode_tb_cb / decode_tb. tbs/8 alone under-allocates by 3 bytes. */
  data_rx = srsran_vec_u8_malloc((tbs + 24 + 7) / 8);
  if (!data_tx || !data_rx) {
    ERROR("Error allocating payload buffers");
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

  printf("  %-14s tti=%2d  nof_re=%6d  tbs=%5d bits  ... OK\n", tc->name, tc->tti, nof_re, tbs);
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

int main(int argc, char** argv)
{
  (void)argc;
  (void)argv;

  const uint32_t nof_prb       = 25; /* valid for every FeMBMS SCS (0.37 kHz max is 75) */
  const uint32_t mcs_idx       = 2;  /* low MCS -> QPSK, modest TBS */
  const uint16_t mbsfn_area_id = 1;

  /* For 0.37 kHz SL4 the pilot stagger is 3*((tti/3)%4); cover all four values
   * (staggers 0/3/6/9) to exercise the per-PRB RS-hole alignment in pmch_cp and
   * the alternating per-PRB RS count in srsran_ra_dl_grant_nof_re. */
  const fembms_case_t cases[] = {
      {"7.5 kHz", SRSRAN_SCS_7KHZ5, 1},
      {"2.5 kHz", SRSRAN_SCS_2KHZ5, 1},
      {"1.25 kHz", SRSRAN_SCS_1KHZ25, 1},
      {"0.37 SL2", SRSRAN_SCS_370HZ_SL2, 1},
      {"0.37 SL4", SRSRAN_SCS_370HZ_SL4, 1},
      {"0.37 SL4", SRSRAN_SCS_370HZ_SL4, 4},
      {"0.37 SL4", SRSRAN_SCS_370HZ_SL4, 7},
      {"0.37 SL4", SRSRAN_SCS_370HZ_SL4, 10},
  };
  const int nof_cases = (int)(sizeof(cases) / sizeof(cases[0]));

  printf("FeMBMS PMCH loopback self-test (nof_prb=%d, mcs=%d):\n", nof_prb, mcs_idx);

  int failures = 0;
  for (int i = 0; i < nof_cases; i++) {
    if (run_case(&cases[i], nof_prb, mcs_idx, mbsfn_area_id) != 0) {
      failures++;
    }
  }

  if (failures) {
    printf("FeMBMS PMCH loopback: %d/%d cases FAILED\n", failures, nof_cases);
    exit(SRSRAN_ERROR);
  }
  printf("FeMBMS PMCH loopback: all %d cases passed\n", nof_cases);
  exit(SRSRAN_SUCCESS);
}
