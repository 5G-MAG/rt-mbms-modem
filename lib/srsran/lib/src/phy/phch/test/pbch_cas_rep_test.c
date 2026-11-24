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

/* PBCH CAS-repetition (TS 36.211 clause 6.6.4.1) put/get geometry self-test.
 *
 * srsran_pbch_put_cas_rep() writes phase-rotated copies of the slot-1 PBCH
 * symbols into extra "CAS" symbol positions so a UE with only the CAS
 * subframe can still recover PBCH; srsran_pbch_get_cas_rep() extracts them
 * back with the conjugate rotation for soft combining. Neither function is
 * exercised by pbch_test.c (which never sets cell.mbms_dedicated), so this
 * is new coverage.
 *
 * This is a pure resource-grid loopback: no PBCH encode/decode, no OFDM, no
 * channel. It writes known values into the PBCH source band (slot 1, the
 * symbols the CAS map reads from), calls put_cas_rep, then get_cas_rep, and
 * checks that each non-RS destination subcarrier round-trips to the exact
 * source value (theta(k) is always one of {1,-1,j,-j}, so
 * theta*conj(theta)=1 with no floating-point rounding) and that RS-guarded
 * subcarriers are left untouched (still zero).
 *
 * The map tables and RS-hole predicate below mirror pbch.c's internal
 * PBCH_CAS_MAP_NCP / PBCH_CAS_MAP_ECP / pbch_cas_cp exactly (they are not
 * part of the public API), so the test can compute, for each destination
 * subcarrier, whether production code was expected to write it.
 */

#include <srsran/phy/utils/random.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "srsran/srsran.h"

#define PBCH_CAS_NOF_SC 72

typedef struct {
  uint32_t src_l, dst_ns, dst_l;
} pbch_cas_map_t;

/* Mirrors pbch.c PBCH_CAS_MAP_NCP / PBCH_CAS_MAP_ECP (TS 36.211 Table 6.6.4.1-1). */
static const pbch_cas_map_t CAS_MAP_NCP[5] = {{0, 0, 4}, {1, 1, 4}, {2, 1, 5}, {3, 0, 3}, {3, 1, 6}};
static const pbch_cas_map_t CAS_MAP_ECP[3] = {{1, 0, 3}, {2, 1, 4}, {3, 1, 5}};

/* One cell.id / cp combination.  Returns 0 on success, -1 on any mismatch. */
static int run_case(uint32_t cell_id, srsran_cp_t cp, uint32_t nof_prb)
{
  int             ret    = -1;
  srsran_random_t random = srsran_random_init(cell_id + (uint32_t)cp + 1u);

  srsran_cell_t cell   = {0};
  cell.nof_prb         = nof_prb;
  cell.nof_ports       = 1;
  cell.id              = cell_id;
  cell.cp              = cp;
  cell.phich_length    = SRSRAN_PHICH_NORM;
  cell.phich_resources = SRSRAN_PHICH_R_1_6;
  cell.frame_type      = SRSRAN_FDD;
  cell.mbms_dedicated  = true; /* required guard in put_cas_rep/get_cas_rep */

  bool                  ncp  = SRSRAN_CP_ISNORM(cp);
  const pbch_cas_map_t* map  = ncp ? CAS_MAP_NCP : CAS_MAP_ECP;
  int                   nmap = ncp ? 5 : 3;

  const uint32_t slot_re = SRSRAN_SLOT_LEN_RE(nof_prb, cp);
  const uint32_t sf_re   = SRSRAN_SF_LEN_RE(nof_prb, cp);
  const uint32_t center  = nof_prb * SRSRAN_NRE / 2 - 36;

  cf_t* grid    = NULL;
  cf_t* cas_out = NULL;
  cf_t  src_syms[4][PBCH_CAS_NOF_SC]; /* indexed by src_l (0..3); only used entries are filled */
  bool  src_filled[4] = {false, false, false, false};

  grid = srsran_vec_cf_malloc(sf_re);
  cas_out = srsran_vec_cf_malloc((uint32_t)nmap * PBCH_CAS_NOF_SC);
  if (!grid || !cas_out) {
    ERROR("cell_id=%d cp=%s: error allocating buffers", cell_id, srsran_cp_string(cp));
    goto cleanup;
  }
  srsran_vec_cf_zero(grid, sf_re);
  srsran_vec_cf_zero(cas_out, (uint32_t)nmap * PBCH_CAS_NOF_SC);

  /* --- Fill the PBCH source band (slot 1, symbol src_l) for every distinct
   * src_l the map reads from, with known random unit-ish values. --- */
  for (int i = 0; i < nmap; i++) {
    uint32_t src_l = map[i].src_l;
    if (src_filled[src_l]) {
      continue;
    }
    for (uint32_t k = 0; k < PBCH_CAS_NOF_SC; k++) {
      float re          = srsran_random_uniform_real_dist(random, -1.0f, 1.0f);
      float im          = srsran_random_uniform_real_dist(random, -1.0f, 1.0f);
      src_syms[src_l][k] = re + _Complex_I * im;
    }
    cf_t* src = grid + slot_re + src_l * nof_prb * SRSRAN_NRE + center;
    memcpy(src, src_syms[src_l], PBCH_CAS_NOF_SC * sizeof(cf_t));
    src_filled[src_l] = true;
  }

  /* --- TX: write phase-rotated copies into the CAS destination symbols --- */
  srsran_pbch_put_cas_rep(grid, cell);

  /* --- RX: extract + conjugate-rotate back into cas_out --- */
  srsran_pbch_get_cas_rep(grid, cell, cas_out);

  /* --- Verify every map entry against the exact same RS-hole predicate
   * pbch_cas_cp uses, and against the source values written above. --- */
  for (int i = 0; i < nmap; i++) {
    uint32_t src_l  = map[i].src_l;
    uint32_t dst_lp = map[i].dst_l;

    bool src_has_rs = ncp ? (src_l == 0 || src_l == 1) : (src_l == 0 || src_l == 3);
    bool dst_has_rs = ncp ? (dst_lp == 0 || dst_lp == 4) : (dst_lp == 0 || dst_lp == 3);

    for (uint32_t k = 0; k < PBCH_CAS_NOF_SC; k++) {
      bool  is_rs = ((k % 3u) == (cell_id % 3u));
      bool  skip  = (src_has_rs || dst_has_rs) && is_rs;
      cf_t  got   = cas_out[i * PBCH_CAS_NOF_SC + k];
      if (skip) {
        if (got != 0.0f) {
          ERROR("cell_id=%d cp=%s map=%d k=%d: expected RS-hole (untouched, 0) but got (%f,%f)",
                cell_id, srsran_cp_string(cp), i, k, crealf(got), cimagf(got));
          goto cleanup;
        }
      } else {
        cf_t expect = src_syms[src_l][k];
        cf_t diff   = got - expect;
        if (fabsf(crealf(diff)) > 1e-5f || fabsf(cimagf(diff)) > 1e-5f) {
          ERROR("cell_id=%d cp=%s map=%d k=%d: round-trip mismatch got=(%f,%f) expect=(%f,%f)",
                cell_id, srsran_cp_string(cp), i, k, crealf(got), cimagf(got), crealf(expect), cimagf(expect));
          goto cleanup;
        }
      }
    }
  }

  /* --- Guard test: mbms_dedicated=false must make both calls a no-op. --- */
  {
    srsran_cell_t cell_off  = cell;
    cell_off.mbms_dedicated = false;

    cf_t* grid2 = srsran_vec_cf_malloc(sf_re);
    if (!grid2) {
      ERROR("cell_id=%d cp=%s: error allocating guard-test buffer", cell_id, srsran_cp_string(cp));
      goto cleanup;
    }
    memcpy(grid2, grid, sf_re * sizeof(cf_t));
    /* Overwrite the CAS destinations with a sentinel so any write would be visible. */
    for (int i = 0; i < nmap; i++) {
      cf_t* dst = grid2 + map[i].dst_ns * slot_re + map[i].dst_l * nof_prb * SRSRAN_NRE + center;
      for (uint32_t k = 0; k < PBCH_CAS_NOF_SC; k++) {
        dst[k] = 12345.0f;
      }
    }
    srsran_pbch_put_cas_rep(grid2, cell_off);
    bool put_is_noop = true;
    for (int i = 0; i < nmap && put_is_noop; i++) {
      cf_t* dst = grid2 + map[i].dst_ns * slot_re + map[i].dst_l * nof_prb * SRSRAN_NRE + center;
      for (uint32_t k = 0; k < PBCH_CAS_NOF_SC; k++) {
        if (dst[k] != 12345.0f) {
          put_is_noop = false;
          break;
        }
      }
    }
    if (!put_is_noop) {
      ERROR("cell_id=%d cp=%s: srsran_pbch_put_cas_rep wrote to the grid with mbms_dedicated=false",
            cell_id, srsran_cp_string(cp));
      free(grid2);
      goto cleanup;
    }

    cf_t* cas_out2 = srsran_vec_cf_malloc((uint32_t)nmap * PBCH_CAS_NOF_SC);
    if (!cas_out2) {
      ERROR("cell_id=%d cp=%s: error allocating guard cas_out2", cell_id, srsran_cp_string(cp));
      free(grid2);
      goto cleanup;
    }
    srsran_vec_cf_zero(cas_out2, (uint32_t)nmap * PBCH_CAS_NOF_SC);
    srsran_pbch_get_cas_rep(grid2, cell_off, cas_out2);
    bool get_is_noop = true;
    for (int i = 0; i < nmap * PBCH_CAS_NOF_SC; i++) {
      if (cas_out2[i] != 0.0f) {
        get_is_noop = false;
        break;
      }
    }
    free(grid2);
    free(cas_out2);
    if (!get_is_noop) {
      ERROR("cell_id=%d cp=%s: srsran_pbch_get_cas_rep wrote to cas_out with mbms_dedicated=false",
            cell_id, srsran_cp_string(cp));
      goto cleanup;
    }
  }

  printf("  cell_id=%d cp=%-4s nmap=%d ... OK\n", cell_id, srsran_cp_string(cp), nmap);
  ret = 0;

cleanup:
  if (grid) {
    free(grid);
  }
  if (cas_out) {
    free(cas_out);
  }
  srsran_random_free(random);
  return ret;
}

int main(int argc, char** argv)
{
  (void)argc;
  (void)argv;

  const uint32_t nof_prb = 25; /* > 6, required by the put_cas_rep/get_cas_rep guard */

  printf("PBCH CAS-repetition put/get geometry self-test (nof_prb=%d):\n", nof_prb);

  int failures  = 0;
  int nof_cases = 0;
  for (uint32_t cell_id = 0; cell_id < 3; cell_id++) { /* sweep the RS-hole pattern (k%3 == id%3) */
    const srsran_cp_t cps[2] = {SRSRAN_CP_NORM, SRSRAN_CP_EXT};
    for (int c = 0; c < 2; c++) {
      nof_cases++;
      if (run_case(cell_id, cps[c], nof_prb) != 0) {
        failures++;
      }
    }
  }

  if (failures) {
    printf("PBCH CAS-repetition self-test: %d/%d cases FAILED\n", failures, nof_cases);
    exit(SRSRAN_ERROR);
  }
  printf("PBCH CAS-repetition self-test: all %d cases passed\n", nof_cases);
  exit(SRSRAN_SUCCESS);
}
