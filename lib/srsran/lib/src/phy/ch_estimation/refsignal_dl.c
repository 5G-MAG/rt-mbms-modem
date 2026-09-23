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

#include "srsran/srsran.h"
#include <complex.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

#include "srsran/phy/ch_estimation/refsignal_dl.h"
#include "srsran/phy/common/phy_common.h"
#include "srsran/phy/common/sequence.h"
#include "srsran/phy/utils/debug.h"
#include "srsran/phy/utils/vector.h"

/** Allocates memory for the 20 slots in a subframe
 */
int srsran_refsignal_cs_init(srsran_refsignal_t* q, uint32_t max_prb)
{
  int ret = SRSRAN_ERROR_INVALID_INPUTS;

  if (q != NULL) {
    ret = SRSRAN_ERROR;
    bzero(q, sizeof(srsran_refsignal_t));
    for (int p = 0; p < 2; p++) {
      for (int i = 0; i < SRSRAN_NOF_SF_X_FRAME; i++) {
        q->pilots[p][i] = srsran_vec_cf_malloc(SRSRAN_REFSIGNAL_MAX_NUM_SF_MBSFN(max_prb));
        if (!q->pilots[p][i]) {
          perror("malloc");
          goto free_and_exit;
        }
      }
    }
    ret = SRSRAN_SUCCESS;
  }
free_and_exit:
  if (ret == SRSRAN_ERROR) {
    srsran_refsignal_free(q);
  }
  return ret;
}

/** Allocates and precomputes the Cell-Specific Reference (CSR) signal for
 * the 20 slots in a subframe
 */
int srsran_refsignal_cs_set_cell(srsran_refsignal_t* q, srsran_cell_t cell)
{
  uint32_t          c_init;
  uint32_t          N_cp, mp;
  srsran_sequence_t seq;
  int               ret = SRSRAN_ERROR_INVALID_INPUTS;

  if (q != NULL && srsran_cell_isvalid(&cell)) {
    if (cell.id != q->cell.id || q->cell.nof_prb == 0) {
      q->cell = cell;

      bzero(&seq, sizeof(srsran_sequence_t));
      if (srsran_sequence_init(&seq, 2 * 2 * SRSRAN_MAX_PRB)) {
        return SRSRAN_ERROR;
      }

      if (SRSRAN_CP_ISNORM(cell.cp)) {
        N_cp = 1;
      } else {
        N_cp = 0;
      }

      srsran_dl_sf_cfg_t sf_cfg;
      ZERO_OBJECT(sf_cfg);

      for (uint32_t ns = 0; ns < SRSRAN_NSLOTS_X_FRAME; ns++) {
        for (uint32_t p = 0; p < 2; p++) {
          sf_cfg.tti        = ns / 2;
          uint32_t nsymbols = srsran_refsignal_cs_nof_symbols(q, &sf_cfg, 2 * p) / 2;
          for (uint32_t l = 0; l < nsymbols; l++) {
            /* Compute sequence init value */
            uint32_t lp = srsran_refsignal_cs_nsymbol(l, cell.cp, 2 * p);
            c_init      = 1024 * (7 * (ns + 1) + lp + 1) * (2 * cell.id + 1) + 2 * cell.id + N_cp;

            /* generate sequence for this symbol and slot */
            srsran_sequence_set_LTE_pr(&seq, 2 * 2 * SRSRAN_MAX_PRB, c_init);

            /* Compute signal */
            for (uint32_t i = 0; i < 2 * q->cell.nof_prb; i++) {
              uint32_t idx = SRSRAN_REFSIGNAL_PILOT_IDX(i, (ns % 2) * nsymbols + l, q->cell);
              mp           = i + SRSRAN_MAX_PRB - cell.nof_prb;
              /* save signal */
              __real__ q->pilots[p][ns / 2][idx] = (1 - 2 * (float)seq.c[2 * mp + 0]) * M_SQRT1_2;
              __imag__ q->pilots[p][ns / 2][idx] = (1 - 2 * (float)seq.c[2 * mp + 1]) * M_SQRT1_2;
            }
          }
        }
      }
      srsran_sequence_free(&seq);
    }
    ret = SRSRAN_SUCCESS;
  }
  return ret;
}

/** Deallocates a srsran_refsignal_cs_t object allocated with srsran_refsignal_cs_init */
void srsran_refsignal_free(srsran_refsignal_t* q)
{
  for (int p = 0; p < 2; p++) {
    for (int i = 0; i < SRSRAN_MBSFN_NOF_SF_40MS; i++) {
      if (q->pilots[p][i]) {
        free(q->pilots[p][i]);
      }
    }
  }
  bzero(q, sizeof(srsran_refsignal_t));
}

uint32_t srsran_refsignal_cs_v(uint32_t port_id, uint32_t ref_symbol_idx)
{
  uint32_t v = 0;
  switch (port_id) {
    case 0:
      if (!(ref_symbol_idx % 2)) {
        v = 0;
      } else {
        v = 3;
      }
      break;
    case 1:
      if (!(ref_symbol_idx % 2)) {
        v = 3;
      } else {
        v = 0;
      }
      break;
    case 2:
      if (ref_symbol_idx == 0) {
        v = 0;
      } else {
        v = 3;
      }
      break;
    case 3:
      if (ref_symbol_idx == 0) {
        v = 3;
      } else {
        v = 0;
      }
      break;
  }
  return v;
}

inline uint32_t srsran_refsignal_cs_nof_symbols(srsran_refsignal_t* q, srsran_dl_sf_cfg_t* sf, uint32_t port_id)
{
  if (q == NULL || sf == NULL || q->cell.frame_type == SRSRAN_FDD || !sf->tdd_config.configured ||
      srsran_sfidx_tdd_type(sf->tdd_config, sf->tti % 10) == SRSRAN_TDD_SF_D) {
    if (port_id < 2) {
      return 4;
    } else {
      return 2;
    }
  } else {
    uint32_t nof_dw_symbols = srsran_sfidx_tdd_nof_dw(sf->tdd_config);
    if (q->cell.cp == SRSRAN_CP_NORM) {
      if (nof_dw_symbols >= 12) {
        if (port_id < 2) {
          return 4;
        } else {
          return 2;
        }
      } else if (nof_dw_symbols >= 9) {
        if (port_id < 2) {
          return 3;
        } else {
          return 2;
        }
      } else if (nof_dw_symbols >= 5) {
        if (port_id < 2) {
          return 2;
        } else {
          return 1;
        }
      } else {
        return 1;
      }
    } else {
      if (nof_dw_symbols >= 10) {
        if (port_id < 2) {
          return 4;
        } else {
          return 2;
        }
      } else if (nof_dw_symbols >= 8) {
        if (port_id < 2) {
          return 3;
        } else {
          return 2;
        }
      } else if (nof_dw_symbols >= 4) {
        if (port_id < 2) {
          return 2;
        } else {
          return 1;
        }
      } else {
        return 1;
      }
    }
  }
}

inline uint32_t srsran_refsignal_cs_nof_pilots_x_slot(uint32_t nof_ports)
{
  switch (nof_ports) {
    case 2:
      return 8;
    case 4:
      return 12;
    default:
      return 4;
  }
}

inline uint32_t srsran_refsignal_cs_nof_re(srsran_refsignal_t* q, srsran_dl_sf_cfg_t* sf, uint32_t port_id)
{
  uint32_t nof_re = srsran_refsignal_cs_nof_symbols(q, sf, port_id);
  if (q != NULL) {
    nof_re *= q->cell.nof_prb * 2; // 2 RE per PRB
  }
  return nof_re;
}

inline uint32_t srsran_refsignal_cs_fidx(srsran_cell_t cell, uint32_t l, uint32_t port_id, uint32_t m)
{
  return 6 * m + ((srsran_refsignal_cs_v(port_id, l) + (cell.id % 6)) % 6);
}

inline uint32_t srsran_refsignal_cs_nsymbol(uint32_t l, srsran_cp_t cp, uint32_t port_id)
{
  if (port_id < 2) {
    if (l % 2) {
      return (l / 2 + 1) * SRSRAN_CP_NSYMB(cp) - 3;
    } else {
      return (l / 2) * SRSRAN_CP_NSYMB(cp);
    }
  } else {
    return 1 + l * SRSRAN_CP_NSYMB(cp);
  }
}

/* Maps a reference signal initialized with srsran_refsignal_cs_init() into an array of subframe symbols */
int srsran_refsignal_cs_put_sf(srsran_refsignal_t* q, srsran_dl_sf_cfg_t* sf, uint32_t port_id, cf_t* sf_symbols)
{
  uint32_t i, l;
  uint32_t fidx;

  if (q != NULL && port_id < SRSRAN_MAX_PORTS && sf_symbols != NULL) {
    cf_t* pilots = q->pilots[port_id / 2][sf->tti % 10];
    for (l = 0; l < srsran_refsignal_cs_nof_symbols(q, sf, port_id); l++) {
      uint32_t nsymbol = srsran_refsignal_cs_nsymbol(l, q->cell.cp, port_id);
      /* Compute offset frequency index */
      fidx = ((srsran_refsignal_cs_v(port_id, l) + (q->cell.id % 6)) % 6);
      for (i = 0; i < 2 * q->cell.nof_prb; i++) {
        sf_symbols[SRSRAN_RE_IDX(q->cell.nof_prb, nsymbol, fidx)] = pilots[SRSRAN_REFSIGNAL_PILOT_IDX(i, l, q->cell)];
        fidx += SRSRAN_NRE / 2; // 1 reference every 6 RE
      }
    }
    return SRSRAN_SUCCESS;
  } else {
    return SRSRAN_ERROR_INVALID_INPUTS;
  }
}

/** Copies the RE containing references from an array of subframe symbols to the pilots array. */
int srsran_refsignal_cs_get_sf(srsran_refsignal_t* q,
                               srsran_dl_sf_cfg_t* sf,
                               uint32_t            port_id,
                               cf_t*               sf_symbols,
                               cf_t*               pilots)
{
  uint32_t i, l;
  uint32_t fidx;

  if (q != NULL && pilots != NULL && sf_symbols != NULL) {
    for (l = 0; l < srsran_refsignal_cs_nof_symbols(q, sf, port_id); l++) {
      uint32_t nsymbol = srsran_refsignal_cs_nsymbol(l, q->cell.cp, port_id);
      /* Compute offset frequency index */
      fidx = srsran_refsignal_cs_fidx(q->cell, l, port_id, 0);
      for (i = 0; i < 2 * q->cell.nof_prb; i++) {
        pilots[SRSRAN_REFSIGNAL_PILOT_IDX(i, l, q->cell)] = sf_symbols[SRSRAN_RE_IDX(q->cell.nof_prb, nsymbol, fidx)];
        fidx += SRSRAN_NRE / 2; // 2 references per PRB
      }
    }
    return SRSRAN_SUCCESS;
  } else {
    return SRSRAN_ERROR_INVALID_INPUTS;
  }
}

SRSRAN_API int srsran_refsignal_mbsfn_put_sf(srsran_cell_t cell,
                                             uint32_t      port_id,
                                             cf_t*         cs_pilots,
                                             cf_t*         mbsfn_pilots,
                                             cf_t*         sf_symbols,
                                             srsran_scs_t  scs,
                                             uint32_t      tti)
{
  uint32_t i, l;
  uint32_t fidx;

  if (srsran_cell_isvalid(&cell) && srsran_portid_isvalid(port_id) && cs_pilots != NULL && mbsfn_pilots != NULL &&
      sf_symbols != NULL) {

    if (scs == SRSRAN_SCS_15KHZ) {
      /* CS references for the non-MBSFN section of the subframe. */
      fidx = ((srsran_refsignal_cs_v(port_id, 0) + (cell.id % 6)) % 6);
      for (i = 0; i < 2 * cell.nof_prb; i++) {
        sf_symbols[SRSRAN_RE_IDX(cell.nof_prb, 0, fidx)] = cs_pilots[SRSRAN_REFSIGNAL_PILOT_IDX(i, 0, cell)];
        fidx += SRSRAN_NRE / 2;
      }
    }

    /* For extended BW (mbsfn_prb < nof_prb), RS covers only the MBSFN allocation.
     * pmch_cp writes data linearly from subcarrier 0, so RS must also be left-aligned
     * (starting at subcarrier 0, not centered) to stay consistent with data placement. */
    uint32_t act_prb_put = cell.mbsfn_prb ? cell.mbsfn_prb : cell.nof_prb;
    /* ns must match the slot index used to select mbsfn_pilots (q->mbsfnr_signal.pilots[0][ns]
     * in the caller, e.g. enb_dl.c's put_refs): 40 ms period, 13 slots of 3 ms, with the first
     * slot (ns=0) absorbing the extra TTI (0..3) so that 4 + 3*12 = 40. A plain tti/3 does not
     * have that offset and disagrees with this from tti=3 onward within every period. */
    uint32_t pos40 = tti % 40u;
    /* TS 36.211 §6.10.2.2.4: the stagger's "ns" is the ABSOLUTE 3ms slot number
     * (ns = ns' + 13*nf/4, nf = radio frame number), not the period-local slot
     * index -- folding in the period count makes the stagger phase advance by
     * one step every successive 40 ms period (13 mod 4 = 1 for SL4, 13 mod 2 = 1
     * for SL2) instead of always resetting to phase 0, as clause 6.10.2.2.4
     * requires. This does NOT apply to which row of the local 13-row pilot
     * VALUE table gets used elsewhere (that stays period-local, unaffected). */
    uint32_t ns_37 = ((pos40 > 0u) ? (pos40 - 1u) / 3u : 0u) + 13u * (tti / 40u);
    if (scs == SRSRAN_SCS_370HZ_SL4) {
      /* TS 36.211 §6.10.2.2.4 type1: k = 12*m' + stagger, stagger = 3*(ns mod 4).
       * Pilot spacing 12 does not divide NscRB=486, so per-RB framework is not used.
       * Must mirror the get_sf SL4 path exactly (same k formula, same base SCS). */
      uint32_t stagger      = 3 * (ns_37 % 4);
      uint32_t total_pilots = (SRSRAN_NRE_SCS_370HZ * act_prb_put) / 12;
      for (i = 0; i < total_pilots; i++) {
        uint32_t k = 12 * i + stagger;
        sf_symbols[SRSRAN_RE_IDX_MBSFN(cell.nof_prb, 0, k, SRSRAN_SCS_370HZ)] =
          mbsfn_pilots[SRSRAN_REFSIGNAL_PILOT_IDX_MBSFN(i, 0, cell, scs)];
      }
    } else {
      for (l = 0; l < srsran_refsignal_mbsfn_nof_symbols(scs); l++) {
        uint32_t nsymbol = srsran_refsignal_mbsfn_nsymbol(l, scs);
        if (scs == SRSRAN_SCS_1KHZ25) {
          /* TS 36.211 §6.10.2.2.2 "Mapping to resource elements for 1.25 kHz":
           *   k = 6m if n_sf mod 2 = 0, k = 6m+3 if n_sf mod 2 = 1
           * where n_sf is the SUBFRAME NUMBER WITHIN THE RADIO FRAME (0..9,
           * changes every 1 ms) - NOT the system frame number sfn=tti/10
           * (changes every 10 ms). Since tti = 10*sfn + n_sf and 10 is even,
           * n_sf%2 == tti%2. Previously used sfn%2 (wrong clause cited too:
           * §6.10.2.2.3 is the 2.5 kHz clause, not 1.25 kHz) - this changed
           * the stagger only once every 10 ms instead of every 1 ms, silently
           * misaligning the assumed pilot subcarriers from every OTHER
           * subframe onward relative to what pmch_cp's data mapping and the
           * chest_dl interpolator expect (see the matching fix there). */
          fidx = tti % 2 == 0 ? 0 : 3;
        } else if (scs == SRSRAN_SCS_370HZ_SL2) {
          /* TS 36.211 §6.10.2.2.4 type2: stagger = 3*(ns mod 2). */
          fidx = 3 * (ns_37 % 2);
        } else {
          fidx = srsran_refsignal_mbsfn_fidx(l, scs);
        }
        for (i = 0; i < srsran_refsignal_mbsfn_rs_per_symbol(scs) * act_prb_put; i++) {
          sf_symbols[SRSRAN_RE_IDX_MBSFN(cell.nof_prb, nsymbol, fidx, scs)] =
            mbsfn_pilots[SRSRAN_REFSIGNAL_PILOT_IDX_MBSFN(i, l, cell, scs)];
          fidx += SRSRAN_NRE_SCS(scs) / srsran_refsignal_mbsfn_rs_per_symbol(scs);
        }
      }
    }

    return SRSRAN_SUCCESS;
  } else {
    return SRSRAN_ERROR_INVALID_INPUTS;
  }
}

uint32_t srsran_refsignal_mbsfn_nof_symbols(srsran_scs_t scs)
{
  /* For 2.5 kHz: RS in l=0 and l=1 (TS 36.211 Table 6.10.2.2.2-1).
   * For 0.37 kHz SL2/SL4: l=0 only (one RS symbol per 3 ms slot). */
  if (scs == SRSRAN_SCS_2KHZ5)   return 2;
  if (scs == SRSRAN_SCS_370HZ_SL2 || scs == SRSRAN_SCS_370HZ_SL4) return 1;
  if (scs == SRSRAN_SCS_1KHZ25)  return 1;
  if (scs == SRSRAN_SCS_7KHZ5)   return 3; /* TS 36.211 Table 6.10.2.2.1-1: l=0,1,2 */
  return 3; /* SRSRAN_SCS_15KHZ: TS 36.211 Table 6.10.2.2.1-1 */
}

uint32_t srsran_refsignal_mbsfn_rs_per_symbol(srsran_scs_t scs)
{
  /* 2.5 kHz: k=4m, NscRB=72, pilots per RB per symbol = 72/4 = 18.
   * SL2 (type2): k=6m, NscRB=486, 486/6=81 pilots per RB per symbol.
   * SL4 (type1): k=12m, NscRB=486, 486/12=40.5 — not integer per RB;
   *   return 41 (ceiling) for buffer sizing; put/get_sf use total-bandwidth loop. */
  if (scs == SRSRAN_SCS_2KHZ5)    return 18;
  if (scs == SRSRAN_SCS_370HZ_SL2) return 81;
  if (scs == SRSRAN_SCS_370HZ_SL4) return 41;  /* ceiling; real count = (486*prb)/12 */
  return scs == SRSRAN_SCS_1KHZ25 ? 24 : 6;
}

uint32_t srsran_refsignal_mbsfn_rs_per_rb(srsran_scs_t scs)
{
  return srsran_refsignal_mbsfn_nof_symbols(scs) * srsran_refsignal_mbsfn_rs_per_symbol(scs);
}

uint32_t srsran_symbols_per_mbsfn_subframe(srsran_scs_t scs)
{
  switch (scs) {
    case SRSRAN_SCS_15KHZ:       return 6;
    case SRSRAN_SCS_7KHZ5:       return 3;
    case SRSRAN_SCS_2KHZ5:       return 2;
    case SRSRAN_SCS_1KHZ25:      return 1;
    case SRSRAN_SCS_370HZ_SL2:
    case SRSRAN_SCS_370HZ_SL4:   return 1;  /* one 3 ms symbol per 1 ms window (partial) */
    default:                      return 0;
  }
}

inline uint32_t srsran_refsignal_mbsfn_fidx(uint32_t l, srsran_scs_t scs)
{
  uint32_t ret = 0;
    switch (scs) {
    case SRSRAN_SCS_15KHZ:
      if (l == 0) {
        ret = 0;
      } else if (l == 1) {
        ret = 1;
      } else if (l == 2) {
        ret = 0;
      }
      break;
    case SRSRAN_SCS_7KHZ5:
      if (l == 0) {
        ret = 0;
      } else if (l == 1) {
        ret = 2;
      } else if (l == 2) {
        ret = 0;
      }
      break;
    case SRSRAN_SCS_2KHZ5:
      /* TS 36.211 clause 6.10.2.2.3: k=4m for l=0, k=4m+2 for l=1. */
      ret = (l == 0) ? 0 : 2;
      break;
    case SRSRAN_SCS_1KHZ25:
      ret = 0;
      break;
    case SRSRAN_SCS_370HZ_SL2:
    case SRSRAN_SCS_370HZ_SL4:
      /* Stagger is slot-dependent; computed from sf_idx in get_sf/put_sf. */
      ret = 0;
      break;
    default:
      ret = 0;
      break;
  }

  return ret;
}

inline uint32_t srsran_refsignal_mbsfn_offset(uint32_t l, uint32_t s, uint32_t tti, srsran_scs_t scs)
{
  uint32_t ret    = 0;
  /* ns must match put_sf/get_sf's ns (40 ms period, 13 slots of 3 ms, first slot
   * absorbs the extra TTI) — a plain tti/3 disagrees with that from tti=3 onward. */
  uint32_t pos40  = tti % 40u;
  /* See put_sf/get_sf's matching comment: "ns" here is the ABSOLUTE 3ms slot
   * number (TS 36.211 §6.10.2.2.4: ns = ns' + 13*nf/4), not period-local. */
  uint32_t ns_37  = ((pos40 > 0u) ? (pos40 - 1u) / 3u : 0u) + 13u * (tti / 40u);

  switch (scs) {
    case SRSRAN_SCS_15KHZ:
      if (s == 1 && l == 0) {
        ret = 1;
      }
      break;
    case SRSRAN_SCS_7KHZ5:
      if (s == 1 && l == 0) {
        ret = 2;
      }
      break;
    case SRSRAN_SCS_2KHZ5:
      /* TS 36.211 clause 6.10.2.2.3: offset within each symbol is fixed (0 or 2),
       * carried by fidx; no slot-based stagger for 2.5 kHz. */
      ret = (l == 0) ? 0 : 2;
      break;
    case SRSRAN_SCS_1KHZ25:
      /* TS 36.211 §6.10.2.2.2 "Mapping to resource elements for 1.25 kHz": stagger
       * is 3*(n_sf mod 2) where n_sf is the subframe number within the radio frame
       * (0..9, changes every 1 ms) - not the frame number sfn=tti/10 (changes every
       * 10 ms; wrong clause was cited too - §6.10.2.2.3 is 2.5 kHz's). tti%2 ==
       * n_sf%2 since tti = 10*sfn+n_sf and 10 is even. See matching fix in
       * put_sf/get_sf and chest_dl.c's interpolate_pilots. */
      if (tti % 2 != 0) {
        ret = 3;
      }
      break;
    case SRSRAN_SCS_370HZ_SL2:
      /* TS 36.211 clause 6.10.2.2.4 type2: stagger = 3*(ns mod 2). */
      ret = 3 * (ns_37 % 2);
      break;
    case SRSRAN_SCS_370HZ_SL4:
      /* TS 36.211 clause 6.10.2.2.4 type1: stagger = 3*(ns mod 4). */
      ret = 3 * (ns_37 % 4);
      break;
    default:
      break;
  }
  return ret;
}

inline uint32_t srsran_refsignal_mbsfn_nsymbol(uint32_t l, srsran_scs_t scs)
{
  uint32_t ret = 0;

  switch (scs) {
    case SRSRAN_SCS_15KHZ:
      if (l == 0) {
        ret = 2;
      } else if (l == 1) {
        ret = 6;
      } else if (l == 2) {
        ret = 10;
      }
      break;
    case SRSRAN_SCS_7KHZ5:
      if (l == 0) {
        ret = 1;
      } else if (l == 1) {
        ret = 3;
      } else if (l == 2) {
        ret = 5;
      }
      break;
    case SRSRAN_SCS_2KHZ5:
      /* At 2.5 kHz a 1 ms subframe contains exactly 2 OFDM symbols (l=0 and l=1). */
      ret = l;
      break;
    case SRSRAN_SCS_1KHZ25:
      ret = 0;
      break;
    case SRSRAN_SCS_370HZ_SL2:
    case SRSRAN_SCS_370HZ_SL4:
      /* RS always in l=0 (first/only symbol of the 3 ms slot). */
      ret = 0;
      break;
    default:
      ret = 0;
      break;
  }

  return ret;
}

int srsran_refsignal_mbsfn_gen_seq(srsran_refsignal_t* q, srsran_cell_t cell, uint32_t N_mbsfn_id, srsran_scs_t scs)
{
  uint32_t c_init;
  uint32_t i, ns, l, p;
  uint32_t mp;
  int      ret = SRSRAN_ERROR;

  srsran_sequence_t seq_mbsfn;
  bzero(&seq_mbsfn, sizeof(srsran_sequence_t));
  if (srsran_sequence_init(&seq_mbsfn, 20 * SRSRAN_REFSIGNAL_NUM_SF_MBSFN(SRSRAN_MAX_PRB, scs))) {
    goto free_and_exit;
  }

  /* TS 36.211 §6.10.2.1: RS sequence periods per SCS.
   * 2.5 kHz (clause 6.10.2.1.3): n_sf = 0..9, 10 ms period (same frame as 7.5/1.25/15 kHz).
   * 0.37 kHz (clause 6.10.2.1.4): n_s = 0..12, 40 ms period (13 slots of 3 ms each).
   * Other SCS: standard 10 ms frame period. */
  uint32_t nof_sf = SRSRAN_NOF_SF_X_FRAME;
  if (scs == SRSRAN_SCS_370HZ_SL4 || scs == SRSRAN_SCS_370HZ_SL2) {
    nof_sf = 13;
  }
  for (ns = 0; ns < nof_sf; ns++) {
    for (p = 0; p < 2; p++) {
      uint32_t nsymbols = srsran_refsignal_mbsfn_nof_symbols(scs);
      for (l = 0; l < nsymbols; l++) {
        uint32_t lp   = (srsran_refsignal_mbsfn_nsymbol(l, scs)) % srsran_symbols_per_mbsfn_subframe(scs);
        uint32_t slot = (l) ? (ns * 2 + 1) : (ns * 2);
        if (scs == SRSRAN_SCS_2KHZ5 || scs == SRSRAN_SCS_370HZ_SL2 || scs == SRSRAN_SCS_370HZ_SL4) {
          /* TS 36.211 clauses 6.10.2.1.3 (2.5 kHz) and 6.10.2.1.4 (0.37 kHz):
           *   cinit = 512*(7*(n+1) + l + 1)*(2*N_ID^MBSFN + 1) + N_ID^MBSFN
           * Same structure as 7.5 kHz (clause 6.10.2.1.2); only the time index differs.
           * For 2.5 kHz: n = n_sf (0..9, subframe index within 10 ms frame).
           * For 0.37 kHz: n = n_s (0..12, 3 ms slot index within 40 ms period).
           * Note: prior code used 297*(n+1)+l+12N(N+1)+N which is a misread of the OOXML
           * math (2^9 superscript adjacent to the factor 7 renders as "297" in text). */
          c_init = 512u * (7u * (ns + 1u) + lp + 1u) * (2u * N_mbsfn_id + 1u) + N_mbsfn_id;
        } else if (scs == SRSRAN_SCS_7KHZ5) {
          /* TS 36.211 clause 6.10.2.1.2 (7.5 kHz MBSFN RS):
           *   cinit = 512*(7*(ns+1) + lp + 1)*(2*N_ID^MBSFN + 1) + N_ID^MBSFN
           * where ns is the subframe index (0..9) and lp = nsymbol mod 3.
           * At 7.5 kHz there is one slot per 1 ms subframe, so ns (not slot) is
           * the correct time index; using slot (= ns*2 or ns*2+1) would produce
           * the wrong cinit and broken pilot values. */
          c_init = 512 * (7 * (ns + 1) + lp + 1) * (2 * N_mbsfn_id + 1) + N_mbsfn_id;
        } else if (scs == SRSRAN_SCS_1KHZ25) {
          /* TS 36.211 clause 6.10.2.1.1 (15 kHz / 1.25 kHz form, shared):
           *   cinit = 512*(7*(ns+1) + l + 1)*(2*N+1) + N */
          c_init = 512 * (7 * (ns + 1) + l + 1) * (2 * N_mbsfn_id + 1) + N_mbsfn_id;
        } else {
          /* SRSRAN_SCS_15KHZ: TS 36.211 clause 6.10.2.1.1:
           *   cinit = 512*(7*(slot+1) + lp + 1)*(2*N+1) + N */
          c_init = 512 * (7 * (slot + 1) + lp + 1) * (2 * N_mbsfn_id + 1) + N_mbsfn_id;
        }

        srsran_sequence_set_LTE_pr(&seq_mbsfn, 10 * SRSRAN_REFSIGNAL_NUM_SF_MBSFN(SRSRAN_MAX_PRB, scs), c_init);
        if (scs == SRSRAN_SCS_370HZ_SL4) {
          /* TS 36.211 clause 6.10.2.2.4 type1: m'=0..(NscRB/12 * prb)-1, m = m' + shift,
           * shift = NscRB/12 * (N_RB^max,DL - N_RB^DL) — same centering idea as the other
           * numerologies below, applied here since guard bands (N_RB^DL < N_RB^max,DL)
           * are the common case for a real deployment, not just NRBmax,DL. NscRB/12 =
           * 40.5 is non-integer; use floor for both the pilot count and the shift. */
          uint32_t act_prb       = q->cell.mbsfn_prb ? q->cell.mbsfn_prb : q->cell.nof_prb;
          uint32_t total_pilots  = (SRSRAN_NRE_SCS_370HZ * act_prb) / 12;
          uint32_t center_offset = (SRSRAN_NRE_SCS_370HZ * (SRSRAN_MAX_PRB - act_prb)) / 24;
          for (i = 0; i < total_pilots; i++) {
            mp                            = i + center_offset;
            __real__ q->pilots[p][ns][i] = (1 - 2 * (float)seq_mbsfn.c[2 * mp + 0]) * M_SQRT1_2;
            __imag__ q->pilots[p][ns][i] = (1 - 2 * (float)seq_mbsfn.c[2 * mp + 1]) * M_SQRT1_2;
          }
        } else if (scs == SRSRAN_SCS_370HZ_SL2) {
          /* TS 36.211 clause 6.10.2.2.4 type2: m'=0..(NscRB/6 * prb)-1 = 81*prb-1,
           * shift = NscRB/6 * (N_RB^max,DL - N_RB^DL) / 2 = NscRB/12 * (...) — same
           * centering as SL4 above, halved pilot density. */
          uint32_t act_prb       = q->cell.mbsfn_prb ? q->cell.mbsfn_prb : q->cell.nof_prb;
          uint32_t total_pilots  = SRSRAN_NRE_SCS_370HZ / 6 * act_prb;  /* 81 * act_prb */
          uint32_t center_offset = (SRSRAN_NRE_SCS_370HZ * (SRSRAN_MAX_PRB - act_prb)) / 12;
          for (i = 0; i < total_pilots; i++) {
            mp                            = i + center_offset;
            __real__ q->pilots[p][ns][i] = (1 - 2 * (float)seq_mbsfn.c[2 * mp + 0]) * M_SQRT1_2;
            __imag__ q->pilots[p][ns][i] = (1 - 2 * (float)seq_mbsfn.c[2 * mp + 1]) * M_SQRT1_2;
          }
        } else {
          /* Centering offset m' = m + centering_factor*(N_RB^max,DL - N_RB^DL): shifts
           * which slice of the master pilot table (sized for the spec's max bandwidth,
           * SRSRAN_MAX_PRB=110) gets used for this deployment's actual bandwidth.
           * rs_per_symbol(scs)/2 is a generic heuristic that happens to match 2.5 kHz's
           * own clause (6.10.2.2.3: m' = m + (N_sc^RB/4)*Delta, Delta=(max-act)/2,
           * N_sc^RB=72 -> coefficient 9 = rs_per_symbol(24)/2... wait, verified
           * separately this reduces to exactly rs_per_symbol/2 for 2.5 kHz) but does
           * NOT hold for 1.25 kHz: TS 36.211 clause 6.10.2.2.2 "Mapping to resource
           * elements for 1.25 kHz" gives m' = m + 3*(N_RB^max,DL - N_RB^DL), a flat
           * coefficient of 3 - not rs_per_symbol(24)/2=12. At the wrong (12x) offset
           * this pulls pilot VALUES from the wrong slice of the master Gold-sequence
           * table: finite, well-formed-looking numbers that are nevertheless wrong,
           * rather than an obviously-broken value - hence why it survived every
           * previous internal-consistency check in this investigation. 7.5 kHz and
           * 15 kHz's coefficients have NOT been independently re-verified against
           * their own clause (6.10.2.2.1) and are left as rs_per_symbol/2 for now. */
          uint32_t act_prb          = q->cell.mbsfn_prb ? q->cell.mbsfn_prb : q->cell.nof_prb;
          uint32_t centering_factor = (scs == SRSRAN_SCS_1KHZ25) ? 3u : (srsran_refsignal_mbsfn_rs_per_symbol(scs) / 2u);
          uint32_t center_offset = centering_factor * (SRSRAN_MAX_PRB - act_prb);
          for (i = 0; i < srsran_refsignal_mbsfn_rs_per_symbol(scs) * act_prb; i++) {
            uint32_t idx                   = SRSRAN_REFSIGNAL_PILOT_IDX_MBSFN(i, l, q->cell, scs);
            mp                             = i + center_offset;
            __real__ q->pilots[p][ns][idx] = (1 - 2 * (float)seq_mbsfn.c[2 * mp + 0]) * M_SQRT1_2;
            __imag__ q->pilots[p][ns][idx] = (1 - 2 * (float)seq_mbsfn.c[2 * mp + 1]) * M_SQRT1_2;
          }
        }
      }
    }
  }

  srsran_sequence_free(&seq_mbsfn);
  ret = SRSRAN_SUCCESS;

free_and_exit:
  if (ret == SRSRAN_ERROR) {
    srsran_sequence_free(&seq_mbsfn);
    srsran_refsignal_free(q);
  }
  return ret;
}

int srsran_refsignal_mbsfn_init(srsran_refsignal_t* q, uint32_t max_prb, srsran_scs_t scs)
{
  int      ret = SRSRAN_ERROR_INVALID_INPUTS;
  uint32_t i, p;
  if (q != NULL) {
    ret = SRSRAN_ERROR;
    bzero(q, sizeof(srsran_refsignal_t));

    q->type = SRSRAN_SF_MBSFN;

    /* Size for the densest MBSFN RS pattern (SL2, see SRSRAN_REFSIGNAL_MAX_NUM_SF_MBSFN's
     * own doc comment), not just the scs passed to THIS call. q->pilots[area_id][*] is
     * shared across every scs that ever uses this area_id - chest_dl.c's
     * srsran_chest_dl_set_mbsfn_area_id() only (re)allocates it the first time a given
     * area_id is seen (`if (!q->mbsfn_refs[mbsfn_area_id])`), so a later call for the
     * SAME area_id with a DIFFERENT, denser scs (e.g. an eNB using one area_id for both
     * 15 kHz MTCH data and 1.25 kHz MCCH, as this repo's own TX fork does) reused this
     * same, now-undersized buffer instead of resizing it. Confirmed via AddressSanitizer:
     * heap-buffer-overflow reading past a 7200-byte (900-element, sized for 15 kHz)
     * region in chest_dl_estimate_correct_sync_error's srsran_vec_prod_conj_ccc call,
     * for a subframe using 1.25 kHz (needs 1200 elements at 50 PRB) - the exact MCCH
     * scenario that made MCCH decode fail with a corrupted channel estimate. */
    for (p = 0; p < 2; p++) {
      for (i = 0; i < SRSRAN_MBSFN_NOF_SF_40MS; i++) {
        q->pilots[p][i] = srsran_vec_cf_malloc(SRSRAN_REFSIGNAL_MAX_NUM_SF_MBSFN(max_prb));
        if (!q->pilots[p][i]) {
          perror("malloc");
          goto free_and_exit;
        }
      }
    }

    ret = SRSRAN_SUCCESS;
  }

free_and_exit:
  if (ret == SRSRAN_ERROR) {
    srsran_refsignal_free(q);
  }
  return ret;
}

int srsran_refsignal_mbsfn_set_cell(srsran_refsignal_t* q, srsran_cell_t cell, uint16_t mbsfn_area_id, srsran_scs_t scs)
{
  int ret = SRSRAN_SUCCESS;

  if (q == NULL) {
    ret = SRSRAN_ERROR_INVALID_INPUTS;
    goto exit;
  }

  q->cell          = cell;
  q->mbsfn_area_id = mbsfn_area_id;
  if (srsran_refsignal_mbsfn_gen_seq(q, q->cell, q->mbsfn_area_id, scs) < SRSRAN_SUCCESS) {
    ret = SRSRAN_ERROR;
    goto exit;
  }

exit:
  return ret;
}

int srsran_refsignal_mbsfn_get_sf(srsran_cell_t cell, uint32_t port_id, cf_t* sf_symbols, cf_t* pilots, srsran_scs_t scs, uint32_t tti)
{
  uint32_t i, l;
  uint32_t fidx;
  uint32_t nsymbol;
  uint32_t nonmbsfn_offset = 0;

  if (srsran_cell_isvalid(&cell) && srsran_portid_isvalid(port_id) && pilots != NULL && sf_symbols != NULL) {
    if (scs == SRSRAN_SCS_15KHZ) {
      // getting refs from non mbsfn section of subframe
      nsymbol = srsran_refsignal_cs_nsymbol(0, cell.cp, port_id);
      fidx    = ((srsran_refsignal_cs_v(port_id, 0) + (cell.id % 6)) % 6);
      for (i = 0; i < 2 * cell.nof_prb; i++) {
        pilots[SRSRAN_REFSIGNAL_PILOT_IDX(i, 0, cell)] = sf_symbols[SRSRAN_RE_IDX(cell.nof_prb, nsymbol, fidx)];
        fidx += SRSRAN_NRE / 2; // 2 references per PRB
      }
      nonmbsfn_offset = 2 * cell.nof_prb;
    }

    /* ns must match put_sf's ns (40 ms period, 13 slots of 3 ms, first slot absorbs the
     * extra TTI) or this reads the RS from the wrong subcarriers whenever the eNB used a
     * different stagger than a plain tti/3 would predict (from tti=3 onward each period). */
    uint32_t pos40 = tti % 40u;
    /* TS 36.211 §6.10.2.2.4: the stagger's "ns" is the ABSOLUTE 3ms slot number
     * (ns = ns' + 13*nf/4, nf = radio frame number), not the period-local slot
     * index -- folding in the period count makes the stagger phase advance by
     * one step every successive 40 ms period (13 mod 4 = 1 for SL4, 13 mod 2 = 1
     * for SL2) instead of always resetting to phase 0, as clause 6.10.2.2.4
     * requires. This does NOT apply to which row of the local 13-row pilot
     * VALUE table gets used elsewhere (that stays period-local, unaffected). */
    uint32_t ns_37 = ((pos40 > 0u) ? (pos40 - 1u) / 3u : 0u) + 13u * (tti / 40u);
    if (scs == SRSRAN_SCS_370HZ_SL4) {
      /* TS 36.211 clause 6.10.2.2.4 type1: k = 12*m' + stagger, stagger = 3*(ns mod 4).
       * Pilot spacing 12 does not divide NscRB=486, so per-RB framework is not used.
       * Use act_prb (mbsfn_prb or nof_prb) to match put_sf; k must stay within sf_symbols. */
      uint32_t act_prb_get  = cell.mbsfn_prb ? cell.mbsfn_prb : cell.nof_prb;
      uint32_t stagger      = 3 * (ns_37 % 4);
      uint32_t total_pilots = (SRSRAN_NRE_SCS_370HZ * act_prb_get) / 12;
      for (i = 0; i < total_pilots; i++) {
        uint32_t k = 12 * i + stagger;
        pilots[i + nonmbsfn_offset] = sf_symbols[SRSRAN_RE_IDX_MBSFN(cell.nof_prb, 0, k, SRSRAN_SCS_370HZ)];
      }
    } else {
      /* For extended BW (mbsfn_prb < nof_prb), RS covers only the MBSFN allocation. */
      uint32_t act_prb_get = cell.mbsfn_prb ? cell.mbsfn_prb : cell.nof_prb;
      for (l = 0; l < srsran_refsignal_mbsfn_nof_symbols(scs); l++) {
        nsymbol = srsran_refsignal_mbsfn_nsymbol(l, scs);
        if (scs == SRSRAN_SCS_1KHZ25) {
          /* TS 36.211 §6.10.2.2.2 "Mapping to resource elements for 1.25 kHz": stagger
           * is 3*(n_sf mod 2), n_sf = subframe number within the radio frame (0..9,
           * changes every 1 ms) - not sfn=tti/10 (changes every 10 ms; wrong clause
           * was cited too). tti%2 == n_sf%2. Must match put_sf's fidx and the
           * chest_dl.c interpolator's fidx_offset exactly, or this reads pilots from
           * the wrong subcarriers on every other subframe. */
          fidx = tti%2==0 ? 0 : 3;
        } else if (scs == SRSRAN_SCS_370HZ_SL2) {
          /* TS 36.211 clause 6.10.2.2.4 type2: stagger = 3*(ns mod 2). */
          fidx = 3 * (ns_37 % 2);
        } else {
          fidx = srsran_refsignal_mbsfn_fidx(l, scs);
        }
        for (i = 0; i < srsran_refsignal_mbsfn_rs_per_symbol(scs) * act_prb_get; i++) {
          pilots[SRSRAN_REFSIGNAL_PILOT_IDX_MBSFN(i, l, cell, scs) + nonmbsfn_offset] =
            sf_symbols[SRSRAN_RE_IDX_MBSFN(cell.nof_prb, nsymbol, fidx, scs)];
          fidx += SRSRAN_NRE_SCS(scs) / srsran_refsignal_mbsfn_rs_per_symbol(scs);
        }
      }
    }

    return SRSRAN_SUCCESS;
  } else {
    return SRSRAN_ERROR_INVALID_INPUTS;
  }
}
