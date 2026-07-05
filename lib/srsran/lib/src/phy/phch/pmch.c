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

#include <assert.h>
#include <complex.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

#include "prb_dl.h"
#include "srsran/phy/common/phy_common.h"
#include "srsran/phy/common/sequence.h"
#include "srsran/phy/phch/pmch.h"
#include "srsran/phy/utils/bit.h"
#include "srsran/phy/utils/debug.h"
#include "srsran/phy/utils/vector.h"

/* Maximum data RE per PRB per subframe across all MBSFN SCS.
 * SL4 (0.37 kHz) has 486 sc/PRB × 1 symbol = 486, which exceeds the
 * 15 kHz MBSFN value of 2 slots × 6 symbols × 12 NRE = 144. */
#define MAX_PMCH_RE SRSRAN_NRE_SCS_370HZ

/* Rel-19 adds 256QAM for PMCH (TS 36.213 Table 11.1-2). */
const static srsran_mod_t modulations[5] = {
    SRSRAN_MOD_BPSK, SRSRAN_MOD_QPSK, SRSRAN_MOD_16QAM, SRSRAN_MOD_64QAM, SRSRAN_MOD_256QAM
};

/* =========================================================================
 * Rel-19 LTE_terr_bcast_Ph2 helpers (TS 36.211 §§6.5.1–6.5.2)
 * ========================================================================= */

static uint32_t pmch_gcd(uint32_t a, uint32_t b)
{
    while (b) { uint32_t t = b; b = a % b; a = t; }
    return a;
}

/* TS 36.306 Table 4.1-1 ("Total number of soft channel bits" column), for
 * PMCH-SoftBufferSizeParameters-r19's N_soft-per-UE-category lookup (TS
 * 36.212 §5.1.4.1.2's N_IR = floor(scalingFactorBeta * N_soft / M)). Verified
 * directly from the primary source (TS 36.306 V19.3.0, 2026-06, page 31) --
 * not derived/guessed. Index 0 is unused (categories are 1-based); 0 return
 * value means "unsupported category, caller must fall back to the uncapped
 * K_w" (categories beyond 15, and the special BL/CE M1/M2/0/1bis categories,
 * are out of scope for a broadcast reference category and not included). */
static const uint32_t pmch_n_soft_table[16] = {
    0,        /* (unused, categories are 1-based) */
    250368,   /* Category 1 */
    1237248,  /* Category 2 */
    1237248,  /* Category 3 */
    1827072,  /* Category 4 */
    3667200,  /* Category 5 */
    3654144,  /* Category 6 */
    3654144,  /* Category 7 */
    35982720, /* Category 8 */
    5481216,  /* Category 9 */
    5481216,  /* Category 10 */
    7308288,  /* Category 11 */
    7308288,  /* Category 12 */
    3654144,  /* Category 13 */
    47431680, /* Category 14 */
    9744384,  /* Category 15 */
};

static uint32_t srsran_pmch_n_soft_for_category(uint8_t category)
{
  if (category == 0 || category >= sizeof(pmch_n_soft_table) / sizeof(pmch_n_soft_table[0])) {
    return 0;
  }
  return pmch_n_soft_table[category];
}

/* TS 36.212 §5.1.4.1.2: N_cb = min(floor(N_IR/C), K_w), where
 * N_IR = floor(scalingFactorBeta * N_soft / M). Returns 0 (meaning "no real
 * cap, caller falls back to the uncapped K_w") when n_soft_ref_category is 0
 * or unsupported, beta_den is 0, M is 0, or C is 0 -- any of which mean the
 * inputs aren't (yet) meaningfully configured, not that N_cb should become 0. */
static uint32_t srsran_pmch_n_cb_cap(uint8_t n_soft_ref_category,
                                      uint8_t beta_num,
                                      uint8_t beta_den,
                                      uint8_t M,
                                      uint32_t C)
{
  uint32_t n_soft = srsran_pmch_n_soft_for_category(n_soft_ref_category);
  if (n_soft == 0 || beta_den == 0 || M == 0 || C == 0) {
    return 0;
  }
  uint32_t n_ir = ((uint32_t)beta_num * n_soft) / ((uint32_t)beta_den * (uint32_t)M);
  return n_ir / C;
}

/* Compute per-OFDM-symbol data RE count for PMCH.
 * Fills sym_re[0..L-1] and returns L (number of PMCH symbols). */
static uint32_t pmch_sym_re_per_sym(srsran_pmch_t* q, srsran_scs_t scs, uint32_t lstart, uint32_t sf,
                                     uint32_t* sym_re, uint32_t max_syms)
{
    uint32_t nof_refs  = srsran_refsignal_mbsfn_rs_per_symbol(scs);
    uint32_t nre       = SRSRAN_NRE_SCS(scs);
    uint32_t act_prb   = q->cell.mbsfn_prb ? q->cell.mbsfn_prb : q->cell.nof_prb;
    uint32_t L         = 0;

    /* SL4 type-1: NRE_SCS=486 is not a multiple of 12, so per-PRB RS count
     * alternates between even and odd PRBs depending on the global stagger.
     * Use exact per-PRB formula to match the TX interleaver. */
    uint32_t sl4_rs_sym_re = 0;
    if (scs == SRSRAN_SCS_370HZ_SL4) {
        uint32_t g  = 3u * ((sf / 3u) % 4u);  /* global stagger; sf = absolute TTI */
        uint32_t s1 = g % 12u;                  /* local stagger for even-indexed PRBs */
        uint32_t s2 = (g + 6u) % 12u;           /* local stagger for odd-indexed PRBs */
        uint32_t rs1 = (nre - s1 + 11u) / 12u;  /* RS count per even PRB */
        uint32_t rs2 = (nre - s2 + 11u) / 12u;  /* RS count per odd PRB */
        uint32_t n1  = (act_prb + 1u) / 2u;     /* ceil(act_prb/2) even PRBs */
        uint32_t n2  = act_prb / 2u;             /* floor(act_prb/2) odd PRBs */
        sl4_rs_sym_re = act_prb * nre - (n1 * rs1 + n2 * rs2);
    }

    for (uint32_t s = 0; s < SRSRAN_MBSFN_NOF_SLOTS(scs); s++) {
        uint32_t ls = (s == 0) ? lstart : 0;
        for (uint32_t l = ls; l < SRSRAN_MBSFN_NOF_SYMBOLS(scs); l++) {
            if (L < max_syms) {
                if (SRSRAN_SYMBOL_HAS_REF_MBSFN_SCS(l, s, scs)) {
                    if (scs == SRSRAN_SCS_370HZ_SL4) {
                        sym_re[L] = sl4_rs_sym_re;
                    } else if (scs == SRSRAN_SCS_370HZ_SL2) {
                        /* SL2: RS spacing=6, 486/6=81 RS per 486-sc; exact for any prb. */
                        sym_re[L] = act_prb * (nre - 81u);
                    } else {
                        sym_re[L] = act_prb * (nre - nof_refs);
                    }
                } else {
                    sym_re[L] = act_prb * nre;
                }
            }
            L++;
        }
    }
    return L;
}

/* TS 36.211 §6.5.1: left cyclic shift — d'(k) = d((k + Xi) mod Mbit).
 * TX shifts left by Xi; RX undoes it (shift by Mbit-Xi) after descrambling. */
static void pmch_cyclic_shift_bits(uint8_t* bit_buf, uint32_t Mbit_bits, uint32_t Xi)
{
    if (Mbit_bits == 0) return;
    Xi = Xi % Mbit_bits;
    if (Xi == 0) return;
    uint32_t nbytes = (Mbit_bits + 7) / 8;
    uint8_t* tmp    = malloc(nbytes);
    if (!tmp) return;
    memcpy(tmp, bit_buf, nbytes);
    /* Left shift: first Xi bits move to the back. */
    srsran_bit_copy(bit_buf, 0,            tmp, Xi, Mbit_bits - Xi);
    srsran_bit_copy(bit_buf, Mbit_bits - Xi, tmp, 0, Xi);
    free(tmp);
}

/* RX anti-shift: undo the TX left cyclic shift on the soft LLR buffer.
 * LLR'[k] = LLR[(k + Xi) % Mbit]  (shift by anti_Xi = Mbit - Xi). */
static void pmch_cyclic_shift_llr(int16_t* llr_buf, uint32_t Mbit_bits, uint32_t anti_Xi)
{
    if (Mbit_bits == 0) return;
    anti_Xi = anti_Xi % Mbit_bits;
    if (anti_Xi == 0) return;
    int16_t* tmp = malloc(Mbit_bits * sizeof(int16_t));
    if (!tmp) return;
    memcpy(tmp, llr_buf, Mbit_bits * sizeof(int16_t));
    memcpy(llr_buf,                      tmp + anti_Xi, (Mbit_bits - anti_Xi) * sizeof(int16_t));
    memcpy(llr_buf + (Mbit_bits - anti_Xi), tmp,        anti_Xi               * sizeof(int16_t));
    free(tmp);
}

/* TS 36.211 §6.5.1: compute cyclic shift amount Xi for subframe i.
 * Er[r] = rate-matched bits for codeblock r ≈ floor or ceil of Mbit/C. */
static uint32_t pmch_cyclic_shift_Xi(uint32_t Mbit_bits, uint32_t C, uint32_t i, uint32_t alpha)
{
    if (C == 0) return 0;
    uint32_t Si      = (i * alpha) % C;
    uint32_t E_floor = Mbit_bits / C;
    uint32_t rem     = Mbit_bits % C;
    uint32_t Xi = 0;
    for (uint32_t r = C - Si; r < C; r++) {
        Xi += (r >= C - rem) ? E_floor + 1 : E_floor;
    }
    return Xi;
}

/* TS 36.211 §6.5.1: map the RRC pmch-CyclicShiftAlpha enum (alpha1/alpha2/alpha3,
 * carried as 1/2/3 in cfg->cyclic_shift_alpha) to the physical-layer alpha consumed
 * by pmch_cyclic_shift_Xi's Si=i*alpha mod C. Confirmed via CR0580 (R1-2506643) and
 * CR0584 (R1-2509640) - both explicitly replace clause 6.5.1's placeholder
 * "parameter XXX"/alphaOne/alphaOther text with the final RRC names: alpha1 <->
 * alphaOne (alpha=1), alpha2 <-> alphaOther (alpha=C/(N*L)). alpha3 does not use
 * this mechanism at all; see pmch_cyclic_shift_alpha3_Xi. The extracted CR text does
 * not state a rounding rule for C/(N*L); integer division is used here (assumes C is
 * chosen to divide evenly by N*L for valid configurations). */
static uint32_t pmch_cyclic_shift_alpha_value(uint8_t rrc_alpha, uint32_t C, uint32_t N, uint32_t L)
{
    if (rrc_alpha == 1 || N == 0 || L == 0) {
        return 1; /* alpha1, or a degenerate-input fallback to alphaOne */
    }
    return C / (N * L);
}

/* TS 36.211 §6.5.1, pmch-CyclicShiftAlpha=alpha3 (CR0580 R1-2506643): a pseudo-random
 * subframe-granularity shift, structurally distinct from alpha1/alpha2's Si mechanism:
 *   Xi = A_i(nf,nsf) * floor(Nsc*Qm / N)
 *   A_i(nf,nsf) = ( sum_{m=0}^{7} c(8*(10*(nf mod 128)+nsf)+m) * 2^m ) mod N
 * c(i) is the clause 7.2 Gold sequence, seeded with cinit=N_ID^MBSFN at the start of
 * every radio frame where nf mod 128 == 0 and run continuously from there (i counts
 * bits since that reset, 8 consumed per subframe). Nsc = Mbit_sf/(Qm*L) per the CR's
 * own definition (not a direct RE count). Reconstructed from the CR's raw OOXML math
 * markup (fraction/floor/summation structure survives there even though the CR's
 * flattened text loses the operators), not guessed from the flattened text alone. */
static uint32_t pmch_cyclic_shift_alpha3_Xi(
    uint32_t nf, uint32_t nsf, uint32_t N, uint32_t Mbit_sf, uint32_t Qm_bits, uint32_t L, uint32_t n_id_mbsfn)
{
    if (N == 0 || Qm_bits == 0 || L == 0) {
        return 0;
    }
    uint32_t subf_ix = 10u * (nf % 128u) + nsf; /* subframes since last cinit reset */
    uint32_t nbits    = 8u * subf_ix + 8u;      /* need c(0..8*subf_ix+7) */

    srsran_sequence_t seq;
    if (srsran_sequence_LTE_pr(&seq, nbits, n_id_mbsfn) != SRSRAN_SUCCESS) {
        return 0;
    }
    uint32_t A_i = 0;
    for (uint32_t m = 0; m < 8; m++) {
        A_i += ((uint32_t)seq.c[8u * subf_ix + m]) << m;
    }
    srsran_sequence_free(&seq);
    A_i %= N;

    uint32_t Nsc = Mbit_sf / (Qm_bits * L);
    return A_i * ((Nsc * Qm_bits) / N);
}

/* TS 36.211 §6.5.2 row permutation: fill perm[0..R-1] with the row index sequence r_i. */
static void pmch_interleave_row_perm(uint32_t R, uint32_t K, uint32_t* perm)
{
    if (R == 0) return;
    /* d = floor( R / floor( R / floor( sqrt(R*K) - K + 1 ) ) ) */
    double sqrtRK = sqrt((double)R * K);
    int    inner  = (int)floor(sqrtRK - K + 1);
    if (inner < 1) inner = 1;
    uint32_t mid  = (uint32_t)(R / (uint32_t)inner);
    if (mid < 1)  mid = 1;
    uint32_t d    = R / mid;
    if (d == 0)   d = 1;

    perm[0] = 0;
    for (uint32_t i = 1; i < R; i++) {
        if (perm[i-1] + d <= R - 1) {
            perm[i] = perm[i-1] + d;
        } else {
            perm[i] = (uint32_t)(((uint64_t)i * d) / R);
        }
    }
}

/* Column cyclic shift offset: sr = (-1)^r * floor((r+1)/2) mod K */
static uint32_t pmch_col_shift(uint32_t r, uint32_t K)
{
    if (K == 0) return 0;
    int s = (int)((r + 1) / 2);
    if (r % 2 == 1) s = -s;
    int sr = s % (int)K;
    if (sr < 0) sr += (int)K;
    return (uint32_t)sr;
}

/* TS 36.211 §6.5.2: apply or reverse frequency-domain interleaving in-place on q->d.
 * inverse=false → TX interleave; inverse=true → RX de-interleave. */
static void pmch_freq_interleave(cf_t* symbols, uint32_t C, uint32_t L,
                                  const uint32_t* sym_re, bool inverse)
{
    if (C == 0 || L == 0) return;
    uint32_t K = C / pmch_gcd(L, C);

    uint32_t offset = 0;
    for (uint32_t l = 0; l < L; l++) {
        uint32_t Ml = sym_re[l];
        if (Ml == 0) { continue; }

        uint32_t R        = (Ml + K - 1) / K;
        uint32_t grid_sz  = R * K;

        uint32_t* perm = malloc(R * sizeof(uint32_t));
        uint32_t* sr   = malloc(R * sizeof(uint32_t));
        pmch_interleave_row_perm(R, K, perm);
        for (uint32_t r = 0; r < R; r++) sr[r] = pmch_col_shift(r, K);

        cf_t* grid      = calloc(grid_sz, sizeof(cf_t));
        bool* grid_null = calloc(grid_sz, sizeof(bool));

        if (!inverse) {
            /* TX: write col-wise → row-perm → col-shift → read row-wise */
            for (uint32_t i = 0; i < grid_sz; i++) {
                uint32_t row = i % R, col = i / R;
                grid[row * K + col]      = (i < Ml) ? symbols[offset + i] : 0;
                grid_null[row * K + col] = (i >= Ml);
            }
            cf_t* grid2      = malloc(grid_sz * sizeof(cf_t));
            bool* grid2_null = malloc(grid_sz * sizeof(bool));
            for (uint32_t i = 0; i < R; i++) {
                memcpy(&grid2[i * K],      &grid[perm[i] * K],      K * sizeof(cf_t));
                memcpy(&grid2_null[i * K], &grid_null[perm[i] * K], K * sizeof(bool));
            }
            for (uint32_t r = 0; r < R; r++) {
                uint32_t s = sr[r];
                if (s == 0) {
                    memcpy(&grid[r * K],      &grid2[r * K],      K * sizeof(cf_t));
                    memcpy(&grid_null[r * K], &grid2_null[r * K], K * sizeof(bool));
                } else {
                    for (uint32_t c = 0; c < K; c++) {
                        uint32_t src = (c + K - s) % K;
                        grid[r * K + c]      = grid2[r * K + src];
                        grid_null[r * K + c] = grid2_null[r * K + src];
                    }
                }
            }
            free(grid2); free(grid2_null);
            uint32_t out_idx = 0;
            for (uint32_t r = 0; r < R && out_idx < Ml; r++) {
                for (uint32_t c = 0; c < K && out_idx < Ml; c++) {
                    if (!grid_null[r * K + c]) {
                        symbols[offset + out_idx++] = grid[r * K + c];
                    }
                }
            }
        } else {
            /* RX inverse: write row-wise (skipping NULL) → undo col-shift → undo row-perm → read col-wise */
            uint32_t* inv_perm = malloc(R * sizeof(uint32_t));
            for (uint32_t i = 0; i < R; i++) inv_perm[perm[i]] = i;
            memset(grid_null, false, grid_sz * sizeof(bool));
            for (uint32_t i = Ml; i < grid_sz; i++) {
                uint32_t row_orig = i % R, col_orig = i / R;
                uint32_t new_row  = inv_perm[row_orig];
                uint32_t new_col  = (col_orig + sr[new_row]) % K;
                grid_null[new_row * K + new_col] = true;
            }
            free(inv_perm);
            uint32_t in_idx = 0;
            for (uint32_t r = 0; r < R && in_idx < Ml; r++) {
                for (uint32_t c = 0; c < K && in_idx < Ml; c++) {
                    if (!grid_null[r * K + c]) {
                        grid[r * K + c] = symbols[offset + in_idx++];
                    } else {
                        grid[r * K + c] = 0;
                    }
                }
            }
            cf_t* grid2 = malloc(grid_sz * sizeof(cf_t));
            for (uint32_t r = 0; r < R; r++) {
                uint32_t s = sr[r];
                if (s == 0) {
                    memcpy(&grid2[r * K], &grid[r * K], K * sizeof(cf_t));
                } else {
                    for (uint32_t c = 0; c < K; c++) {
                        uint32_t src = (c + s) % K;
                        grid2[r * K + c] = grid[r * K + src];
                    }
                }
            }
            for (uint32_t i = 0; i < R; i++) {
                memcpy(&grid[perm[i] * K], &grid2[i * K], K * sizeof(cf_t));
            }
            free(grid2);
            for (uint32_t i = 0; i < Ml; i++) {
                uint32_t row = i % R, col = i / R;
                symbols[offset + i] = grid[row * K + col];
            }
        }

        free(perm); free(sr); free(grid); free(grid_null);
        offset += Ml;
    }
}

/* =========================================================================
 * End of Rel-19 helpers
 * ========================================================================= */

/* TS 36.211 §6.10.2.2.4 SL4 RS type 1: pilots at k = stagger + 12*i (step=12).
 * 486 is not divisible by 12 so the last interval is shorter — prb_cp_ref_scs
 * uses (486/41)-1=10 as the step, which is off by 1 and misaligns all pilots
 * after the first.  This helper walks the 486 subcarriers directly. */
static void pmch_cp_sl4_prb(cf_t** in_ptr, cf_t** out_ptr, uint32_t stagger, bool advance_output)
{
  uint32_t nre = SRSRAN_NRE_SCS_370HZ; /* 486 */
  /* Data before first RS */
  if (stagger > 0) {
    memcpy(*out_ptr, *in_ptr, stagger * sizeof(cf_t));
    *in_ptr += stagger; *out_ptr += stagger;
  }
  /* Walk RS positions: k_rs = stagger, stagger+12, stagger+24, ... */
  for (uint32_t k_rs = stagger; k_rs < nre; k_rs += 12) {
    /* Skip the RS at k_rs */
    if (advance_output) { (*out_ptr)++; } else { (*in_ptr)++; }
    /* Data between this RS (exclusive) and the next RS (exclusive) or PRB end */
    uint32_t next_rs  = k_rs + 12;
    uint32_t data_end = (next_rs < nre) ? next_rs : nre;
    uint32_t data_len = data_end - (k_rs + 1);
    if (data_len > 0) {
      memcpy(*out_ptr, *in_ptr, data_len * sizeof(cf_t));
      *in_ptr += data_len; *out_ptr += data_len;
    }
  }
}

static int pmch_cp(srsran_pmch_t* q, cf_t* input, cf_t* output, uint32_t lstart_grant, bool put, srsran_scs_t scs, uint32_t tti)
{
  uint32_t s, n, l, lp, lstart, lend, nof_refs;
  cf_t *   in_ptr = input, *out_ptr = output;
  uint32_t offset = 0;

#ifdef DEBUG_IDX
  indices_ptr = 0;
  if (put) {
    offset_original = output;
  } else {
    offset_original = input;
  }
#endif
  nof_refs             = srsran_refsignal_mbsfn_rs_per_symbol(scs);
  uint32_t act_prb_cp  = q->cell.mbsfn_prb ? q->cell.mbsfn_prb : q->cell.nof_prb;
  for (s = 0; s < SRSRAN_MBSFN_NOF_SLOTS(scs); s++) {
    for (l = 0; l < SRSRAN_MBSFN_NOF_SYMBOLS(scs); l++) {
      for (n = 0; n < act_prb_cp; n++) {
        // If this PRB is assigned
        if (s == 0) {
          lstart = lstart_grant;
        } else {
          lstart = 0;
        }
        lend = SRSRAN_MBSFN_NOF_SYMBOLS(scs);
        lp   = l + s * SRSRAN_MBSFN_NOF_SYMBOLS(scs);
        /* Symbol stride is nof_prb (full OFDM bandwidth), not act_prb_cp (PMCH bandwidth).
         * sf_symbols has nof_prb*NRE_SCS samples per MBSFN symbol; using act_prb_cp as
         * stride corrupts all symbols beyond the first when mbsfn_prb < nof_prb. */
        if (put) {
          out_ptr = &output[(lp * q->cell.nof_prb + n) * SRSRAN_NRE_SCS(scs)];
        } else {
          in_ptr = &input[(lp * q->cell.nof_prb + n) * SRSRAN_NRE_SCS(scs)];
        }
        // This is a symbol in a normal PRB with or without references
        if (l >= lstart && l < lend) {
          if (SRSRAN_SYMBOL_HAS_REF_MBSFN_SCS(l, s, scs)) {
            offset = srsran_refsignal_mbsfn_offset(l, s, tti, scs);
            if (scs == SRSRAN_SCS_370HZ_SL4) {
              /* SL4 RS are placed globally at k = global_stagger + 12*i across
               * the full carrier.  NRE_SCS=486 is not a multiple of 12 (486%12=6),
               * so the local RS start within PRB n shifts by (n*6)%12 relative to
               * the global stagger.  Pass the per-PRB local stagger so RS holes
               * align with the pilots written by put_sf. */
              uint32_t prb_stagger = (offset + n * (SRSRAN_NRE_SCS_370HZ % 12u)) % 12u;
              pmch_cp_sl4_prb(&in_ptr, &out_ptr, prb_stagger, put);
            } else {
              prb_cp_ref_scs(&in_ptr, &out_ptr, offset, nof_refs, nof_refs, put, scs);
            }
          } else {
            prb_cp_scs(&in_ptr, &out_ptr, 1, scs);
          }
        }
      }
    }
  }

  int r;
  if (put) {
    r = abs((int)(input - in_ptr));
  } else {
    r = abs((int)(output - out_ptr));
  }

  return r;
}

/**
 * Puts PMCH in slot number 1
 *
 * Returns the number of symbols written to sf_symbols
 *
 * 36.211 10.3 section 6.3.5
 */
static int pmch_put(srsran_pmch_t* q, cf_t* symbols, cf_t* sf_symbols, srsran_scs_t scs, uint32_t lstart, uint32_t tti)
{
  return pmch_cp(q, symbols, sf_symbols, lstart, true, scs, tti);
}

/**
 * Extracts PMCH from slot number 1
 *
 * Returns the number of symbols written to PMCH
 *
 * 36.211 10.3 section 6.3.5
 */
static int pmch_get(srsran_pmch_t* q, cf_t* sf_symbols, cf_t* symbols, uint32_t lstart, srsran_scs_t scs, uint32_t tti)
{
  return pmch_cp(q, sf_symbols, symbols, lstart, false, scs, tti);
}

int srsran_pmch_init(srsran_pmch_t* q, uint32_t max_prb, uint32_t nof_rx_antennas)
{
  int ret = SRSRAN_ERROR_INVALID_INPUTS;

  if (q != NULL && nof_rx_antennas <= SRSRAN_MAX_PORTS) {
    bzero(q, sizeof(srsran_pmch_t));
    ret = SRSRAN_ERROR;

    q->cell.nof_prb    = max_prb;
    q->cell.nof_ports  = 1;
    q->max_re          = max_prb * MAX_PMCH_RE;
    q->nof_rx_antennas = nof_rx_antennas;

    INFO("Init PMCH: %d PRBs, max_symbols: %d", max_prb, q->max_re);

    for (int i = 0; i < 5; i++) {
      if (srsran_modem_table_lte(&q->mod[i], modulations[i])) {
        goto clean;
      }
      srsran_modem_table_bytes(&q->mod[i]);
    }

    srsran_sch_init(&q->dl_sch);

    // Allocate int16_t for reception (LLRs)
    q->e = srsran_vec_i16_malloc(q->max_re * srsran_mod_bits_x_symbol(SRSRAN_MOD_256QAM));
    if (!q->e) {
      goto clean;
    }

    /* Rel-19 time interleaving: ti_rx_buf sized for max N=16 subframes (see
     * pmch.h - unused by the current decode path, left allocated). Per-slot
     * ti_tx_buf[m] / ti_decoded[m] are lazily allocated on first use instead
     * (see srsran_pmch_encode) - bzero above already left them NULL/false. */
    q->ti_buf_nre = q->max_re;
    uint32_t ti_buf_bits = 16 * q->ti_buf_nre * srsran_mod_bits_x_symbol(SRSRAN_MOD_256QAM);
    q->ti_rx_buf = calloc(ti_buf_bits, sizeof(int16_t));
    if (!q->ti_rx_buf) {
      goto clean;
    }

    q->d = srsran_vec_cf_malloc(q->max_re);
    if (!q->d) {
      goto clean;
    }

    for (int i = 0; i < SRSRAN_MAX_PORTS; i++) {
      q->x[i] = srsran_vec_cf_malloc(q->max_re);
      if (!q->x[i]) {
        goto clean;
      }
      for (int j = 0; j < q->nof_rx_antennas; j++) {
        q->ce[i][j] = srsran_vec_cf_malloc(q->max_re);
        if (!q->ce[i][j]) {
          goto clean;
        }
      }
    }
    for (int j = 0; j < q->nof_rx_antennas; j++) {
      q->symbols[j] = srsran_vec_cf_malloc(q->max_re);
      if (!q->symbols[j]) {
        goto clean;
      }
    }

    q->seqs = calloc(SRSRAN_MAX_MBSFN_AREA_IDS, sizeof(srsran_pmch_seq_t*));
    if (!q->seqs) {
      perror("calloc");
      goto clean;
    }

    ret = SRSRAN_SUCCESS;
  }
clean:
  if (ret == SRSRAN_ERROR) {
    srsran_pmch_free(q);
  }
  return ret;
}

void srsran_pmch_free(srsran_pmch_t* q)
{
  if (q->e) {
    free(q->e);
  }
  if (q->d) {
    free(q->d);
  }
  if (q->ti_rx_buf) {
    free(q->ti_rx_buf);
  }
  for (uint32_t i = 0; i < SRSRAN_PMCH_MAX_TI_M; i++) {
    if (q->ti_tx_buf[i]) {
      free(q->ti_tx_buf[i]);
    }
  }
  for (uint32_t i = 0; i < SRSRAN_MAX_PORTS; i++) {
    if (q->x[i]) {
      free(q->x[i]);
    }
    for (uint32_t j = 0; j < q->nof_rx_antennas; j++) {
      if (q->ce[i][j]) {
        free(q->ce[i][j]);
      }
    }
  }
  for (uint32_t i = 0; i < q->nof_rx_antennas; i++) {
    if (q->symbols[i]) {
      free(q->symbols[i]);
    }
  }
  if (q->seqs) {
    for (uint32_t i = 0; i < SRSRAN_MAX_MBSFN_AREA_IDS; i++) {
      if (q->seqs[i]) {
        srsran_pmch_free_area_id(q, i);
      }
    }
    free(q->seqs);
  }
  for (uint32_t i = 0; i < 5; i++) {
    srsran_modem_table_free(&q->mod[i]);
  }

  srsran_sch_free(&q->dl_sch);

  bzero(q, sizeof(srsran_pmch_t));
}

int srsran_pmch_set_cell(srsran_pmch_t* q, srsran_cell_t cell)
{
  int ret = SRSRAN_ERROR_INVALID_INPUTS;

  if (q != NULL && srsran_cell_isvalid(&cell)) {
    q->cell   = cell;
    uint32_t act_prb = cell.mbsfn_prb ? cell.mbsfn_prb : cell.nof_prb;
    q->max_re = act_prb * MAX_PMCH_RE;  /* MAX_PMCH_RE=486 covers all SCS; use act_prb for extended BW */

    INFO("PMCH: Cell config PCI=%d, %d ports, %d PRBs (%d MBSFN PRBs), max_symbols: %d",
         q->cell.nof_ports,
         q->cell.id,
         q->cell.nof_prb,
         act_prb,
         q->max_re);

    ret = SRSRAN_SUCCESS;
  }
  return ret;
}

/* Precalculate the scramble sequences for a given MBSFN area ID. This function takes a while
 * to execute.
 */
int srsran_pmch_set_area_id(srsran_pmch_t* q, uint16_t area_id)
{
  uint32_t i;
  if (!q->seqs[area_id]) {
    q->seqs[area_id] = calloc(1, sizeof(srsran_pmch_seq_t));
    if (q->seqs[area_id]) {
      for (i = 0; i < SRSRAN_NOF_SF_X_FRAME; i++) {
        if (srsran_sequence_pmch(
                &q->seqs[area_id]->seq[i], 2 * i, area_id, q->max_re * srsran_mod_bits_x_symbol(SRSRAN_MOD_256QAM))) {
          return SRSRAN_ERROR;
        }
      }
    }
  }
  return SRSRAN_SUCCESS;
}

void srsran_pmch_free_area_id(srsran_pmch_t* q, uint16_t area_id)
{
  if (q->seqs[area_id]) {
    for (int i = 0; i < SRSRAN_NOF_SF_X_FRAME; i++) {
      srsran_sequence_free(&q->seqs[area_id]->seq[i]);
    }
    free(q->seqs[area_id]);
    q->seqs[area_id] = NULL;
  }
}

/** Decodes the pmch from the received symbols
 */
int srsran_pmch_decode(srsran_pmch_t*         q,
                       srsran_dl_sf_cfg_t*    sf,
                       srsran_pmch_cfg_t*     cfg,
                       srsran_chest_dl_res_t* channel,
                       cf_t*                  sf_symbols[SRSRAN_MAX_PORTS],
                       srsran_pdsch_res_t*    out)
{
  uint32_t i, n;

  if (q != NULL && sf_symbols != NULL && out != NULL && cfg != NULL) {
    INFO("Decoding PMCH SF: %d, MBSFN area ID: 0x%x, Mod %s, TBS: %d, NofSymbols: %d, NofBitsE: %d, rv_idx: %d, "
         "C_prb=%d, cfi=%d",
         sf->tti % 10,
         cfg->area_id,
         srsran_mod_string(cfg->pdsch_cfg.grant.tb[0].mod),
         cfg->pdsch_cfg.grant.tb[0].tbs,
         cfg->pdsch_cfg.grant.nof_re,
         cfg->pdsch_cfg.grant.tb[0].nof_bits,
         0,
         cfg->pdsch_cfg.grant.nof_prb,
         sf->cfi);

    /* All FeMBMS SCS types (0.37/1.25/2.5/7.5 kHz) have no control region; lstart=0.
     * Standard 15 kHz MBSFN uses sf->cfi control symbols. */
    uint32_t lstart = (sf->subcarrier_spacing != SRSRAN_SCS_15KHZ) ? 0u
                                                                    : SRSRAN_NOF_CTRL_SYMBOLS(q->cell, sf->cfi);
    for (int j = 0; j < q->nof_rx_antennas; j++) {
      /* extract symbols */
      n = pmch_get(q, sf_symbols[j], q->symbols[j], lstart, sf->subcarrier_spacing, sf->tti);
      if (n != cfg->pdsch_cfg.grant.nof_re) {
        ERROR("PMCH 1 extract symbols error expecting %d symbols but got %d, lstart %d",
              cfg->pdsch_cfg.grant.nof_re,
              n,
              lstart);
        return SRSRAN_ERROR;
      }

      /* extract channel estimates */
      for (i = 0; i < q->cell.nof_ports; i++) {
        /* DIAG (PMCH_RE_DUMP): dump the FULL, per-RE, post-interpolation channel
         * estimate grid (before pmch_get compacts it to data-only REs) so it can
         * be inspected directly against the sparse pilot_estimates dump added in
         * chest_dl.c, to check whether interpolate_pilots() fills the full grid
         * smoothly/consistently or scrambles/misindexes between pilot positions. */
        if (getenv("PMCH_RE_DUMP") && i == 0 && j == 0 && sf->subcarrier_spacing != SRSRAN_SCS_15KHZ) {
          uint32_t dump_n = SRSRAN_NRE_SCS(sf->subcarrier_spacing) * q->cell.nof_prb;
          char     fn[128];
          snprintf(fn, sizeof(fn), "/tmp/pmch_rx_fullce_tti%u.bin", sf->tti);
          FILE* ffce = fopen(fn, "wb");
          if (ffce) {
            fwrite(channel->ce[0][0], sizeof(cf_t), dump_n, ffce);
            fclose(ffce);
          }
          fprintf(stderr, "[PMCH_RE_DUMP] DIAG fullce tti=%u dump_n=%u\n", sf->tti, dump_n);
        }
        n = pmch_get(q, channel->ce[i][j], q->ce[i][j], lstart, sf->subcarrier_spacing, sf->tti);
        if (n != cfg->pdsch_cfg.grant.nof_re) {
          ERROR("PMCH 2 extract chest error expecting %d symbols but got %d", cfg->pdsch_cfg.grant.nof_re, n);
          return SRSRAN_ERROR;
        }
      }
    }

    // No tx diversity in MBSFN
    srsran_predecoding_single_multi(q->symbols,
                                    q->ce[0],
                                    q->d,
                                    NULL,
                                    q->nof_rx_antennas,
                                    cfg->pdsch_cfg.grant.nof_re,
                                    1.0f,
                                    channel->noise_estimate);

    /* PMCH_RE_DUMP: scratch instrumentation, see the matching comment in
     * srsran_pmch_encode above. Dumps the post-equalization data symbols (same
     * point TX dumps its post-modulation symbols) so the two can be diffed
     * directly for the same tti. */
    if (getenv("PMCH_RE_DUMP")) {
      char fn[128];
      snprintf(fn, sizeof(fn), "/tmp/pmch_rx_sym_tti%u.bin", sf->tti);
      FILE* fsym = fopen(fn, "wb");
      if (fsym) {
        fwrite(q->d, sizeof(cf_t), cfg->pdsch_cfg.grant.nof_re, fsym);
        fclose(fsym);
      }
      snprintf(fn, sizeof(fn), "/tmp/pmch_rx_rawsym_tti%u.bin", sf->tti);
      FILE* fraw = fopen(fn, "wb");
      if (fraw) {
        fwrite(q->symbols[0], sizeof(cf_t), cfg->pdsch_cfg.grant.nof_re, fraw);
        fclose(fraw);
      }
      snprintf(fn, sizeof(fn), "/tmp/pmch_rx_ce_tti%u.bin", sf->tti);
      FILE* fce = fopen(fn, "wb");
      if (fce) {
        fwrite(q->ce[0][0], sizeof(cf_t), cfg->pdsch_cfg.grant.nof_re, fce);
        fclose(fce);
      }
      fprintf(stderr,
              "[PMCH_RE_DUMP] RX tti=%u area_id=%u tbs=%u mcs=%u mod=%s nof_re=%u noise_est=%e\n",
              sf->tti,
              cfg->area_id,
              cfg->pdsch_cfg.grant.tb[0].tbs,
              cfg->pdsch_cfg.grant.tb[0].mcs_idx,
              srsran_mod_string(cfg->pdsch_cfg.grant.tb[0].mod),
              cfg->pdsch_cfg.grant.nof_re,
              channel->noise_estimate);
    }

    if (SRSRAN_VERBOSE_ISDEBUG()) {
      DEBUG("SAVED FILE subframe.dat: received subframe symbols");
      srsran_vec_save_file("subframe2.dat", q->symbols[0], cfg->pdsch_cfg.grant.nof_re * sizeof(cf_t));
      DEBUG("SAVED FILE hest0.dat: channel estimates for port 4");
      printf("nof_prb=%d, cp=%d, nof_re=%d, grant_re=%d\n",
             q->cell.nof_prb,
             q->cell.cp,
             SRSRAN_NOF_RE(q->cell),
             cfg->pdsch_cfg.grant.nof_re);
      srsran_vec_save_file("hest2.dat", channel->ce[0][0], SRSRAN_NOF_RE(q->cell) * sizeof(cf_t));
      DEBUG("SAVED FILE pmch_symbols.dat: symbols after equalization");
      srsran_vec_save_file("pmch_symbols.bin", q->d, cfg->pdsch_cfg.grant.nof_re * sizeof(cf_t));
    }

    /* Rel-19 §6.5.1/§6.5.2 both need C (codeblocks) and L (PMCH OFDM symbol count);
     * compute once, only when a Rel-19 feature that needs them is actually enabled. */
    srsran_cbsegm_t cb_segm = {0};
    uint32_t        sym_re[64];
    uint32_t        L = 0;
    if (cfg->freq_interleaving || (cfg->cyclic_shift && cfg->cyclic_shift_alpha > 0)) {
      srsran_cbsegm(&cb_segm, cfg->pdsch_cfg.grant.tb[0].tbs);
      L = pmch_sym_re_per_sym(q, sf->subcarrier_spacing, 0, sf->tti, sym_re, 64);
    }

    /* Rel-19 §6.5.2: frequency-domain de-interleaving (undo TX interleave, before demodulation) */
    if (cfg->freq_interleaving) {
      pmch_freq_interleave(q->d, (uint32_t)cb_segm.C, L, sym_re, true);
    }

    /* No per-subcarrier phase de-rotation here: see the matching TX-side comment
     * below (pmch_encode) - TS 36.211 §6.5.1 has no complex-domain step, only the
     * bit-domain shift undone below via pmch_cyclic_shift_llr. */

    /* demodulate symbols
     * The MAX-log-MAP algorithm used in turbo decoding is unsensitive to SNR estimation,
     * thus we don't need tot set it in thde LLRs normalization
     */
    srsran_demod_soft_demodulate_s(cfg->pdsch_cfg.grant.tb[0].mod, q->d, q->e, cfg->pdsch_cfg.grant.nof_re);

    uint32_t Mbit_sf_rx = cfg->pdsch_cfg.grant.tb[0].nof_bits;
    uint8_t  N_rx       = cfg->time_interleaving_n;

    /* descramble */
    srsran_scrambling_s_offset(&q->seqs[cfg->area_id]->seq[sf->tti % 10], q->e, 0, Mbit_sf_rx);

    /* TS 36.211 §6.5.1: undo bit-level left cyclic shift applied by TX.
     * anti_Xi = Mbit - Xi undoes a left shift of Xi bits. */
    if (cfg->cyclic_shift && cfg->cyclic_shift_alpha > 0) {
      uint32_t Xi;
      if (cfg->cyclic_shift_alpha == 3) {
        uint32_t Qm_bits = srsran_mod_bits_x_symbol(cfg->pdsch_cfg.grant.tb[0].mod);
        Xi = pmch_cyclic_shift_alpha3_Xi(sf->tti / 10, sf->tti % 10, N_rx, Mbit_sf_rx, Qm_bits, L, cfg->area_id);
      } else {
        uint32_t alpha = pmch_cyclic_shift_alpha_value(cfg->cyclic_shift_alpha, cb_segm.C, N_rx, L);
        Xi             = pmch_cyclic_shift_Xi(Mbit_sf_rx, cb_segm.C, cfg->subframe_idx, alpha);
      }
      uint32_t anti_Xi = (Xi == 0) ? 0 : Mbit_sf_rx - Xi;
      pmch_cyclic_shift_llr((int16_t*)q->e, Mbit_sf_rx, anti_Xi);
    }

    /* PMCH_RE_DUMP: scratch instrumentation, see the matching comment in
     * srsran_pmch_encode above. Dumps the final LLRs (post-demod, post-descramble,
     * post-anti-cyclic-shift - i.e. immediately before dlsch_decode) so they can be
     * hard-thresholded and diffed bit-for-bit against TX's scrambled bit dump for
     * the same tti. */
    if (getenv("PMCH_RE_DUMP")) {
      char fn[128];
      snprintf(fn, sizeof(fn), "/tmp/pmch_rx_llr_tti%u.bin", sf->tti);
      FILE* fllr = fopen(fn, "wb");
      if (fllr) {
        fwrite(q->e, sizeof(int16_t), Mbit_sf_rx, fllr);
        fclose(fllr);
      }
      fprintf(stderr, "[PMCH_RE_DUMP] RX tti=%u Mbit_sf_rx=%u llr dumped\n", sf->tti, Mbit_sf_rx);
    }

    if (N_rx > 1) {
      /* TS 36.213 §11.1 (see the matching comment in srsran_pmch_encode below
       * for the full derivation): subframe s=cfg->subframe_idx belongs to slot
       * m=s%M with redundancy version n=(s%(N*M))/M. Attempt an INDEPENDENT
       * rate-matching + soft-combine + decode pass on EVERY subframe at
       * rv_idx=n, instead of only accumulating raw LLRs and decoding once at
       * the end of a span. This also implements the early-decode-attempt
       * behavior (TS 36.321 §5.12): the codeblock-CRC-skip machinery inside
       * decode_tb_cb (reused unchanged via srsran_dlsch_decode_mch) means
       * later calls within the same slot's span are cheap no-ops for any
       * codeblock that already succeeded.
       *
       * REQUIRES the caller (MbsfnFrameProcessor.cpp on the RX side) to keep
       * one softbuffer PER SLOT m (cfg->pdsch_cfg.softbuffers.rx[0] must point
       * at slot m's own buffer for this call), reset only when slot m starts a
       * new TB (n==0 for that slot) - otherwise this accumulation never has
       * anything to combine across calls, or combines two different slots'
       * data together. ti_decoded[m] (reset at n==0 for slot m) guards against
       * re-reporting crc=true (and so re-delivering the same TB) more than
       * once per slot per block. */
      /* Clamp defensively: time_interleaving_m ultimately derives from
       * broadcast MCCH data on the RX side - a value above the spec max
       * (only reachable via a malformed/unexpected broadcast) must not
       * become an out-of-bounds ti_tx_buf[]/ti_decoded[] access below. */
      uint8_t M = cfg->time_interleaving_m;
      if (M == 0) {
        M = 1;
      } else if (M > SRSRAN_PMCH_MAX_TI_M) {
        M = SRSRAN_PMCH_MAX_TI_M;
      }
      uint32_t block_len = (uint32_t)N_rx * (uint32_t)M;
      uint32_t s_mod      = cfg->subframe_idx % block_len;
      uint32_t slot_m      = s_mod % M;
      uint32_t slot_n      = s_mod / M;

      srsran_cbsegm_t ti_cb_segm;
      srsran_cbsegm(&ti_cb_segm, cfg->pdsch_cfg.grant.tb[0].tbs);
      uint32_t Qm_ti = srsran_mod_bits_x_symbol(cfg->pdsch_cfg.grant.tb[0].mod);
      uint32_t Gp_ti = Mbit_sf_rx / Qm_ti;
      uint32_t e_min = Qm_ti * (Gp_ti / ti_cb_segm.C);
      // TS 36.212 §5.1.4.1.2 N_cb = min(floor(N_IR/C), K_w). 0 = category/beta not
      // (yet) configured -- srsran_dlsch_decode_mch falls back to the uncapped K_w.
      uint32_t n_cb_cap = srsran_pmch_n_cb_cap(
          cfg->n_soft_ref_category, cfg->scaling_factor_beta_num, cfg->scaling_factor_beta_den, M, ti_cb_segm.C);

      if (slot_n == 0) {
        q->ti_decoded[slot_m] = false;
      }
      if (getenv("PMCH_TI_DIAG")) {
        fprintf(stderr,
                "TI_DIAG_RX q=%p sb=%p subframe_idx=%u slot_m=%u slot_n=%u\n",
                (void*)q, (void*)cfg->pdsch_cfg.softbuffers.rx[0], cfg->subframe_idx, slot_m, slot_n);
      }
      bool cb_crc_ok = (srsran_dlsch_decode_mch(&q->dl_sch,
                                                &cfg->pdsch_cfg,
                                                (int16_t*)q->e,
                                                out[0].payload,
                                                slot_n,
                                                e_min,
                                                n_cb_cap) == 0);
      if (cb_crc_ok && !q->ti_decoded[slot_m]) {
        q->ti_decoded[slot_m] = true;
        out[0].crc             = true;
      } else {
        out[0].crc = false;
      }
      out[0].avg_iterations_block = srsran_sch_last_noi(&q->dl_sch);
    } else {
      if (SRSRAN_VERBOSE_ISDEBUG()) {
        DEBUG("SAVED FILE llr.dat: LLR estimates after demodulation and descrambling");
        srsran_vec_save_file("llr.dat", q->e, Mbit_sf_rx * sizeof(int16_t));
      }
      out[0].crc                  = (srsran_dlsch_decode(&q->dl_sch, &cfg->pdsch_cfg, q->e, out[0].payload) == 0);
      out[0].avg_iterations_block = srsran_sch_last_noi(&q->dl_sch);
    }

    return SRSRAN_SUCCESS;
  } else {
    return SRSRAN_ERROR_INVALID_INPUTS;
  }
}
void srsran_configure_pmch(srsran_pmch_cfg_t* pmch_cfg, srsran_cell_t* cell, srsran_mbsfn_cfg_t* mbsfn_cfg)
{
  pmch_cfg->area_id                       = mbsfn_cfg->mbsfn_area_id;
  pmch_cfg->pdsch_cfg.rnti                = SRSRAN_MRNTI;
  pmch_cfg->pdsch_cfg.grant.nof_layers    = 1;
  pmch_cfg->pdsch_cfg.grant.nof_prb       = cell->mbsfn_prb ? cell->mbsfn_prb : cell->nof_prb;
  pmch_cfg->pdsch_cfg.grant.tb[0].mcs_idx = mbsfn_cfg->mbsfn_mcs;
  pmch_cfg->pdsch_cfg.grant.tb[0].enabled = mbsfn_cfg->enable;
  pmch_cfg->pdsch_cfg.grant.tb[0].rv      = SRSRAN_PMCH_RV;
  pmch_cfg->pdsch_cfg.grant.last_tbs[0]   = 0;
  pmch_cfg->use_mcs_table2                = mbsfn_cfg->use_mcs_table2;
  pmch_cfg->time_interleaving_n           = mbsfn_cfg->time_interleaving_n;
  pmch_cfg->time_interleaving_m           = mbsfn_cfg->time_interleaving_m;
  pmch_cfg->n_soft_ref_category           = mbsfn_cfg->n_soft_ref_category;
  pmch_cfg->scaling_factor_beta_num       = mbsfn_cfg->scaling_factor_beta_num;
  pmch_cfg->scaling_factor_beta_den       = mbsfn_cfg->scaling_factor_beta_den;
  /* Rel-19: use PMCH-specific MCS table (TS 36.213 §11.1) instead of PDSCH table */
  srsran_pmch_fill_ra_mcs(&pmch_cfg->pdsch_cfg.grant.tb[0],
                           pmch_cfg->pdsch_cfg.grant.nof_prb,
                           mbsfn_cfg->use_mcs_table2,
                           mbsfn_cfg->subcarrier_spacing);
  /* Time interleaving: scale TBS by N subframes (TS 36.213 §11.1).
   * CONVENTION: after this block, tb[0].tbs holds the TOTAL transport block size
   * across all N subframes (not the per-subframe size). The encode and decode paths
   * in this file both use tbs as the full accumulated size. CB segmentation
   * (srsran_cbsegm) is called on this total value at both ends and is therefore
   * consistent. External callers that need the per-subframe size must divide by N. */
  if (mbsfn_cfg->time_interleaving_n > 1) {
    int base_tbs = pmch_cfg->pdsch_cfg.grant.tb[0].tbs;
    int scaled   = base_tbs * (int)mbsfn_cfg->time_interleaving_n;
    /* Round to the nearest valid TBS table entry. TX and RX must use the same
     * rounding so that CB segmentation operates on identical total sizes. */
    int tbs_idx  = srsran_ra_tbs_to_table_idx((uint32_t)scaled, pmch_cfg->pdsch_cfg.grant.nof_prb,
                                               SRSRAN_RA_NOF_TBS_IDX - 1);
    if (tbs_idx >= (int)SRSRAN_RA_NOF_TBS_IDX) tbs_idx = (int)SRSRAN_RA_NOF_TBS_IDX - 1;
    if (tbs_idx < 0) tbs_idx = 0;
    pmch_cfg->pdsch_cfg.grant.tb[0].tbs = srsran_ra_tbs_from_idx((uint32_t)tbs_idx,
                                                                   pmch_cfg->pdsch_cfg.grant.nof_prb);
  }
  pmch_cfg->pdsch_cfg.grant.nof_tb     = 1;
  pmch_cfg->pdsch_cfg.grant.nof_layers = 1;
  for (int i = 0; i < 2; i++) {
    for (uint32_t j = 0; j < pmch_cfg->pdsch_cfg.grant.nof_prb; j++) {
      pmch_cfg->pdsch_cfg.grant.prb_idx[i][j] = true;
    }
  }
  if (getenv("PMCH_TI_DIAG")) {
    fprintf(stderr,
            "TI_DIAG_CFGPMCH cell.nof_prb=%u cell.mbsfn_prb=%u grant.nof_prb=%u enable=%d scs=%d tbs=%d\n",
            cell->nof_prb, cell->mbsfn_prb, pmch_cfg->pdsch_cfg.grant.nof_prb, (int)mbsfn_cfg->enable,
            (int)mbsfn_cfg->subcarrier_spacing, pmch_cfg->pdsch_cfg.grant.tb[0].tbs);
  }
}

int srsran_pmch_encode(srsran_pmch_t*      q,
                       srsran_dl_sf_cfg_t* sf,
                       srsran_pmch_cfg_t*  cfg,
                       uint8_t*            data,
                       cf_t*               sf_symbols[SRSRAN_MAX_PORTS])
{
  int i;
  int ret = SRSRAN_ERROR_INVALID_INPUTS;
  if (q != NULL && cfg != NULL) {
    for (i = 0; i < q->cell.nof_ports; i++) {
      if (sf_symbols[i] == NULL) {
        return SRSRAN_ERROR_INVALID_INPUTS;
      }
    }

    if (cfg->pdsch_cfg.grant.tb[0].tbs == 0) {
      return SRSRAN_ERROR_INVALID_INPUTS;
    }

    if (cfg->pdsch_cfg.grant.nof_re > q->max_re) {
      ERROR("Error too many RE per subframe (%d). PMCH configured for %d RE (%d PRB)",
            cfg->pdsch_cfg.grant.nof_re,
            q->max_re,
            q->cell.nof_prb);
      return SRSRAN_ERROR_INVALID_INPUTS;
    }

    INFO("Encoding PMCH SF: %d, Mod %s, NofBits: %d, NofSymbols: %d, NofBitsE: %d, rv_idx: %d",
         sf->tti % 10,
         srsran_mod_string(cfg->pdsch_cfg.grant.tb[0].mod),
         cfg->pdsch_cfg.grant.tb[0].tbs,
         cfg->pdsch_cfg.grant.nof_re,
         cfg->pdsch_cfg.grant.tb[0].nof_bits,
         0);

    uint32_t Mbit_sf = cfg->pdsch_cfg.grant.tb[0].nof_bits; /* coded bits for one subframe */
    uint8_t  N       = cfg->time_interleaving_n;

    if (N > 1) {
      /* TS 36.213 §11.1 (R1-2504967 CR1448 / R1-2601739 CR1459, clause 11.1):
       * within a block of N*M subframes, subframe s=cfg->subframe_idx
       * belongs to slot m=s%M (one of M independently pipelined transport
       * blocks TBm) with redundancy version n=(s%(N*M))/M - NOT a single
       * ongoing TB cycling rv=0..N-1 (that pattern only matches this formula
       * at M==1, which is unreachable once N>1 since RRC clamps M>=N; an
       * earlier version of this function implemented exactly that
       * unreachable M==1 case - see the roadmap's Phase 1/2/3 findings log
       * for the correction).
       *
       * Each slot m independently rate-matches via the MCH-specific
       * k0 = 2*R_TC + rv_idx*e_min formula (srsran_dlsch_encode_mch) at
       * rv_idx=n, not a contiguous slice of one big rv=0 pass.
       *
       * ti_tx_buf[m] caches slot m's raw TB payload (not pre-encoded bits)
       * across its own N calls: MAC/caller only supplies a non-NULL data
       * pointer at n==0 for slot m, so n=1..N-1 need the SAME payload -
       * srsran_dlsch_encode_mch re-encodes from scratch each time (turbo
       * encoding doesn't depend on rv_idx, only rate matching does - this
       * redundant re-encode is an accepted inefficiency, not a correctness
       * concern, since MCH encoding isn't a hot path). */
      /* Clamp defensively: time_interleaving_m ultimately derives from
       * broadcast MCCH data on the RX side - a value above the spec max
       * (only reachable via a malformed/unexpected broadcast) must not
       * become an out-of-bounds ti_tx_buf[]/ti_decoded[] access below. */
      uint8_t M = cfg->time_interleaving_m;
      if (M == 0) {
        M = 1;
      } else if (M > SRSRAN_PMCH_MAX_TI_M) {
        M = SRSRAN_PMCH_MAX_TI_M;
      }
      uint32_t block_len = (uint32_t)N * (uint32_t)M;
      uint32_t s_mod      = cfg->subframe_idx % block_len;
      uint32_t slot_m      = s_mod % M;
      uint32_t slot_n      = s_mod / M;

      srsran_cbsegm_t ti_cb_segm;
      srsran_cbsegm(&ti_cb_segm, cfg->pdsch_cfg.grant.tb[0].tbs);
      uint32_t Qm_ti = srsran_mod_bits_x_symbol(cfg->pdsch_cfg.grant.tb[0].mod);
      uint32_t Gp_ti = Mbit_sf / Qm_ti;
      uint32_t e_min = Qm_ti * (Gp_ti / ti_cb_segm.C);
      // TS 36.212 §5.1.4.1.2 N_cb = min(floor(N_IR/C), K_w). 0 = category/beta not
      // (yet) configured -- srsran_dlsch_encode_mch falls back to the uncapped K_w.
      uint32_t n_cb_cap = srsran_pmch_n_cb_cap(
          cfg->n_soft_ref_category, cfg->scaling_factor_beta_num, cfg->scaling_factor_beta_den, M, ti_cb_segm.C);

      if (!q->ti_tx_buf[slot_m]) {
        /* Lazily allocated: same worst-case-N=16 sizing as the single buffer
         * this replaced, now one instance per slot actually used instead of
         * a single unconditional allocation (see pmch.h). */
        uint32_t buf_bytes = 16 * q->ti_buf_nre * srsran_mod_bits_x_symbol(SRSRAN_MOD_256QAM);
        q->ti_tx_buf[slot_m] = calloc(buf_bytes, sizeof(uint8_t));
        if (!q->ti_tx_buf[slot_m]) {
          ERROR("Error allocating ti_tx_buf for slot %d", slot_m);
          return SRSRAN_ERROR;
        }
      }
      if (slot_n == 0 && data != NULL) {
        memcpy(q->ti_tx_buf[slot_m], data, ((uint32_t)cfg->pdsch_cfg.grant.tb[0].tbs + 7) / 8);
      }
      if (srsran_dlsch_encode_mch(&q->dl_sch, &cfg->pdsch_cfg, q->ti_tx_buf[slot_m], q->e, slot_n, e_min, n_cb_cap)) {
        ERROR("Error encoding TB for time interleaving");
        return SRSRAN_ERROR;
      }
    } else {
      // TODO: use tb_encode directly
      if (srsran_dlsch_encode(&q->dl_sch, &cfg->pdsch_cfg, data, q->e)) {
        ERROR("Error encoding TB");
        return SRSRAN_ERROR;
      }
    }

    /* Rel-19 §6.5.1/§6.5.2 both need C (codeblocks) and L (PMCH OFDM symbol count);
     * compute once, only when a Rel-19 feature that needs them is actually enabled. */
    srsran_cbsegm_t cb_segm = {0};
    uint32_t        sym_re[64];
    uint32_t        L = 0;
    if (cfg->freq_interleaving || (cfg->cyclic_shift && cfg->cyclic_shift_alpha > 0)) {
      srsran_cbsegm(&cb_segm, cfg->pdsch_cfg.grant.tb[0].tbs);
      L = pmch_sym_re_per_sym(q, sf->subcarrier_spacing, 0, sf->tti, sym_re, 64);
    }

    /* TS 36.211 §6.5.1: bit-level left cyclic shift after rate matching, before
     * scrambling. (This encode() copy was previously missing this step entirely -
     * pmch_decode's anti-shift and rt-mbms-tx's pmch_encode both had it. The shared
     * PHY library should implement both directions consistently even though this
     * modem app itself is receive-only.) */
    if (cfg->cyclic_shift && cfg->cyclic_shift_alpha > 0) {
      uint32_t Xi;
      if (cfg->cyclic_shift_alpha == 3) {
        uint32_t Qm_bits = srsran_mod_bits_x_symbol(cfg->pdsch_cfg.grant.tb[0].mod);
        Xi = pmch_cyclic_shift_alpha3_Xi(sf->tti / 10, sf->tti % 10, N, Mbit_sf, Qm_bits, L, cfg->area_id);
      } else {
        uint32_t alpha = pmch_cyclic_shift_alpha_value(cfg->cyclic_shift_alpha, cb_segm.C, N, L);
        Xi             = pmch_cyclic_shift_Xi(Mbit_sf, cb_segm.C, cfg->subframe_idx, alpha);
      }
      pmch_cyclic_shift_bits((uint8_t*)q->e, Mbit_sf, Xi);
    }

    /* scramble */
    srsran_scrambling_bytes(
        &q->seqs[cfg->area_id]->seq[sf->tti % 10], (uint8_t*)q->e, Mbit_sf);

    srsran_mod_modulate_bytes(
        &q->mod[cfg->pdsch_cfg.grant.tb[0].mod], (uint8_t*)q->e, q->d, Mbit_sf);

    /* No per-subcarrier phase rotation here: TS 36.211 §6.5.1 (confirmed against
     * R1-2505059) defines cyclic shift as a bit-domain operation only
     * (bn = b(n-Xi mod Mbit), applied above via pmch_cyclic_shift_bits) with no
     * complex-domain multiplication step. A prior version of this function
     * additionally applied d[k] *= e^(j*alpha*k) here, which has no basis in the
     * spec text - self-consistent for this eNB/its own receiver (both TX and RX
     * applied the same spurious rotation), but not interoperable with a
     * spec-compliant third-party receiver. */

    /* Rel-19 §6.5.2: frequency-domain interleaving (applied after modulation, before RE mapping) */
    if (cfg->freq_interleaving) {
      pmch_freq_interleave(q->d, (uint32_t)cb_segm.C, L, sym_re, false);
    }

    /* No tx diversity in MBSFN */
    memcpy(q->symbols[0], q->d, cfg->pdsch_cfg.grant.nof_re * sizeof(cf_t));

    /* PMCH_RE_DUMP: scratch instrumentation for tracing an MCCH decode failure
     * across the TX/RX loopback (not part of normal operation - gated behind an
     * env var, safe to leave in, meant to be removed once the investigation is
     * done). Dumps the modulated data symbols (post-modulation, pre-RE-mapping -
     * same content as q->symbols[0] just copied above) and the scrambled bits
     * (post-scramble, pre-modulation), one file per tti so a specific subframe's
     * TX-side data can be diffed against the matching RX-side dump (see the
     * decode()-side PMCH_RE_DUMP block below) for the same tti. */
    if (getenv("PMCH_RE_DUMP")) {
      char fn[128];
      snprintf(fn, sizeof(fn), "/tmp/pmch_tx_sym_tti%u.bin", sf->tti);
      FILE* fsym = fopen(fn, "wb");
      if (fsym) {
        fwrite(q->d, sizeof(cf_t), cfg->pdsch_cfg.grant.nof_re, fsym);
        fclose(fsym);
      }
      snprintf(fn, sizeof(fn), "/tmp/pmch_tx_bits_tti%u.bin", sf->tti);
      FILE* fbits = fopen(fn, "wb");
      if (fbits) {
        fwrite(q->e, 1, (Mbit_sf + 7) / 8, fbits);
        fclose(fbits);
      }
      fprintf(stderr,
              "[PMCH_RE_DUMP] TX tti=%u area_id=%u tbs=%u mcs=%u mod=%s nof_re=%u Mbit_sf=%u\n",
              sf->tti,
              cfg->area_id,
              cfg->pdsch_cfg.grant.tb[0].tbs,
              cfg->pdsch_cfg.grant.tb[0].mcs_idx,
              srsran_mod_string(cfg->pdsch_cfg.grant.tb[0].mod),
              cfg->pdsch_cfg.grant.nof_re,
              Mbit_sf);
    }

    /* mapping to resource elements */
    /* All FeMBMS SCS types have no control region; lstart=0.
     * Standard 15 kHz MBSFN uses sf->cfi control symbols. */
    uint32_t lstart = (sf->subcarrier_spacing != SRSRAN_SCS_15KHZ)
                         ? 0u
                         : SRSRAN_NOF_CTRL_SYMBOLS(q->cell, sf->cfi);
    for (i = 0; i < q->cell.nof_ports; i++) {
      pmch_put(q, q->symbols[i], sf_symbols[i], sf->subcarrier_spacing, lstart, sf->tti);
    }

    /* PMCH_RE_DUMP: scratch instrumentation, see the matching comment in
     * rt-mbms-tx's copy of this function (this app is receive-only, so this path is
     * not actually live here, but kept mirrored for consistency). */
    if (getenv("PMCH_RE_DUMP")) {
      uint32_t dump_n = SRSRAN_NRE_SCS(sf->subcarrier_spacing) * q->cell.nof_prb;
      char     fn[128];
      snprintf(fn, sizeof(fn), "/tmp/pmch_tx_preifft_tti%u.bin", sf->tti);
      FILE* fpre = fopen(fn, "wb");
      if (fpre) {
        fwrite(sf_symbols[0], sizeof(cf_t), dump_n, fpre);
        fclose(fpre);
      }
      fprintf(stderr, "[PMCH_RE_DUMP] TX preifft tti=%u scs=%d nof_prb=%u dump_n=%u\n",
              sf->tti, (int)sf->subcarrier_spacing, q->cell.nof_prb, dump_n);
    }

    ret = SRSRAN_SUCCESS;
  }
  return ret;
}
