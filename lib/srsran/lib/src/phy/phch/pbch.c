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
#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

#include "prb_dl.h"
#include "srsran/phy/common/phy_common.h"
#include "srsran/phy/phch/pbch.h"
#include "srsran/phy/utils/bit.h"
#include "srsran/phy/utils/debug.h"
#include "srsran/phy/utils/vector.h"

#define PBCH_RE_CP_NORM 240
#define PBCH_RE_CP_EXT 216

const uint8_t srsran_crc_mask[4][16] = {{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                                        {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
                                        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                                        {0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1}};

bool srsran_pbch_exists(int nframe, int nslot)
{
  return (!(nframe % 5) && nslot == 1);
}

int srsran_pbch_cp(cf_t* input, cf_t* output, srsran_cell_t cell, bool put)
{
  int   i;
  cf_t* ptr;

  if (put) {
    ptr = input;
    output += cell.nof_prb * SRSRAN_NRE / 2 - 36;
  } else {
    ptr = output;
    input += cell.nof_prb * SRSRAN_NRE / 2 - 36;
  }

  /* symbol 0 & 1 */
  for (i = 0; i < 2; i++) {
    prb_cp_ref(&input, &output, cell.id % 3, 4, 4 * 6, put);
    if (put) {
      output += cell.nof_prb * SRSRAN_NRE - 2 * 36 + (cell.id % 3 == 2 ? 1 : 0);
    } else {
      input += cell.nof_prb * SRSRAN_NRE - 2 * 36 + (cell.id % 3 == 2 ? 1 : 0);
    }
  }
  /* symbols 2 & 3 */
  if (SRSRAN_CP_ISNORM(cell.cp)) {
    for (i = 0; i < 2; i++) {
      prb_cp(&input, &output, 6);
      if (put) {
        output += cell.nof_prb * SRSRAN_NRE - 2 * 36;
      } else {
        input += cell.nof_prb * SRSRAN_NRE - 2 * 36;
      }
    }
  } else {
    prb_cp(&input, &output, 6);
    if (put) {
      output += cell.nof_prb * SRSRAN_NRE - 2 * 36;
    } else {
      input += cell.nof_prb * SRSRAN_NRE - 2 * 36;
    }
    prb_cp_ref(&input, &output, cell.id % 3, 4, 4 * 6, put);
  }
  if (put) {
    return input - ptr;
  } else {
    return output - ptr;
  }
}

/* TS 36.211 clause 6.6.4.1: PBCH CAS repetition for FeMBMS dedicated carriers.
 *
 * cinit = 2^13*(N_ID^cell+1)*(N_symb^DL*ns'+l'+1) + 2^4*N_ID^cell + N_symb^DL*ns'+l'
 * theta(k,l') = e^(j*pi*c(2k)/2) * e^(j*pi*c(2k+1)),  k = subcarrier in PBCH 72-band
 *
 * Table 6.6.4.1-1 (Normal CP):
 *   src l=0 → (ns'=0, l'=4)      src l=2 → (ns'=1, l'=5)
 *   src l=1 → (ns'=1, l'=4)      src l=3 → (ns'=0, l'=3), (ns'=1, l'=6)
 *
 * Table 6.6.4.1-1 (Extended CP):
 *   src l=0 → "-" (no entry)      src l=2 → (ns'=1, l'=4)
 *   src l=1 → (ns'=0, l'=3)       src l=3 → (ns'=1, l'=5)
 */
#define PBCH_CAS_NOF_SC 72

typedef struct { uint32_t src_l, dst_ns, dst_l; } pbch_cas_map_t;
static const pbch_cas_map_t PBCH_CAS_MAP_NCP[5] = {
    {0, 0, 4}, {1, 1, 4}, {2, 1, 5}, {3, 0, 3}, {3, 1, 6}
};
static const pbch_cas_map_t PBCH_CAS_MAP_ECP[3] = {
    {1, 0, 3}, {2, 1, 4}, {3, 1, 5}
};

static void pbch_cas_gen_theta(uint32_t cell_id, uint32_t ns_prime, uint32_t l_prime,
                                uint32_t n_symb_dl, cf_t* theta)
{
    uint32_t cinit = (1U << 13) * (cell_id + 1) * (n_symb_dl * ns_prime + l_prime + 1)
                     + (1U << 4) * cell_id
                     + n_symb_dl * ns_prime + l_prime;
    srsran_sequence_t seq = {};
    srsran_sequence_LTE_pr(&seq, 2 * PBCH_CAS_NOF_SC, cinit);
    for (uint32_t k = 0; k < PBCH_CAS_NOF_SC; k++) {
        /* θk,l' = e^(jπc(2k)/2) · e^(jπc(2k+1)):
         *   c(2k)=0 → 1+j0;  c(2k)=1 → 0+j1;  then flip sign if c(2k+1)=1 */
        float re = (seq.c[2 * k]     == 0) ? 1.0f : 0.0f;
        float im = (seq.c[2 * k]     == 1) ? 1.0f : 0.0f;
        if (seq.c[2 * k + 1] == 1) { re = -re; im = -im; }
        theta[k] = re + _Complex_I * im;
    }
    srsran_sequence_free(&seq);
}

/**
 * Puts or gets a CAS-repeated PBCH symbol block.
 *
 * sf_symbols: full subframe resource grid
 * cell:       cell configuration
 * put:        true = TX (write rotated copies), false = RX (read into cas_out)
 * cas_out:    RX only — NCP: 5*PBCH_CAS_NOF_SC elements; ECP: 3*PBCH_CAS_NOF_SC elements
 *
 * Frame condition (checked by caller):
 *   n_f mod 4 = 0 for N_RB^DL >= 25;  n_f mod 8 = 4 for 6 < N_RB^DL < 25.
 * Applies only when cell.mbms_dedicated = true and cell.nof_prb > 6.
 */
static void pbch_cas_cp(cf_t* sf_symbols, srsran_cell_t cell, bool put, cf_t* cas_out)
{
    bool ncp = SRSRAN_CP_ISNORM(cell.cp);
    const pbch_cas_map_t* map  = ncp ? PBCH_CAS_MAP_NCP : PBCH_CAS_MAP_ECP;
    int                   nmap = ncp ? 5 : 3;

    uint32_t n_symb_dl = SRSRAN_CP_NSYMB(cell.cp);   /* 7 NCP, 6 ECP */
    uint32_t slot_re   = SRSRAN_SLOT_LEN_RE(cell.nof_prb, cell.cp);
    uint32_t center    = cell.nof_prb * SRSRAN_NRE / 2 - 36;

    cf_t theta[PBCH_CAS_NOF_SC];

    for (int i = 0; i < nmap; i++) {
        uint32_t src_l  = map[i].src_l;
        uint32_t dst_ns = map[i].dst_ns;
        uint32_t dst_lp = map[i].dst_l;

        /* RS positions: NCP at l=0,4 (ports 0,1) and l=1 (ports 2,3);
         *               ECP at l=0,3 (ports 0,1) — ports 2,3 not used for PBCH. */
        bool src_has_rs = ncp ? (src_l == 0 || src_l == 1) : (src_l == 0 || src_l == 3);
        bool dst_has_rs = ncp ? (dst_lp == 0 || dst_lp == 4) : (dst_lp == 0 || dst_lp == 3);

        /* Source subcarriers in slot 1, symbol src_l. */
        cf_t* src = sf_symbols + slot_re + src_l * cell.nof_prb * SRSRAN_NRE + center;
        /* Destination subcarriers in slot dst_ns, symbol dst_lp. */
        cf_t* dst = sf_symbols + dst_ns * slot_re + dst_lp * cell.nof_prb * SRSRAN_NRE + center;

        pbch_cas_gen_theta(cell.id, dst_ns, dst_lp, n_symb_dl, theta);

        for (uint32_t k = 0; k < PBCH_CAS_NOF_SC; k++) {
            bool is_rs = ((k % 3) == (cell.id % 3));
            if ((src_has_rs || dst_has_rs) && is_rs) {
                continue;  /* Skip RS subcarrier — do not overwrite CS-RS. */
            }
            if (put) {
                dst[k] = src[k] * theta[k];
            } else if (cas_out) {
                /* Conjugate-rotate for coherent soft combination. */
                cas_out[i * PBCH_CAS_NOF_SC + k] = dst[k] * conjf(theta[k]);
            }
        }
    }
}

/**
 * Puts PBCH in slot number 1
 *
 * Returns the number of symbols written to slot1_data
 *
 * 36.211 10.3 section 6.6.4
 *
 * @param[in] pbch PBCH complex symbols to place in slot1_data
 * @param[out] slot1_data Complex symbol buffer for slot1
 * @param[in] cell Cell configuration
 */
int srsran_pbch_put(cf_t* pbch, cf_t* slot1_data, srsran_cell_t cell)
{
  return srsran_pbch_cp(pbch, slot1_data, cell, true);
}

/**
 * Extracts PBCH from slot number 1
 *
 * Returns the number of symbols written to pbch
 *
 * 36.211 10.3 section 6.6.4
 *
 * @param[in] slot1_data Complex symbols for slot1
 * @param[out] pbch Extracted complex PBCH symbols
 * @param[in] cell Cell configuration
 */
int srsran_pbch_get(cf_t* slot1_data, cf_t* pbch, srsran_cell_t cell)
{
  return srsran_pbch_cp(slot1_data, pbch, cell, false);
}

/**
 * Writes phase-rotated PBCH CAS repetition symbols into sf_symbols.
 *
 * TS 36.211 clause 6.6.4.1. Call after srsran_pbch_encode on an FeMBMS dedicated carrier
 * when cell.mbms_dedicated=true, cell.nof_prb>6, and the frame condition is satisfied:
 *   n_f mod 4 = 0 for N_RB^DL >= 25;  n_f mod 8 = 4 for 6 < N_RB^DL < 25.
 *
 * @param[in,out] sf_symbols Full subframe resource grid (both slots).
 * @param[in]     cell       Cell configuration.
 */
void srsran_pbch_put_cas_rep(cf_t* sf_symbols, srsran_cell_t cell)
{
  if (cell.mbms_dedicated && cell.nof_prb > 6) {
    pbch_cas_cp(sf_symbols, cell, true, NULL);
  }
}

/**
 * Extracts conjugate-rotated PBCH CAS repetition symbols from sf_symbols.
 *
 * TS 36.211 clause 6.6.4.1. Returns 5 * PBCH_CAS_NOF_SC = 360 complex values for
 * Normal CP, or 3 * PBCH_CAS_NOF_SC = 216 for Extended CP (fewer source/destination
 * symbol pairs, see PBCH_CAS_MAP_NCP/PBCH_CAS_MAP_ECP), suitable for soft-combining
 * with the main PBCH LLRs. The caller is responsible for channel equalization
 * before combining.
 *
 * @param[in]  sf_symbols Full subframe resource grid.
 * @param[in]  cell       Cell configuration.
 * @param[out] cas_out    Output buffer: 360 (Normal CP) or 216 (Extended CP) complex elements.
 */
void srsran_pbch_get_cas_rep(cf_t* sf_symbols, srsran_cell_t cell, cf_t* cas_out)
{
  if (cell.mbms_dedicated && cell.nof_prb > 6 && cas_out) {
    pbch_cas_cp(sf_symbols, cell, false, cas_out);
  }
}

/** Initializes the PBCH transmitter and receiver.
 * At the receiver, the field nof_ports in the cell structure indicates the
 * maximum number of BS transmitter ports to look for.
 */
int srsran_pbch_init(srsran_pbch_t* q)
{
  int ret = SRSRAN_ERROR_INVALID_INPUTS;

  if (q != NULL) {
    ret = SRSRAN_ERROR;

    bzero(q, sizeof(srsran_pbch_t));

    if (srsran_modem_table_lte(&q->mod, SRSRAN_MOD_QPSK)) {
      goto clean;
    }
    int poly[3] = {0x6D, 0x4F, 0x57};
    if (srsran_viterbi_init(&q->decoder, SRSRAN_VITERBI_37, poly, 40, true)) {
      goto clean;
    }
    if (srsran_crc_init(&q->crc, SRSRAN_LTE_CRC16, 16)) {
      goto clean;
    }
    q->encoder.K           = 7;
    q->encoder.R           = 3;
    q->encoder.tail_biting = true;
    memcpy(q->encoder.poly, poly, 3 * sizeof(int));

    q->nof_symbols = PBCH_RE_CP_NORM;

    q->d = srsran_vec_cf_malloc(q->nof_symbols);
    if (!q->d) {
      goto clean;
    }
    int i;
    for (i = 0; i < SRSRAN_MAX_PORTS; i++) {
      q->ce[i] = srsran_vec_cf_malloc(q->nof_symbols);
      if (!q->ce[i]) {
        goto clean;
      }
      q->x[i] = srsran_vec_cf_malloc(q->nof_symbols);
      if (!q->x[i]) {
        goto clean;
      }
      q->symbols[i] = srsran_vec_cf_malloc(q->nof_symbols);
      if (!q->symbols[i]) {
        goto clean;
      }
    }
    q->llr = srsran_vec_f_malloc(q->nof_symbols * 4 * 2);
    if (!q->llr) {
      goto clean;
    }
    q->temp = srsran_vec_f_malloc(q->nof_symbols * 4 * 2);
    if (!q->temp) {
      goto clean;
    }
    q->rm_b = srsran_vec_u8_malloc(q->nof_symbols * 4 * 2);
    if (!q->rm_b) {
      goto clean;
    }
    q->cas_syms = srsran_vec_cf_malloc(5 * PBCH_CAS_NOF_SC);
    if (!q->cas_syms) {
      goto clean;
    }
    q->cas_frame_idx_hint = 1;

    ret = SRSRAN_SUCCESS;
  }
clean:
  if (ret == SRSRAN_ERROR) {
    srsran_pbch_free(q);
  }
  return ret;
}

void srsran_pbch_free(srsran_pbch_t* q)
{
  srsran_sequence_free(&q->seq);
  srsran_modem_table_free(&q->mod);
  srsran_viterbi_free(&q->decoder);
  int i;
  for (i = 0; i < SRSRAN_MAX_PORTS; i++) {
    if (q->ce[i]) {
      free(q->ce[i]);
    }
    if (q->x[i]) {
      free(q->x[i]);
    }
    if (q->symbols[i]) {
      free(q->symbols[i]);
    }
  }
  if (q->llr) {
    free(q->llr);
  }
  if (q->temp) {
    free(q->temp);
  }
  if (q->rm_b) {
    free(q->rm_b);
  }
  if (q->d) {
    free(q->d);
  }
  if (q->cas_syms) {
    free(q->cas_syms);
  }
  bzero(q, sizeof(srsran_pbch_t));
}

int srsran_pbch_set_cell(srsran_pbch_t* q, srsran_cell_t cell)
{
  int ret = SRSRAN_ERROR_INVALID_INPUTS;

  if (q != NULL && srsran_cell_isvalid(&cell)) {
    if (cell.nof_ports == 0) {
      q->search_all_ports = true;
      cell.nof_ports      = SRSRAN_MAX_PORTS;
    } else {
      q->search_all_ports = false;
    }

    if (q->cell.id != cell.id || q->cell.nof_prb == 0) {
      q->cell = cell;
      if (srsran_sequence_pbch(&q->seq, q->cell.cp, q->cell.id, q->cell.mbms_dedicated)) {
        return SRSRAN_ERROR;
      }
    }
    q->nof_symbols = (SRSRAN_CP_ISNORM(q->cell.cp)) ? PBCH_RE_CP_NORM : PBCH_RE_CP_EXT;

    ret = SRSRAN_SUCCESS;
  }
  return ret;
}

/**
 * Unpacks MIB from PBCH message.
 *
 * @param[in] msg PBCH in an unpacked bit array of size 24
 * @param[out] sfn System frame number
 * @param[out] cell MIB information about PHICH and system bandwidth will be saved here
 */
void srsran_pbch_mib_unpack(uint8_t* msg, srsran_cell_t* cell, uint32_t* sfn)
{
  int phich_res;

  uint32_t bw_idx = srsran_bit_pack(&msg, 3);
  switch (bw_idx) {
    case 0:
      cell->nof_prb = 6;
      break;
    case 1:
      cell->nof_prb = 15;
      break;
    default:
      cell->nof_prb = (bw_idx - 1) * 25;
      break;
  }
  if (*msg) {
    cell->phich_length = SRSRAN_PHICH_EXT;
  } else {
    cell->phich_length = SRSRAN_PHICH_NORM;
  }
  msg++;

  phich_res = srsran_bit_pack(&msg, 2);
  switch (phich_res) {
    case 0:
      cell->phich_resources = SRSRAN_PHICH_R_1_6;
      break;
    case 1:
      cell->phich_resources = SRSRAN_PHICH_R_1_2;
      break;
    case 2:
      cell->phich_resources = SRSRAN_PHICH_R_1;
      break;
    case 3:
      cell->phich_resources = SRSRAN_PHICH_R_2;
      break;
  }
  if (sfn) {
    *sfn = srsran_bit_pack(&msg, 8) << 2;
  }
}

/**
 * Unpacks MIB-MBMS from PBCH message.
 *
 * @param[in] msg PBCH in an unpacked bit array of size 24
 * @param[out] sfn System frame number
 * @param[out] cell MIB information about PHICH and system bandwidth will be saved here
 */
void srsran_pbch_mib_mbms_unpack(uint8_t* msg, srsran_cell_t* cell, uint32_t* sfn, uint32_t* additional_non_mbsfn_subframes, int8_t override_prb)
{
  uint32_t bw_idx = srsran_bit_pack(&msg, 3);
  switch (bw_idx) {
    case 0:
      cell->nof_prb = 6;
      break;
    case 1:
      cell->nof_prb = 15;
      break;
    default:
      cell->nof_prb = (bw_idx - 1) * 25;
      break;
  }
  if (override_prb != -1) {
      cell->nof_prb = override_prb;
  }

  /* Always advance msg past sfn bits to keep alignment for subsequent fields. */
  uint32_t sfn_bits = srsran_bit_pack(&msg, 6);
  if (sfn) {
    *sfn = sfn_bits << 4;
  }

  /* bits [9-10]: additionalNonMBSFNSubframes-r14 — always advance. */
  uint32_t add_non_mbsfn = srsran_bit_pack(&msg, 2);
  if (additional_non_mbsfn_subframes) {
    *additional_non_mbsfn_subframes = add_non_mbsfn;
  }

  /* bits [11-12]: semiStaticCFI-MBMS-r16 — INTEGER(0..3), TS 36.213 §9.1.3: 0 = derive CFI
   * from PCFICH, 1/2/3 directly ARE the CFI value. cell->semi_static_cfi already stores
   * exactly this convention (see phy_common.h) -- decode it verbatim, no remapping. */
  cell->semi_static_cfi = srsran_bit_pack(&msg, 2);
}

/**
 * Packs MIB to PBCH message.
 *
 * @param[out] payload Output unpacked bit array of size 24
 * @param[in] sfn System frame number
 * @param[in] cell Cell configuration to be encoded in MIB
 */
void srsran_pbch_mib_pack(srsran_cell_t* cell, uint32_t sfn, uint8_t* payload)
{
  int bw, phich_res = 0;

  uint8_t* msg = payload;

  bzero(msg, 24);

  if (cell->nof_prb <= 6) {
    bw = 0;
  } else if (cell->nof_prb <= 15) {
    bw = 1;
  } else {
    bw = 1 + cell->nof_prb / 25;
  }
  srsran_bit_unpack(bw, &msg, 3);

  *msg = cell->phich_length == SRSRAN_PHICH_EXT;
  msg++;

  switch (cell->phich_resources) {
    case SRSRAN_PHICH_R_1_6:
      phich_res = 0;
      break;
    case SRSRAN_PHICH_R_1_2:
      phich_res = 1;
      break;
    case SRSRAN_PHICH_R_1:
      phich_res = 2;
      break;
    case SRSRAN_PHICH_R_2:
      phich_res = 3;
      break;
  }
  srsran_bit_unpack(phich_res, &msg, 2);
  srsran_bit_unpack(sfn >> 2, &msg, 8);
}

void srsran_pbch_decode_reset(srsran_pbch_t* q)
{
  q->frame_idx          = 0;
  q->cas_frame_idx_hint = 1;
}

void srsran_crc_set_mask(uint8_t* data, int nof_ports)
{
  int i;
  for (i = 0; i < 16; i++) {
    data[SRSRAN_BCH_PAYLOAD_LEN + i] = (data[SRSRAN_BCH_PAYLOAD_LEN + i] + srsran_crc_mask[nof_ports - 1][i]) % 2;
  }
}

/* Checks CRC after applying the mask for the given number of ports.
 *
 * The bits buffer size must be at least 40 bytes.
 *
 * Returns 0 if the data is correct, -1 otherwise
 */
uint32_t srsran_pbch_crc_check(srsran_pbch_t* q, uint8_t* bits, uint32_t nof_ports)
{
  uint8_t data[SRSRAN_BCH_PAYLOADCRC_LEN];
  memcpy(data, bits, SRSRAN_BCH_PAYLOADCRC_LEN * sizeof(uint8_t));
  srsran_crc_set_mask(data, nof_ports);
  int ret = srsran_crc_checksum(&q->crc, data, SRSRAN_BCH_PAYLOADCRC_LEN);
  if (ret == 0) {
    uint32_t chkzeros = 0;
    for (int i = 0; i < SRSRAN_BCH_PAYLOAD_LEN; i++) {
      chkzeros += data[i];
    }
    if (chkzeros) {
      return 0;
    } else {
      return SRSRAN_ERROR;
    }
  } else {
    return ret;
  }
}

int decode_frame(srsran_pbch_t* q, uint32_t src, uint32_t dst, uint32_t n, uint32_t nof_bits, uint32_t nof_ports)
{
  int j;

  if (dst + n <= 4 && src + n <= 4) {
    srsran_vec_f_copy(&q->temp[dst * nof_bits], &q->llr[src * nof_bits], n * nof_bits);

    /* descramble */
    srsran_scrambling_f_offset(&q->seq, &q->temp[dst * nof_bits], dst * nof_bits, n * nof_bits);

    for (j = 0; j < dst * nof_bits; j++) {
      q->temp[j] = SRSRAN_RX_NULL;
    }
    for (j = (dst + n) * nof_bits; j < 4 * nof_bits; j++) {
      q->temp[j] = SRSRAN_RX_NULL;
    }

    /* unrate matching */
    srsran_rm_conv_rx(q->temp, 4 * nof_bits, q->rm_f, SRSRAN_BCH_ENCODED_LEN);

    /* Normalize LLR */
    srsran_vec_sc_prod_fff(q->rm_f, 1.0 / ((float)2 * n), q->rm_f, SRSRAN_BCH_ENCODED_LEN);

    /* decode */
    srsran_viterbi_decode_f(&q->decoder, q->rm_f, q->data, SRSRAN_BCH_PAYLOADCRC_LEN);

    if (!srsran_pbch_crc_check(q, q->data, nof_ports)) {
      return 1;
    } else {
      return SRSRAN_SUCCESS;
    }
  } else {
    ERROR("Error in PBCH decoder: Invalid frame pointers dst=%d, src=%d, n=%d", src, dst, n);
    return -1;
  }
}

/* Decodes the PBCH channel
 *
 * The PBCH spans in 40 ms. This function is called every 10 ms. It tries to decode the MIB
 * given the symbols of a subframe (1 ms). Successive calls will use more subframes
 * to help the decoding process.
 *
 * Returns 1 if successfully decoded MIB, 0 if not and -1 on error
 */
int srsran_pbch_decode(srsran_pbch_t*         q,
                       srsran_chest_dl_res_t* channel,
                       cf_t*                  sf_symbols[SRSRAN_MAX_PORTS],
                       uint8_t                bch_payload[SRSRAN_BCH_PAYLOAD_LEN],
                       uint32_t*              nof_tx_ports,
                       int*                   sfn_offset)
{
  uint32_t src, dst, nb;
  uint32_t nant;
  int      i;
  int      nof_bits;
  cf_t*    x[SRSRAN_MAX_LAYERS];

  int ret = SRSRAN_ERROR_INVALID_INPUTS;

  if (q != NULL && sf_symbols != NULL) {
    cf_t* slot1_symbols = &sf_symbols[0][SRSRAN_SLOT_LEN_RE(q->cell.nof_prb, q->cell.cp)];

    cf_t* ce_slot1[SRSRAN_MAX_PORTS];
    for (int i = 0; i < SRSRAN_MAX_PORTS; i++) {
      ce_slot1[i] = &channel->ce[i][0][SRSRAN_SLOT_LEN_RE(q->cell.nof_prb, q->cell.cp)];
    }

    /* Set pointers for layermapping & precoding */
    nof_bits = 2 * q->nof_symbols;

    /* number of layers equals number of ports */
    for (i = 0; i < SRSRAN_MAX_PORTS; i++) {
      x[i] = q->x[i];
    }

    /* extract symbols */
    if (q->nof_symbols != srsran_pbch_get(slot1_symbols, q->symbols[0], q->cell)) {
      ERROR("There was an error getting the PBCH symbols");
      return SRSRAN_ERROR;
    }

    /* extract channel estimates */
    for (i = 0; i < q->cell.nof_ports; i++) {
      if (q->nof_symbols != srsran_pbch_get(ce_slot1[i], q->ce[i], q->cell)) {
        ERROR("There was an error getting the PBCH symbols");
        return SRSRAN_ERROR;
      }
    }

    q->frame_idx++;
    ret = 0;

    uint32_t frame_idx = q->frame_idx;

    /* Try decoding for 1 to cell.nof_ports antennas */
    if (q->search_all_ports) {
      nant = 1;
    } else {
      nant = q->cell.nof_ports;
    }

    do {
      if (nant != 3) {
        DEBUG("Trying %d TX antennas with %d frames", nant, frame_idx);

        /* in control channels, only diversity is supported */
        if (nant == 1) {
          /* no need for layer demapping */
          srsran_predecoding_single(q->symbols[0], q->ce[0], q->d, NULL, q->nof_symbols, 1.0f, channel->noise_estimate);
        } else {
          srsran_predecoding_diversity(q->symbols[0], q->ce, x, nant, q->nof_symbols, 1.0f);
          srsran_layerdemap_diversity(x, q->d, nant, q->nof_symbols / nant);
        }

        /* demodulate symbols */
        srsran_demod_soft_demodulate(SRSRAN_MOD_QPSK, q->d, &q->llr[nof_bits * (frame_idx - 1)], q->nof_symbols);

        /* CAS soft-combining: accumulate phase-corrected PBCH repetition LLRs (TS 36.211 §6.6.4.1).
         * Uses the average CE from the standard PBCH position as a flat-channel approximation.
         * Per TS 36.211 §6.6.4.1 the CAS repetition exists in exactly one frame per 4-frame PBCH
         * window (sfn%4==0 for wide carriers, sfn%8==4 for narrow).  Combining the CAS symbols
         * on a non-CAS frame corrupts the LLRs with noise.
         * cas_frame_idx_hint is derived from sfn_offset at each successful decode and pins the
         * combine to the correct window position.  Before the first decode it defaults to 1
         * (conservative: applies to the first accumulated frame as a best-guess fallback). */
        if (q->cell.mbms_dedicated && q->cell.nof_prb > 6 && frame_idx == q->cas_frame_idx_hint) {
          int   nmap     = SRSRAN_CP_ISNORM(q->cell.cp) ? 5 : 3;
          int   cas_re   = nmap * PBCH_CAS_NOF_SC;
          int   cas_bits = 2 * cas_re;
          srsran_pbch_get_cas_rep(sf_symbols[0], q->cell, q->cas_syms);
          cf_t  h_avg = srsran_vec_acc_cc(q->ce[0], q->nof_symbols) / (float)q->nof_symbols;
          float h_sq  = crealf(h_avg * conjf(h_avg)) + 1e-6f;
          for (int k = 0; k < cas_re; k++) {
            q->cas_syms[k] = q->cas_syms[k] * conjf(h_avg) / h_sq;
          }
          srsran_demod_soft_demodulate(SRSRAN_MOD_QPSK, q->cas_syms, q->temp, cas_re);
          float* dst_llr = &q->llr[nof_bits * (frame_idx - 1)];
          for (int k = 0; k < cas_bits; k++) {
            dst_llr[k % nof_bits] += q->temp[k];
          }
          INFO("CAS PBCH soft-combining: added %d rep LLRs to frame_idx=%d", cas_bits, frame_idx);
        }

        /* We don't know where the 40 ms begin, so we try all combinations. E.g. if we received
         * 4 frames, try 1,2,3,4 individually, 12, 23, 34 in pairs, 123, 234 and finally 1234.
         * We know they are ordered.
         */
        for (nb = 0; nb < frame_idx; nb++) {
          for (dst = 0; (dst < 4 - nb); dst++) {
            for (src = 0; src < frame_idx - nb; src++) {
              ret = decode_frame(q, src, dst, nb + 1, nof_bits, nant);
              if (ret == 1) {
                if (sfn_offset) {
                  *sfn_offset = (int)dst - src + frame_idx - 1;
                }
                if (nof_tx_ports) {
                  *nof_tx_ports = nant;
                }
                if (bch_payload) {
                  memcpy(bch_payload, q->data, sizeof(uint8_t) * SRSRAN_BCH_PAYLOAD_LEN);
                }
                INFO("Decoded PBCH: src=%d, dst=%d, nb=%d, sfn_offset=%d",
                     src,
                     dst,
                     nb + 1,
                     (int)dst - src + frame_idx - 1);
                /* Derive CAS frame position for the next accumulation window.
                 * sfn_offset = (dst - src + frame_idx - 1) gives sfn%4 of the
                 * current frame.  The next window starts at sfn%4 = (sfn_offset+1)%4,
                 * so the CAS frame (sfn%4==0) falls at frame_idx = 1 + (4 - (sfn_offset+1)%4)%4. */
                {
                  uint32_t so = (uint32_t)((int)dst - src + (int)frame_idx - 1);
                  uint32_t nch = 1u + (4u - (so + 1u) % 4u) % 4u;
                  srsran_pbch_decode_reset(q);
                  q->cas_frame_idx_hint = nch;
                }
                return 1;
              }
            }
          }
        }
      }
      nant++;
    } while (nant <= q->cell.nof_ports);

    /* If not found, make room for the next packet of radio frame symbols */
    if (q->frame_idx == 4) {
      memmove(q->llr, &q->llr[nof_bits], nof_bits * 3 * sizeof(float));
      q->frame_idx = 3;
    }
  }
  return ret;
}

/** Converts the MIB message to symbols mapped to SLOT #1 ready for transmission
 */
int srsran_pbch_encode(srsran_pbch_t* q,
                       uint8_t        bch_payload[SRSRAN_BCH_PAYLOAD_LEN],
                       cf_t*          sf_symbols[SRSRAN_MAX_PORTS],
                       uint32_t       frame_idx,
                       uint32_t       sfn)
{
  int   i;
  int   nof_bits;
  cf_t* x[SRSRAN_MAX_LAYERS];

  if (q != NULL && bch_payload != NULL) {
    /* Set pointers for layermapping & precoding */
    nof_bits = 2 * q->nof_symbols;

    /* number of layers equals number of ports */
    for (i = 0; i < q->cell.nof_ports; i++) {
      x[i] = q->x[i];
    }
    memset(&x[q->cell.nof_ports], 0, sizeof(cf_t*) * (SRSRAN_MAX_LAYERS - q->cell.nof_ports));

    frame_idx = frame_idx % 4;

    memcpy(q->data, bch_payload, sizeof(uint8_t) * SRSRAN_BCH_PAYLOAD_LEN);

    /* encode & modulate */
    srsran_crc_attach(&q->crc, q->data, SRSRAN_BCH_PAYLOAD_LEN);
    srsran_crc_set_mask(q->data, q->cell.nof_ports);

    srsran_convcoder_encode(&q->encoder, q->data, q->data_enc, SRSRAN_BCH_PAYLOADCRC_LEN);

    srsran_rm_conv_tx(q->data_enc, SRSRAN_BCH_ENCODED_LEN, q->rm_b, 4 * nof_bits);

    srsran_scrambling_b_offset(&q->seq, &q->rm_b[frame_idx * nof_bits], frame_idx * nof_bits, nof_bits);
    srsran_mod_modulate(&q->mod, &q->rm_b[frame_idx * nof_bits], q->d, nof_bits);

    /* layer mapping & precoding */
    if (q->cell.nof_ports > 1) {
      srsran_layermap_diversity(q->d, x, q->cell.nof_ports, q->nof_symbols);
      srsran_precoding_diversity(x, q->symbols, q->cell.nof_ports, q->nof_symbols / q->cell.nof_ports, 1.0f);
    } else {
      memcpy(q->symbols[0], q->d, q->nof_symbols * sizeof(cf_t));
    }

    /* mapping to resource elements */
    for (i = 0; i < q->cell.nof_ports; i++) {
      srsran_pbch_put(q->symbols[i], &sf_symbols[i][SRSRAN_SLOT_LEN_RE(q->cell.nof_prb, q->cell.cp)], q->cell);
    }

    /* TS 36.211 clause 6.6.4.1: CAS repetition for FeMBMS dedicated carriers (nof_prb > 6).
     * N_RB^DL >= 25: every frame with n_f mod 4 = 0.
     * 6 < N_RB^DL < 25: every frame with n_f mod 8 = 4.
     * CAS-muting (CR 0577) awareness is checked here, not left to the caller: a
     * muted frame must not get CAS-rep symbols regardless of who calls this public
     * API, since muted frames are MBSFN data, not CAS. */
    bool put_cas = q->cell.mbms_dedicated && q->cell.nof_prb > 6;
    if (put_cas) {
      put_cas = (q->cell.nof_prb >= 25) ? (sfn % 4 == 0) : (sfn % 8 == 4);
    }
    if (put_cas && q->cell.cas_muting) {
      put_cas = sfn % (16u * (uint32_t)q->cell.n_cas) < 4u * (uint32_t)q->cell.k_cas;
    }
    if (put_cas) {
      for (i = 0; i < q->cell.nof_ports; i++) {
        srsran_pbch_put_cas_rep(sf_symbols[i], q->cell);
      }
    }

    return SRSRAN_SUCCESS;
  } else {
    return SRSRAN_ERROR_INVALID_INPUTS;
  }
}
