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
 *  File:         rm_turbo.h
 *
 *  Description:  Rate matching for turbo coded transport channels.
 *
 *  Reference:    3GPP TS 36.212 version 10.0.0 Release 10 Sec. 5.1.4.1
 *********************************************************************************************/

#ifndef SRSRAN_RM_TURBO_H
#define SRSRAN_RM_TURBO_H

#include "srsran/config.h"
#include "srsran/phy/fec/turbo/turbodecoder.h"

#ifndef SRSRAN_RX_NULL
#define SRSRAN_RX_NULL 10000
#endif

#ifndef SRSRAN_TX_NULL
#define SRSRAN_TX_NULL 100
#endif

#include "srsran/config.h"

SRSRAN_API int srsran_rm_turbo_tx(uint8_t* w_buff,
                                  uint32_t buff_len,
                                  uint8_t* input,
                                  uint32_t in_len,
                                  uint8_t* output,
                                  uint32_t out_len,
                                  uint32_t rv_idx);

SRSRAN_API void srsran_rm_turbo_gentables();

SRSRAN_API void srsran_rm_turbo_free_tables();

SRSRAN_API int srsran_rm_turbo_tx_lut(uint8_t* w_buff,
                                      uint8_t* systematic,
                                      uint8_t* parity,
                                      uint8_t* output,
                                      uint32_t cb_idx,
                                      uint32_t out_len,
                                      uint32_t w_offset,
                                      uint32_t rv_idx);

SRSRAN_API int srsran_rm_turbo_rx(float*   w_buff,
                                  uint32_t buff_len,
                                  float*   input,
                                  uint32_t in_len,
                                  float*   output,
                                  uint32_t out_len,
                                  uint32_t rv_idx,
                                  uint32_t nof_filler_bits);

SRSRAN_API int
srsran_rm_turbo_rx_lut(int16_t* input, int16_t* output, uint32_t in_len, uint32_t cb_idx, uint32_t rv_idx);

SRSRAN_API int srsran_rm_turbo_rx_lut_(int16_t* input,
                                       int16_t* output,
                                       uint32_t in_len,
                                       uint32_t cb_idx,
                                       uint32_t rv_idx,
                                       bool     enable_input_tdec);

SRSRAN_API int
srsran_rm_turbo_rx_lut_8bit(int8_t* input, int8_t* output, uint32_t in_len, uint32_t cb_idx, uint32_t rv_idx);

/* TS 36.212 §5.1.4.1.2, MCH configured with pmch-TimeInterleaving-N: rv_idx
 * ranges 0..N-1 (N up to 16), which srsran_rm_turbo_tx_lut/rx_lut cannot
 * support (their k0_vec[][]/deinterleaver[][] tables are sized for exactly
 * rv_idx<4). Same packed-byte (_tx) / int16-LLR-softbuffer (_rx) I/O shape as
 * the _lut functions otherwise, so these are drop-in alternatives for the
 * MCH time-interleaving case, not a new format. */
/* n_cb_cap: TS 36.212 §5.1.4.1.2 N_cb = min(floor(N_IR/C), K_w) soft-buffer
 * limitation, pre-computed by the caller (0 = no real cap available, fall
 * back to the uncapped K_w -- exactly today's behavior). */
SRSRAN_API int srsran_rm_turbo_tx_mch(uint8_t* systematic,
                                      uint8_t* parity,
                                      uint8_t* output,
                                      uint32_t cb_idx,
                                      uint32_t out_len,
                                      uint32_t w_offset,
                                      uint32_t rv_idx,
                                      uint32_t e_min,
                                      uint32_t n_cb_cap);

SRSRAN_API int srsran_rm_turbo_rx_mch(
    int16_t* input, int16_t* output, uint32_t in_len, uint32_t cb_idx, uint32_t rv_idx, uint32_t e_min, uint32_t n_cb_cap);

#endif // SRSRAN_RM_TURBO_H
