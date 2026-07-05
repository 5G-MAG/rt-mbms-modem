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
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <unistd.h>

#include "srsran/phy/ue/ue_mib.h"

#include "srsran/phy/utils/debug.h"
#include "srsran/phy/utils/vector.h"

#define MIB_BUFFER_MAX_SAMPLES_FOR_PRB(prb)   (8 * SRSRAN_SF_LEN_PRB(prb))
#define MIB_BUFFER_MAX_SAMPLES                MIB_BUFFER_MAX_SAMPLES_FOR_PRB(SRSRAN_UE_MIB_NOF_PRB)

int srsran_ue_mib_init(srsran_ue_mib_t* q, cf_t* in_buffer, uint32_t max_prb)
{
  int ret = SRSRAN_ERROR_INVALID_INPUTS;

  if (q != NULL) {
    ret = SRSRAN_ERROR;
    bzero(q, sizeof(srsran_ue_mib_t));

    if (srsran_pbch_init(&q->pbch)) {
      ERROR("Error initiating PBCH");
      goto clean_exit;
    }

    q->sf_symbols = srsran_vec_cf_malloc(SRSRAN_SF_LEN_RE(max_prb, SRSRAN_CP_NORM));
    if (!q->sf_symbols) {
      perror("malloc");
      goto clean_exit;
    }

    if (srsran_ofdm_rx_init(&q->fft, SRSRAN_CP_NORM, in_buffer, q->sf_symbols, max_prb)) {
      ERROR("Error initializing FFT");
      goto clean_exit;
    }
    if (srsran_chest_dl_init(&q->chest, max_prb, 1)) {
      ERROR("Error initializing reference signal");
      goto clean_exit;
    }
    if (srsran_chest_dl_res_init(&q->chest_res, max_prb)) {
      ERROR("Error initializing reference signal");
      goto clean_exit;
    }
    srsran_ue_mib_reset(q);

    ret = SRSRAN_SUCCESS;
  }

clean_exit:
  if (ret == SRSRAN_ERROR) {
    srsran_ue_mib_free(q);
  }
  return ret;
}

void srsran_ue_mib_free(srsran_ue_mib_t* q)
{
  if (q->sf_symbols) {
    free(q->sf_symbols);
  }
  srsran_sync_free(&q->sfind);
  srsran_chest_dl_res_free(&q->chest_res);
  srsran_chest_dl_free(&q->chest);
  srsran_pbch_free(&q->pbch);
  srsran_ofdm_rx_free(&q->fft);

  bzero(q, sizeof(srsran_ue_mib_t));
}

int srsran_ue_mib_set_cell(srsran_ue_mib_t* q, srsran_cell_t cell)
{
  int ret = SRSRAN_ERROR_INVALID_INPUTS;

  if (q != NULL && cell.nof_ports <= SRSRAN_MAX_PORTS) {
    if (srsran_pbch_set_cell(&q->pbch, cell)) {
      ERROR("Error initiating PBCH");
      return SRSRAN_ERROR;
    }
    if (cell.mbsfn_prb != 0 && cell.nof_prb != cell.mbsfn_prb) {
      if (srsran_ofdm_rx_set_prb_symbol_sz(&q->fft, cell.cp, cell.nof_prb,
            srsran_symbol_sz(cell.mbsfn_prb))) {
        ERROR("Error resizing FFT\n");
        return SRSRAN_ERROR;
      }
    } else {
      if (srsran_ofdm_rx_set_prb(&q->fft, cell.cp, cell.nof_prb)) {
        ERROR("Error initializing FFT\n");
        return SRSRAN_ERROR;
      }
    }

    if (cell.nof_ports == 0) {
      cell.nof_ports = SRSRAN_MAX_PORTS;
    }

    if (srsran_chest_dl_set_cell(&q->chest, cell)) {
      ERROR("Error initializing reference signal");
      return SRSRAN_ERROR;
    }
    srsran_ue_mib_reset(q);

    ret = SRSRAN_SUCCESS;
  }
  return ret;
}

void srsran_ue_mib_reset(srsran_ue_mib_t* q)
{
  q->frame_cnt = 0;
  srsran_pbch_decode_reset(&q->pbch);
}

int srsran_ue_mib_decode(srsran_ue_mib_t* q,
                         uint8_t          bch_payload[SRSRAN_BCH_PAYLOAD_LEN],
                         uint32_t*        nof_tx_ports,
                         int*             sfn_offset)
{
  int ret = SRSRAN_SUCCESS;

  /* Run FFT for the slot symbols */
  srsran_ofdm_rx_sf(&q->fft);

  // sf_idx is always 0 in MIB
  srsran_dl_sf_cfg_t sf_cfg;
  ZERO_OBJECT(sf_cfg);

  // Current MIB decoder implementation uses a single antenna
  cf_t* sf_buffer[SRSRAN_MAX_PORTS] = {};
  sf_buffer[0]                      = q->sf_symbols;

  /* Get channel estimates of sf idx #0 for each port */
  ret = srsran_chest_dl_estimate(&q->chest, &sf_cfg, sf_buffer, &q->chest_res);
  if (ret < 0) {
    return SRSRAN_ERROR;
  }
  /* Reset decoder if we missed a frame */
  if (q->frame_cnt > 8) {
    INFO("Resetting PBCH decoder after %d frames", q->frame_cnt);
    srsran_ue_mib_reset(q);
  }

  /* Decode PBCH */
  ret = srsran_pbch_decode(&q->pbch, &q->chest_res, sf_buffer, bch_payload, nof_tx_ports, sfn_offset);
  if (ret < 0) {
    ERROR("Error decoding PBCH (%d)", ret);
  } else if (ret == 1) {
    INFO("MIB decoded: %u, snr=%.1f dB", q->frame_cnt, q->chest_res.snr_db);
    srsran_ue_mib_reset(q);
    ret = SRSRAN_UE_MIB_FOUND;
  } else {
    ret = SRSRAN_UE_MIB_NOTFOUND;
    INFO("MIB not decoded: %u, snr=%.1f dB", q->frame_cnt, q->chest_res.snr_db);
    q->frame_cnt++;
  }

  return ret;
}

int srsran_ue_mib_sync_init_multi(srsran_ue_mib_sync_t* q,
                                  int(recv_callback)(void*, cf_t* [SRSRAN_MAX_CHANNELS], uint32_t, srsran_timestamp_t*),
                                  uint32_t nof_rx_channels,
                                  void*    stream_handler)
{
  return srsran_ue_mib_sync_init_multi_prb(q, recv_callback, nof_rx_channels, stream_handler, SRSRAN_UE_MIB_NOF_PRB);
}

int srsran_ue_mib_sync_init_multi_prb(srsran_ue_mib_sync_t* q,
    int(recv_callback)(void*, cf_t* [SRSRAN_MAX_CHANNELS], uint32_t, srsran_timestamp_t*),
    uint32_t nof_rx_channels,
    void*    stream_handler,
    uint8_t nof_prb)
{
  // sf_buffer is the raw sample storage that srsran_ue_mib_init() below hands to
  // srsran_ofdm_rx_init() (and that ue_sync fills, one subframe at a time, via the
  // receive_callback) - it must hold a full subframe AT nof_prb, not at the fixed
  // SRSRAN_UE_MIB_NOF_PRB (6) that MIB_BUFFER_MAX_SAMPLES is defined for. Sizing it from the
  // constant regardless of nof_prb undersizes the buffer for any nof_prb > 6 (e.g. exactly at
  // capacity at 50 PRB, and a 2x overflow at 100 PRB), which corrupts the heap the first time a
  // subframe is written/read and crashes later and unpredictably (seen both as a
  // MultichannelRingbuffer assertion and as a segfault inside fftwf_execute, depending on what
  // the corrupted memory was reused for next).
  for (int i = 0; i < nof_rx_channels; i++) {
    q->sf_buffer[i] = srsran_vec_cf_malloc(MIB_BUFFER_MAX_SAMPLES_FOR_PRB(nof_prb));
  }
  q->nof_rx_channels = nof_rx_channels;

  // Use 1st RF channel only to receive MIB
  if (srsran_ue_mib_init(&q->ue_mib, q->sf_buffer[0], nof_prb)) {
    ERROR("Error initiating ue_mib");
    return SRSRAN_ERROR;
  }
  // Configure ue_sync to receive all channels
  if (srsran_ue_sync_init_multi(
          &q->ue_sync, nof_prb, false, recv_callback, nof_rx_channels, stream_handler)) {
    fprintf(stderr, "Error initiating ue_sync\n");
    srsran_ue_mib_free(&q->ue_mib);
    return SRSRAN_ERROR;
  }
  return SRSRAN_SUCCESS;
}

int srsran_ue_mib_sync_set_cell(srsran_ue_mib_sync_t* q, srsran_cell_t cell)
{
  // MIB search is done at 6 PRB by default
  return srsran_ue_mib_sync_set_cell_prb(q, cell, SRSRAN_UE_MIB_NOF_PRB);
}

int srsran_ue_mib_sync_set_cell_prb(srsran_ue_mib_sync_t* q, srsran_cell_t cell, uint8_t nof_prb)
{
  // If the ports are set to 0, ue_mib goes through 1, 2 and 4 ports to blindly detect nof_ports
  cell.nof_ports = 0;

  cell.nof_prb = nof_prb;

  if (srsran_ue_mib_set_cell(&q->ue_mib, cell)) {
    ERROR("Error initiating ue_mib");
    return SRSRAN_ERROR;
  }
  if (srsran_ue_sync_set_cell(&q->ue_sync, cell)) {
    ERROR("Error initiating ue_sync");
    srsran_ue_mib_free(&q->ue_mib);
    return SRSRAN_ERROR;
  }
  return SRSRAN_SUCCESS;
}

void srsran_ue_mib_sync_free(srsran_ue_mib_sync_t* q)
{
  for (int i = 0; i < q->nof_rx_channels; i++) {
    if (q->sf_buffer[i]) {
      free(q->sf_buffer[i]);
    }
  }
  srsran_ue_mib_free(&q->ue_mib);
  srsran_ue_sync_free(&q->ue_sync);
}

void srsran_ue_mib_sync_reset(srsran_ue_mib_sync_t* q)
{
  srsran_ue_mib_reset(&q->ue_mib);
  srsran_ue_sync_reset(&q->ue_sync);
}

int srsran_ue_mib_sync_decode(srsran_ue_mib_sync_t* q,
                              uint32_t              max_frames_timeout,
                              uint8_t               bch_payload[SRSRAN_BCH_PAYLOAD_LEN],
                              uint32_t*             nof_tx_ports,
                              int*                  sfn_offset)
{
  return srsran_ue_mib_sync_decode_prb(q, max_frames_timeout, bch_payload, nof_tx_ports, sfn_offset, SRSRAN_UE_MIB_NOF_PRB);
}

int srsran_ue_mib_sync_decode_prb(srsran_ue_mib_sync_t* q,
                              uint32_t              max_frames_timeout,
                              uint8_t               bch_payload[SRSRAN_BCH_PAYLOAD_LEN],
                              uint32_t*             nof_tx_ports,
                              int*                  sfn_offset,
                              uint8_t               nof_prb)
{
  int      ret        = SRSRAN_ERROR_INVALID_INPUTS;
  uint32_t nof_frames = 0;
  int      mib_ret    = SRSRAN_UE_MIB_NOTFOUND;

  if (q == NULL) {
    return ret;
  }

  srsran_ue_mib_sync_reset(q);

  do {
    mib_ret = SRSRAN_UE_MIB_NOTFOUND;
    ret     = srsran_ue_sync_zerocopy(&q->ue_sync, q->sf_buffer, MIB_BUFFER_MAX_SAMPLES_FOR_PRB(nof_prb));
    if (ret < 0) {
      ERROR("Error calling srsran_ue_sync_work()");
      return -1;
    }

    if (srsran_ue_sync_get_sfn(&q->ue_sync)%4 == 0 && srsran_ue_sync_get_sfidx(&q->ue_sync) == 0) {
   // if (srsran_ue_sync_get_sfidx(&q->ue_sync) == 0 ) { // [TODO] fix for mixed-mode
      if (ret == 1) {
        mib_ret = srsran_ue_mib_decode(&q->ue_mib, bch_payload, nof_tx_ports, sfn_offset);
      } else {
        DEBUG("Resetting PBCH decoder after %d frames", q->ue_mib.frame_cnt);
        srsran_ue_mib_reset(&q->ue_mib);
      }
      nof_frames++;
    }
  } while (mib_ret == SRSRAN_UE_MIB_NOTFOUND && nof_frames < max_frames_timeout);

  return mib_ret;
}
