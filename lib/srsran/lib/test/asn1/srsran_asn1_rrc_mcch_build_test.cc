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

/* MCCH-Message (MBSFNAreaConfiguration-r9) build-from-scratch pack/unpack round trip.
 *
 * srsran_asn1_rrc_mcch_test.cc (in this same directory) only UNPACKS a
 * captured reference byte string and re-packs the already-unpacked object —
 * it never exercises pack() starting from a struct populated by hand. This
 * test does exactly that: constructs an mcch_msg_s field by field (as the
 * eNB's RRC layer would when building an MCCH message for a FeMBMS carrier,
 * see rrc.cc/pack_mcch()), packs it, unpacks the result into a second
 * object, checks every field round-trips, then re-packs the unpacked copy
 * and checks the two byte streams are bit-for-bit identical.
 *
 * Two PMCH-Info-r9 entries with distinct field values (different MCC/MNC
 * length, TMGI, session/lc-ch IDs, sf_alloc_end, MCS, scheduling period) are
 * used so a field-index or list-length mixup would be caught.
 */

#include "srsran/asn1/rrc.h"
#include "srsran/asn1/rrc_utils.h"
#include "srsran/common/bcd_helpers.h"
#include "srsran/interfaces/rrc_interface_types.h"
#include <iostream>

using namespace asn1::rrc;

#define TESTASSERT(cond)                                                                                              \
  {                                                                                                                   \
    if (!(cond)) {                                                                                                   \
      std::cout << "[" << __FUNCTION__ << "][Line " << __LINE__ << "]: FAIL at " << (#cond) << std::endl;            \
      return -1;                                                                                                     \
    }                                                                                                                 \
  }

static void build_mcch(mcch_msg_s* msg)
{
  msg->msg.set_c1();
  mbsfn_area_cfg_r9_s& area = msg->msg.c1().mbsfn_area_cfg_r9();
  area.non_crit_ext_present = false;

  area.common_sf_alloc_r9.resize(2);
  area.common_sf_alloc_r9[0].radioframe_alloc_period = mbsfn_sf_cfg_s::radioframe_alloc_period_e_::n16;
  area.common_sf_alloc_r9[0].radioframe_alloc_offset = 3;
  area.common_sf_alloc_r9[0].sf_alloc.set_one_frame().from_string("101010");
  area.common_sf_alloc_r9[1].radioframe_alloc_period = mbsfn_sf_cfg_s::radioframe_alloc_period_e_::n4;
  area.common_sf_alloc_r9[1].radioframe_alloc_offset = 1;
  area.common_sf_alloc_r9[1].sf_alloc.set_four_frames().from_string("110011001100110011001100");
  area.common_sf_alloc_period_r9 = mbsfn_area_cfg_r9_s::common_sf_alloc_period_r9_e_::rf64;

  area.pmch_info_list_r9.resize(2);

  // --- PMCH 0: 3-digit MNC, small IDs ---
  pmch_info_r9_s& pmch0 = area.pmch_info_list_r9[0];
  pmch0.ext                       = false;
  pmch0.pmch_cfg_r9.ext           = false;
  pmch0.pmch_cfg_r9.sf_alloc_end_r9 = 200;
  pmch0.pmch_cfg_r9.data_mcs_r9      = 10;
  pmch0.pmch_cfg_r9.mch_sched_period_r9 = pmch_cfg_r9_s::mch_sched_period_r9_e_::rf512;
  pmch0.mbms_session_info_list_r9.resize(1);
  mbms_session_info_r9_s& sess0 = pmch0.mbms_session_info_list_r9[0];
  sess0.ext                     = false;
  sess0.session_id_r9_present   = true;
  plmn_id_s& plmn0               = sess0.tmgi_r9.plmn_id_r9.set_explicit_value_r9();
  plmn0.mcc_present               = true;
  plmn0.mcc                       = {1, 2, 3};
  plmn0.mnc.resize(2);
  plmn0.mnc[0] = 4;
  plmn0.mnc[1] = 5;
  sess0.tmgi_r9.service_id_r9.from_number(7);
  sess0.session_id_r9.from_number(3);
  sess0.lc_ch_id_r9 = 5;

  // --- PMCH 1: 2-digit MNC, larger IDs, different scheduling ---
  pmch_info_r9_s& pmch1 = area.pmch_info_list_r9[1];
  pmch1.ext                       = false;
  pmch1.pmch_cfg_r9.ext           = false;
  pmch1.pmch_cfg_r9.sf_alloc_end_r9 = 1535;
  pmch1.pmch_cfg_r9.data_mcs_r9      = 27;
  pmch1.pmch_cfg_r9.mch_sched_period_r9 = pmch_cfg_r9_s::mch_sched_period_r9_e_::rf8;
  pmch1.mbms_session_info_list_r9.resize(1);
  mbms_session_info_r9_s& sess1 = pmch1.mbms_session_info_list_r9[0];
  sess1.ext                     = false;
  sess1.session_id_r9_present   = true;
  plmn_id_s& plmn1               = sess1.tmgi_r9.plmn_id_r9.set_explicit_value_r9();
  plmn1.mcc_present               = true;
  plmn1.mcc                       = {9, 8, 7};
  plmn1.mnc.resize(3);
  plmn1.mnc[0] = 6;
  plmn1.mnc[1] = 5;
  plmn1.mnc[2] = 4;
  sess1.tmgi_r9.service_id_r9.from_number(99);
  sess1.session_id_r9.from_number(200);
  sess1.lc_ch_id_r9 = 9;
}

static int check_mcch(const mcch_msg_s& msg)
{
  TESTASSERT(msg.msg.type() == mcch_msg_type_c::types::c1);
  TESTASSERT(msg.msg.c1().type() == mcch_msg_type_c::c1_c_::types::mbsfn_area_cfg_r9);
  const mbsfn_area_cfg_r9_s& area = msg.msg.c1().mbsfn_area_cfg_r9();

  TESTASSERT(not area.non_crit_ext_present);
  TESTASSERT(area.common_sf_alloc_r9.size() == 2);
  TESTASSERT(area.common_sf_alloc_r9[0].radioframe_alloc_period ==
             mbsfn_sf_cfg_s::radioframe_alloc_period_e_::n16);
  TESTASSERT(area.common_sf_alloc_r9[0].radioframe_alloc_offset == 3);
  TESTASSERT(area.common_sf_alloc_r9[0].sf_alloc.type() == mbsfn_sf_cfg_s::sf_alloc_c_::types::one_frame);
  TESTASSERT(area.common_sf_alloc_r9[0].sf_alloc.one_frame().to_string() == "101010");
  TESTASSERT(area.common_sf_alloc_r9[1].radioframe_alloc_period == mbsfn_sf_cfg_s::radioframe_alloc_period_e_::n4);
  TESTASSERT(area.common_sf_alloc_r9[1].radioframe_alloc_offset == 1);
  TESTASSERT(area.common_sf_alloc_r9[1].sf_alloc.type() == mbsfn_sf_cfg_s::sf_alloc_c_::types::four_frames);
  TESTASSERT(area.common_sf_alloc_r9[1].sf_alloc.four_frames().to_string() == "110011001100110011001100");
  TESTASSERT(area.common_sf_alloc_period_r9 == mbsfn_area_cfg_r9_s::common_sf_alloc_period_r9_e_::rf64);
  TESTASSERT(area.pmch_info_list_r9.size() == 2);

  const pmch_info_r9_s& pmch0 = area.pmch_info_list_r9[0];
  TESTASSERT(not pmch0.ext);
  TESTASSERT(not pmch0.pmch_cfg_r9.ext);
  TESTASSERT(pmch0.pmch_cfg_r9.sf_alloc_end_r9 == 200);
  TESTASSERT(pmch0.pmch_cfg_r9.data_mcs_r9 == 10);
  TESTASSERT(pmch0.pmch_cfg_r9.mch_sched_period_r9 == pmch_cfg_r9_s::mch_sched_period_r9_e_::rf512);
  TESTASSERT(pmch0.mbms_session_info_list_r9.size() == 1);
  const mbms_session_info_r9_s& sess0 = pmch0.mbms_session_info_list_r9[0];
  TESTASSERT(not sess0.ext);
  TESTASSERT(sess0.session_id_r9_present);
  TESTASSERT(sess0.tmgi_r9.plmn_id_r9.type() == tmgi_r9_s::plmn_id_r9_c_::types::explicit_value_r9);
  const plmn_id_s& plmn0 = sess0.tmgi_r9.plmn_id_r9.explicit_value_r9();
  TESTASSERT(plmn0.mcc_present);
  TESTASSERT(plmn0.mcc[0] == 1 && plmn0.mcc[1] == 2 && plmn0.mcc[2] == 3);
  TESTASSERT(plmn0.mnc.size() == 2 && plmn0.mnc[0] == 4 && plmn0.mnc[1] == 5);
  TESTASSERT(sess0.tmgi_r9.service_id_r9.to_number() == 7);
  TESTASSERT(sess0.session_id_r9.to_number() == 3);
  TESTASSERT(sess0.lc_ch_id_r9 == 5);

  const pmch_info_r9_s& pmch1 = area.pmch_info_list_r9[1];
  TESTASSERT(not pmch1.ext);
  TESTASSERT(not pmch1.pmch_cfg_r9.ext);
  TESTASSERT(pmch1.pmch_cfg_r9.sf_alloc_end_r9 == 1535);
  TESTASSERT(pmch1.pmch_cfg_r9.data_mcs_r9 == 27);
  TESTASSERT(pmch1.pmch_cfg_r9.mch_sched_period_r9 == pmch_cfg_r9_s::mch_sched_period_r9_e_::rf8);
  TESTASSERT(pmch1.mbms_session_info_list_r9.size() == 1);
  const mbms_session_info_r9_s& sess1 = pmch1.mbms_session_info_list_r9[0];
  TESTASSERT(not sess1.ext);
  TESTASSERT(sess1.session_id_r9_present);
  TESTASSERT(sess1.tmgi_r9.plmn_id_r9.type() == tmgi_r9_s::plmn_id_r9_c_::types::explicit_value_r9);
  const plmn_id_s& plmn1 = sess1.tmgi_r9.plmn_id_r9.explicit_value_r9();
  TESTASSERT(plmn1.mcc_present);
  TESTASSERT(plmn1.mcc[0] == 9 && plmn1.mcc[1] == 8 && plmn1.mcc[2] == 7);
  TESTASSERT(plmn1.mnc.size() == 3 && plmn1.mnc[0] == 6 && plmn1.mnc[1] == 5 && plmn1.mnc[2] == 4);
  TESTASSERT(sess1.tmgi_r9.service_id_r9.to_number() == 99);
  TESTASSERT(sess1.session_id_r9.to_number() == 200);
  TESTASSERT(sess1.lc_ch_id_r9 == 9);

  return 0;
}

int mcch_build_from_scratch_test()
{
  mcch_msg_s tx_msg;
  build_mcch(&tx_msg);

  // Sanity: the struct we just built reads back as expected before any pack/unpack.
  TESTASSERT(check_mcch(tx_msg) == 0);

  // --- Pack the hand-built message ---
  const uint32_t buf_cap = 256;
  uint8_t        buf1[buf_cap];
  bzero(buf1, sizeof(buf1));
  asn1::bit_ref bref1(&buf1[0], buf_cap);
  TESTASSERT(tx_msg.pack(bref1) == asn1::SRSASN_SUCCESS);
  TESTASSERT(bref1.distance() > 0);
  const uint32_t nof_bytes = (bref1.distance() + 7) / 8;
  TESTASSERT(nof_bytes <= buf_cap);

  // --- Unpack into a fresh object and check every field round-tripped ---
  mcch_msg_s     rx_msg;
  asn1::cbit_ref cbref(&buf1[0], nof_bytes);
  TESTASSERT(rx_msg.unpack(cbref) == asn1::SRSASN_SUCCESS);
  TESTASSERT(cbref.distance() == bref1.distance());
  TESTASSERT(check_mcch(rx_msg) == 0);

  // --- Re-pack the unpacked copy: must reproduce the exact same bitstream ---
  uint8_t buf2[buf_cap];
  bzero(buf2, sizeof(buf2));
  asn1::bit_ref bref2(&buf2[0], buf_cap);
  TESTASSERT(rx_msg.pack(bref2) == asn1::SRSASN_SUCCESS);
  TESTASSERT(bref2.distance() == bref1.distance());
  TESTASSERT(memcmp(buf1, buf2, nof_bytes) == 0);

  return 0;
}

int main(int argc, char** argv)
{
  auto& asn1_logger = srslog::fetch_basic_logger("ASN1", false);
  asn1_logger.set_level(srslog::basic_levels::debug);
  asn1_logger.set_hex_dump_max_size(-1);

  srslog::init();

  TESTASSERT(mcch_build_from_scratch_test() == 0);

  return 0;
}
