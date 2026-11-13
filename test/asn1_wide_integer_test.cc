// 5G-MAG Reference Tools
// MBMS Modem Process
//
// A constrained integer wider than 32 bits must survive a pack/unpack round trip. SystemInformationBlockType16 carries
// timeInfoUTC-r11 as INTEGER (0..549755813887), 39 bits (3GPP TS 36.331 V19.4.0 clause 6.3.1); a decoder that shifts a
// 32-bit intermediate loses everything above bit 31.

#include "srsran/asn1/rrc/bcch_msg.h"

#include <cstdint>
#include <cstdio>
#include <cstdlib>

using namespace asn1;
using namespace asn1::rrc;

#define CHECK(cond)                                                                         \
  do {                                                                                      \
    if (!(cond)) {                                                                          \
      std::fprintf(stderr, "%s:%d: check failed: %s\n", __FILE__, __LINE__, #cond);          \
      std::exit(1);                                                                         \
    }                                                                                       \
  } while (0)

static void round_trip(uint64_t time_info_utc, bool with_r15)
{
  sib_type16_r11_s in;
  in.time_info_r11_present                  = true;
  in.time_info_r11.time_info_utc_r11        = time_info_utc;
  in.time_info_r11.leap_seconds_r11_present = true;
  in.time_info_r11.leap_seconds_r11         = 18;
  if (with_r15) {
    in.ext = true;
    in.time_ref_info_r15.reset(new time_ref_info_r15_s());
    in.time_ref_info_r15->time_r15.ref_days_r15                  = 17113;
    in.time_ref_info_r15->time_r15.ref_seconds_r15               = 63491;
    in.time_ref_info_r15->time_r15.ref_milli_seconds_r15         = 61;
    in.time_ref_info_r15->time_r15.ref_quarter_micro_seconds_r15 = 3867;
    in.time_ref_info_r15->uncertainty_r15_present                = true;
    in.time_ref_info_r15->uncertainty_r15                        = 12;
  }
  uint8_t  buf[64] = {};
  bit_ref  bref(buf, sizeof(buf));
  CHECK(in.pack(bref) == SRSASN_SUCCESS);
  cbit_ref          cref(buf, (uint32_t)bref.distance_bytes());
  sib_type16_r11_s  out;
  CHECK(out.unpack(cref) == SRSASN_SUCCESS);
  CHECK(out.time_info_r11.time_info_utc_r11 == time_info_utc);
  CHECK(out.time_info_r11.leap_seconds_r11 == 18);
  if (with_r15) {
    CHECK(out.time_ref_info_r15.get() != nullptr);
    CHECK(out.time_ref_info_r15->time_r15.ref_days_r15 == 17113);
    CHECK(out.time_ref_info_r15->time_r15.ref_seconds_r15 == 63491);
    CHECK(out.time_ref_info_r15->time_r15.ref_milli_seconds_r15 == 61);
    CHECK(out.time_ref_info_r15->time_r15.ref_quarter_micro_seconds_r15 == 3867);
    CHECK(out.time_ref_info_r15->uncertainty_r15 == 12);
  }
}

int main()
{
  const uint64_t values[] = {0ULL,
                             1ULL,
                             4294967295ULL,            // 2^32 - 1
                             4294967296ULL,            // 2^32
                             400358242773ULL,          // 2026-11-13 in 10 ms units since 1900
                             0x5555555555ULL,          // alternating bits across the whole 39-bit width
                             0x2AAAAAAAAAULL,
                             549755813887ULL};         // the upper bound, 2^39 - 1
  for (uint64_t v : values) {
    round_trip(v, false);
    round_trip(v, true);
  }
  std::printf("asn1_wide_integer_test: all passed\n");
  return 0;
}
