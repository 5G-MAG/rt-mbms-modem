// 5G-MAG Reference Tools
// MBMS Modem Process
//
// A SystemInformationBlockType16 names a time if it carries timeInfo-r11 or timeReferenceInfo-r15: both are OPTIONAL (3GPP TS 36.331 V19.4.0 clauses 6.3.1 and 6.3.2).
// The decoder of the modem used to look at the r15 time reference only inside the timeInfo-r11 branch, so a SIB16 with the time reference alone was dropped.

#include "../src/Sib16Time.h"
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

static void add_r15(sib_type16_r11_s& s)
{
  s.ext = true;
  s.time_ref_info_r15.reset(new time_ref_info_r15_s());
  s.time_ref_info_r15->time_r15.ref_days_r15 = 17113;
}

// pack and unpack, so the object is what the receiver decodes
static sib_type16_r11_s through_the_air(const sib_type16_r11_s& in)
{
  uint8_t buf[256];
  bit_ref bw(buf, sizeof(buf));
  CHECK(in.pack(bw) == SRSASN_SUCCESS);
  sib_type16_r11_s out;
  cbit_ref br(buf, bw.distance_bytes());
  CHECK(out.unpack(br) == SRSASN_SUCCESS);
  return out;
}

int main()
{
  using MBMS_RT::sib16_time::carries_time;
  bool r11 = false, r15 = false;

  sib_type16_r11_s both;
  both.time_info_r11_present = true;
  both.time_info_r11.time_info_utc_r11 = 2524953600ULL * 100;
  add_r15(both);
  CHECK(carries_time(through_the_air(both), &r11, &r15) && r11 && r15);

  sib_type16_r11_s only11;
  only11.time_info_r11_present = true;
  only11.time_info_r11.time_info_utc_r11 = 2524953600ULL * 100;
  CHECK(carries_time(through_the_air(only11), &r11, &r15) && r11 && !r15);

  sib_type16_r11_s only15;                 // timeInfo-r11 absent: the case that was dropped
  add_r15(only15);
  CHECK(carries_time(through_the_air(only15), &r11, &r15) && !r11 && r15);

  sib_type16_r11_s zero_with_15;           // timeInfoUTC 0 is not a time, the r15 reference still is
  zero_with_15.time_info_r11_present = true;
  zero_with_15.time_info_r11.time_info_utc_r11 = 0;
  add_r15(zero_with_15);
  CHECK(carries_time(through_the_air(zero_with_15), &r11, &r15) && !r11 && r15);

  sib_type16_r11_s zero_only;
  zero_only.time_info_r11_present = true;
  zero_only.time_info_r11.time_info_utc_r11 = 0;
  CHECK(!carries_time(through_the_air(zero_only), &r11, &r15) && !r11 && !r15);

  sib_type16_r11_s nothing;
  CHECK(!carries_time(through_the_air(nothing), &r11, &r15));

  std::printf("sib16_presence_test: all passed\n");
  return 0;
}
