// 5G-MAG Reference Tools
// MBMS Modem Process
//
// Tests for src/Sib16Time.h. Expected values are worked out from the text of 3GPP TS 36.331 V19.4.0 clauses 5.2.3b, 6.3.1 and
// 6.3.2, and from what the transmitter log reported on a live run (window SFN 516, boundary SFN 520).

#include "../src/Sib16Time.h"

#include <cstdio>
#include <cstdlib>

using namespace MBMS_RT::sib16_time;

#define CHECK(cond)                                                                         \
  do {                                                                                      \
    if (!(cond)) {                                                                          \
      std::fprintf(stderr, "%s:%d: check failed: %s\n", __FILE__, __LINE__, #cond);          \
      std::exit(1);                                                                         \
    }                                                                                       \
  } while (0)

static uint16_t boundary(uint32_t n, uint32_t w, uint32_t t, uint32_t rx)
{
  uint16_t out = 0xFFFF;
  CHECK(boundary_sfn(n, w, t, rx, &out));
  return out;
}

int main()
{
  // n=2, w=40, T=256: x=40, window starts at SFN mod 256 = 4, ends 40 ms later: boundary SFN mod 256 = 8.
  CHECK(boundary(2, 40, 256, 4) == 8);
  CHECK(boundary(2, 40, 256, 516) == 520);   // seen live
  CHECK(boundary(2, 40, 256, 519) == 520);   // decoded in the last frame of the window
  CHECK(boundary(2, 40, 256, 772) == 776);
  // n=1, w=20, T=64: window [0,20), boundary SFN mod 64 = 2.
  CHECK(boundary(1, 20, 64, 64) == 66);
  CHECK(boundary(1, 20, 64, 65) == 66);
  CHECK(boundary(1, 20, 64, 960) == 962);
  // n=2, w=20, T=256 (the shipped schedule): x=20, window [20,40), boundary SFN mod 256 = 4.
  CHECK(boundary(2, 20, 256, 2) == 4);
  CHECK(boundary(2, 20, 256, 258) == 260);
  // A window that starts inside a frame: n=2, w=15, x=15, a=5: window [15,30) ms, boundary at 30 ms = SFN mod T = 3.
  CHECK(boundary(2, 15, 64, 1) == 3);
  // Boundary after the SFN wrap: n=8, w=80, T=64, x=560: window [560,640), boundary 640 = the next period, SFN 0.
  CHECK(boundary(8, 80, 64, 1020) == 0);
  CHECK(boundary(8, 80, 64, 1023) == 0);
  // Impossible schedules.
  uint16_t out = 0;
  CHECK(!boundary_sfn(0, 20, 64, 0, &out));
  CHECK(!boundary_sfn(1, 0, 64, 0, &out));
  CHECK(!boundary_sfn(1, 20, 0, 0, &out));

  // timeInfoUTC: the GPS epoch (1980-01-06) is 2,524,953,600 s after 1900-01-01 and 315,964,800 s after the Unix epoch.
  CHECK(time_info_utc_to_unix_ms(2524953600ULL * 100) == 315964800LL * 1000);
  CHECK(time_info_utc_to_unix_ms(2524953600ULL * 100 + 1) == 315964800LL * 1000 + 10);
  CHECK(time_info_utc_to_unix_ms(2208988800ULL * 100) == 0);
  // ReferenceTime-r15, 0.25 us units.
  CHECK(reference_time_ns(0, 0, 0, 1) == 250);
  CHECK(reference_time_ns(0, 0, 0, 3999) == 999750);
  CHECK(reference_time_ns(0, 0, 1, 0) == 1000000);
  CHECK(reference_time_ns(0, 1, 0, 0) == 1000000000LL);
  CHECK(reference_time_ns(1, 0, 0, 0) == 86400LL * 1000000000LL);
  CHECK(reference_time_ns(17113, 63491, 61, 3867) == 1478626691061966750LL); // value decoded from a live SIB16

  std::printf("sib16_time_test: all passed\n");
  return 0;
}
