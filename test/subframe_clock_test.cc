// 5G-MAG Reference Tools
// MBMS Modem Process
//
// Tests for src/SubframeClock.h.

#include "../src/SubframeClock.h"

#include <cstdio>
#include <cstdlib>

using MBMS_RT::SubframeClock;

#define CHECK(cond)                                                                         \
  do {                                                                                      \
    if (!(cond)) {                                                                          \
      std::fprintf(stderr, "%s:%d: check failed: %s\n", __FILE__, __LINE__, #cond);          \
      std::exit(1);                                                                         \
    }                                                                                       \
  } while (0)

static SubframeClock::Request req(uint32_t tti, int64_t label)
{
  SubframeClock::Request r;
  r.tti       = tti;
  r.sfn       = (uint16_t)(tti / 10);
  r.label_ns  = label;
  return r;
}

int main()
{
  // A subframe that has already passed is answered at once.
  {
    SubframeClock c;
    for (uint32_t t = 0; t < 100; t++) c.note(t, (uint64_t)t * 1000000);
    CHECK(!c.latest());
    c.request(req(50, 7));
    CHECK(c.latest() && c.latest()->sample_clock_ns == 50u * 1000000 && c.latest()->request.label_ns == 7);
  }
  // A future subframe is answered when it arrives, with its own clock value.
  {
    SubframeClock c;
    for (uint32_t t = 0; t < 100; t++) c.note(t, (uint64_t)t * 1000000);
    c.request(req(150, 9));
    CHECK(!c.latest());
    for (uint32_t t = 100; t < 150; t++) c.note(t, (uint64_t)t * 1000000);
    CHECK(!c.latest());
    c.note(150, 150123456);
    CHECK(c.latest() && c.latest()->sample_clock_ns == 150123456 && c.latest()->request.label_ns == 9);
    // a later subframe does not disturb the stored anchor
    c.note(151, 151000000);
    CHECK(c.latest()->sample_clock_ns == 150123456);
  }
  // Across the TTI wrap at 10240.
  {
    SubframeClock c;
    uint64_t clk = 0;
    for (uint32_t i = 0; i < 20; i++) { c.note((10235 + i) % 10240, clk); clk += 1000000; }
    c.request(req(2, 1));
    CHECK(c.latest() && c.latest()->sample_clock_ns == 7u * 1000000);
  }
  // A subframe older than the history is not answered from a stale entry; it waits for the next occurrence.
  {
    SubframeClock c;
    for (uint32_t t = 0; t < 5000; t++) c.note(t % 10240, (uint64_t)t * 1000000);
    c.request(req(10, 3)); // seen 4990 subframes ago, beyond the 2048-subframe history
    CHECK(!c.latest());
    for (uint32_t t = 5000; t < 10251; t++) c.note(t % 10240, (uint64_t)t * 1000000);
    CHECK(c.latest() && c.latest()->sample_clock_ns == 10250ull * 1000000); // t = 10250 is tti 10 again
  }
  // The latest note is where the clock stands now.
  {
    SubframeClock c;
    CHECK(!c.last_note());
    c.note(7, 7000000);
    c.note(8, 8000000);
    CHECK(c.last_note() && c.last_note()->tti == 8 && c.last_note()->sample_clock_ns == 8000000);
  }
  std::printf("subframe_clock_test: all passed\n");
  return 0;
}
