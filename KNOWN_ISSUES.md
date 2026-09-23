# Known issues

## n_prb=25 PMCH decode: ~98% BLER, plus a separate periodic full sync loss

**Status: RESOLVED 2026-07-15.** Root cause found and fixed; verified live,
repeatedly, across many restarts: `MCH BLER 0.0`, zero sync losses in
multi-minute runs, raw ZMQ throughput ~91% of nominal (up from ~20%).

### Root cause

`~/soapy-zmq-bridge/build.sh` compiled `ZmqRxDevice.cpp` with **no compiler
optimization flags at all** (`g++ -std=c++17 -shared -fPIC ...`, no
`-O2`/`-O3`). The FIR decimation filter in `Decimator::process()` — active
only when the bridge's ratio is >1, i.e. only at n_prb=25, never at n_prb=50
— is a nested `std::complex<float>` multiply-accumulate loop that, at `-O0`,
pegged its own thread at a sustained 100% CPU (confirmed via `pidstat -t`)
and could only drain ~20% of the eNB's actual send rate (the eNB itself was
confirmed idle via `pidstat`, sending at ~92% of nominal via a direct
`TX_SEND_RATE_DIAG` measurement — it was never the bottleneck). The bridge
silently dropped the majority of subframes before they ever reached the
modem, which explains both symptoms below simultaneously: PMCH content
genuinely wasn't arriving (Problem 1), and PSS/SSS tracking genuinely lost
signal for stretches long enough to exceed `TRACK_MAX_LOST=3` (Problem 2).

**Fix**: added `-O3 -march=native` to `build.sh`. Decimator thread CPU
dropped to ~35-45%; raw throughput went from ~195 Mbps to ~890-899 Mbps
(~91% of the 983 Mbps nominal for a continuous 15.36 Msps native stream);
sync losses went from ~15-20/minute to zero in every subsequent test run;
`MCHDIAG` shows `crc=1` on every line, `MCH BLER 0.0`.

**Important**: do not re-increase the decimation filter's tap count without
re-testing carefully — a 4x tap-count increase (tried once, to test a
follow-up CINR question, see below) reintroduced the *exact* original
failure mode even at `-O3`, because 4x the taps is 4x the compute per
sample and there isn't headroom to spare. If touching `design_decim_lowpass()`
again, verify decimator thread CPU via `pidstat -t` stays well under 100%
before trusting any functional result.

### Everything below this line is the original investigation log (kept for
### context — most of it was ruled out as a red herring once the real cause
### was found, not because the reasoning was wrong, just because it wasn't
### the dominant effect). TX's grid population, TX's IFFT, the decimator's
### own math (once running fast enough), and RX's FFT/extraction were all
### independently proven correct during this investigation via bit-exact
### capture-and-diff against from-scratch reimplementations — none of that
### work was wasted, it just wasn't where the bug actually was.

### Context / how to reproduce

Test setup: `rt-mbms-tx` eNB configured for `n_prb = 25` (5 MHz cell), receiving
through `soapy-zmq-bridge`'s ZMQ bridge with its decimator active (native wire
rate 15.36 Msps, decimated to 7.68 Msps, ratio 2 — enabled via
`device_args=...,native_srate=15.36e6` in `modem_zmqtest.conf`). `pmch_bandwidth`
is left unset in `enb_baseline.conf` (PMCH spans the full 25-PRB carrier, no
extended-bandwidth signaling — confirmed via `/modem-api/sib_info`:
`sib13.areas[0].pmch_bandwidth: 0`, which is the *correct* value for this
config, not a decode failure — see `Phy.cpp:290` and `rrc_utils.cc:1257-1261`
for why 0 means "no r17 extension present").

Two distinct problems showed up at this configuration that did **not** occur
at the n_prb=50 baseline (which ran clean for ~5 hours in the same test rig)
— both are now understood to be downstream symptoms of the single root cause
above, not two separate bugs.

### Problem 1 (RESOLVED, was: PMCH content BLER ~98%)

CAS/PDSCH and MCCH both decode correctly (PDSCH BLER 0.0, MCCH BLER ~14%,
SIB13/SIB15/SIB16 all confirmed via the REST API to be decoding correctly).
PMCH (the actual MCH data channel) failed CRC on ~98% of subframes prior to
the fix; now passes on effectively 100%.

Per-subframe diagnostics (`MCH_SF5_DIAG=1` env var, prints via
`MbsfnFrameProcessor.cpp`'s `MCHDIAG` stderr lines) showed:
- TX and RX **agreed** on TBS (4008 bits / 501 bytes), MCS (9), modulation
  (QPSK), nof_prb (25), nof_re (3000), rv (0) — confirmed by comparing
  `MCHDIAG` (RX) against the eNB's own `PMCH: ...` log line
  (`cc_worker.cc:610`, needs `phy_level=info` in `enb_baseline.conf` to see).
- Failing subframes showed real received power (rxpwr ~100-800, not
  DTX/near-zero) and the turbo decoder hit max iterations (10) without
  converging — consistent with genuinely missing/corrupted data arriving at
  the decoder, which is exactly what an overwhelmed, sample-dropping
  decimator produces.

**Ruled out during the investigation** (real findings, just not the dominant
cause):
1. TBS/MCS/PRB mismatch between TX and RX — directly compared, they agreed.
2. Softbuffer not reset between independent PMCH subframes —
   `MbsfnFrameProcessor.cpp:237-249`: confirmed the reset condition fires
   correctly every subframe when TI is inactive.
3. MBSFN subcarrier-spacing switch/restore costing time via FFTW replanning
   — directly measured, never triggered in this config.
4. `cyclic_shift_alpha` mishandling — confirmed correctly gated, not the cause.
5. Several TX/RX OFDM-chain hypotheses tested via bit-exact live capture this
   session (TX grid population, TX IFFT modulation, decimator math
   correctness at the algorithm level, RX FFT/extraction fidelity) — all
   independently verified correct by reimplementing each stage from scratch
   in Python and diffing against real captured samples (6+ nines
   correlation in every case). None of these were the bug.

### Problem 2 (RESOLVED, was: periodic full "Synchronization lost while processing")

Happened roughly every ~2000-2300 TTIs (~2-2.3s, though the exact period
varied run to run — a signature of a load-dependent effect, not a fixed
algorithmic constant, which in hindsight was already a clue pointing at the
real cause). Traced to `srsran_ue_sync_run_track_pss_mode()` internally
declaring `"3 frames lost. Going back to FIND"` after 3 consecutive PSS
correlation misses. Directly measured (via a `SYNC_OFFSET_DIAG` diagnostic
added this session): healthy tracking shows PSS peak values of 4.6-9.7;
every miss showed peak values of 1.0-1.1 — not a marginal threshold effect,
an ~8x collapse consistent with genuinely absent/stale data, not reduced
signal quality.

Already tried and kept as real (if insufficient alone) improvements during
the investigation, before the actual root cause was found:
- `SdrReader::get_samples()`'s buffer-fill-reactive pacing nudge — removed;
  real bug, real fix, did not eliminate the issue alone.
- A `pthread_setschedparam`/`SCHED_RR` priority elevation on the bridge's
  decimation thread — added; turned out to be irrelevant once the actual
  cause (the thread being compute-bound, not scheduling-starved) was found,
  but harmless to leave.
- Reduced the ring-buffer high-watermark refill target from 50% capacity
  down to just above the low-water mark — real, measurable ~25% reduction
  in sync-loss frequency on its own, kept regardless of the larger fix.

### Open follow-up, not blocking: CINR ceiling at n_prb=25

After the fix, `CasFrameProcessor`'s reported CINR sits at ~30-46dB at
n_prb=25, versus ~131-137dB at n_prb=50 (confirmed via a same-rig, clean A/B
comparison — genuinely decimation-specific, not a rig-wide characteristic).
Ruled out: decimation filter stopband rejection (Hamming vs Blackman-Harris
made zero difference); the noise-estimation algorithm choice (already
`SRSRAN_NOISE_ALG_EMPTY` on both TX and RX, and its measurement points sit
in the center of the band, not near where a filter's transition band would
bite). Not cleanly tested: passband ripple / filter length (an attempt at a
4x-longer filter reintroduced the original CPU-saturation failure before it
could produce a valid measurement — see the tap-count warning above) and
float32 precision in the FIR accumulation (not investigated at all). Does
not affect functional correctness — BLER is 0.0 either way, and 30-46dB is
far more margin than QPSK needs — but worth a careful (small, incremental
tap-count) re-test sometime.

**Not yet tried:** re-testing whether a modest (not 4x) tap-count increase,
kept within the CPU headroom demonstrated at 31 taps (~35-45%), meaningfully
improves CINR — would need `pidstat -t` verification at each step to avoid
repeating the compute-cost regression.
