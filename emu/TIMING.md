# Timing mode: measuring like the watch

Plain TCG runs the firmware as fast as the host can, so time measured inside the guest says
little about the watch: integer code runs several times faster than a 208 MHz Cortex-M33,
floating point and memory-mapped I/O far slower, and the GPU model rasterises in host time
inside the vCPU thread. Timing mode makes guest time follow a model of the hardware instead.

## Turning it on

Per device, through Mirage's `extraArguments` (device.json, device stopped):

    "-icount", "ips=130000000,align=on,sleep=on,exact-io=off",
    "-global", "driver=vsi.vglite,property=timing,value=on"

* `-icount ips=N` (added here): every guest instruction advances virtual time by 1/N s. Stock
  QEMU only offers powers of two (`shift`: 4 ns, 8 ns ...). `align=on` keeps the guest from
  running ahead of real time when the host is faster; when the host is slower, guest time
  simply runs slower than the wall clock (screens, timeouts and recordings all stretch) while
  every measurement inside the guest stays in modelled time.
* `exact-io=off` (added here): memory-mapped I/O in the middle of a translation block happens
  right away and sees the virtual clock as of the end of the block (a few dozen instructions
  ahead at most), instead of QEMU re-translating the block so that the access ends it. The
  firmware polls timer and status registers in tight loops; with exact I/O the guest ran at
  about a tenth of real time, without it at about half (BAL2D rendering a lobby).
* `vsi.vglite.timing=on`: a command buffer completes (END interrupt, idle bits) after the time
  a GC355 would take for it, on the virtual clock, instead of at once.

Without `-icount`, `timing=on` would add the modelled GPU time on top of the host's own
rendering time; use the two together.

## The model and where its numbers come from

CPU (`ips`). The Band 10 Pro's application core is a Cortex-M33 (`bes_board.c`: best1503)
whose fastest system clock is 208 MHz (SDK `HAL_CMU_FREQ_T`, see `best1503.c`). 130 MIPS
assumes 1.6 cycles per instruction: about 1.2 for the core itself (single-cycle ALU and FPU
operations, two-cycle loads and taken branches) plus misses of the PSRAM caches the code and
heap live behind. Every instruction costs the same; divisions (VDIV 14 cycles, SDIV up to 12)
and cache behaviour are averaged in. This is the figure to calibrate (below).

GPU (`hw/misc/vsi_vglite.c`, `vgl_cost`): per draw operation
`setup = op-cycles + window / 32 + edges * 8` GPU cycles (the tessellator covers the path's
whole window, plus work per edge), then the covered pixels at one per clock (two with the 3x3
Gaussian filter) or as fast as memory allows, whichever is slower: per pixel the target is
written (and read first when blending), and about one texel is fetched (the texture cache
serves the rest of a bilinear footprint). A submit adds fetching its command buffer and 2 us
for the interrupt round trip. Properties: `gpu-mhz` (208: assumed to run at the system
clock), `mem-mbps` (400: an assumed share of the PSRAM bandwidth for the GPU, after the CPU and
the display controller; not measured), `op-cycles` (150).

Display (`hw/misc/bes_lcdc.c`, unchanged): the panel's TE pulse at `refresh-hz` (60) paces
frame starts and a full-frame QSPI transfer takes `frame-us` (8000). The firmware always sends
the full 336 x 480 frame.

## Calibrating against a watch

All figures above are estimates from specifications, not measurements. With one real watch the
two that matter can be fitted:

1. Run the same workload on the watch and in timing mode (BAL2D shows, per frame, the
   animation, drawing, driver and GPU-wait milliseconds, and the screen refresh).
2. CPU-only phases (BAL2D's animation time) scale with `ips`: new `ips` = old `ips` x
   (Mirage ms / watch ms).
3. With `ips` fitted, the GPU wait at the end of a frame scales with `mem-mbps` (fill-bound
   frames) or `gpu-mhz` (many small draws).

## Profiling in guest time

`MIRAGE_PROFILE=FILE` (environment of the QEMU process; `hw/arm/mirage_profile.c`) samples the
CPU's PC and LR every `MIRAGE_PROFILE_US` (50) microseconds of virtual time and rewrites FILE
every 20000 samples (cumulative "pc lr count" lines). In timing mode the samples are exact (the
timer fires between translation blocks at the instruction where the time is due), so the
difference of two dumps is where a real CPU of the modelled speed spends its time, idle and
firmware included. BAL2D's `scripts/guestprof.py` symbolises such dumps.

What it found on BAL2D, as examples of what plain TCG hides: libm's `sinf`/`cosf` compute in
double precision, which the M33's single-precision FPU runs as software calls (a fifth of all CPU
time on a heavy lobby, invisible on the host); and the Canopus build's `opt-level = "z"` left
slice indexing out of line in the skinning loop.
