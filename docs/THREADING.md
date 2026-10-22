# Performance and threading plan

## Implemented execution model

1. The window thread owns SFML graphics and events. Display defaults to 30 Hz; `--fps=60` is available. Pipe triangles, grid, legend and waveform are drawn in batches. Text and rectangle objects are reused. Pipe vertex positions are built once and only colors change per frame.
2. A dedicated simulation thread runs the fixed 44.1 kHz clock and acoustic DSP in sample order. It uses a long-running task rather than occupying a thread-pool worker. It consumes input commands and publishes an immutable snapshot only when the window requests one. No renderer accesses changing solver arrays.
3. SFML's audio thread consumes the single-producer/single-consumer ring. It interpolates playback, removes DC, smooths speed changes, applies output gain and saturation, and fills pinned PCM memory. It never waits for the simulation.

The 16,384-frame queue is prefilled to 8,192 frames. The producer waits at that target, in 1 ms cancellable waits, and computes 64 frames per batch. At 1x this gives roughly 186 ms of queued audio; slower playback increases input-to-sound latency. This is a deliberate scheduling cushion for weak CPUs. Queue size and underrun count are visible. A faster producer does not make the simulation run ahead without limit.

## Numerical work already optimized

- SIMD Rusanov fluxes, CFL wave-speed scanning, and fused updates with stability protection.
- On common 16-cell pipes, a final overlapping flux vector replaces a seven-face scalar tail. This is safe because all fluxes read the old state before any cell update. Cell updates do not overlap.
- Damping coefficients computed once per substep and minimum cell length cached for unchanged topology.
- Each volume's boundary thermodynamic state cached once per global step; its outgoing-mass budget accumulated directly. Those states were already constant throughout the substeps.
- Compressible orifice shape stored in a 1025-entry table parameterized by sqrt(pressure difference / upstream pressure). Choked flow is analytic. The table is checked against the analytic formula over the unchoked/choked domain; equal pressure stays zero.
- Ambient characteristic density uses the integer fifth power for gamma=1.4.
- Darcy wall-friction update runs in the SIMD path too. It preserves total energy while dissipating resolved momentum.

`--benchmark` measures the scalar and vector pipe loops after warmup, then measures the complete B205 simulation and acoustic path. `--headless --profile` reports boundary, pipe and component time; headless mode also validates states, which adds work absent from interactive playback. Compare Release builds, on the same machine, without competing runs. Allocation reporting excludes immutable render snapshots.

## Next threading stage: optional DSP worker

First profile a sustained high-RPM run and record wall time, allocations, queue fill and audio underruns. If DSP consumes enough of the producer budget to justify another active core, split `Scenario.Process` into physics advancement and acoustic mixing. Physics would emit blocks of 128–256 excitation records containing intake/exhaust mass flow, crank angle, RPM and combustion excitation. A bounded SPSC queue would connect physics to one DSP worker. That worker alone would own radiation filters, delays and reverb state and would produce the existing stereo queue.

This transfers work in blocks, adding about 3–6 ms of block latency. Backpressure must propagate from stereo output through the excitation queue to physics. Playback rate must continue to affect consumption, not the fixed numerical timestep. Shutdown cancels both workers and joins them before disposing audio resources. Keep the current combined worker as a fallback on CPUs where two workers worsen throughput or scheduling jitter. Benchmark and verify identical sample ordering before enabling this automatically.

## Larger networks: partition pipes with persistent workers

For substantially larger networks, benchmark two persistent workers at 512, 2048 and 8192 cells; use the results to choose a threshold instead of assuming 212 cells will benefit. A substep has these dependencies:

1. The coordinator freezes component states and resolves boundaries in deterministic order. Each volume's mass budget and port accumulators remain owned by this coordinator.
2. Workers advance disjoint sets of whole pipes with SIMD. A pipe belongs to exactly one worker; its state and flux-array slices have one writer. Each worker maintains its own numerical-repair count.
3. A barrier completes all pipe updates before the next boundary resolution. Reduce repair counters after the barrier. Component integration remains once per global step on the coordinator.

Do not let independent cylinder workers advance complete timesteps: cylinders share plenums, collectors and a crankshaft. Do not run `Parallel.For` or create tasks for every pipe at 44.1 kHz. At two substeps per frame there would be at least 88,200 job submissions and synchronization points per second; small networks can spend more time scheduling than calculating. Persistent workers still need a substep barrier, so they should be enabled only when full-pipeline measurements show a worthwhile gain.

If boundary resolution eventually becomes the limiting phase, parallelize it by component ownership, not by arbitrary pipe endpoints. Each worker would write private endpoint fluxes and private integrated port transfers. A deterministic reduction would enforce shared donor limits and apply transfers before any pipe advances. This requires additional conservation and reproducibility checks and is a later stage.

## Acceptance checks for further concurrency

- Scalar/SIMD agreement, segment boundaries, positivity, mass and energy accounting remain within stated tolerances.
- Firing order, valve events and sample sequence are independent of worker scheduling.
- No render or audio callback waits for a barrier or acquires a solver lock.
- Queue wrap, starvation recovery and shutdown remain correct under CPU contention.
- Measure total throughput and underruns, rather than reporting only kernel speedup. Retain one simulation worker when extra workers do not improve the measured workload.
