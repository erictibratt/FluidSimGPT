# FluidSim

A desktop engine-flow laboratory built from `Idea.txt`. It couples a 1D compressible Euler solver to a roughly 600 cc parallel twin, renders pressure and temperature in SFML, and synthesizes stereo sound from intake/exhaust mass flow. This is a new implementation of the description; the original source, appearance, and exact tuning values were not available.

## Run

Install the **.NET 10 SDK** and run from this folder:

```powershell
dotnet run -c Release
```

NuGet restores SFML.Net 3.0.0 and the CSFML native runtime. A graphics-capable desktop and audio output device are needed. The bundled Inter font is under the SIL Open Font License in `Assets/OFL.txt`.

The default is the historical `SingleCylScenario` name, containing two four-stroke cylinders on a shared crankshaft, phased 180 degrees apart. It starts in neutral at normal (1x) playback speed, so the engine is heard at its intended pitch. Space switches to quarter speed and back. Automatic low-RPM starter assistance keeps the engine from getting stuck at rest.

| Input | Action |
| --- | --- |
| Hold W | Apply the throttle target |
| Up / Down | Adjust target throttle |
| Right / Left | Increase / decrease brake load |
| Hold C | Disengage clutch |
| Q / E | Shift down / up; gear 0 is neutral |
| Mouse wheel | Playback speed, from 0.05x to 2x |
| Space | Toggle real time and previous selected speed |
| T | Switch pressure / temperature coloring |
| Escape | Exit |

Release builds are recommended for real-time operation. Playback interpolation changes both perceived pitch and simulation speed; it is not pitch-preserving time stretching. At 2x, the producer needs to compute two simulated seconds per wall-clock second. Underruns are counted and produce silence rather than blocking the audio callback.

## Other scenarios and offline audio

```powershell
dotnet run -c Release -- --scenario=test
dotnet run -c Release -- --scenario=helmholtz
dotnet run -c Release -- --scenario=two-stroke
dotnet run -c Release -- --headless --seconds=15 --wav=engine-preview.wav
```

The shock tube has reflecting ends and an initial pressure discontinuity. The Helmholtz case releases an initially overpressurized cavity through a neck. The two-stroke alternative uses piston-controlled intake and exhaust windows with one combustion event per revolution; it is a simplified ported engine, without a detailed crankcase/scavenging model. Headless mode applies 65% throttle after one second and optionally exports stereo 16-bit PCM at 44.1 kHz without opening SFML.

## Implementation

- `Core/PipeSystem.cs`: flat conserved-state arrays, SIMD Rusanov interior face fluxes, SIMD CFL wave-speed scans, and fused SIMD cell updates, positivity checks, damping, and ambient thermal relaxation. Uses `System.Numerics.Vector<float>` with runtime-selected vector width and a scalar fallback. Each face batch stays inside its pipe; short segments and remainder cells use scalar code. Damping coefficients are computed once per substep. Seven ten-cell pipes in the default scenario. Area affects boundary coupling; there is no area-gradient source term.
- `Core/BoundarySystem.cs`: reflecting walls, choked/unchoked isentropic orifices, optional inertance, donor limits, and ambient characteristic boundaries. Positive port flow enters the component. Component connections use a shared conservative numerical face flux rather than independently solving a ghost-state Rusanov flux, so pipe and reservoir mass/enthalpy transfers agree exactly before floating-point rounding.
- `Core/Solver.cs`: CFL <= 0.8, at least two substeps per global timestep, integrated boundary transfers, and one reservoir update per global step.
- `Components/`: ideal-gas reservoirs, valve lift, crank-slider cylinders, adiabatic compression/expansion, wall heat transfer, trapped-air fueling, Wiebe combustion, rev limiting, crankshaft and a simplified clutch/vehicle model.
- `Core/Acoustics.cs`: differentiated and smoothed radiation flow, soft saturation, distance attenuation, binaural delay/head shadow, mechanical excitation, early reflections, and an eight-line orthogonal feedback reverb with three-band decay.
- `Audio/`: bounded single-producer/single-consumer stereo ring buffer, interpolated playback, 10 Hz DC blocking, 512-frame PCM blocks. The application reuses a permanently pinned PCM array so garbage collection cannot move memory exposed to native SFML. This matters because the [SFML.Net callback](https://github.com/SFML/SFML.Net/blob/3.0.0/src/SFML.Audio/SoundStream.cs) passes the array address beyond a temporary `fixed` scope. The callback also smooths speed changes and fades out/re-enters on starvation to avoid hard-zero clicks. The binding itself may allocate internally.
- `App.cs`: window-thread events/drawing, queued input, immutable render snapshots, producer backpressure, cancellation and disposal. `Dashboard.cs` renders the network and telemetry. Audio history is sampled at render rate and is an activity indicator, not a sample-accurate oscilloscope.

The global timestep is fixed at 1/44,100 seconds. Each scenario processing call produces one stereo frame. Drawing never reads changing thermodynamic arrays directly. Startup warms the solver/JIT for 0.1 simulated seconds, then prefills 8,192 frames before starting native playback. The producer targets 8,192 queued frames to absorb scheduling jitter; this trades some input latency for uninterrupted audio.

## Tuning and limitations

Scenario topology, pipe lengths/areas, reservoir sizes, throttle leakage, and source wiring are in `Scenarios/Scenario.cs`. Cylinder geometry, timing, heat transfer, combustion, rev limit, starter, friction and drivetrain parameters are in `Components/Engine.cs` and `Scenarios/Scenario.cs`. Mixer/source/reverb parameters are in `Core/Acoustics.cs`. Defaults are inferred from the specification and are not calibrated to a measured engine. The model omits detailed chemistry, fuel evaporation, variable specific heats, full viscous pipe losses and a physical muffler geometry. Numerical floors are counted as repairs and nonfinite states fail visibly.

## Validation

```powershell
dotnet build -c Release
dotnet run -c Release --no-build -- --self-test
dotnet run -c Release --no-build -- --smoke
dotnet run -c Release --no-build -- --benchmark
```

The self-test runner checks SIMD/scalar agreement across segment lengths 1, 3, 7, 8, 10, 17 and 31, uniform Euler flux and pipe preservation, wall reflection, orifice reversal/choking/closure, closed pipe/reservoir mass and energy conservation, cycle wrap and phase, compression geometry, ring-buffer wrap/interpolation/underruns, PCM pointer stability under compacting GC, callback continuity and starvation recovery, engine stability under throttle/shift/clutch/brake inputs, and all alternative scenarios. Failures return a nonzero exit code. The smoke mode runs the actual window/audio pipeline for five seconds with scripted playback/control changes, saves `smoke.png`, prints runtime counters, and exits.

`--benchmark` compares scalar and SIMD pipe loops on identical seven-segment, 70-cell networks after warmup, alternating test order. On the development machine (eight float lanes), 90,000 substeps took 0.375 seconds scalar versus 0.061 seconds SIMD, a 6.17x improvement in that loop. Full 10-second headless engine runs, including validation and acoustic processing, took 4.34 seconds scalar and 2.06 seconds SIMD. These are local measurements, not a guarantee on other CPUs. Headless `--scalar` disables SIMD for comparison; `--flow-only` disables mechanical audio and reverb for isolating the physical radiation signals.

Validation on Windows with .NET 10: Release build succeeded without warnings; all ten test groups passed. A 15-second simulated run completed in 5.04 seconds with finite positive states, peak left audio amplitude 0.276, RMS 0.038, and zero positivity repairs. After removing hot-loop LINQ allocations and caching boundary fluxes, the actual SFML graphics/audio smoke run completed its 0.25x/1x/2x/0.5x speed changes with zero underruns and zero repairs. Startup/shutdown passed, and the screenshot was inspected. Speaker output and subjective sound quality have not been verified by listening. This is numerical smoke coverage, not experimental validation of engine performance or acoustics.
