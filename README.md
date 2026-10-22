# FluidSim

A desktop engine-flow laboratory built from `Idea.txt`. It couples a 1D compressible Euler solver to a Saab B205 with a custom naturally aspirated intake and 4-2-1 exhaust, renders pressure and temperature in SFML, and synthesizes stereo sound from intake/exhaust mass flow. The original 600 cc twin and other experimental scenarios remain available.

## Run

Install the **.NET 10 SDK** and run from this folder:

```powershell
dotnet run -c Release
```

NuGet restores SFML.Net 3.0.0 and the CSFML native runtime. A graphics-capable desktop and audio output device are needed. The bundled Inter font is under the SIL Open Font License in `Assets/OFL.txt`.

The default is `SaabB205Scenario`: four cylinders, 1985 cc, stock 8.8:1 compression and B205L/R valve timing, with custom NA plumbing. It starts in neutral at normal (1x) playback speed. Space switches to quarter speed and back. Low-RPM assistance targets 825 rpm. The window defaults to 30 FPS for weak CPUs; use `--fps=60` for a higher display rate. See [B205 specifications, sources and design assumptions](docs/SAAB_B205.md).

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

Release builds are recommended for real-time operation. Playback interpolation changes both perceived pitch and simulation speed. At 2x, the producer needs to compute two simulated seconds per wall-clock second. Underruns are counted and fade toward silence without blocking the audio callback.

## Other scenarios and offline audio

```powershell
dotnet run -c Release -- --scenario=saab
dotnet run -c Release -- --scenario=twin
dotnet run -c Release -- --scenario=test
dotnet run -c Release -- --scenario=helmholtz
dotnet run -c Release -- --scenario=two-stroke
dotnet run -c Release -- --headless --seconds=15 --wav=engine-preview.wav
dotnet run -c Release -- --headless --seconds=10 --throttle=0 --profile
```

The shock tube has reflecting ends and an initial pressure discontinuity. The Helmholtz case releases an initially overpressurized cavity through a neck. The two-stroke alternative uses piston-controlled intake and exhaust windows with one combustion event per revolution; it is a simplified ported engine, without a detailed crankcase/scavenging model. Headless mode applies 65% throttle after one second and optionally exports stereo 16-bit PCM at 44.1 kHz without opening SFML.

## Implementation

- `Core/PipeSystem.cs`: flat conserved-state arrays, SIMD Rusanov interior face fluxes, SIMD CFL wave-speed scans, and fused SIMD cell updates, positivity checks, damping, optional Darcy friction, and ambient thermal relaxation. Uses `System.Numerics.Vector<float>` with runtime-selected vector width and a scalar fallback. Each face batch stays inside its pipe. Fifteen ducts with 212 cells in the B205 scenario. Area affects boundary coupling; there is no area-gradient source term.
- `Core/BoundarySystem.cs`: reflecting walls, choked/unchoked isentropic orifices, optional inertance, donor limits, and ambient characteristic boundaries. Positive port flow enters the component. Component connections use a shared conservative numerical face flux rather than independently solving a ghost-state Rusanov flux, so pipe and reservoir mass/enthalpy transfers agree exactly before floating-point rounding.
- `Core/Solver.cs`: CFL <= 0.8, at least two substeps per global timestep, integrated boundary transfers, and one reservoir update per global step.
- `Components/`: ideal-gas reservoirs, valve lift, crank-slider cylinders, adiabatic compression/expansion, wall heat transfer, trapped-air fueling, Wiebe combustion, rev limiting, crankshaft and a simplified clutch/vehicle model.
- `Core/Acoustics.cs`: differentiated and smoothed radiation flow, soft saturation, distance attenuation, binaural delay/head shadow, mechanical excitation, early reflections, and an eight-line orthogonal feedback reverb with three-band decay.
- `Audio/`: bounded single-producer/single-consumer stereo ring buffer, interpolated playback, 10 Hz DC blocking, 512-frame PCM blocks. The application reuses a permanently pinned PCM array so garbage collection cannot move memory exposed to native SFML. This matters because the [SFML.Net callback](https://github.com/SFML/SFML.Net/blob/3.0.0/src/SFML.Audio/SoundStream.cs) passes the array address beyond a temporary `fixed` scope. The callback also smooths speed changes and fades out/re-enters on starvation to avoid hard-zero clicks. The binding itself may allocate internally.
- `App.cs`: window-thread events/drawing, queued input, a dedicated simulation thread, immutable render snapshots requested by the display, producer backpressure, cancellation and disposal. `Dashboard.cs` batches the network, grid, legend and waveform. Audio history is sampled at render rate and is an activity indicator, not a sample-accurate oscilloscope. See the [performance and threading plan](docs/THREADING.md) for ownership, synchronization and proposed next stages.

The global timestep is fixed at 1/44,100 seconds. Each scenario processing call produces one stereo frame. Drawing never reads changing thermodynamic arrays directly. Startup warms the solver/JIT for 0.1 simulated seconds, then prefills 8,192 frames before starting native playback. The producer targets 8,192 queued frames to absorb scheduling jitter; this trades some input latency for uninterrupted audio.

## Tuning and limitations

The B205 parameters and topology are in `Scenarios/SaabB205Scenario.cs`; older scenarios are in `Scenarios/Scenario.cs`. Common cylinder geometry, valve timing, curtain area, combustion and drivetrain behavior are in `Components/Engine.cs`. Mixer/source/reverb parameters are in `Core/Acoustics.cs`. B205 geometry and cam-event values have workshop sources; manifold dimensions, loss coefficients and combustion settings are design assumptions. No measured NA engine calibration is claimed. The model omits detailed chemistry, fuel evaporation, variable specific heats, full viscous losses and silencer internals. Numerical floors are counted as repairs and nonfinite states fail visibly.

## Validation

```powershell
dotnet build -c Release
dotnet run -c Release --no-build -- --self-test
dotnet run -c Release --no-build -- --smoke
dotnet run -c Release --no-build -- --benchmark
```

The twelve self-test groups cover B205 geometry, cam events, firing order and 4-2-1 pairing; orifice lookup against the analytic equation; SIMD/scalar agreement including segment edges, friction, and overlapping flux vectors; uniform flow, wall reflection, mass and energy conservation; cycle timing; ring-buffer behavior and PCM lifetime; starvation recovery; and engine stability with throttle/drivetrain inputs. Failures return a nonzero exit code. Smoke mode runs the actual window/audio pipeline for five seconds with scripted playback/control changes, saves `smoke.png`, prints runtime counters, and exits.

`--benchmark` compares scalar and SIMD on both a seven-segment pipe kernel and the complete 212-cell B205 simulation plus DSP. The latest local B205 result was 5.022 seconds scalar versus 1.612 seconds SIMD for three simulated seconds, a 3.12x improvement. The warmed simulation/DSP loop allocated zero bytes across 44,100 frames, excluding render snapshots. Headless `--scalar` disables SIMD for comparison; `--flow-only` disables mechanical audio and reverb for isolating physical radiation signals. These are local measurements, not a guarantee on other CPUs.

Validation on Windows with .NET 10: Release build succeeded without warnings; all twelve test groups passed. A ten-second B205 throttle run completed in 5.13 seconds including state validation and profiling, with zero positivity repairs and bounded stereo output. A closed-throttle run settled near 825 rpm. The SFML smoke run completed its 0.25x/1x/2x/0.5x speed changes with zero underruns and zero repairs. Startup/shutdown passed, and the four-cylinder screenshot was inspected. Speaker output and subjective sound quality have not been verified by listening. This is numerical smoke coverage, not experimental validation of engine performance or acoustics.
