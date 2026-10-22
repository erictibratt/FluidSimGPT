# Saab B205 naturally aspirated simulation

The default scenario is a custom NA conversion of a B205 long block. The reference is the B205L/R data in Saab's 2000/2001 workshop documentation. The selected internals and cam timing stay at those specifications. Intake and exhaust are a new, configurable design. There is no turbocharger, compressor, intercooler, boost controller or turbine restriction in this network.

## Workshop data used

| Parameter | Implemented value | Source |
| --- | --- | --- |
| Cylinders / cycle / valves | Inline four, four stroke, four valves per cylinder | [Saab WIS general data, MY2001](https://saabwisonline.com/9-3-9400/2001/2-engine/basic-engine-4-cylinder-petrol/technical-data/general-data) |
| Bore × stroke | 90 × 78 mm; calculated displacement 1984.91 cc | Same general data; Saab rounds displacement to 1985 cc |
| Compression | 8.8:1 | Same general data |
| Firing order | 1-3-4-2, events 180 crank degrees apart | Same general data |
| Rod centre distance | 159 mm | [Saab WIS connecting rods](https://saabwisonline.com/9-5-9600/2005/2-engine/basic-engine-4-cylinder-petrol/technical-data/connecting-rods) |
| Inlet timing | Opens 12° BTDC; closes 39° ABDC | [Saab WIS camshafts, MY2000 B205L/R](https://saabwisonline.com/9-3-9400/2000/2-engine/basic-engine-4-cylinder-petrol/technical-data/camshafts) |
| Exhaust timing | Opens 34° BBDC; closes 14° ATDC | Same camshaft data |
| Peak lift | Inlet 8.31 mm; exhaust 8.07 mm | Same camshaft data |
| Valve head diameters | Inlet 33 mm; exhaust 29 mm | [Saab WIS valve gear](https://saabwisonline.com/9-3-9400/2001/2-engine/basic-engine-4-cylinder-petrol/technical-data/valve-gear) |
| Warm idle target | 825 rpm | [Saab WIS general data, MY2004](https://saabwisonline.com/9-5-9600/2004/2-engine/basic-engine-4-cylinder-petrol/technical-data/general-data) |

Phase offsets for cylinders 1, 2, 3, 4 are 0°, 180°, 540°, 360°. The solver adds these to the common crank angle. This sign convention produces the specified firing order. Intake TDC is local 0° and combustion TDC is local 360°. The lift function interpolates between published opening/closing events and peak lift; it is not a measured cam-lobe trace. Effective valve area is the curtain area of two valves, capped at their combined disk area, with an assumed 0.72 discharge coefficient and the adjoining duct's area limit.

## Intake design assumptions

The arrangement follows the general NA airbox/resonator/throttle architecture described by [Saab for the B204i](https://saabwisonline.com/9-3-9400/1998/2-engine/intake-system-petrol/technical-description/system-overview-normally-aspirated-engine-b204i). That source supports the arrangement, not the dimensions below. These are selected dimensions for this B205 conversion:

| Element | Length | Internal diameter / volume | Cells |
| --- | --- | --- | --- |
| Ambient snorkel | 450 mm | 65 mm | 8 |
| Airbox / intake silencer | — | 3 L | 0D |
| Filter and throttle feed | 250 mm | 65 mm, 60 mm throttle | 8 |
| Shared plenum | — | 3 L | 0D |
| Four equal runners, including effective head-port length | 400 mm each | 38 mm | 16 each |

Throttle leakage is equivalent to a 3 mm circular opening. The idle assistance is a simple crank-torque controller around 825 rpm, not a reproduction of Trionic 7 idle control. The fully open throttle area follows the 60 mm bore. An upstream port coefficient of 0.85 approximates filter loss. Other non-valve junctions use 0.95. Runner and exhaust lengths are candidates for a road engine with stock cams; no length sweep or measured volumetric-efficiency optimization has established an optimum.

## Exhaust design assumptions

The 4-2-1 header pairs 1 with 4 and 2 with 3. Each first merge then receives events separated by 360°, and the final merge alternates the two secondary branches at 180°. Physical pipe lengths are equal within each group even though schematic line lengths differ.

| Element | Length | Internal diameter / volume | Cells |
| --- | --- | --- | --- |
| Four primaries | 550 mm each | 38 mm | 16 each |
| Pair collectors | — | 0.20 L each | 0D |
| Two secondaries | 420 mm each | 45 mm | 12 each |
| Final merge | — | 0.35 L | 0D |
| Main exhaust | 700 mm | 54 mm | 16 |
| Centre resonator | — | 3 L | 0D |
| Rear link | 650 mm | 54 mm | 16 |
| Rear silencer | — | 6 L | 0D |
| Tailpipe | 450 mm | 54 mm | 12 |

There are 15 pipe segments and 212 cells. Both the snorkel and tailpipe have ambient characteristic boundaries. Silencers are well-mixed expansion reservoirs, so the network models their storage and reflection effects but not perforations, packing absorption, internal baffles or catalytic chemistry. The 54 mm line is a design assumption, not a claim about Saab's original exhaust.

All ducts use an assumed Darcy friction factor of 0.025. Momentum decays with the local velocity according to f*u*abs(u)/(2*D); total cell energy is retained, so the removed kinetic energy becomes internal energy before the existing ambient thermal relaxation. The model still uses constant-area 1D segments and lumped junctions, without a quasi-1D area-gradient source.

## Other assumed settings

Ignition advance is fixed at 24° before combustion TDC; the Wiebe burn spans 65°. The rev cut is 6500 rpm, with the existing hysteresis. Crank/flywheel inertia is 0.24 kg m². Friction, heat transfer, discharge coefficients and fueling are simplified. Vehicle mass is 1400 kg with a generic five-speed drivetrain and a 240 Nm clutch capacity; gear ratios are not asserted to match a particular Saab gearbox.

The stock 8.8:1 compression ratio is intentionally preserved for this conversion. A purpose-built NA engine could use different pistons and cams, but that would be a different specification. Factory turbocharged power and torque figures are not calibration targets for this NA simulation. No measured NA B205 dyno or sound data has been used, so calculated engine behavior and acoustic output remain experimental.

## Run and inspect

```powershell
dotnet run -c Release
dotnet run -c Release -- --scenario=saab --headless --seconds=10 --profile
dotnet run -c Release -- --scenario=saab --headless --seconds=10 --throttle=0
dotnet run -c Release -- --scenario=saab --headless --seconds=10 --wav=saab-b205-na.wav
dotnet run -c Release -- --scenario=twin
```

Configuration is in `Scenarios/SaabB205Scenario.cs`; common engine behavior is in `Components/Engine.cs`. The screenshot schematic is a connectivity diagram, not a scale drawing. This simulation is intended for experimenting with the network and its sound, not dimensioning a real exhaust or making mechanical modification decisions.
