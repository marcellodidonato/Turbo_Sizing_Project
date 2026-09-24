# Turbo_Sizing_Project
MATLAB turbocharger sizing tool: compressor/turbine mean-line design, joint geometric feasibility search across 100k+ candidate wheel pairings, mass/inertia derivation from first-principles geometry, and closed-form spool-time modeling to select the fastest-spooling turbo meeting a target power curve.
# Turbocharger Sizing & Spool-Time Optimization

A MATLAB model that sizes a turbocharger from engine parameters and a
target power curve, then selects the compressor/turbine combination
with the fastest spool response among all combinations that genuinely
meet that power curve across the full engine speed range.

## What it does

Given engine displacement, cylinder count, and a desired power curve
across an RPM sweep, the model:

1. **Derives compressor requirements** (mass flow, pressure ratio, boost)
   at every RPM point from first principles — mass conservation and an
   implicit isentropic-efficiency solve, not a fitted curve.
2. **Searches compressor geometry** (inducer/exducer diameter) for every
   combination that meets those requirements at *every* RPM point
   simultaneously, not just at a single design point.
3. **Matches the turbine to each feasible compressor candidate**
   independently (shared shaft speed), then jointly searches turbine
   D1, D2, and inlet flow angle for aerodynamic feasibility (inducer and
   exducer relative Mach number) across the same full RPM range.
4. **Builds a meridional silhouette** for each surviving wheel and
   derives volume, mass, and polar moment of inertia directly from that
   geometry — hub modeled as a bored annulus (accounting for the shaft
   bore), blades modeled as discrete radial plates from blade count and
   thickness (not an assumed fill fraction).
5. **Sizes the shaft** as a separate component (different material,
   diameter set by the smaller wheel bore) and combines all three
   components into total rotating assembly inertia.
6. **Solves spool time** for a throttle-stab-from-idle scenario at
   multiple engine speeds, using a torque model derived from turbine
   efficiency's known dependence on velocity ratio (literature-sourced
   correlation), rather than assuming flat torque through the transient.
7. **Selects the fastest-spooling design** — and explicitly checks
   whether minimum total inertia actually predicts it (it doesn't
   always: torque availability matters too).

## Key result

Across ~22,000 feasible turbine/compressor pairings for a 400 hp,
2.0 L target, the design with minimum total rotating inertia was
**not** the design with the fastest spool time — confirming that
inertia alone is an insufficient proxy for turbo lag, and that torque
availability during the transient has to be modeled explicitly.

## Assumptions worth knowing

- Mean-line, 1D aerodynamic design — not CFD-validated.
- No-intercooler closure for the compressor PR solve (a stated
  conservative upper bound, not an assumed real intercooler).
- Turbine off-design efficiency uses a literature parabolic
  correlation (peak near velocity ratio ≈ 0.707), capped below a
  velocity ratio of 0.5 where the correlation is no longer validated.
- Blade counts, shaft core length, and shaft-to-bore ratio are stated
  engineering assumptions, chosen to match typical automotive
  turbocharger hardware, not derived from the aerodynamics.

## Requirements

MATLAB (no toolboxes beyond base — no Optimization Toolbox, no ODE
solvers required; the spool-time integration is either closed-form or
a simple fixed-step vectorized scheme).

## Usage

Run `Engine_Study.m` end to end. Edit Section 1.1 to change engine
parameters, displacement, or the target power curve.
