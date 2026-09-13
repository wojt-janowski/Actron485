# Que replacement thermostat design

Status: implemented experimental design; not a claim that the proprietary control law
has been recovered. This adds a new control loop to the existing responder.

## Evidence and uncertainties

The original controller selects Heat (reg 2 high byte 01) or Cool (02), with
low-byte values changing from 00 in standby to 14/5E/64 during heating and
43/48 during cooling. LCD Auto selects these thermal branches in software.
The low byte is consistent with demand, but neither its precise meaning nor
its control law has been proven. Fixed maximum-demand replay is not a
replacement thermostat. Thermal fan Auto also selects output speeds in the
original controller; its complete fan algorithm is unknown.

## Intended behavior

- Keep the user's requested mode, fan setting, targets, and zone enables
  separate from the selected thermal branch and generated wire commands.
  Downstream slave-11 echoes must not overwrite command intent.
- Heat calls only below its target; Cool calls only above its target. Auto
  calls Heat below the lower comfort target, Cool above the upper target,
  and neither inside the band. The UI exposes explicit lower/upper number
  controls, preserving the native single-target climate control. Zone targets
  are independent centers sharing the master's band width.
- Only enabled zones with fresh, finite temperature readings contribute.
  Missing/stale readings cannot sustain thermal demand. Do not promote
  echoed temperatures, boot defaults, or an off-room average to fresh sensor
  input. Expose why demand is inhibited.
- Start with explicit, bounded experimental demand conversion; isolate this
  policy from protocol encoding so captures and physical results can refine
  it. Document coefficients and limits as implementation choices, not
  recovered Actron facts.
- Rate-limit demand changes and impose minimum off and Heat/Cool changeover
  delays. OFF, sensor loss, and disabling the responder remove demand
  immediately. Do not claim these replace the equipment's own protections.
- Fan-only remains independent of thermal temperature demand and uses the
  capture-tested speed encodings. Thermal Auto fan output must not change
  the displayed user preference.
- Compute on the main loop, not in HTTP callbacks. Commands and sensor input
  cross from API callbacks through ESPHome deferred callbacks; bus
  parsing, control decisions, and response rendering share one owner.
- Keep the responder disabled during deployment and leave original wall
  controller operation intact. Physical testing will start with zone 7.

## Integration

A host-testable thermal state machine in the Actron485 library handles
branch selection, freshness, hysteresis, and timers. The controller adapter
supplies inputs and renders reg 2, reg 3, zone targets/offsets, and diagnostics.
ESPHome climate traits/control and REST state/setpoint handling surface the
same targets. Persist Auto bounds, never demand outputs or fresh-sensor
claims. Retain compatibility with existing single-target Heat/Cool calls.

## Validation before deployment

Tests must cover Heat/Cool starts and stops; Auto below/in/above band; exact
boundaries; disabled zones; conflicting zone requests; missing, non-finite
and expired readings; startup/restart and direction-change delays; immediate
OFF; monotonic-clock wrap; downstream echo rejection; requested Auto fan
retention; mode changes without temperature updates; REST input validation;
and coherent update of both Auto limits. Re-run the captured fan protocol
suite and compile the complete bridge firmware. Check firmware identity,
passive responder state, and bus reception after OTA. Publish release assets
and manifests only for the exact deployed build.

Hardware validation remains required: a passing simulation cannot prove the
inferred demand field operates the physical system as intended.
