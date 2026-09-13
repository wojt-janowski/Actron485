# Que thermal control implementation plan

Goal: wire Heat, Cool and controller-side Auto through the existing bridge,
keeping takeover disabled until the user disconnects the original controller.

1. Capture-backed regression: add a host test that requests Heat and Cool
   using fresh zone readings and verifies nonzero demand, target satisfaction,
   immediate OFF, and no output in passive mode. Demonstrate current failure.
2. Add a pure QueThermostat state machine. Inputs are selected mode, per-zone
   targets/readings/freshness, enabled zones, and monotonic time. Outputs are
   selected Heat/Cool branch, bounded demand, and an inhibition reason.
   Test hysteresis, Auto band, stale/invalid sensors, min-off/changeover delays,
   startup, time wrap and opposing zones. Demand conversion is experimental,
   explicitly separate from protocol facts.
3. Integrate with the controller main loop. Track sensor timestamps separately
   from passive bus temperatures. Reject state echoes while responding; preserve
   requested Auto/fan settings. Render generated demand and selected targets.
4. Add coherent Auto low/high setters, ESPHome climate traits and REST fields;
   reject invalid/inverted ranges. Persist limits separately from old blobs.
   Queue HTTP mutations to the main loop. Expose demand/branch/inhibit reason.
5. Re-run host regression suite, build pinned bridge, inspect generated config.
   Deploy passive, verify version and bus, push sources, publish the exact
   firmware release and manifests, refresh update metadata. Physical test is
   deferred until the user disconnects the original controller.
