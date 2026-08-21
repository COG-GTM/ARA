# Skydio X10D Mission Executor — TraverseTo Test Plan

This plan verifies that the `skydio_me_node` Mission Executor correctly translates the
MPMS `TraverseTo` behavior into the Skydio X10D's native RAS-A/MAVLink protocol
(per the "X10D Control and Telemetry ICD") and executes a waypoint traversal.

Behavior-to-protocol mapping under test:

| MPMS signal | Native protocol messages |
|---|---|
| `start` | `MISSION_COUNT` → `MISSION_REQUEST(_INT)` / `MISSION_ITEM_INT` → `MISSION_ACK`; `COMMAND_LONG(MAV_CMD_COMPONENT_ARM_DISARM)`; `COMMAND_LONG(MAV_CMD_MISSION_START)` |
| `update` | hold via `MAV_CMD_DO_PAUSE_CONTINUE(0)`, then re-upload + restart with the new waypoint list |
| `pause` | `COMMAND_LONG(MAV_CMD_DO_PAUSE_CONTINUE, param1=0)` |
| `resume` | `COMMAND_LONG(MAV_CMD_DO_PAUSE_CONTINUE, param1=1)` |
| `stop` | `MAV_CMD_DO_PAUSE_CONTINUE(0)` + `MISSION_CLEAR_ALL` |
| completion | `MISSION_ITEM_REACHED` (final seq) → `WaypointListComplete` output signal fired from the mission worker thread |
| telemetry | `HEARTBEAT`, `GLOBAL_POSITION_INT` → asset params `position`, `altitude`, `heading`, `speed` |

## Stage 1 — Unit tests (no vehicle) — AUTOMATED

Implemented in `tests/` (GoogleTest, no MPMS SDK or vehicle required; the small SDK
surface used by the behavior code is stubbed under `tests/stubs/`). Run with:

```sh
cmake -S tests -B build-tests
cmake --build build-tests -j
ctest --test-dir build-tests --output-on-failure
```

Framing and encoding, tested with a loopback UDP socket (`tests/FakeVehicle.h`
decodes every frame with an independent table-driven CRC-16/MCRF4XX
implementation and the standard `CRC_EXTRA` seeds):

1. **MAVLink v2 framing** — capture frames emitted by `SkydioMavlinkClient` and assert:
   magic `0xFD`, correct payload length after zero-truncation, little-endian field order,
   CRC-16/MCRF4XX with the message's `CRC_EXTRA` (validate against a reference
   implementation, e.g. `pymavlink`).
2. **MISSION_ITEM_INT encoding** — for a known lat/lon (e.g. 40.446° N, -79.982° E):
   `x == 404460000`, `y == -799820000`, `frame == MAV_FRAME_GLOBAL_RELATIVE_ALT_INT`,
   `z == altitude_m`, `command == MAV_CMD_NAV_WAYPOINT`, acceptance radius in `param2`.
3. **Flight plan construction** — `TraverseTo_impl::buildFlightPlan` produces:
   item 0 `NAV_TAKEOFF` (`current=1`), item 1 `DO_CHANGE_SPEED` (clamped to
   `[minVelocity_mps, maxVelocity_mps]`), then one `NAV_WAYPOINT` per point, sequential `seq`.
4. **GeoJSON parsing** — `resolveWaypoints` accepts Point / LineString / Polygon
   Features; empty, malformed, or numeric-placeholder (`"0"`) input yields an empty
   list and `start` is rejected without commanding the vehicle (input validation).
5. **Telemetry decode** — feed canned `GLOBAL_POSITION_INT`, `HEARTBEAT`,
   `MISSION_CURRENT`, `MISSION_ITEM_REACHED`, `COMMAND_ACK`, `MISSION_ACK` payloads
   into the decoder and assert the `Telemetry` snapshot and ack state.
6. **Robustness** — garbage datagrams, wrong-magic frames, truncated headers, and
   CRC-corrupted frames are discarded without affecting later valid frames;
   zero-truncated payloads (e.g. `MISSION_ACK` with `type == 0`) decode correctly.

## Stage 2 — Protocol transaction tests — AUTOMATED (scripted vehicle)

Automated in the same suite (`tests/test_mavlink_client.cpp`,
`tests/test_traverse_to.cpp`) using a scripted loopback stand-in for the X10D
that runs the vehicle side of the Mission Protocol. Items 1–6 below are covered
there end-to-end through the real `TraverseTo_impl` signal handlers; item 7
(asset-parameter publication) still requires the MPMS SDK runtime and remains
a simulator/HIL check:

1. **Heartbeat exchange** — node emits a 1 Hz GCS `HEARTBEAT`; vehicle heartbeat sets
   `heartbeatOk`; loss of heartbeat for >5 s clears it and is logged.
2. **Upload handshake** — on `start`, verify `MISSION_COUNT` with the right count, one
   `MISSION_ITEM_INT` per `MISSION_REQUEST(_INT)` with matching `seq`, and that upload
   succeeds only on `MISSION_ACK == MAV_MISSION_ACCEPTED`.
3. **Rejection / timeout paths** — simulator replies `MAV_MISSION_ERROR`, or never
   replies: node reports failure, status returns to `PENDING`, no `MISSION_START` sent.
4. **Execution** — after accepted upload: `ARM` then `MISSION_START` with correct
   first/last seq; `COMMAND_ACK` retry logic (3 attempts) on dropped acks.
5. **Pause/resume/stop** — each MPMS signal produces exactly the mapped
   `COMMAND_LONG`/`MISSION_CLEAR_ALL` traffic above.
6. **Completion** — simulator publishes `MISSION_ITEM_REACHED` for the final seq;
   verify behavior status transitions `PENDING → EXECUTING_NOMINAL → COMPLETE` and the
   `WaypointListComplete` MPMS signal fires (from the worker thread, not the handler).
7. **Telemetry publication** — simulator streams `GLOBAL_POSITION_INT`; verify the
   node's asset params update at 10 Hz with correct unit conversions
   (lat/lon 1e-7 deg, alt mm→m, vel cm/s→m/s, hdg cdeg→deg).

## Stage 3 — Hardware-in-the-loop (bench, props removed) — NOT AUTOMATABLE HERE

Requires a physical X10D and the proprietary MPMS SDK runtime. With an X10D on
the bench connected over the configured UDP link:

1. Confirm the vehicle's RAS-A heartbeat is decoded (system id matches
   `SKYDIO_TARGET_SYSTEM_ID`).
2. Upload a 3-waypoint plan; confirm acceptance in the Skydio Enterprise
   controller/Cloud UI.
3. Issue `pause`/`resume`/`stop` and confirm `COMMAND_ACK` results are `ACCEPTED`.
4. Verify telemetry matches the controller's displayed position/heading.

Additional bench-only checks the loopback suite cannot provide:

5. Confirm the X10D actually answers `MISSION_COUNT` with `MISSION_REQUEST_INT`
   (vs. the legacy float `MISSION_REQUEST`, which the node also answers with
   `MISSION_ITEM_INT`) and accepts `MAV_FRAME_GLOBAL_RELATIVE_ALT_INT`.
6. Confirm the X10D's RAS-A profile accepts `MAV_CMD_NAV_TAKEOFF` as mission
   item 0 while on the ground, and its behavior when already airborne.
7. Confirm `MAV_CMD_DO_PAUSE_CONTINUE` hold/continue semantics on the X10D
   (hold position vs. loiter behavior) and that `MISSION_CLEAR_ALL` while
   holding does not trigger RTL.

## Stage 4 — Flight test — NOT AUTOMATABLE HERE

In a cleared test area, execute a full MPMS mission: send `start` with a 3–5 waypoint
LineString at a safe altitude (e.g. 30 m AGL):

1. Vehicle takes off, flies waypoints in order at the commanded speed.
2. `pause` mid-leg → vehicle holds; `resume` → continues from the same leg.
3. `update` with a diverted route → vehicle flies the new list.
4. `stop` → vehicle holds and the plan is cleared (vehicle remains under
   operator control for landing/RTL).
5. On final waypoint arrival, `WaypointListComplete` is received by the MPMS
   mission manager and status reads `COMPLETE`.

## Pass/fail criteria

- All Stage 1–2 checks automated and green.
- No unacknowledged commands (after retries) during Stages 3–4.
- Completion signal observed exactly once per traversal.
- No manual pilot intervention required during the nominal Stage 4 run.
