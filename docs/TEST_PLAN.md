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

### Connection and discovery

1. Confirm the node connects to the X10D at the ICD default endpoint
   `192.168.42.10:15667/UDP` with no configuration overrides, and that an
   explicitly configured `SKYDIO_VEHICLE_IP`/`SKYDIO_VEHICLE_PORT` overrides it.
2. Confirm the vehicle's RAS-A heartbeat is decoded (system id matches
   `SKYDIO_TARGET_SYSTEM_ID`) and target-system discovery is stable across
   node restarts.

### Command and mission handling

3. Issue an ARM (`MAV_CMD_COMPONENT_ARM_DISARM`) and record the real
   `COMMAND_ACK` sequence — specifically whether the X10D emits
   `MAV_RESULT_IN_PROGRESS` before the terminal ACK, and how long arming takes.
4. Upload a 3+ waypoint plan; confirm acceptance in the Skydio Enterprise
   controller/Cloud UI.
5. Issue `pause`/`resume`/`stop` and confirm `COMMAND_ACK` results are `ACCEPTED`.
6. Verify telemetry (position, altitude, heading, speed) agrees with the
   Skydio controller's display.

### Bench-only protocol checks the loopback suite cannot provide

7. Confirm the X10D actually answers `MISSION_COUNT` with `MISSION_REQUEST_INT`
   (vs. the legacy float `MISSION_REQUEST`, which the node also answers with
   `MISSION_ITEM_INT`) and accepts `MAV_FRAME_GLOBAL_RELATIVE_ALT_INT`.
8. Confirm the X10D's RAS-A profile accepts `MAV_CMD_NAV_TAKEOFF` as mission
   item 0 while on the ground, and its behavior when already airborne
   (takeoff item should be skipped/accepted without landing first).
9. Confirm `MAV_CMD_DO_PAUSE_CONTINUE` hold/continue semantics on the X10D
   (hold position vs. loiter behavior) and that `MISSION_CLEAR_ALL` while
   holding does not trigger RTL.

### Packet-loss / retry injection (bench, where practical)

Using a UDP impairment tool (e.g. `tc netem` loss on the link, or an
interposing proxy) inject loss during each Mission Protocol stage and verify
retries occur within the RAS-A bounds (~250 ms item timing, ~1500 ms protocol
timing, max 5 retries) and that failure is cleanly reported (status returns to
`PENDING`, no `MISSION_START`) after retry exhaustion:

10. Drop the initial `MISSION_COUNT` — node re-announces.
11. Drop a `MISSION_REQUEST_INT` from the vehicle — node retransmits the
    previous item (or `MISSION_COUNT` if no item sent yet).
12. Drop a `MISSION_ITEM_INT` — vehicle re-requests; node answers the
    duplicate request.
13. Drop the final `MISSION_ACK` — node retransmits the last item until the
    ACK arrives or retries are exhausted.
14. Drop `MISSION_CLEAR_ALL` / its ACK — node retries the clear; after
    exhaustion `stop` still holds the vehicle and reports the failure.

## Stage 4 — Flight test — NOT AUTOMATABLE HERE

In a cleared test area, execute a full MPMS mission: send `start` with a 3–5 waypoint
LineString at a safe altitude (e.g. 30 m AGL):

1. Vehicle takes off (`NAV_TAKEOFF` while landed), flies waypoints in the
   expected order.
2. Vehicle holds the requested altitude (`altitude_m`) and the requested speed
   (`DO_CHANGE_SPEED` value) within tolerance, verified against controller
   telemetry.
3. `pause` mid-leg → vehicle holds; `resume` → continues from the same leg.
4. `update` with a diverted route while the mission is active → vehicle flies
   the new list without corrupting mission state.
5. `stop` while traversing → vehicle holds and the plan is cleared
   (`MISSION_CLEAR_ALL`); verify NO unexpected RTL, landing, or continued
   mission execution after the stop (vehicle remains under operator control
   for landing/RTL).
6. On final waypoint arrival, `WaypointListComplete` is received by the MPMS
   mission manager exactly once and status reads `COMPLETE`.
7. Repeat one upload with induced packet loss (weak-link placement or
   impairment tool) and confirm the mission still uploads via retries or
   fails cleanly without partial execution.

## Pass/fail criteria

- All Stage 1–2 checks automated and green.
- No unacknowledged commands (after retries) during Stages 3–4.
- Completion signal observed exactly once per traversal.
- No manual pilot intervention required during the nominal Stage 4 run.
