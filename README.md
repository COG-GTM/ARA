# skydio_me_node — Skydio X10D Mission Executor

An MPMS Mission Executor (ME) node that commands a Skydio X10D to traverse a set
of waypoints. It translates the abstract MPMS `TraverseTo` behavior into the
X10D's native RAS-A/MAVLink v2 protocol (per the "X10D Control and Telemetry ICD").

## What's in this repo

**Implementation** (developer-editable code, per the MPMS Integration Toolkit workflow):

| File | Purpose |
|---|---|
| `src/TraverseTo_impl.cpp` | The `TraverseTo` behavior: implements the `start`/`update`/`pause`/`resume`/`stop` signal handlers and fires `WaypointListComplete` when the traverse finishes |
| `src/SkydioMavlinkClient.cpp` | Self-contained MAVLink v2 UDP driver: framing + CRC, mission upload handshake, `COMMAND_LONG` with ACK/retry, telemetry decode (no external MAVLink library needed) |
| `src/skydio_me_nodeAsset_impl.cpp` | Publishes the required asset parameters (`position`, `altitude`, `heading`, `speed`) from X10D telemetry at 10 Hz |
| `src/main_skydio_me_node.cpp` | Node entry point: config, MPMS comms, telemetry loop |

Generated files live in `gen_include/` and `gen_src/` and are **never edited**.

**Tests** (`tests/`, GoogleTest — run without the MPMS SDK or a vehicle):

- `test_mavlink_client.cpp` — protocol-level: MAVLink framing/CRC, `MISSION_ITEM_INT`
  encoding, upload handshake (incl. rejection/timeout/retry), command ACK semantics,
  telemetry conversion, corrupt-frame handling
- `test_traverse_to.cpp` — behavior-level: full start → upload → arm → start →
  complete lifecycle through the real handlers, pause/resume/stop/update traffic,
  malformed-input rejection
- `test_utility.cpp` — GeoJSON parsing and geometry/config helpers
- `FakeVehicle.h` — loopback UDP vehicle with an independent MAVLink decoder,
  so encoder bugs can't self-verify; `stubs/` stands in for the proprietary MPMS SDK

## Behavior → protocol mapping

| MPMS `TraverseTo` signal | Native X10D messages |
|---|---|
| `start` | Mission upload (`MISSION_COUNT` → `MISSION_ITEM_INT` → `MISSION_ACK`), then arm + `MAV_CMD_MISSION_START` |
| `update` | Hold, re-upload the new waypoint list, restart |
| `pause` / `resume` | `MAV_CMD_DO_PAUSE_CONTINUE` (param1 = 0 / 1) |
| `stop` | Hold + `MISSION_CLEAR_ALL` |
| `WaypointListComplete` | Fired when `MISSION_ITEM_REACHED` reports the final waypoint |

Waypoints arrive as GeoJSON (via signal data or the `waypoint_list` config param)
and are flown at the configured `altitude_m` and `speed_mps` (clamped to
`[minVelocity_mps, maxVelocity_mps]`).

## Running the tests

Requires CMake, GoogleTest, and GDAL dev packages:

```sh
cmake -S tests -B build-tests
cmake --build build-tests -j
ctest --test-dir build-tests --output-on-failure
```

## Building the node

Requires the MPMS SDK (`MMSLib`, `MMSTypes`, `UDPCommInterface`), nlohmann/json,
and GDAL. Compile everything under `src/` and `gen_src/` with `include/` and
`gen_include/` on the include path.

Runtime configuration (`skydio_me_node.cfg`): `SKYDIO_VEHICLE_IP` (default
`192.168.42.10`), `SKYDIO_VEHICLE_PORT` (`15667`), `SKYDIO_LOCAL_PORT` (`14551`),
`SKYDIO_GCS_SYSTEM_ID` (`255`), `SKYDIO_TARGET_SYSTEM_ID` (`1`), plus MPMS-side
`UDP_RX_PORT` (`5001`) and `UDP_HOSTS_FILE` (`./hosts.json`).

## Further testing

The unit and protocol stages above are automated here. Remaining stages need
hardware — see [docs/TEST_PLAN.md](docs/TEST_PLAN.md):

1. **Bench HIL** (X10D, props removed): confirm the real vehicle accepts the
   mission upload, takeoff item, and hold/clear semantics
2. **Live flight**: full waypoint traversal exercising pause/resume/update/stop
   and the completion signal
