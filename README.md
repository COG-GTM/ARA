# skydio_me_node — Skydio X10D Mission Executor

MPMS Mission Executor (ME) node that commands a Skydio X10D to traverse a set of
waypoints. It bridges the abstract MPMS `TraverseTo` behavior to the X10D's native
RAS-A/MAVLink control link (see the "X10D Control and Telemetry ICD").

## Layout

Per the MPMS Integration Toolkit workflow, generated files are separated from
developer-editable files:

```
gen_include/  Generated behavior/asset interfaces (do not edit)
  TraverseToInterface.h     TraverseTo behavior interface (signals, config params)
  skydio_me_nodeAsset.h     Asset interface (position/altitude/heading/speed params)
gen_src/
  skydio_me_nodeAsset.cpp   Generated asset implementation (do not edit)
include/      Developer-editable headers
  TraverseTo_impl.h         TraverseTo behavior implementation
  SkydioMavlinkClient.h     Native RAS-A/MAVLink v2 UDP driver for the X10D
  skydio_me_nodeAsset_impl.h, utility.h, UTM.h
src/          Developer-editable sources
  TraverseTo_impl.cpp       start/update/pause/resume/stop handlers -> MAVLink
  SkydioMavlinkClient.cpp   Mission Protocol upload, COMMAND_LONG, telemetry decode
  main_skydio_me_node.cpp   Node entry point (config, comms, 10 Hz telemetry loop)
  skydio_me_nodeAsset_impl.cpp, utility.cpp, UTM.cpp
docs/
  TEST_PLAN.md              Verification plan (unit -> simulator -> HIL -> flight)
```

## Behavior → native protocol mapping

| MPMS `TraverseTo` | Skydio X10D (RAS-A/MAVLink) |
|---|---|
| `start` | Mission Protocol upload (`MISSION_COUNT`/`MISSION_ITEM_INT`/`MISSION_ACK`), `MAV_CMD_COMPONENT_ARM_DISARM`, `MAV_CMD_MISSION_START` |
| `update` | hold + re-upload + restart with new waypoint list |
| `pause` / `resume` | `MAV_CMD_DO_PAUSE_CONTINUE` (param1 = 0 / 1) |
| `stop` | hold + `MISSION_CLEAR_ALL` |
| `WaypointListComplete` | fired when `MISSION_ITEM_REACHED` reports the final item |
| asset telemetry | `GLOBAL_POSITION_INT` → `position`, `altitude`, `heading`, `speed` |

Waypoints arrive as GeoJSON (Point/LineString/Polygon Feature) via the `start`/`update`
signal data or the `waypoint_list` config parameter, and are encoded as
`MISSION_ITEM_INT` items in `MAV_FRAME_GLOBAL_RELATIVE_ALT_INT` at the configured
`altitude_m` and `speed_mps` (clamped to `[minVelocity_mps, maxVelocity_mps]`).

## Configuration (`skydio_me_node.cfg`)

| Key | Default | Description |
|---|---|---|
| `UDP_RX_PORT`, `UDP_HOSTS_FILE` | `5001`, `./hosts.json` | MPMS-side comms |
| `SKYDIO_VEHICLE_IP` | `192.168.10.1` | X10D control link address |
| `SKYDIO_VEHICLE_PORT` | `14550` | X10D MAVLink UDP port |
| `SKYDIO_LOCAL_PORT` | `14551` | Local bind port for the MAVLink socket |
| `SKYDIO_GCS_SYSTEM_ID` | `255` | System id this node uses on the link |
| `SKYDIO_TARGET_SYSTEM_ID` | `1` | X10D system id |

## Building

Requires the MPMS SDK (`MMSLib`, `MMSTypes`, `UDPCommInterface`), nlohmann/json, and
GDAL (for `UTM.cpp`). Compile all files under `src/` and `gen_src/` with `include/` and
`gen_include/` on the include path, e.g.:

```
g++ -std=c++14 -Iinclude -Igen_include $(MPMS_SDK_FLAGS) src/*.cpp gen_src/*.cpp \
    -o skydio_me_node -lpthread
```

`SkydioMavlinkClient` is a self-contained MAVLink v2 encoder/decoder for the message
subset TraverseTo needs, so no external MAVLink library is required.

## Testing

Automated unit/protocol tests (no MPMS SDK or vehicle required — the SDK surface
used by the behavior code is stubbed under `tests/stubs/`; requires GoogleTest
and GDAL dev packages):

```
cmake -S tests -B build-tests
cmake --build build-tests -j
ctest --test-dir build-tests --output-on-failure
```

Full verification plan (unit → simulator → HIL → flight):
see [docs/TEST_PLAN.md](docs/TEST_PLAN.md).
