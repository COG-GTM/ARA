/**
 * SkydioMavlinkClient.h
 *
 * Native protocol driver for the Skydio X10D.
 *
 * Per the "X10D Control and Telemetry ICD" the X10D implements the Robotics
 * and Autonomous Systems - Air (RAS-A) MAVLink Control Link Interoperability
 * Profile (IOP) v1.2 over UDP. Waypoint traversal is commanded through the
 * standard MAVLink Mission Protocol microservice:
 *
 *   upload:   MISSION_COUNT -> (MISSION_REQUEST_INT ... MISSION_ITEM_INT) -> MISSION_ACK
 *   execute:  MAV_CMD_COMPONENT_ARM_DISARM, MAV_CMD_MISSION_START (COMMAND_LONG)
 *   monitor:  MISSION_CURRENT / MISSION_ITEM_REACHED / GLOBAL_POSITION_INT
 *   pause:    MAV_CMD_DO_PAUSE_CONTINUE (param1 = 0 hold / 1 continue)
 *   abort:    MISSION_CLEAR_ALL + hold
 *
 * This is a deliberately small, dependency-free MAVLink v2 encoder/decoder
 * that implements only the subset of messages required by the TraverseTo
 * behavior, so the ME node does not need the full mavlink header tree.
 */

#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace skydio
{

// ---- MAVLink message IDs (common.xml / RAS-A dialect) ----
constexpr uint32_t MSG_HEARTBEAT            = 0;
constexpr uint32_t MSG_GLOBAL_POSITION_INT  = 33;
constexpr uint32_t MSG_MISSION_REQUEST      = 40;   // legacy float variant, answered with MISSION_ITEM_INT
constexpr uint32_t MSG_MISSION_CURRENT      = 42;
constexpr uint32_t MSG_MISSION_COUNT        = 44;
constexpr uint32_t MSG_MISSION_CLEAR_ALL    = 45;
constexpr uint32_t MSG_MISSION_ITEM_REACHED = 46;
constexpr uint32_t MSG_MISSION_ACK          = 47;
constexpr uint32_t MSG_MISSION_REQUEST_INT  = 51;
constexpr uint32_t MSG_MISSION_ITEM_INT     = 73;
constexpr uint32_t MSG_COMMAND_LONG         = 76;
constexpr uint32_t MSG_COMMAND_ACK          = 77;

// ---- MAV_CMD ids used for traverse-to ----
constexpr uint16_t MAV_CMD_NAV_WAYPOINT           = 16;
constexpr uint16_t MAV_CMD_NAV_RETURN_TO_LAUNCH   = 20;
constexpr uint16_t MAV_CMD_NAV_LAND               = 21;
constexpr uint16_t MAV_CMD_NAV_TAKEOFF            = 22;
constexpr uint16_t MAV_CMD_DO_CHANGE_SPEED        = 178;
constexpr uint16_t MAV_CMD_DO_PAUSE_CONTINUE      = 193;
constexpr uint16_t MAV_CMD_DO_SET_MISSION_CURRENT = 224;
constexpr uint16_t MAV_CMD_MISSION_START          = 300;
constexpr uint16_t MAV_CMD_COMPONENT_ARM_DISARM   = 400;

// ---- enums ----
constexpr uint8_t MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6;
constexpr uint8_t MAV_MISSION_TYPE_MISSION          = 0;
constexpr uint8_t MAV_MISSION_ACCEPTED              = 0;
constexpr uint8_t MAV_RESULT_ACCEPTED               = 0;
constexpr uint8_t MAV_TYPE_GCS                      = 6;
constexpr uint8_t MAV_AUTOPILOT_INVALID             = 8;
constexpr uint8_t MAV_STATE_ACTIVE                  = 4;
constexpr uint8_t MAV_MODE_FLAG_SAFETY_ARMED        = 0x80;

/// One entry of a MAVLink flight plan (maps 1:1 onto MISSION_ITEM_INT).
struct MissionItem
{
    uint16_t seq          = 0;
    uint8_t  frame        = MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
    uint16_t command      = MAV_CMD_NAV_WAYPOINT;
    uint8_t  current      = 0;
    uint8_t  autocontinue = 1;
    float    param1       = 0.f;
    float    param2       = 0.f;
    float    param3       = 0.f;
    float    param4       = 0.f;
    int32_t  x            = 0;   // latitude  * 1e7
    int32_t  y            = 0;   // longitude * 1e7
    float    z            = 0.f; // altitude, meters (frame-relative)
};

/// Snapshot of vehicle telemetry decoded from the RAS-A stream.
struct Telemetry
{
    bool   positionValid = false;
    double latitudeDeg   = 0.0;
    double longitudeDeg  = 0.0;
    float  relAltitudeM  = 0.f;
    float  headingDeg    = 0.f;
    float  speedMps      = 0.f;
    bool   armed         = false;
    int    currentSeq    = -1;  // MISSION_CURRENT
    int    reachedSeq    = -1;  // MISSION_ITEM_REACHED
    bool   heartbeatOk   = false;
};

/**
 * Minimal MAVLink v2 / RAS-A UDP client for the Skydio X10D.
 * Owns a receive thread and a 1 Hz GCS heartbeat thread.
 * Shared by the asset implementation (telemetry) and the behavior
 * implementations (commanding), hence exposed as a singleton.
 */
class SkydioMavlinkClient
{
public:
    static SkydioMavlinkClient & instance();

    // Must be called before connect().
    void configure( const std::string & vehicleIp,
                    unsigned short vehiclePort,
                    unsigned short localPort,
                    uint8_t gcsSystemId,
                    uint8_t targetSystemId );

    bool connect();
    void disconnect();
    bool isConnected() const { return m_connected.load(); }

    /// Blocks until a vehicle HEARTBEAT is decoded or the timeout elapses.
    bool waitForHeartbeat( std::chrono::milliseconds timeout );

    /// Full Mission Protocol upload transaction (MISSION_COUNT ->
    /// MISSION_REQUEST(_INT)/MISSION_ITEM_INT -> MISSION_ACK == accepted).
    bool uploadMission( const std::vector<MissionItem> & items,
                        std::chrono::milliseconds timeout );

    /// MISSION_CLEAR_ALL, acknowledged by MISSION_ACK.
    bool clearMission( std::chrono::milliseconds timeout );

    /// COMMAND_LONG helpers; each waits for COMMAND_ACK == MAV_RESULT_ACCEPTED.
    bool sendCommand( uint16_t command,
                      float p1 = 0.f, float p2 = 0.f, float p3 = 0.f,
                      float p4 = 0.f, float p5 = 0.f, float p6 = 0.f,
                      float p7 = 0.f,
                      std::chrono::milliseconds timeout = std::chrono::milliseconds( 3000 ),
                      int retries = 3 );

    bool arm( bool armed );
    bool startMission( uint16_t firstSeq, uint16_t lastSeq );
    bool pauseMission();   // MAV_CMD_DO_PAUSE_CONTINUE, param1 = 0 (hold)
    bool resumeMission();  // MAV_CMD_DO_PAUSE_CONTINUE, param1 = 1 (continue)
    bool setCurrentItem( uint16_t seq );
    bool changeSpeed( float speedMps );

    Telemetry getTelemetry() const;

    /// Resets the reached/current mission item trackers before a new run.
    void resetMissionProgress();

private:
    SkydioMavlinkClient() = default;
    ~SkydioMavlinkClient();
    SkydioMavlinkClient( const SkydioMavlinkClient & ) = delete;
    SkydioMavlinkClient & operator=( const SkydioMavlinkClient & ) = delete;

    // framing
    void sendMessage( uint32_t msgId, const uint8_t * payload, uint8_t length );
    void sendHeartbeat();
    void sendMissionItemInt( const MissionItem & item );
    void sendMissionCount( uint16_t count );

    // rx path
    void rxLoop();
    void heartbeatLoop();
    void handleMessage( uint32_t msgId, const uint8_t * payload, uint8_t length );

    static uint16_t crcExtra( uint32_t msgId );

    // configuration
    std::string    m_vehicleIp   = "192.168.10.1";
    unsigned short m_vehiclePort = 14550;
    unsigned short m_localPort   = 14550;
    uint8_t        m_systemId    = 255; // GCS-side system id
    uint8_t        m_componentId = 190; // MAV_COMP_ID_MISSIONPLANNER
    uint8_t        m_targetSystem    = 1;
    uint8_t        m_targetComponent = 1; // MAV_COMP_ID_AUTOPILOT1

    // socket / threads
    int               m_socket = -1;
    std::atomic<bool> m_connected{ false };
    std::atomic<bool> m_shutdown{ false };
    std::thread       m_rxThread;
    std::thread       m_heartbeatThread;
    uint8_t           m_txSequence = 0;
    mutable std::mutex m_txMutex;

    // protocol state shared with the rx thread
    mutable std::mutex      m_stateMutex;
    std::condition_variable m_stateCondition;
    Telemetry               m_telemetry;
    int                     m_missionRequestSeq = -1; // last MISSION_REQUEST(_INT) seq
    int                     m_missionAckType    = -1; // last MISSION_ACK type
    int                     m_commandAckCmd     = -1; // last COMMAND_ACK command
    int                     m_commandAckResult  = -1; // last COMMAND_ACK result
    std::chrono::steady_clock::time_point m_lastHeartbeat{};
};

} // namespace skydio
