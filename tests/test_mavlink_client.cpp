/**
 * Protocol-level tests for SkydioMavlinkClient using a scripted FakeVehicle
 * on a loopback UDP socket. Verifies MAVLink v2 framing (magic, header,
 * zero-truncation, CRC-16/MCRF4XX + CRC_EXTRA via an independent decoder),
 * payload field encoding, the Mission Protocol upload transaction, command
 * retry/rejection handling, and telemetry decoding.
 */
#include <gtest/gtest.h>

#include <atomic>
#include <cmath>
#include <future>
#include <thread>

#include "FakeVehicle.h"
#include "SkydioMavlinkClient.h"

using namespace std::chrono_literals;
using testutil::FakeVehicle;
using testutil::Frame;
using testutil::readField;
using testutil::writeField;

namespace
{

unsigned short nextPort()
{
    static std::atomic<unsigned short> port{ 24550 };
    return port++;
}

std::vector<uint8_t> heartbeatPayload( uint8_t baseMode )
{
    std::vector<uint8_t> payload( 9, 0 );
    payload[4] = 2;        // MAV_TYPE_QUADROTOR
    payload[5] = 12;       // autopilot
    payload[6] = baseMode; // base_mode
    payload[7] = skydio::MAV_STATE_ACTIVE;
    payload[8] = 3;
    return payload;
}

std::vector<uint8_t> missionRequestIntPayload( uint16_t seq )
{
    std::vector<uint8_t> payload( 5, 0 );
    writeField<uint16_t>( payload, 0, seq );
    payload[2] = 255; // target_system (the GCS/node)
    payload[3] = 190; // target_component
    payload[4] = skydio::MAV_MISSION_TYPE_MISSION;
    return payload;
}

std::vector<uint8_t> missionAckPayload( uint8_t type )
{
    std::vector<uint8_t> payload( 3, 0 );
    payload[0] = 255;
    payload[1] = 190;
    payload[2] = type;
    return payload;
}

std::vector<uint8_t> commandAckPayload( uint16_t command, uint8_t result )
{
    std::vector<uint8_t> payload( 3, 0 );
    writeField<uint16_t>( payload, 0, command );
    payload[2] = result;
    return payload;
}

class MavlinkClientTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        const unsigned short vehiclePort = nextPort();
        m_vehicle = std::make_unique<FakeVehicle>( vehiclePort );

        auto & client = skydio::SkydioMavlinkClient::instance();
        client.configure( "127.0.0.1", vehiclePort, nextPort(), 255, 1 );
        ASSERT_TRUE( client.connect() );
    }

    void TearDown() override
    {
        skydio::SkydioMavlinkClient::instance().disconnect();
        // The independent decoder must have accepted every frame the node sent.
        EXPECT_EQ( m_vehicle->decodeErrors(), 0 );
        m_vehicle.reset();
    }

    skydio::SkydioMavlinkClient & client() { return skydio::SkydioMavlinkClient::instance(); }

    std::unique_ptr<FakeVehicle> m_vehicle;
};

// ---- framing / heartbeat -------------------------------------------------

TEST_F( MavlinkClientTest, GcsHeartbeatFramingAndFields )
{
    Frame frame;
    ASSERT_TRUE( m_vehicle->waitForMessage( skydio::MSG_HEARTBEAT, frame ) );

    // Header identity fields.
    EXPECT_EQ( frame.sysid, 255 );  // configured GCS system id
    EXPECT_EQ( frame.compid, 190 ); // MAV_COMP_ID_MISSIONPLANNER

    // HEARTBEAT payload: custom_mode u32, type, autopilot, base_mode,
    // system_status, mavlink_version (zero-truncated fields read as 0).
    EXPECT_EQ( readField<uint32_t>( frame.payload, 0 ), 0u );
    EXPECT_EQ( readField<uint8_t>( frame.payload, 4 ), skydio::MAV_TYPE_GCS );
    EXPECT_EQ( readField<uint8_t>( frame.payload, 5 ), skydio::MAV_AUTOPILOT_INVALID );
    EXPECT_EQ( readField<uint8_t>( frame.payload, 6 ), 0 );
    EXPECT_EQ( readField<uint8_t>( frame.payload, 7 ), skydio::MAV_STATE_ACTIVE );
    EXPECT_EQ( readField<uint8_t>( frame.payload, 8 ), 3 );
}

TEST_F( MavlinkClientTest, TxSequenceIncrementsAcrossFrames )
{
    Frame first, second;
    ASSERT_TRUE( m_vehicle->waitForMessage( skydio::MSG_HEARTBEAT, first ) );
    ASSERT_TRUE( m_vehicle->waitForMessage( skydio::MSG_HEARTBEAT, second, 2500ms ) );
    EXPECT_EQ( second.seq, static_cast<uint8_t>( first.seq + 1 ) );
}

TEST_F( MavlinkClientTest, WaitForHeartbeatAndArmedFlag )
{
    Frame frame;
    ASSERT_TRUE( m_vehicle->waitForMessage( skydio::MSG_HEARTBEAT, frame ) ); // learn peer

    m_vehicle->sendMessage( skydio::MSG_HEARTBEAT, heartbeatPayload( skydio::MAV_MODE_FLAG_SAFETY_ARMED ) );
    EXPECT_TRUE( client().waitForHeartbeat( 3000ms ) );

    // Poll until the rx thread applies the armed flag.
    const auto deadline = std::chrono::steady_clock::now() + 2s;
    while ( !client().getTelemetry().armed && std::chrono::steady_clock::now() < deadline )
        std::this_thread::sleep_for( 20ms );
    EXPECT_TRUE( client().getTelemetry().armed );
    EXPECT_TRUE( client().getTelemetry().heartbeatOk );
}

// ---- mission upload transaction -------------------------------------------

TEST_F( MavlinkClientTest, MissionUploadHappyPathAndItemEncoding )
{
    std::vector<skydio::MissionItem> plan( 3 );
    plan[0].seq     = 0;
    plan[0].command = skydio::MAV_CMD_NAV_TAKEOFF;
    plan[0].current = 1;
    plan[0].z       = 25.f;
    plan[1].seq     = 1;
    plan[1].command = skydio::MAV_CMD_DO_CHANGE_SPEED;
    plan[1].param1  = 1.f;
    plan[1].param2  = 3.5f;
    plan[1].param3  = -1.f;
    plan[2].seq     = 2;
    plan[2].command = skydio::MAV_CMD_NAV_WAYPOINT;
    plan[2].param2  = 2.f;
    plan[2].x       = 404460000;  // 40.446 deg * 1e7
    plan[2].y       = -799820000; // -79.982 deg * 1e7
    plan[2].z       = 25.f;

    auto result = std::async( std::launch::async, [&]() { return client().uploadMission( plan, 10s ); } );

    Frame count;
    ASSERT_TRUE( m_vehicle->waitForMessage( skydio::MSG_MISSION_COUNT, count ) );
    EXPECT_EQ( readField<uint16_t>( count.payload, 0 ), 3 );
    EXPECT_EQ( readField<uint8_t>( count.payload, 2 ), 1 ); // target_system
    EXPECT_EQ( readField<uint8_t>( count.payload, 4 ), skydio::MAV_MISSION_TYPE_MISSION );

    for ( uint16_t seq = 0; seq < 3; ++seq )
    {
        m_vehicle->sendMessage( skydio::MSG_MISSION_REQUEST_INT, missionRequestIntPayload( seq ) );
        Frame item;
        ASSERT_TRUE( m_vehicle->waitForMessage( skydio::MSG_MISSION_ITEM_INT, item ) );
        EXPECT_EQ( readField<uint16_t>( item.payload, 28 ), seq );
        EXPECT_EQ( readField<uint16_t>( item.payload, 30 ), plan[seq].command );
        EXPECT_EQ( readField<uint8_t>( item.payload, 32 ), 1 );                      // target_system
        EXPECT_EQ( readField<uint8_t>( item.payload, 34 ), plan[seq].frame );        // frame
        EXPECT_EQ( readField<uint8_t>( item.payload, 35 ), plan[seq].current );      // current
        EXPECT_EQ( readField<uint8_t>( item.payload, 36 ), 1 );                      // autocontinue
        EXPECT_EQ( readField<int32_t>( item.payload, 16 ), plan[seq].x );            // lat * 1e7
        EXPECT_EQ( readField<int32_t>( item.payload, 20 ), plan[seq].y );            // lon * 1e7
        EXPECT_FLOAT_EQ( readField<float>( item.payload, 24 ), plan[seq].z );        // alt
        EXPECT_FLOAT_EQ( readField<float>( item.payload, 4 ), plan[seq].param2 );
    }

    m_vehicle->sendMessage( skydio::MSG_MISSION_ACK, missionAckPayload( skydio::MAV_MISSION_ACCEPTED ) );
    EXPECT_TRUE( result.get() );
}

TEST_F( MavlinkClientTest, MissionUploadAnswersLegacyMissionRequest )
{
    std::vector<skydio::MissionItem> plan( 1 );
    auto result = std::async( std::launch::async, [&]() { return client().uploadMission( plan, 10s ); } );

    Frame count;
    ASSERT_TRUE( m_vehicle->waitForMessage( skydio::MSG_MISSION_COUNT, count ) );

    // Legacy MISSION_REQUEST (40) must also be answered with MISSION_ITEM_INT.
    m_vehicle->sendMessage( skydio::MSG_MISSION_REQUEST, missionRequestIntPayload( 0 ) );
    Frame item;
    ASSERT_TRUE( m_vehicle->waitForMessage( skydio::MSG_MISSION_ITEM_INT, item ) );

    m_vehicle->sendMessage( skydio::MSG_MISSION_ACK, missionAckPayload( skydio::MAV_MISSION_ACCEPTED ) );
    EXPECT_TRUE( result.get() );
}

TEST_F( MavlinkClientTest, MissionUploadRejectedByVehicle )
{
    std::vector<skydio::MissionItem> plan( 2 );
    auto result = std::async( std::launch::async, [&]() { return client().uploadMission( plan, 10s ); } );

    Frame count;
    ASSERT_TRUE( m_vehicle->waitForMessage( skydio::MSG_MISSION_COUNT, count ) );
    m_vehicle->sendMessage( skydio::MSG_MISSION_ACK, missionAckPayload( 1 ) ); // MAV_MISSION_ERROR
    EXPECT_FALSE( result.get() );
}

TEST_F( MavlinkClientTest, MissionUploadTimesOutAndRetriesCount )
{
    std::vector<skydio::MissionItem> plan( 1 );
    auto result = std::async( std::launch::async, [&]() { return client().uploadMission( plan, 1500ms ); } );

    // With no vehicle response, MISSION_COUNT should be re-announced.
    Frame first, second;
    ASSERT_TRUE( m_vehicle->waitForMessage( skydio::MSG_MISSION_COUNT, first ) );
    EXPECT_TRUE( m_vehicle->waitForMessage( skydio::MSG_MISSION_COUNT, second, 2000ms ) );
    EXPECT_FALSE( result.get() );
}

TEST_F( MavlinkClientTest, EmptyMissionUploadFails )
{
    EXPECT_FALSE( client().uploadMission( {}, 100ms ) );
}

// ---- COMMAND_LONG helpers --------------------------------------------------

TEST_F( MavlinkClientTest, ArmCommandEncodingAndAck )
{
    auto result = std::async( std::launch::async, [&]() { return client().arm( true ); } );

    Frame cmd;
    ASSERT_TRUE( m_vehicle->waitForMessage( skydio::MSG_COMMAND_LONG, cmd ) );
    EXPECT_EQ( readField<uint16_t>( cmd.payload, 28 ), skydio::MAV_CMD_COMPONENT_ARM_DISARM );
    EXPECT_FLOAT_EQ( readField<float>( cmd.payload, 0 ), 1.f ); // param1: arm
    EXPECT_EQ( readField<uint8_t>( cmd.payload, 30 ), 1 );      // target_system
    EXPECT_EQ( readField<uint8_t>( cmd.payload, 32 ), 0 );      // confirmation (first attempt)

    m_vehicle->sendMessage( skydio::MSG_COMMAND_ACK,
                            commandAckPayload( skydio::MAV_CMD_COMPONENT_ARM_DISARM, skydio::MAV_RESULT_ACCEPTED ) );
    EXPECT_TRUE( result.get() );
}

TEST_F( MavlinkClientTest, MissionStartPauseResumeAndSpeedParams )
{
    struct Case
    {
        std::function<bool()> invoke;
        uint16_t              command;
        float                 p1;
        float                 p2;
    };
    const std::vector<Case> cases = {
        { [&]() { return client().startMission( 0, 4 ); }, skydio::MAV_CMD_MISSION_START, 0.f, 4.f },
        { [&]() { return client().pauseMission(); }, skydio::MAV_CMD_DO_PAUSE_CONTINUE, 0.f, 0.f },
        { [&]() { return client().resumeMission(); }, skydio::MAV_CMD_DO_PAUSE_CONTINUE, 1.f, 0.f },
        { [&]() { return client().changeSpeed( 3.5f ); }, skydio::MAV_CMD_DO_CHANGE_SPEED, 1.f, 3.5f },
        { [&]() { return client().setCurrentItem( 2 ); }, skydio::MAV_CMD_DO_SET_MISSION_CURRENT, 2.f, 0.f },
    };

    for ( const auto & testCase : cases )
    {
        auto result = std::async( std::launch::async, testCase.invoke );
        Frame cmd;
        ASSERT_TRUE( m_vehicle->waitForMessage( skydio::MSG_COMMAND_LONG, cmd ) );
        EXPECT_EQ( readField<uint16_t>( cmd.payload, 28 ), testCase.command );
        EXPECT_FLOAT_EQ( readField<float>( cmd.payload, 0 ), testCase.p1 );
        EXPECT_FLOAT_EQ( readField<float>( cmd.payload, 4 ), testCase.p2 );
        m_vehicle->sendMessage( skydio::MSG_COMMAND_ACK,
                                commandAckPayload( testCase.command, skydio::MAV_RESULT_ACCEPTED ) );
        EXPECT_TRUE( result.get() );
    }
}

TEST_F( MavlinkClientTest, CommandRetriesWithIncrementedConfirmation )
{
    auto result = std::async( std::launch::async, [&]()
                              { return client().sendCommand( skydio::MAV_CMD_MISSION_START, 0.f, 2.f,
                                                             0.f, 0.f, 0.f, 0.f, 0.f, 700ms, 3 ); } );

    Frame first;
    ASSERT_TRUE( m_vehicle->waitForMessage( skydio::MSG_COMMAND_LONG, first ) );
    EXPECT_EQ( readField<uint8_t>( first.payload, 32 ), 0 ); // confirmation 0, deliberately not acked

    Frame second;
    ASSERT_TRUE( m_vehicle->waitForMessage( skydio::MSG_COMMAND_LONG, second, 2000ms ) );
    EXPECT_EQ( readField<uint8_t>( second.payload, 32 ), 1 ); // retried with confirmation 1

    m_vehicle->sendMessage( skydio::MSG_COMMAND_ACK,
                            commandAckPayload( skydio::MAV_CMD_MISSION_START, skydio::MAV_RESULT_ACCEPTED ) );
    EXPECT_TRUE( result.get() );
}

TEST_F( MavlinkClientTest, RejectedCommandFailsWithoutFurtherRetries )
{
    auto result = std::async( std::launch::async, [&]() { return client().pauseMission(); } );

    Frame cmd;
    ASSERT_TRUE( m_vehicle->waitForMessage( skydio::MSG_COMMAND_LONG, cmd ) );
    m_vehicle->sendMessage( skydio::MSG_COMMAND_ACK,
                            commandAckPayload( skydio::MAV_CMD_DO_PAUSE_CONTINUE, 4 ) ); // MAV_RESULT_DENIED
    EXPECT_FALSE( result.get() );

    Frame extra;
    EXPECT_FALSE( m_vehicle->waitForMessage( skydio::MSG_COMMAND_LONG, extra, 500ms ) );
}

TEST_F( MavlinkClientTest, UnackedCommandFailsAfterRetries )
{
    EXPECT_FALSE( client().sendCommand( skydio::MAV_CMD_MISSION_START, 0.f, 0.f,
                                        0.f, 0.f, 0.f, 0.f, 0.f, 200ms, 2 ) );
}

// ---- clear / telemetry -----------------------------------------------------

TEST_F( MavlinkClientTest, ClearMissionEncodingAndZeroTruncatedAck )
{
    auto result = std::async( std::launch::async, [&]() { return client().clearMission( 5s ); } );

    Frame clear;
    ASSERT_TRUE( m_vehicle->waitForMessage( skydio::MSG_MISSION_CLEAR_ALL, clear ) );
    EXPECT_EQ( readField<uint8_t>( clear.payload, 0 ), 1 ); // target_system
    EXPECT_EQ( readField<uint8_t>( clear.payload, 2 ), skydio::MAV_MISSION_TYPE_MISSION );

    // MISSION_ACK with type == 0 (ACCEPTED): the trailing zero byte is
    // zero-truncated on the wire; the client must still decode it as accepted.
    m_vehicle->sendMessage( skydio::MSG_MISSION_ACK, missionAckPayload( skydio::MAV_MISSION_ACCEPTED ) );
    EXPECT_TRUE( result.get() );
}

TEST_F( MavlinkClientTest, GlobalPositionIntTelemetryDecoding )
{
    Frame frame;
    ASSERT_TRUE( m_vehicle->waitForMessage( skydio::MSG_HEARTBEAT, frame ) ); // learn peer

    std::vector<uint8_t> payload( 28, 0 );
    writeField<uint32_t>( payload, 0, 123456 );       // time_boot_ms
    writeField<int32_t>( payload, 4, 404460000 );     // lat
    writeField<int32_t>( payload, 8, -799820000 );    // lon
    writeField<int32_t>( payload, 12, 300000 );       // alt (MSL mm)
    writeField<int32_t>( payload, 16, 25000 );        // relative_alt mm
    writeField<int16_t>( payload, 20, 300 );          // vx cm/s
    writeField<int16_t>( payload, 22, 400 );          // vy cm/s
    writeField<int16_t>( payload, 24, -100 );         // vz cm/s
    writeField<uint16_t>( payload, 26, 9000 );        // hdg cdeg
    m_vehicle->sendMessage( skydio::MSG_GLOBAL_POSITION_INT, payload );

    const auto deadline = std::chrono::steady_clock::now() + 2s;
    while ( !client().getTelemetry().positionValid && std::chrono::steady_clock::now() < deadline )
        std::this_thread::sleep_for( 20ms );

    const skydio::Telemetry telemetry = client().getTelemetry();
    ASSERT_TRUE( telemetry.positionValid );
    EXPECT_NEAR( telemetry.latitudeDeg, 40.446, 1e-9 );
    EXPECT_NEAR( telemetry.longitudeDeg, -79.982, 1e-9 );
    EXPECT_NEAR( telemetry.relAltitudeM, 25.0, 1e-3 );
    EXPECT_NEAR( telemetry.speedMps, 5.0, 1e-3 );   // sqrt(3^2 + 4^2)
    EXPECT_NEAR( telemetry.headingDeg, 90.0, 1e-3 );
}

TEST_F( MavlinkClientTest, UnknownHeadingIsIgnored )
{
    Frame frame;
    ASSERT_TRUE( m_vehicle->waitForMessage( skydio::MSG_HEARTBEAT, frame ) );

    std::vector<uint8_t> payload( 28, 0 );
    writeField<uint16_t>( payload, 26, 4500 );
    m_vehicle->sendMessage( skydio::MSG_GLOBAL_POSITION_INT, payload );
    std::this_thread::sleep_for( 200ms );
    EXPECT_NEAR( client().getTelemetry().headingDeg, 45.0, 1e-3 );

    writeField<uint16_t>( payload, 26, UINT16_MAX ); // UINT16_MAX == unknown
    m_vehicle->sendMessage( skydio::MSG_GLOBAL_POSITION_INT, payload );
    std::this_thread::sleep_for( 200ms );
    EXPECT_NEAR( client().getTelemetry().headingDeg, 45.0, 1e-3 ); // unchanged
}

TEST_F( MavlinkClientTest, MissionProgressTrackingAndReset )
{
    Frame frame;
    ASSERT_TRUE( m_vehicle->waitForMessage( skydio::MSG_HEARTBEAT, frame ) );

    std::vector<uint8_t> current( 2, 0 );
    writeField<uint16_t>( current, 0, 3 );
    m_vehicle->sendMessage( skydio::MSG_MISSION_CURRENT, current );

    std::vector<uint8_t> reached( 2, 0 );
    writeField<uint16_t>( reached, 0, 2 );
    m_vehicle->sendMessage( skydio::MSG_MISSION_ITEM_REACHED, reached );

    const auto deadline = std::chrono::steady_clock::now() + 2s;
    while ( client().getTelemetry().reachedSeq < 2 && std::chrono::steady_clock::now() < deadline )
        std::this_thread::sleep_for( 20ms );

    EXPECT_EQ( client().getTelemetry().currentSeq, 3 );
    EXPECT_EQ( client().getTelemetry().reachedSeq, 2 );

    client().resetMissionProgress();
    EXPECT_EQ( client().getTelemetry().currentSeq, -1 );
    EXPECT_EQ( client().getTelemetry().reachedSeq, -1 );
}

// ---- robustness ------------------------------------------------------------

TEST_F( MavlinkClientTest, SurvivesMalformedAndCorruptedFrames )
{
    Frame frame;
    ASSERT_TRUE( m_vehicle->waitForMessage( skydio::MSG_HEARTBEAT, frame ) );

    // Garbage, wrong magic, truncated header, and a CRC-corrupted frame.
    m_vehicle->sendRaw( { 0x00, 0x01, 0x02, 0x03 } );
    m_vehicle->sendRaw( { 0xFE, 9, 0, 0, 0, 1, 1, 0, 0, 0 } );
    m_vehicle->sendRaw( { 0xFD, 200, 0, 0, 0, 1 } );
    std::vector<uint8_t> corrupt = { 0xFD, 2, 0, 0, 0, 1, 1, 46, 0, 0, 0x05, 0x00, 0xDE, 0xAD };
    m_vehicle->sendRaw( corrupt );

    std::this_thread::sleep_for( 200ms );
    EXPECT_EQ( client().getTelemetry().reachedSeq, -1 ); // corrupt frame discarded

    // A valid frame after the garbage must still decode.
    std::vector<uint8_t> reached( 2, 0 );
    writeField<uint16_t>( reached, 0, 7 );
    m_vehicle->sendMessage( skydio::MSG_MISSION_ITEM_REACHED, reached );

    const auto deadline = std::chrono::steady_clock::now() + 2s;
    while ( client().getTelemetry().reachedSeq < 7 && std::chrono::steady_clock::now() < deadline )
        std::this_thread::sleep_for( 20ms );
    EXPECT_EQ( client().getTelemetry().reachedSeq, 7 );
}

// ---- endpoint configuration -------------------------------------------------

TEST( MavlinkClientConfigTest, DefaultEndpointMatchesX10dIcd )
{
    skydio::SkydioMavlinkClient fresh;
    EXPECT_EQ( fresh.vehicleIp(), "192.168.42.10" );
    EXPECT_EQ( fresh.vehiclePort(), 15667 );
    EXPECT_EQ( fresh.gcsSystemId(), 255 );
    EXPECT_EQ( fresh.targetSystemId(), 1 );
}

TEST( MavlinkClientConfigTest, ConfiguredEndpointOverridesDefault )
{
    skydio::SkydioMavlinkClient fresh;
    fresh.configure( "10.1.2.3", 14550, 14551, 250, 7 );
    EXPECT_EQ( fresh.vehicleIp(), "10.1.2.3" );
    EXPECT_EQ( fresh.vehiclePort(), 14550 );
    EXPECT_EQ( fresh.localPort(), 14551 );
    EXPECT_EQ( fresh.gcsSystemId(), 250 );
    EXPECT_EQ( fresh.targetSystemId(), 7 );
}

// ---- MAV_RESULT_IN_PROGRESS -------------------------------------------------

TEST_F( MavlinkClientTest, ArmInProgressThenAcceptedSucceeds )
{
    auto result = std::async( std::launch::async, [&]() { return client().arm( true ); } );

    Frame cmd;
    ASSERT_TRUE( m_vehicle->waitForMessage( skydio::MSG_COMMAND_LONG, cmd ) );
    m_vehicle->sendMessage( skydio::MSG_COMMAND_ACK,
                            commandAckPayload( skydio::MAV_CMD_COMPONENT_ARM_DISARM, skydio::MAV_RESULT_IN_PROGRESS ) );
    std::this_thread::sleep_for( 500ms );
    m_vehicle->sendMessage( skydio::MSG_COMMAND_ACK,
                            commandAckPayload( skydio::MAV_CMD_COMPONENT_ARM_DISARM, skydio::MAV_RESULT_ACCEPTED ) );
    EXPECT_TRUE( result.get() );

    // IN_PROGRESS must not have triggered a retransmission.
    Frame extra;
    EXPECT_FALSE( m_vehicle->waitForMessage( skydio::MSG_COMMAND_LONG, extra, 500ms ) );
}

TEST_F( MavlinkClientTest, ArmMultipleInProgressThenAcceptedSucceeds )
{
    auto result = std::async( std::launch::async, [&]() { return client().arm( true ); } );

    Frame cmd;
    ASSERT_TRUE( m_vehicle->waitForMessage( skydio::MSG_COMMAND_LONG, cmd ) );
    for ( int i = 0; i < 3; ++i )
    {
        m_vehicle->sendMessage( skydio::MSG_COMMAND_ACK,
                                commandAckPayload( skydio::MAV_CMD_COMPONENT_ARM_DISARM,
                                                   skydio::MAV_RESULT_IN_PROGRESS ) );
        std::this_thread::sleep_for( 200ms );
    }
    m_vehicle->sendMessage( skydio::MSG_COMMAND_ACK,
                            commandAckPayload( skydio::MAV_CMD_COMPONENT_ARM_DISARM, skydio::MAV_RESULT_ACCEPTED ) );
    EXPECT_TRUE( result.get() );
}

TEST_F( MavlinkClientTest, ArmInProgressThenTerminalRejectionFails )
{
    auto result = std::async( std::launch::async, [&]() { return client().arm( true ); } );

    Frame cmd;
    ASSERT_TRUE( m_vehicle->waitForMessage( skydio::MSG_COMMAND_LONG, cmd ) );
    m_vehicle->sendMessage( skydio::MSG_COMMAND_ACK,
                            commandAckPayload( skydio::MAV_CMD_COMPONENT_ARM_DISARM, skydio::MAV_RESULT_IN_PROGRESS ) );
    std::this_thread::sleep_for( 300ms );
    m_vehicle->sendMessage( skydio::MSG_COMMAND_ACK,
                            commandAckPayload( skydio::MAV_CMD_COMPONENT_ARM_DISARM, 4 ) ); // MAV_RESULT_DENIED
    EXPECT_FALSE( result.get() );
}

TEST_F( MavlinkClientTest, ArmInProgressWithoutTerminalAckTimesOut )
{
    const auto start  = std::chrono::steady_clock::now();
    auto result = std::async( std::launch::async, [&]()
                              { return client().sendCommand( skydio::MAV_CMD_COMPONENT_ARM_DISARM, 1.f,
                                                             0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 400ms, 2 ); } );

    Frame cmd;
    ASSERT_TRUE( m_vehicle->waitForMessage( skydio::MSG_COMMAND_LONG, cmd ) );
    m_vehicle->sendMessage( skydio::MSG_COMMAND_ACK,
                            commandAckPayload( skydio::MAV_CMD_COMPONENT_ARM_DISARM, skydio::MAV_RESULT_IN_PROGRESS ) );
    EXPECT_FALSE( result.get() );

    // Bounded by the overall command deadline, not open-ended.
    EXPECT_LT( std::chrono::steady_clock::now() - start, 5s );
}

TEST_F( MavlinkClientTest, UnrelatedCommandAckIgnoredWhileWaitingForArm )
{
    auto result = std::async( std::launch::async, [&]() { return client().arm( true ); } );

    Frame cmd;
    ASSERT_TRUE( m_vehicle->waitForMessage( skydio::MSG_COMMAND_LONG, cmd ) );
    m_vehicle->sendMessage( skydio::MSG_COMMAND_ACK,
                            commandAckPayload( skydio::MAV_CMD_MISSION_START, skydio::MAV_RESULT_ACCEPTED ) );
    std::this_thread::sleep_for( 300ms );
    EXPECT_NE( result.wait_for( 0s ), std::future_status::ready ); // still waiting

    m_vehicle->sendMessage( skydio::MSG_COMMAND_ACK,
                            commandAckPayload( skydio::MAV_CMD_COMPONENT_ARM_DISARM, skydio::MAV_RESULT_ACCEPTED ) );
    EXPECT_TRUE( result.get() );
}

// ---- mission protocol packet loss / retransmission --------------------------

TEST_F( MavlinkClientTest, MissionUploadAnswersDuplicateRequest )
{
    std::vector<skydio::MissionItem> plan( 2 );
    plan[0].seq = 0;
    plan[1].seq = 1;
    auto result = std::async( std::launch::async, [&]() { return client().uploadMission( plan, 10s ); } );

    Frame count;
    ASSERT_TRUE( m_vehicle->waitForMessage( skydio::MSG_MISSION_COUNT, count ) );

    m_vehicle->sendMessage( skydio::MSG_MISSION_REQUEST_INT, missionRequestIntPayload( 0 ) );
    Frame item;
    ASSERT_TRUE( m_vehicle->waitForMessage( skydio::MSG_MISSION_ITEM_INT, item ) );
    EXPECT_EQ( readField<uint16_t>( item.payload, 28 ), 0 );

    // Duplicate request for seq 0 (e.g. our item got lost): answered again.
    m_vehicle->sendMessage( skydio::MSG_MISSION_REQUEST_INT, missionRequestIntPayload( 0 ) );
    Frame dup;
    ASSERT_TRUE( m_vehicle->waitForMessage( skydio::MSG_MISSION_ITEM_INT, dup ) );
    EXPECT_EQ( readField<uint16_t>( dup.payload, 28 ), 0 );

    m_vehicle->sendMessage( skydio::MSG_MISSION_REQUEST_INT, missionRequestIntPayload( 1 ) );
    Frame last;
    ASSERT_TRUE( m_vehicle->waitForMessage( skydio::MSG_MISSION_ITEM_INT, last ) );
    EXPECT_EQ( readField<uint16_t>( last.payload, 28 ), 1 );

    m_vehicle->sendMessage( skydio::MSG_MISSION_ACK, missionAckPayload( skydio::MAV_MISSION_ACCEPTED ) );
    EXPECT_TRUE( result.get() );
}

TEST_F( MavlinkClientTest, MissionUploadRetransmitsItemWhenNextRequestLost )
{
    std::vector<skydio::MissionItem> plan( 2 );
    plan[0].seq = 0;
    plan[1].seq = 1;
    auto result = std::async( std::launch::async, [&]() { return client().uploadMission( plan, 10s ); } );

    Frame count;
    ASSERT_TRUE( m_vehicle->waitForMessage( skydio::MSG_MISSION_COUNT, count ) );
    m_vehicle->sendMessage( skydio::MSG_MISSION_REQUEST_INT, missionRequestIntPayload( 0 ) );
    Frame item;
    ASSERT_TRUE( m_vehicle->waitForMessage( skydio::MSG_MISSION_ITEM_INT, item ) );

    // Simulate the vehicle's MISSION_REQUEST_INT(1) being lost: the node
    // must retransmit the previous item within the ~250 ms item timing.
    Frame retransmit;
    ASSERT_TRUE( m_vehicle->waitForMessage( skydio::MSG_MISSION_ITEM_INT, retransmit, 1500ms ) );
    EXPECT_EQ( readField<uint16_t>( retransmit.payload, 28 ), 0 );

    m_vehicle->sendMessage( skydio::MSG_MISSION_REQUEST_INT, missionRequestIntPayload( 1 ) );
    Frame last;
    ASSERT_TRUE( m_vehicle->waitForMessage( skydio::MSG_MISSION_ITEM_INT, last ) );
    m_vehicle->sendMessage( skydio::MSG_MISSION_ACK, missionAckPayload( skydio::MAV_MISSION_ACCEPTED ) );
    EXPECT_TRUE( result.get() );
}

TEST_F( MavlinkClientTest, MissionUploadRetransmitsLastItemWhenFinalAckLost )
{
    std::vector<skydio::MissionItem> plan( 1 );
    auto result = std::async( std::launch::async, [&]() { return client().uploadMission( plan, 10s ); } );

    Frame count;
    ASSERT_TRUE( m_vehicle->waitForMessage( skydio::MSG_MISSION_COUNT, count ) );
    m_vehicle->sendMessage( skydio::MSG_MISSION_REQUEST_INT, missionRequestIntPayload( 0 ) );
    Frame item;
    ASSERT_TRUE( m_vehicle->waitForMessage( skydio::MSG_MISSION_ITEM_INT, item ) );

    // Final MISSION_ACK "lost": the node retransmits the last item within
    // the ~1500 ms protocol timing, then the (re)sent ACK completes it.
    Frame retransmit;
    ASSERT_TRUE( m_vehicle->waitForMessage( skydio::MSG_MISSION_ITEM_INT, retransmit, 2500ms ) );
    EXPECT_EQ( readField<uint16_t>( retransmit.payload, 28 ), 0 );

    m_vehicle->sendMessage( skydio::MSG_MISSION_ACK, missionAckPayload( skydio::MAV_MISSION_ACCEPTED ) );
    EXPECT_TRUE( result.get() );
}

TEST_F( MavlinkClientTest, MissionUploadRetryExhaustionMidTransactionFails )
{
    std::vector<skydio::MissionItem> plan( 2 );
    auto result = std::async( std::launch::async, [&]() { return client().uploadMission( plan, 20s ); } );

    Frame count;
    ASSERT_TRUE( m_vehicle->waitForMessage( skydio::MSG_MISSION_COUNT, count ) );
    m_vehicle->sendMessage( skydio::MSG_MISSION_REQUEST_INT, missionRequestIntPayload( 0 ) );

    // Never request seq 1 and never ACK: the node retransmits item 0 up to
    // MISSION_MAX_RETRIES times and then fails well before the 20 s budget.
    int itemFrames = 0;
    Frame item;
    while ( m_vehicle->waitForMessage( skydio::MSG_MISSION_ITEM_INT, item, 1500ms ) )
        ++itemFrames;
    EXPECT_EQ( itemFrames, 1 + skydio::MISSION_MAX_RETRIES );
    EXPECT_FALSE( result.get() );
}

TEST_F( MavlinkClientTest, MissionUploadIgnoresOutOfRangeRequest )
{
    std::vector<skydio::MissionItem> plan( 1 );
    auto result = std::async( std::launch::async, [&]() { return client().uploadMission( plan, 10s ); } );

    Frame count;
    ASSERT_TRUE( m_vehicle->waitForMessage( skydio::MSG_MISSION_COUNT, count ) );

    // Malformed/out-of-range request: must not be answered or corrupt state.
    m_vehicle->sendMessage( skydio::MSG_MISSION_REQUEST_INT, missionRequestIntPayload( 999 ) );
    Frame bogus;
    EXPECT_FALSE( m_vehicle->waitForMessage( skydio::MSG_MISSION_ITEM_INT, bogus, 200ms ) );

    m_vehicle->sendMessage( skydio::MSG_MISSION_REQUEST_INT, missionRequestIntPayload( 0 ) );
    Frame item;
    ASSERT_TRUE( m_vehicle->waitForMessage( skydio::MSG_MISSION_ITEM_INT, item ) );
    EXPECT_EQ( readField<uint16_t>( item.payload, 28 ), 0 );

    m_vehicle->sendMessage( skydio::MSG_MISSION_ACK, missionAckPayload( skydio::MAV_MISSION_ACCEPTED ) );
    EXPECT_TRUE( result.get() );
}

// ---- MISSION_CLEAR_ALL reliability ------------------------------------------

TEST_F( MavlinkClientTest, ClearMissionRetriesWhenAckLost )
{
    auto result = std::async( std::launch::async, [&]() { return client().clearMission( 15s ); } );

    Frame first;
    ASSERT_TRUE( m_vehicle->waitForMessage( skydio::MSG_MISSION_CLEAR_ALL, first ) );
    // Ignore the first CLEAR_ALL (simulated loss); expect a retry.
    Frame second;
    ASSERT_TRUE( m_vehicle->waitForMessage( skydio::MSG_MISSION_CLEAR_ALL, second, 2500ms ) );

    m_vehicle->sendMessage( skydio::MSG_MISSION_ACK, missionAckPayload( skydio::MAV_MISSION_ACCEPTED ) );
    EXPECT_TRUE( result.get() );
}

TEST_F( MavlinkClientTest, ClearMissionMultipleLossesThenSuccess )
{
    auto result = std::async( std::launch::async, [&]() { return client().clearMission( 15s ); } );

    Frame frame;
    ASSERT_TRUE( m_vehicle->waitForMessage( skydio::MSG_MISSION_CLEAR_ALL, frame ) );
    ASSERT_TRUE( m_vehicle->waitForMessage( skydio::MSG_MISSION_CLEAR_ALL, frame, 2500ms ) );
    ASSERT_TRUE( m_vehicle->waitForMessage( skydio::MSG_MISSION_CLEAR_ALL, frame, 2500ms ) );

    m_vehicle->sendMessage( skydio::MSG_MISSION_ACK, missionAckPayload( skydio::MAV_MISSION_ACCEPTED ) );
    EXPECT_TRUE( result.get() );
}

TEST_F( MavlinkClientTest, ClearMissionRetryExhaustionFails )
{
    auto result = std::async( std::launch::async, [&]() { return client().clearMission( 4s ); } );

    int clears = 0;
    Frame frame;
    while ( m_vehicle->waitForMessage( skydio::MSG_MISSION_CLEAR_ALL, frame, 2500ms ) )
        ++clears;
    EXPECT_GE( clears, 2 );
    EXPECT_FALSE( result.get() );
}

TEST_F( MavlinkClientTest, ClearMissionExplicitRejectionFails )
{
    auto result = std::async( std::launch::async, [&]() { return client().clearMission( 10s ); } );

    Frame frame;
    ASSERT_TRUE( m_vehicle->waitForMessage( skydio::MSG_MISSION_CLEAR_ALL, frame ) );
    m_vehicle->sendMessage( skydio::MSG_MISSION_ACK, missionAckPayload( 1 ) ); // MAV_MISSION_ERROR
    EXPECT_FALSE( result.get() );

    // A terminal rejection must not trigger further retries.
    Frame extra;
    EXPECT_FALSE( m_vehicle->waitForMessage( skydio::MSG_MISSION_CLEAR_ALL, extra, 500ms ) );
}

TEST_F( MavlinkClientTest, ClearMissionIgnoresUnrelatedMissionMessages )
{
    auto result = std::async( std::launch::async, [&]() { return client().clearMission( 10s ); } );

    Frame frame;
    ASSERT_TRUE( m_vehicle->waitForMessage( skydio::MSG_MISSION_CLEAR_ALL, frame ) );

    // Unrelated mission traffic while waiting for the clear ACK.
    std::vector<uint8_t> current( 2, 0 );
    writeField<uint16_t>( current, 0, 2 );
    m_vehicle->sendMessage( skydio::MSG_MISSION_CURRENT, current );
    std::vector<uint8_t> reached( 2, 0 );
    writeField<uint16_t>( reached, 0, 1 );
    m_vehicle->sendMessage( skydio::MSG_MISSION_ITEM_REACHED, reached );
    std::this_thread::sleep_for( 200ms );
    EXPECT_NE( result.wait_for( 0s ), std::future_status::ready );

    m_vehicle->sendMessage( skydio::MSG_MISSION_ACK, missionAckPayload( skydio::MAV_MISSION_ACCEPTED ) );
    EXPECT_TRUE( result.get() );
}

// ---- link health / lifecycle ------------------------------------------------

TEST_F( MavlinkClientTest, HeartbeatTimeoutClearsLinkHealth )
{
    Frame frame;
    ASSERT_TRUE( m_vehicle->waitForMessage( skydio::MSG_HEARTBEAT, frame ) );

    m_vehicle->sendMessage( skydio::MSG_HEARTBEAT, heartbeatPayload( 0 ) );
    ASSERT_TRUE( client().waitForHeartbeat( 3000ms ) );

    // No further vehicle heartbeats: health must drop after the 5 s window.
    const auto deadline = std::chrono::steady_clock::now() + 8s;
    while ( client().getTelemetry().heartbeatOk && std::chrono::steady_clock::now() < deadline )
        std::this_thread::sleep_for( 100ms );
    EXPECT_FALSE( client().getTelemetry().heartbeatOk );
}

TEST_F( MavlinkClientTest, DisconnectWhileCommandPendingIsSafe )
{
    auto result = std::async( std::launch::async, [&]()
                              { return client().sendCommand( skydio::MAV_CMD_COMPONENT_ARM_DISARM, 1.f,
                                                             0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 500ms, 1 ); } );

    Frame cmd;
    ASSERT_TRUE( m_vehicle->waitForMessage( skydio::MSG_COMMAND_LONG, cmd ) );
    client().disconnect(); // shut the socket down under the pending command
    EXPECT_FALSE( result.get() );
}

TEST_F( MavlinkClientTest, RepeatedConnectDisconnectCycles )
{
    for ( int cycle = 0; cycle < 3; ++cycle )
    {
        client().disconnect();
        ASSERT_TRUE( client().connect() );
        Frame frame;
        ASSERT_TRUE( m_vehicle->waitForMessage( skydio::MSG_HEARTBEAT, frame ) );
    }
}

} // namespace
