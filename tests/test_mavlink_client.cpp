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

} // namespace
