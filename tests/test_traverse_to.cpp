/**
 * Behavior-level tests for TraverseTo_impl driving the real
 * SkydioMavlinkClient against a scripted FakeVehicle. Exercises the MPMS
 * signal handlers (start/update/pause/resume/stop) end-to-end and asserts on
 * the native protocol traffic, the flight-plan contents, the behavior status
 * transitions, and the WaypointListComplete output signal.
 */
#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <map>
#include <thread>

#include "FakeVehicle.h"
#include "TraverseTo_impl.h"

using namespace std::chrono_literals;
using testutil::FakeVehicle;
using testutil::Frame;
using testutil::readField;
using testutil::writeField;

namespace
{

unsigned short nextPort()
{
    static std::atomic<unsigned short> port{ 26550 };
    return port++;
}

/// Decoded MISSION_ITEM_INT as captured off the wire.
struct CapturedItem
{
    uint16_t seq;
    uint16_t command;
    uint8_t  frame;
    uint8_t  current;
    uint8_t  autocontinue;
    float    param1, param2, param3, param4;
    int32_t  x, y;
    float    z;
};

/// Runs the vehicle side of the Mission Protocol and COMMAND_LONG handling
/// in a background thread, recording everything the node sends.
class ScriptedVehicle
{
public:
    explicit ScriptedVehicle( unsigned short port ) : m_vehicle( port )
    {
        m_thread = std::thread( &ScriptedVehicle::run, this );
    }

    ~ScriptedVehicle()
    {
        m_running.store( false );
        if ( m_thread.joinable() )
            m_thread.join();
    }

    std::vector<CapturedItem> items()
    {
        std::lock_guard<std::mutex> lock( m_mutex );
        return m_items;
    }

    std::vector<uint16_t> commands()
    {
        std::lock_guard<std::mutex> lock( m_mutex );
        return m_commands;
    }

    std::vector<std::pair<uint16_t, float>> commandParams()
    {
        std::lock_guard<std::mutex> lock( m_mutex );
        return m_commandParams;
    }

    int missionUploads()
    {
        std::lock_guard<std::mutex> lock( m_mutex );
        return m_uploadsCompleted;
    }

    bool sawClearAll()
    {
        std::lock_guard<std::mutex> lock( m_mutex );
        return m_sawClearAll;
    }

    void sendItemReached( uint16_t seq )
    {
        std::vector<uint8_t> payload( 2, 0 );
        writeField<uint16_t>( payload, 0, seq );
        m_vehicle.sendMessage( skydio::MSG_MISSION_ITEM_REACHED, payload );
    }

    bool waitForCommand( uint16_t command, std::chrono::milliseconds timeout = 5000ms )
    {
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        while ( std::chrono::steady_clock::now() < deadline )
        {
            {
                std::lock_guard<std::mutex> lock( m_mutex );
                if ( std::find( m_commands.begin(), m_commands.end(), command ) != m_commands.end() )
                    return true;
            }
            std::this_thread::sleep_for( 20ms );
        }
        return false;
    }

    bool waitForUploads( int count, std::chrono::milliseconds timeout = 10000ms )
    {
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        while ( std::chrono::steady_clock::now() < deadline )
        {
            {
                std::lock_guard<std::mutex> lock( m_mutex );
                if ( m_uploadsCompleted >= count )
                    return true;
            }
            std::this_thread::sleep_for( 20ms );
        }
        return false;
    }

private:
    void run()
    {
        uint16_t expectedCount = 0;
        uint16_t nextSeq       = 0;
        bool     uploading     = false;
        auto     lastHeartbeat = std::chrono::steady_clock::now() - 2s;

        while ( m_running.load() )
        {
            Frame frame;
            const bool got = m_vehicle.waitForAnyMessage( frame, 100ms );

            // Send a vehicle heartbeat once the node's address is known.
            if ( m_vehicle.hasPeer() &&
                 std::chrono::steady_clock::now() - lastHeartbeat > 1s )
            {
                std::vector<uint8_t> heartbeat( 9, 0 );
                heartbeat[4] = 2; // MAV_TYPE_QUADROTOR
                heartbeat[7] = skydio::MAV_STATE_ACTIVE;
                heartbeat[8] = 3;
                m_vehicle.sendMessage( skydio::MSG_HEARTBEAT, heartbeat );
                lastHeartbeat = std::chrono::steady_clock::now();
            }

            if ( !got )
                continue;

            switch ( frame.msgid )
            {
                case skydio::MSG_MISSION_COUNT:
                {
                    expectedCount = readField<uint16_t>( frame.payload, 0 );
                    nextSeq       = 0;
                    uploading     = expectedCount > 0;
                    if ( uploading )
                        requestItem( nextSeq );
                    break;
                }
                case skydio::MSG_MISSION_ITEM_INT:
                {
                    CapturedItem item;
                    item.param1       = readField<float>( frame.payload, 0 );
                    item.param2       = readField<float>( frame.payload, 4 );
                    item.param3       = readField<float>( frame.payload, 8 );
                    item.param4       = readField<float>( frame.payload, 12 );
                    item.x            = readField<int32_t>( frame.payload, 16 );
                    item.y            = readField<int32_t>( frame.payload, 20 );
                    item.z            = readField<float>( frame.payload, 24 );
                    item.seq          = readField<uint16_t>( frame.payload, 28 );
                    item.command      = readField<uint16_t>( frame.payload, 30 );
                    item.frame        = readField<uint8_t>( frame.payload, 34 );
                    item.current      = readField<uint8_t>( frame.payload, 35 );
                    item.autocontinue = readField<uint8_t>( frame.payload, 36 );
                    {
                        std::lock_guard<std::mutex> lock( m_mutex );
                        m_items.push_back( item );
                    }
                    if ( uploading )
                    {
                        ++nextSeq;
                        if ( nextSeq < expectedCount )
                            requestItem( nextSeq );
                        else
                        {
                            uploading = false;
                            std::vector<uint8_t> ack( 3, 0 );
                            ack[0] = 255;
                            ack[1] = 190;
                            ack[2] = skydio::MAV_MISSION_ACCEPTED;
                            m_vehicle.sendMessage( skydio::MSG_MISSION_ACK, ack );
                            std::lock_guard<std::mutex> lock( m_mutex );
                            ++m_uploadsCompleted;
                        }
                    }
                    break;
                }
                case skydio::MSG_MISSION_CLEAR_ALL:
                {
                    {
                        std::lock_guard<std::mutex> lock( m_mutex );
                        m_sawClearAll = true;
                    }
                    std::vector<uint8_t> ack( 3, 0 );
                    ack[0] = 255;
                    ack[1] = 190;
                    ack[2] = skydio::MAV_MISSION_ACCEPTED;
                    m_vehicle.sendMessage( skydio::MSG_MISSION_ACK, ack );
                    break;
                }
                case skydio::MSG_COMMAND_LONG:
                {
                    const uint16_t command = readField<uint16_t>( frame.payload, 28 );
                    const float    param1  = readField<float>( frame.payload, 0 );
                    {
                        std::lock_guard<std::mutex> lock( m_mutex );
                        m_commands.push_back( command );
                        m_commandParams.emplace_back( command, param1 );
                    }
                    std::vector<uint8_t> ack( 3, 0 );
                    writeField<uint16_t>( ack, 0, command );
                    ack[2] = skydio::MAV_RESULT_ACCEPTED;
                    m_vehicle.sendMessage( skydio::MSG_COMMAND_ACK, ack );
                    break;
                }
                default:
                    break; // HEARTBEAT etc.
            }
        }
    }

    void requestItem( uint16_t seq )
    {
        std::vector<uint8_t> request( 5, 0 );
        writeField<uint16_t>( request, 0, seq );
        request[2] = 255;
        request[3] = 190;
        request[4] = skydio::MAV_MISSION_TYPE_MISSION;
        m_vehicle.sendMessage( skydio::MSG_MISSION_REQUEST_INT, request );
    }

    FakeVehicle               m_vehicle;
    std::thread               m_thread;
    std::atomic<bool>         m_running{ true };
    std::mutex                m_mutex;
    std::vector<CapturedItem> m_items;
    std::vector<uint16_t>     m_commands;
    std::vector<std::pair<uint16_t, float>> m_commandParams;
    int                       m_uploadsCompleted = 0;
    bool                      m_sawClearAll      = false;
};

class TraverseToTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        const unsigned short vehiclePort = nextPort();
        m_vehicle = std::make_unique<ScriptedVehicle>( vehiclePort );

        auto & client = skydio::SkydioMavlinkClient::instance();
        client.configure( "127.0.0.1", vehiclePort, nextPort(), 255, 1 );
        ASSERT_TRUE( client.connect() );

        m_behavior = std::make_shared<TraverseTo_impl>();
        m_behavior->initializeCallbacks();
        m_behavior->configureService(); // default config: alt 20 m, speed 2 m/s, min/max 1 m/s
    }

    void TearDown() override
    {
        m_behavior->unconfigureService();
        m_behavior.reset();
        skydio::SkydioMavlinkClient::instance().disconnect();
        m_vehicle.reset();
    }

    void invokeStart( const std::string & waypointJson )
    {
        MMS::Parameter::ParameterMap params;
        params["data"] = MMS::Parameter( "data", "geojson", waypointJson );
        m_behavior->testInvokeIncomingSignal( "start", params );
    }

    void invokeUpdate( const std::string & waypointJson )
    {
        MMS::Parameter::ParameterMap params;
        params["data"] = MMS::Parameter( "data", "geojson", waypointJson );
        m_behavior->testInvokeIncomingSignal( "update", params );
    }

    bool waitForStatus( const std::string & value, std::chrono::milliseconds timeout = 5000ms )
    {
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        while ( std::chrono::steady_clock::now() < deadline )
        {
            if ( m_behavior->testStatusValue( "status" ) == value )
                return true;
            std::this_thread::sleep_for( 20ms );
        }
        return false;
    }

    std::unique_ptr<ScriptedVehicle>  m_vehicle;
    std::shared_ptr<TraverseTo_impl>  m_behavior;
};

const char * const TWO_WAYPOINTS =
    R"({"type":"LineString","coordinates":[[-79.982,40.446],[-79.990,40.450]]})";

TEST_F( TraverseToTest, StartUploadsPlanArmsStartsAndCompletes )
{
    invokeStart( TWO_WAYPOINTS );

    ASSERT_TRUE( m_vehicle->waitForUploads( 1 ) );
    ASSERT_TRUE( m_vehicle->waitForCommand( skydio::MAV_CMD_COMPONENT_ARM_DISARM ) );
    ASSERT_TRUE( m_vehicle->waitForCommand( skydio::MAV_CMD_MISSION_START ) );

    // Flight plan: takeoff, speed change, then one NAV_WAYPOINT per point.
    const auto items = m_vehicle->items();
    ASSERT_EQ( items.size(), 4u );

    EXPECT_EQ( items[0].seq, 0 );
    EXPECT_EQ( items[0].command, skydio::MAV_CMD_NAV_TAKEOFF );
    EXPECT_EQ( items[0].current, 1 );
    EXPECT_FLOAT_EQ( items[0].z, 20.f ); // default altitude_m

    EXPECT_EQ( items[1].seq, 1 );
    EXPECT_EQ( items[1].command, skydio::MAV_CMD_DO_CHANGE_SPEED );
    EXPECT_FLOAT_EQ( items[1].param1, 1.f );
    // default speed_mps 2 clamped to default maxVelocity_mps 1
    EXPECT_FLOAT_EQ( items[1].param2, 1.f );
    EXPECT_FLOAT_EQ( items[1].param3, -1.f );

    EXPECT_EQ( items[2].seq, 2 );
    EXPECT_EQ( items[2].command, skydio::MAV_CMD_NAV_WAYPOINT );
    EXPECT_EQ( items[2].frame, skydio::MAV_FRAME_GLOBAL_RELATIVE_ALT_INT );
    EXPECT_EQ( items[2].x, 404460000 );
    EXPECT_EQ( items[2].y, -799820000 );
    EXPECT_FLOAT_EQ( items[2].z, 20.f );
    EXPECT_FLOAT_EQ( items[2].param2, 2.f ); // acceptance radius
    EXPECT_EQ( items[2].autocontinue, 1 );

    EXPECT_EQ( items[3].seq, 3 );
    EXPECT_EQ( items[3].x, 404500000 );
    EXPECT_EQ( items[3].y, -799900000 );

    // Arm before start.
    const auto commands = m_vehicle->commands();
    const auto armIt    = std::find( commands.begin(), commands.end(),
                                     skydio::MAV_CMD_COMPONENT_ARM_DISARM );
    const auto startIt  = std::find( commands.begin(), commands.end(),
                                     skydio::MAV_CMD_MISSION_START );
    ASSERT_NE( armIt, commands.end() );
    ASSERT_NE( startIt, commands.end() );
    EXPECT_LT( armIt - commands.begin(), startIt - commands.begin() );

    EXPECT_TRUE( waitForStatus( "TASK_EXECUTING_NOMINAL" ) );

    // Reaching the final seq completes the traverse and fires the output signal.
    m_vehicle->sendItemReached( 3 );
    EXPECT_TRUE( waitForStatus( "TASK_COMPLETE" ) );

    const auto deadline = std::chrono::steady_clock::now() + 3s;
    while ( m_behavior->testFiredSignals().empty() && std::chrono::steady_clock::now() < deadline )
        std::this_thread::sleep_for( 20ms );
    const auto fired = m_behavior->testFiredSignals();
    ASSERT_EQ( fired.size(), 1u );
    EXPECT_EQ( fired[0], "WaypointListComplete" );
}

TEST_F( TraverseToTest, IntermediateWaypointDoesNotComplete )
{
    invokeStart( TWO_WAYPOINTS );
    ASSERT_TRUE( m_vehicle->waitForCommand( skydio::MAV_CMD_MISSION_START ) );

    m_vehicle->sendItemReached( 2 ); // first NAV_WAYPOINT, not the last
    EXPECT_FALSE( waitForStatus( "TASK_COMPLETE", 1000ms ) );
    EXPECT_TRUE( m_behavior->testFiredSignals().empty() );

    m_vehicle->sendItemReached( 3 );
    EXPECT_TRUE( waitForStatus( "TASK_COMPLETE" ) );
}

TEST_F( TraverseToTest, StartWithoutUsableWaypointsDoesNothing )
{
    // Signal with no usable data; default config waypoint_list is the numeric
    // placeholder "0" and must not produce a mission.
    MMS::Parameter::ParameterMap params;
    m_behavior->testInvokeIncomingSignal( "start", params );

    EXPECT_FALSE( m_vehicle->waitForUploads( 1, 1000ms ) );
    EXPECT_TRUE( m_vehicle->commands().empty() );
}

TEST_F( TraverseToTest, StartWithMalformedGeoJsonDoesNothing )
{
    invokeStart( R"({"type":"Point","coordinates":"garbage"})" );
    EXPECT_FALSE( m_vehicle->waitForUploads( 1, 1000ms ) );
}

TEST_F( TraverseToTest, PauseAndResumeMapToDoPauseContinue )
{
    m_behavior->testInvokeIncomingSignal( "pause", {} );
    ASSERT_TRUE( m_vehicle->waitForCommand( skydio::MAV_CMD_DO_PAUSE_CONTINUE ) );

    m_behavior->testInvokeIncomingSignal( "resume", {} );

    const auto deadline = std::chrono::steady_clock::now() + 5s;
    while ( m_vehicle->commandParams().size() < 2 && std::chrono::steady_clock::now() < deadline )
        std::this_thread::sleep_for( 20ms );

    const auto params = m_vehicle->commandParams();
    ASSERT_GE( params.size(), 2u );
    EXPECT_EQ( params[0].first, skydio::MAV_CMD_DO_PAUSE_CONTINUE );
    EXPECT_FLOAT_EQ( params[0].second, 0.f ); // pause = hold
    EXPECT_EQ( params[1].first, skydio::MAV_CMD_DO_PAUSE_CONTINUE );
    EXPECT_FLOAT_EQ( params[1].second, 1.f ); // resume = continue
}

TEST_F( TraverseToTest, StopHoldsClearsAndResetsStatus )
{
    invokeStart( TWO_WAYPOINTS );
    ASSERT_TRUE( m_vehicle->waitForCommand( skydio::MAV_CMD_MISSION_START ) );

    m_behavior->testInvokeIncomingSignal( "stop", {} );

    const auto deadline = std::chrono::steady_clock::now() + 5s;
    while ( !m_vehicle->sawClearAll() && std::chrono::steady_clock::now() < deadline )
        std::this_thread::sleep_for( 20ms );
    EXPECT_TRUE( m_vehicle->sawClearAll() );

    const auto params = m_vehicle->commandParams();
    const auto holdIt = std::find_if( params.begin(), params.end(), []( const auto & p )
                                      { return p.first == skydio::MAV_CMD_DO_PAUSE_CONTINUE && p.second == 0.f; } );
    EXPECT_NE( holdIt, params.end() );

    EXPECT_TRUE( waitForStatus( "TASK_PENDING" ) );
    EXPECT_TRUE( m_behavior->testFiredSignals().empty() ); // no completion on stop
}

TEST_F( TraverseToTest, UpdateReuploadsNewPlan )
{
    invokeStart( TWO_WAYPOINTS );
    ASSERT_TRUE( m_vehicle->waitForUploads( 1 ) );
    ASSERT_TRUE( m_vehicle->waitForCommand( skydio::MAV_CMD_MISSION_START ) );

    invokeUpdate( R"({"type":"LineString","coordinates":[[-80.000,40.460]]})" );
    ASSERT_TRUE( m_vehicle->waitForUploads( 2 ) );

    const auto items = m_vehicle->items();
    ASSERT_EQ( items.size(), 7u ); // 4 (first plan) + 3 (updated plan)
    EXPECT_EQ( items[6].command, skydio::MAV_CMD_NAV_WAYPOINT );
    EXPECT_EQ( items[6].x, 404600000 );
    EXPECT_EQ( items[6].y, -800000000 );

    // The update paused the running mission before restarting.
    const auto params = m_vehicle->commandParams();
    const auto holdIt = std::find_if( params.begin(), params.end(), []( const auto & p )
                                      { return p.first == skydio::MAV_CMD_DO_PAUSE_CONTINUE && p.second == 0.f; } );
    EXPECT_NE( holdIt, params.end() );

    m_vehicle->sendItemReached( 2 ); // last seq of the 3-item updated plan
    EXPECT_TRUE( waitForStatus( "TASK_COMPLETE" ) );
}

TEST_F( TraverseToTest, SpeedClampedToConfiguredBounds )
{
    m_behavior->setConfigParam_maxVelocity_mps( 5.f );
    m_behavior->setConfigParam_minVelocity_mps( 2.f );
    m_behavior->setConfigParam_speed_mps( 99.f );
    m_behavior->user_configure();

    invokeStart( TWO_WAYPOINTS );
    ASSERT_TRUE( m_vehicle->waitForUploads( 1 ) );

    const auto items = m_vehicle->items();
    ASSERT_GE( items.size(), 2u );
    EXPECT_FLOAT_EQ( items[1].param2, 5.f ); // clamped to max

    m_behavior->testInvokeIncomingSignal( "stop", {} );
}

} // namespace
