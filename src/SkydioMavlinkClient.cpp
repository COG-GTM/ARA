/**
 * SkydioMavlinkClient.cpp
 *
 * MAVLink v2 / RAS-A UDP driver for the Skydio X10D. See header for the
 * mapping between TraverseTo behavior actions and ICD messages.
 */

#include "SkydioMavlinkClient.h"
#include "utility.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <iostream>

namespace skydio
{

namespace
{

constexpr uint8_t MAVLINK_V2_MAGIC     = 0xFD;
constexpr size_t  MAVLINK_V2_HDR_LEN   = 10;
constexpr size_t  MAVLINK_CRC_LEN      = 2;
constexpr uint8_t MAVLINK_IFLAG_SIGNED = 0x01;
constexpr size_t  MAVLINK_SIG_LEN      = 13;
constexpr size_t  MAVLINK_MAX_PAYLOAD  = 255;

// CRC-16/MCRF4XX as specified by MAVLink.
void crcAccumulate( uint8_t data, uint16_t & crc )
{
    uint8_t tmp = data ^ static_cast<uint8_t>( crc & 0xFF );
    tmp ^= static_cast<uint8_t>( tmp << 4 );
    crc = static_cast<uint16_t>( ( crc >> 8 ) ^ ( tmp << 8 ) ^ ( tmp << 3 ) ^ ( tmp >> 4 ) );
}

template <typename T>
void put( uint8_t * buffer, size_t offset, T value )
{
    std::memcpy( buffer + offset, &value, sizeof( T ) );
}

template <typename T>
T get( const uint8_t * buffer, size_t offset )
{
    T value;
    std::memcpy( &value, buffer + offset, sizeof( T ) );
    return value;
}

} // namespace

SkydioMavlinkClient & SkydioMavlinkClient::instance()
{
    static SkydioMavlinkClient client;
    return client;
}

SkydioMavlinkClient::~SkydioMavlinkClient()
{
    disconnect();
}

uint16_t SkydioMavlinkClient::crcExtra( uint32_t msgId )
{
    switch ( msgId )
    {
        case MSG_HEARTBEAT:            return 50;
        case MSG_GLOBAL_POSITION_INT:  return 104;
        case MSG_MISSION_REQUEST:      return 230;
        case MSG_MISSION_CURRENT:      return 28;
        case MSG_MISSION_COUNT:        return 221;
        case MSG_MISSION_CLEAR_ALL:    return 232;
        case MSG_MISSION_ITEM_REACHED: return 11;
        case MSG_MISSION_ACK:          return 153;
        case MSG_MISSION_REQUEST_INT:  return 196;
        case MSG_MISSION_ITEM_INT:     return 38;
        case MSG_COMMAND_LONG:         return 152;
        case MSG_COMMAND_ACK:          return 143;
        default:                       return 0xFFFF; // unknown -> not decodable
    }
}

void SkydioMavlinkClient::configure( const std::string & vehicleIp,
                                     unsigned short vehiclePort,
                                     unsigned short localPort,
                                     uint8_t gcsSystemId,
                                     uint8_t targetSystemId )
{
    m_vehicleIp    = vehicleIp;
    m_vehiclePort  = vehiclePort;
    m_localPort    = localPort;
    m_systemId     = gcsSystemId;
    m_targetSystem = targetSystemId;
}

bool SkydioMavlinkClient::connect()
{
    if ( m_connected.load() )
        return true;

    m_socket = ::socket( AF_INET, SOCK_DGRAM, 0 );
    if ( m_socket < 0 )
    {
        std::cerr << RED << "[SkydioMavlinkClient] failed to create UDP socket" << NORMAL << std::endl;
        return false;
    }

    sockaddr_in localAddr{};
    localAddr.sin_family      = AF_INET;
    localAddr.sin_addr.s_addr = htonl( INADDR_ANY );
    localAddr.sin_port        = htons( m_localPort );
    if ( ::bind( m_socket, reinterpret_cast<sockaddr *>( &localAddr ), sizeof( localAddr ) ) < 0 )
    {
        std::cerr << YELLOW << "[SkydioMavlinkClient] could not bind local port " << m_localPort
                  << ", using ephemeral port" << NORMAL << std::endl;
    }

    sockaddr_in vehicleAddr{};
    vehicleAddr.sin_family = AF_INET;
    vehicleAddr.sin_port   = htons( m_vehiclePort );
    if ( ::inet_pton( AF_INET, m_vehicleIp.c_str(), &vehicleAddr.sin_addr ) != 1 )
    {
        std::cerr << RED << "[SkydioMavlinkClient] invalid vehicle IP \"" << m_vehicleIp << "\"" << NORMAL << std::endl;
        ::close( m_socket );
        m_socket = -1;
        return false;
    }
    if ( ::connect( m_socket, reinterpret_cast<sockaddr *>( &vehicleAddr ), sizeof( vehicleAddr ) ) < 0 )
    {
        std::cerr << RED << "[SkydioMavlinkClient] failed to connect UDP socket to "
                  << m_vehicleIp << ":" << m_vehiclePort << NORMAL << std::endl;
        ::close( m_socket );
        m_socket = -1;
        return false;
    }

    timeval rxTimeout{};
    rxTimeout.tv_sec  = 0;
    rxTimeout.tv_usec = 250000;
    ::setsockopt( m_socket, SOL_SOCKET, SO_RCVTIMEO, &rxTimeout, sizeof( rxTimeout ) );

    m_shutdown.store( false );
    m_connected.store( true );
    m_rxThread        = std::thread( &SkydioMavlinkClient::rxLoop, this );
    m_heartbeatThread = std::thread( &SkydioMavlinkClient::heartbeatLoop, this );

    std::cout << GREEN << "[SkydioMavlinkClient] RAS-A/MAVLink link opened to "
              << m_vehicleIp << ":" << m_vehiclePort << NORMAL << std::endl;
    return true;
}

void SkydioMavlinkClient::disconnect()
{
    m_shutdown.store( true );
    if ( m_rxThread.joinable() )
        m_rxThread.join();
    if ( m_heartbeatThread.joinable() )
        m_heartbeatThread.join();
    if ( m_socket >= 0 )
    {
        ::close( m_socket );
        m_socket = -1;
    }
    m_connected.store( false );
}

void SkydioMavlinkClient::sendMessage( uint32_t msgId, const uint8_t * payload, uint8_t length )
{
    if ( m_socket < 0 )
        return;

    // MAVLink v2 payload zero-truncation: trim trailing zero bytes (min 1).
    uint8_t trimmedLength = length;
    while ( trimmedLength > 1 && payload[trimmedLength - 1] == 0 )
        --trimmedLength;

    uint8_t frame[MAVLINK_V2_HDR_LEN + MAVLINK_MAX_PAYLOAD + MAVLINK_CRC_LEN];

    std::lock_guard<std::mutex> lock( m_txMutex );

    frame[0] = MAVLINK_V2_MAGIC;
    frame[1] = trimmedLength;
    frame[2] = 0; // incompat_flags
    frame[3] = 0; // compat_flags
    frame[4] = m_txSequence++;
    frame[5] = m_systemId;
    frame[6] = m_componentId;
    frame[7] = static_cast<uint8_t>( msgId & 0xFF );
    frame[8] = static_cast<uint8_t>( ( msgId >> 8 ) & 0xFF );
    frame[9] = static_cast<uint8_t>( ( msgId >> 16 ) & 0xFF );
    std::memcpy( frame + MAVLINK_V2_HDR_LEN, payload, trimmedLength );

    uint16_t crc = 0xFFFF;
    for ( size_t i = 1; i < MAVLINK_V2_HDR_LEN + trimmedLength; ++i )
        crcAccumulate( frame[i], crc );
    crcAccumulate( static_cast<uint8_t>( crcExtra( msgId ) ), crc );

    frame[MAVLINK_V2_HDR_LEN + trimmedLength]     = static_cast<uint8_t>( crc & 0xFF );
    frame[MAVLINK_V2_HDR_LEN + trimmedLength + 1] = static_cast<uint8_t>( crc >> 8 );

    const size_t frameLength = MAVLINK_V2_HDR_LEN + trimmedLength + MAVLINK_CRC_LEN;
    if ( ::send( m_socket, frame, frameLength, 0 ) != static_cast<ssize_t>( frameLength ) )
        std::cerr << RED << "[SkydioMavlinkClient] UDP send failed for msg " << msgId << NORMAL << std::endl;
}

void SkydioMavlinkClient::sendHeartbeat()
{
    uint8_t payload[9] = { 0 };
    put<uint32_t>( payload, 0, 0 );                // custom_mode
    payload[4] = MAV_TYPE_GCS;                     // type
    payload[5] = MAV_AUTOPILOT_INVALID;            // autopilot
    payload[6] = 0;                                // base_mode
    payload[7] = MAV_STATE_ACTIVE;                 // system_status
    payload[8] = 3;                                // mavlink_version
    sendMessage( MSG_HEARTBEAT, payload, sizeof( payload ) );
}

void SkydioMavlinkClient::sendMissionCount( uint16_t count )
{
    uint8_t payload[5] = { 0 };
    put<uint16_t>( payload, 0, count );
    payload[2] = m_targetSystem;
    payload[3] = m_targetComponent;
    payload[4] = MAV_MISSION_TYPE_MISSION;
    sendMessage( MSG_MISSION_COUNT, payload, sizeof( payload ) );
}

void SkydioMavlinkClient::sendMissionItemInt( const MissionItem & item )
{
    uint8_t payload[38] = { 0 };
    put<float>(    payload,  0, item.param1 );
    put<float>(    payload,  4, item.param2 );
    put<float>(    payload,  8, item.param3 );
    put<float>(    payload, 12, item.param4 );
    put<int32_t>(  payload, 16, item.x );
    put<int32_t>(  payload, 20, item.y );
    put<float>(    payload, 24, item.z );
    put<uint16_t>( payload, 28, item.seq );
    put<uint16_t>( payload, 30, item.command );
    payload[32] = m_targetSystem;
    payload[33] = m_targetComponent;
    payload[34] = item.frame;
    payload[35] = item.current;
    payload[36] = item.autocontinue;
    payload[37] = MAV_MISSION_TYPE_MISSION;
    sendMessage( MSG_MISSION_ITEM_INT, payload, sizeof( payload ) );
}

bool SkydioMavlinkClient::waitForHeartbeat( std::chrono::milliseconds timeout )
{
    std::unique_lock<std::mutex> lock( m_stateMutex );
    return m_stateCondition.wait_for( lock, timeout, [this]() { return m_telemetry.heartbeatOk; } );
}

bool SkydioMavlinkClient::uploadMission( const std::vector<MissionItem> & items,
                                         std::chrono::milliseconds timeout )
{
    if ( items.empty() )
        return false;

    const auto deadline = std::chrono::steady_clock::now() + timeout;

    uint64_t seenRequestCounter = 0;
    {
        std::lock_guard<std::mutex> lock( m_stateMutex );
        m_missionRequestSeq = -1;
        m_missionAckType    = -1;
        seenRequestCounter  = m_missionRequestCounter;
    }

    // Explicit, state-aware Mission Protocol transaction with bounded
    // retransmission per the RAS-A ICD: on a response timeout the previous
    // message (MISSION_COUNT before the first request, otherwise the last
    // MISSION_ITEM_INT) is retransmitted, up to MISSION_MAX_RETRIES per
    // stage. Duplicate requests are answered again; retries reset whenever
    // the transaction makes forward progress.
    sendMissionCount( static_cast<uint16_t>( items.size() ) );

    int lastSentSeq   = -1;
    int stageRetries  = 0;
    while ( std::chrono::steady_clock::now() < deadline )
    {
        const auto stageTimeout = ( lastSentSeq < 0 ) ? PROTOCOL_RESPONSE_TIMEOUT
                                : ( lastSentSeq + 1 < static_cast<int>( items.size() ) )
                                      ? MISSION_ITEM_RESPONSE_TIMEOUT
                                      : PROTOCOL_RESPONSE_TIMEOUT; // final MISSION_ACK

        int  requestSeq = -1;
        int  ackType    = -1;
        bool progressed = false;
        {
            std::unique_lock<std::mutex> lock( m_stateMutex );
            progressed = m_stateCondition.wait_for( lock, stageTimeout, [this, seenRequestCounter]()
                                                    { return m_missionRequestCounter != seenRequestCounter ||
                                                             m_missionAckType >= 0; } );
            seenRequestCounter = m_missionRequestCounter;
            requestSeq         = m_missionRequestSeq;
            ackType            = m_missionAckType;
        }

        if ( ackType >= 0 )
        {
            if ( ackType == MAV_MISSION_ACCEPTED )
            {
                std::cout << GREEN << "[SkydioMavlinkClient] mission of " << items.size()
                          << " items accepted by vehicle" << NORMAL << std::endl;
                return true;
            }
            std::cerr << RED << "[SkydioMavlinkClient] mission upload rejected, MAV_MISSION_RESULT = "
                      << ackType << NORMAL << std::endl;
            return false;
        }

        if ( !progressed )
        {
            // Response timeout: retransmit the previous message.
            if ( ++stageRetries > MISSION_MAX_RETRIES )
            {
                std::cerr << RED << "[SkydioMavlinkClient] mission upload retries exhausted" << NORMAL << std::endl;
                return false;
            }
            if ( lastSentSeq < 0 )
                sendMissionCount( static_cast<uint16_t>( items.size() ) );
            else
                sendMissionItemInt( items[lastSentSeq] );
            continue;
        }

        if ( requestSeq >= 0 && requestSeq < static_cast<int>( items.size() ) )
        {
            // New request or duplicate of an earlier one: answer it either way.
            if ( requestSeq > lastSentSeq )
                stageRetries = 0; // forward progress
            sendMissionItemInt( items[requestSeq] );
            lastSentSeq = std::max( lastSentSeq, requestSeq );
        }
        else
        {
            std::cerr << YELLOW << "[SkydioMavlinkClient] ignoring out-of-range mission request seq "
                      << requestSeq << NORMAL << std::endl;
        }
    }

    std::cerr << RED << "[SkydioMavlinkClient] mission upload timed out" << NORMAL << std::endl;
    return false;
}

bool SkydioMavlinkClient::clearMission( std::chrono::milliseconds timeout )
{
    const auto deadline = std::chrono::steady_clock::now() + timeout;

    for ( int attempt = 0; attempt <= MISSION_MAX_RETRIES; ++attempt )
    {
        {
            std::lock_guard<std::mutex> lock( m_stateMutex );
            m_missionAckType = -1;
        }

        uint8_t payload[3] = { m_targetSystem, m_targetComponent, MAV_MISSION_TYPE_MISSION };
        sendMessage( MSG_MISSION_CLEAR_ALL, payload, sizeof( payload ) );

        auto waitUntil = std::chrono::steady_clock::now() + PROTOCOL_RESPONSE_TIMEOUT;
        if ( waitUntil > deadline )
            waitUntil = deadline;

        std::unique_lock<std::mutex> lock( m_stateMutex );
        const bool acked = m_stateCondition.wait_until( lock, waitUntil, [this]()
                                                        { return m_missionAckType >= 0; } );
        if ( acked )
        {
            if ( m_missionAckType == MAV_MISSION_ACCEPTED )
                return true;
            std::cerr << RED << "[SkydioMavlinkClient] MISSION_CLEAR_ALL rejected, MAV_MISSION_RESULT = "
                      << m_missionAckType << NORMAL << std::endl;
            return false;
        }

        if ( std::chrono::steady_clock::now() >= deadline )
            break;
    }

    std::cerr << RED << "[SkydioMavlinkClient] MISSION_CLEAR_ALL not acknowledged" << NORMAL << std::endl;
    return false;
}

bool SkydioMavlinkClient::sendCommand( uint16_t command,
                                       float p1, float p2, float p3, float p4,
                                       float p5, float p6, float p7,
                                       std::chrono::milliseconds timeout,
                                       int retries )
{
    // Overall bound: even a command that keeps reporting IN_PROGRESS may
    // not be waited on forever if the terminal COMMAND_ACK never arrives.
    const auto hardDeadline = std::chrono::steady_clock::now() + timeout * ( retries + 1 );

    for ( int attempt = 0; attempt < retries; ++attempt )
    {
        {
            std::lock_guard<std::mutex> lock( m_stateMutex );
            m_commandAckCmd    = -1;
            m_commandAckResult = -1;
        }

        uint8_t payload[33] = { 0 };
        put<float>( payload,  0, p1 );
        put<float>( payload,  4, p2 );
        put<float>( payload,  8, p3 );
        put<float>( payload, 12, p4 );
        put<float>( payload, 16, p5 );
        put<float>( payload, 20, p6 );
        put<float>( payload, 24, p7 );
        put<uint16_t>( payload, 28, command );
        payload[30] = m_targetSystem;
        payload[31] = m_targetComponent;
        payload[32] = static_cast<uint8_t>( attempt ); // confirmation
        sendMessage( MSG_COMMAND_LONG, payload, sizeof( payload ) );

        std::unique_lock<std::mutex> lock( m_stateMutex );
        auto waitUntil = std::min( std::chrono::steady_clock::now() + timeout, hardDeadline );
        while ( true )
        {
            const bool acked = m_stateCondition.wait_until( lock, waitUntil, [this, command]()
                                                            { return m_commandAckCmd == command; } );
            if ( !acked )
            {
                if ( std::chrono::steady_clock::now() >= hardDeadline )
                {
                    std::cerr << RED << "[SkydioMavlinkClient] MAV_CMD " << command
                              << " still in progress at timeout" << NORMAL << std::endl;
                    return false;
                }
                break; // no ACK at all within this attempt: retransmit
            }

            if ( m_commandAckResult == MAV_RESULT_ACCEPTED )
                return true;

            if ( m_commandAckResult == MAV_RESULT_IN_PROGRESS )
            {
                // Long-running command (e.g. arming): keep waiting for the
                // terminal COMMAND_ACK without failing or retransmitting.
                m_commandAckCmd    = -1;
                m_commandAckResult = -1;
                waitUntil = hardDeadline;
                continue;
            }

            std::cerr << RED << "[SkydioMavlinkClient] MAV_CMD " << command
                      << " rejected, MAV_RESULT = " << m_commandAckResult << NORMAL << std::endl;
            return false;
        }
    }

    std::cerr << RED << "[SkydioMavlinkClient] MAV_CMD " << command << " not acknowledged" << NORMAL << std::endl;
    return false;
}

bool SkydioMavlinkClient::arm( bool armed )
{
    return sendCommand( MAV_CMD_COMPONENT_ARM_DISARM, armed ? 1.f : 0.f );
}

bool SkydioMavlinkClient::startMission( uint16_t firstSeq, uint16_t lastSeq )
{
    return sendCommand( MAV_CMD_MISSION_START,
                        static_cast<float>( firstSeq ),
                        static_cast<float>( lastSeq ) );
}

bool SkydioMavlinkClient::pauseMission()
{
    return sendCommand( MAV_CMD_DO_PAUSE_CONTINUE, 0.f );
}

bool SkydioMavlinkClient::resumeMission()
{
    return sendCommand( MAV_CMD_DO_PAUSE_CONTINUE, 1.f );
}

bool SkydioMavlinkClient::setCurrentItem( uint16_t seq )
{
    return sendCommand( MAV_CMD_DO_SET_MISSION_CURRENT, static_cast<float>( seq ) );
}

bool SkydioMavlinkClient::changeSpeed( float speedMps )
{
    // param1: speed type 1 = ground speed, param2: speed m/s, param3: throttle (-1 no change)
    return sendCommand( MAV_CMD_DO_CHANGE_SPEED, 1.f, speedMps, -1.f );
}

Telemetry SkydioMavlinkClient::getTelemetry() const
{
    std::lock_guard<std::mutex> lock( m_stateMutex );
    return m_telemetry;
}

void SkydioMavlinkClient::resetMissionProgress()
{
    std::lock_guard<std::mutex> lock( m_stateMutex );
    m_telemetry.currentSeq = -1;
    m_telemetry.reachedSeq = -1;
}

void SkydioMavlinkClient::heartbeatLoop()
{
    while ( !m_shutdown.load() )
    {
        sendHeartbeat();
        std::this_thread::sleep_for( std::chrono::seconds( 1 ) );
    }
}

void SkydioMavlinkClient::rxLoop()
{
    uint8_t datagram[2048];
    uint8_t payload[MAVLINK_MAX_PAYLOAD];

    while ( !m_shutdown.load() )
    {
        const ssize_t received = ::recv( m_socket, datagram, sizeof( datagram ), 0 );
        if ( received <= 0 )
        {
            // Timeout (SO_RCVTIMEO) or transient error: refresh heartbeat health.
            std::lock_guard<std::mutex> lock( m_stateMutex );
            if ( m_telemetry.heartbeatOk &&
                 std::chrono::steady_clock::now() - m_lastHeartbeat > std::chrono::seconds( 5 ) )
            {
                m_telemetry.heartbeatOk = false;
                std::cerr << YELLOW << "[SkydioMavlinkClient] vehicle heartbeat lost" << NORMAL << std::endl;
            }
            continue;
        }

        // A datagram may contain multiple MAVLink v2 frames.
        size_t offset = 0;
        while ( offset + MAVLINK_V2_HDR_LEN + MAVLINK_CRC_LEN <= static_cast<size_t>( received ) )
        {
            if ( datagram[offset] != MAVLINK_V2_MAGIC )
            {
                ++offset;
                continue;
            }

            const uint8_t payloadLength  = datagram[offset + 1];
            const uint8_t incompatFlags  = datagram[offset + 2];
            size_t        frameLength    = MAVLINK_V2_HDR_LEN + payloadLength + MAVLINK_CRC_LEN;
            if ( incompatFlags & MAVLINK_IFLAG_SIGNED )
                frameLength += MAVLINK_SIG_LEN;
            if ( offset + frameLength > static_cast<size_t>( received ) )
                break;

            const uint32_t msgId = static_cast<uint32_t>( datagram[offset + 7] )
                                 | ( static_cast<uint32_t>( datagram[offset + 8] ) << 8 )
                                 | ( static_cast<uint32_t>( datagram[offset + 9] ) << 16 );

            const uint16_t extra = crcExtra( msgId );
            if ( extra != 0xFFFF )
            {
                uint16_t crc = 0xFFFF;
                for ( size_t i = 1; i < MAVLINK_V2_HDR_LEN + payloadLength; ++i )
                    crcAccumulate( datagram[offset + i], crc );
                crcAccumulate( static_cast<uint8_t>( extra ), crc );

                const uint16_t rxCrc = static_cast<uint16_t>( datagram[offset + MAVLINK_V2_HDR_LEN + payloadLength] )
                                     | ( static_cast<uint16_t>( datagram[offset + MAVLINK_V2_HDR_LEN + payloadLength + 1] ) << 8 );
                if ( crc == rxCrc )
                {
                    // Re-expand the zero-truncated payload to full length.
                    std::memset( payload, 0, sizeof( payload ) );
                    std::memcpy( payload, datagram + offset + MAVLINK_V2_HDR_LEN, payloadLength );
                    handleMessage( msgId, payload, payloadLength );
                }
            }

            offset += frameLength;
        }
    }
}

void SkydioMavlinkClient::handleMessage( uint32_t msgId, const uint8_t * payload, uint8_t /*length*/ )
{
    std::lock_guard<std::mutex> lock( m_stateMutex );

    switch ( msgId )
    {
        case MSG_HEARTBEAT:
        {
            const uint8_t baseMode  = payload[6];
            m_telemetry.armed       = ( baseMode & MAV_MODE_FLAG_SAFETY_ARMED ) != 0;
            m_telemetry.heartbeatOk = true;
            m_lastHeartbeat         = std::chrono::steady_clock::now();
            break;
        }
        case MSG_GLOBAL_POSITION_INT:
        {
            m_telemetry.latitudeDeg  = get<int32_t>( payload, 4 ) / 1e7;
            m_telemetry.longitudeDeg = get<int32_t>( payload, 8 ) / 1e7;
            m_telemetry.relAltitudeM = get<int32_t>( payload, 16 ) / 1000.f;
            const float vx           = get<int16_t>( payload, 20 ) / 100.f;
            const float vy           = get<int16_t>( payload, 22 ) / 100.f;
            m_telemetry.speedMps     = std::sqrt( vx * vx + vy * vy );
            const uint16_t heading   = get<uint16_t>( payload, 26 );
            if ( heading != UINT16_MAX )
                m_telemetry.headingDeg = heading / 100.f;
            m_telemetry.positionValid = true;
            break;
        }
        case MSG_MISSION_REQUEST:
        case MSG_MISSION_REQUEST_INT:
            m_missionRequestSeq = get<uint16_t>( payload, 0 );
            ++m_missionRequestCounter;
            break;
        case MSG_MISSION_ACK:
            m_missionAckType = payload[2];
            break;
        case MSG_COMMAND_ACK:
            m_commandAckCmd    = get<uint16_t>( payload, 0 );
            m_commandAckResult = payload[2];
            break;
        case MSG_MISSION_CURRENT:
            m_telemetry.currentSeq = get<uint16_t>( payload, 0 );
            break;
        case MSG_MISSION_ITEM_REACHED:
            m_telemetry.reachedSeq = get<uint16_t>( payload, 0 );
            break;
        default:
            break;
    }

    m_stateCondition.notify_all();
}

} // namespace skydio
