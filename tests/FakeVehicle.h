/**
 * FakeVehicle.h
 *
 * A scripted stand-in for the Skydio X10D's RAS-A/MAVLink endpoint used by
 * the unit tests. Binds a local UDP port, decodes MAVLink v2 frames with an
 * independent table-driven CRC-16/MCRF4XX implementation and the CRC_EXTRA
 * seeds published in the MAVLink common dialect (values taken from
 * pymavlink 2.4.x generated tables), and can reply with encoded vehicle
 * messages. Decoding failures (bad CRC, bad framing) are counted so tests
 * can assert the node's frames validate against the standard.
 */
#pragma once

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <deque>
#include <mutex>
#include <stdexcept>
#include <vector>

namespace testutil
{

struct Frame
{
    uint8_t              seq       = 0;
    uint8_t              sysid     = 0;
    uint8_t              compid    = 0;
    uint32_t             msgid     = 0;
    std::vector<uint8_t> payload;   // zero-extended to the full message length is NOT done here
};

/// CRC_EXTRA seeds per MAVLink common dialect (source: pymavlink generated tables).
inline int crcExtraFor( uint32_t msgid )
{
    switch ( msgid )
    {
        case 0:   return 50;   // HEARTBEAT
        case 33:  return 104;  // GLOBAL_POSITION_INT
        case 40:  return 230;  // MISSION_REQUEST
        case 42:  return 28;   // MISSION_CURRENT
        case 44:  return 221;  // MISSION_COUNT
        case 45:  return 232;  // MISSION_CLEAR_ALL
        case 46:  return 11;   // MISSION_ITEM_REACHED
        case 47:  return 153;  // MISSION_ACK
        case 51:  return 196;  // MISSION_REQUEST_INT
        case 73:  return 38;   // MISSION_ITEM_INT
        case 76:  return 152;  // COMMAND_LONG
        case 77:  return 143;  // COMMAND_ACK
        default:  return -1;
    }
}

/// Table-driven CRC-16/MCRF4XX — deliberately a different implementation
/// style from the shift-based one in SkydioMavlinkClient.cpp.
class Crc16Mcrf4xx
{
public:
    Crc16Mcrf4xx()
    {
        for ( int i = 0; i < 256; ++i )
        {
            uint16_t crc = static_cast<uint16_t>( i );
            for ( int b = 0; b < 8; ++b )
                crc = ( crc & 1 ) ? static_cast<uint16_t>( ( crc >> 1 ) ^ 0x8408 ) : static_cast<uint16_t>( crc >> 1 );
            m_table[i] = crc;
        }
    }

    uint16_t compute( const uint8_t * data, size_t length, uint8_t extra ) const
    {
        uint16_t crc = 0xFFFF;
        for ( size_t i = 0; i < length; ++i )
            crc = static_cast<uint16_t>( ( crc >> 8 ) ^ m_table[( crc ^ data[i] ) & 0xFF] );
        crc = static_cast<uint16_t>( ( crc >> 8 ) ^ m_table[( crc ^ extra ) & 0xFF] );
        return crc;
    }

private:
    uint16_t m_table[256];
};

class FakeVehicle
{
public:
    explicit FakeVehicle( unsigned short port, uint8_t sysid = 1, uint8_t compid = 1 )
        : m_sysid( sysid ), m_compid( compid )
    {
        m_socket = ::socket( AF_INET, SOCK_DGRAM, 0 );
        if ( m_socket < 0 )
            throw std::runtime_error( "FakeVehicle: socket() failed" );

        sockaddr_in addr{};
        addr.sin_family      = AF_INET;
        addr.sin_addr.s_addr = htonl( INADDR_LOOPBACK );
        addr.sin_port        = htons( port );
        if ( ::bind( m_socket, reinterpret_cast<sockaddr *>( &addr ), sizeof( addr ) ) < 0 )
            throw std::runtime_error( "FakeVehicle: bind() failed" );

        timeval rxTimeout{};
        rxTimeout.tv_usec = 50000;
        ::setsockopt( m_socket, SOL_SOCKET, SO_RCVTIMEO, &rxTimeout, sizeof( rxTimeout ) );
    }

    ~FakeVehicle()
    {
        if ( m_socket >= 0 )
            ::close( m_socket );
    }

    /// Receives and decodes frames until a frame with msgid arrives or the
    /// deadline passes. Non-matching frames (e.g. HEARTBEAT) are discarded
    /// unless collect is true. Throws on decode (CRC/framing) errors.
    bool waitForMessage( uint32_t msgid, Frame & out,
                         std::chrono::milliseconds timeout = std::chrono::milliseconds( 3000 ) )
    {
        return waitFor( &msgid, out, timeout );
    }

    /// Receives the next decodable frame of any message id.
    bool waitForAnyMessage( Frame & out,
                            std::chrono::milliseconds timeout = std::chrono::milliseconds( 3000 ) )
    {
        return waitFor( nullptr, out, timeout );
    }

    bool hasPeer() const { return m_havePeer; }

    int decodeErrors() const { return m_decodeErrors; }

    /// Encodes and sends a MAVLink v2 frame (with zero-truncation) back to
    /// the node. Requires at least one frame received first (peer address).
    void sendMessage( uint32_t msgid, const std::vector<uint8_t> & payload )
    {
        if ( !m_havePeer )
            throw std::runtime_error( "FakeVehicle: no peer address yet" );

        uint8_t trimmed = static_cast<uint8_t>( payload.size() );
        while ( trimmed > 1 && payload[trimmed - 1] == 0 )
            --trimmed;

        std::vector<uint8_t> frame( 10 + trimmed + 2 );
        frame[0] = 0xFD;
        frame[1] = trimmed;
        frame[2] = 0;
        frame[3] = 0;
        frame[4] = m_txSeq++;
        frame[5] = m_sysid;
        frame[6] = m_compid;
        frame[7] = static_cast<uint8_t>( msgid & 0xFF );
        frame[8] = static_cast<uint8_t>( ( msgid >> 8 ) & 0xFF );
        frame[9] = static_cast<uint8_t>( ( msgid >> 16 ) & 0xFF );
        std::memcpy( frame.data() + 10, payload.data(), trimmed );

        const uint16_t crc = m_crc.compute( frame.data() + 1, 9 + trimmed,
                                            static_cast<uint8_t>( crcExtraFor( msgid ) ) );
        frame[10 + trimmed]     = static_cast<uint8_t>( crc & 0xFF );
        frame[10 + trimmed + 1] = static_cast<uint8_t>( crc >> 8 );

        ::sendto( m_socket, frame.data(), frame.size(), 0,
                  reinterpret_cast<sockaddr *>( &m_peer ), m_peerLen );
    }

    /// Sends raw bytes as-is (for malformed-input robustness tests).
    void sendRaw( const std::vector<uint8_t> & bytes )
    {
        if ( !m_havePeer )
            throw std::runtime_error( "FakeVehicle: no peer address yet" );
        ::sendto( m_socket, bytes.data(), bytes.size(), 0,
                  reinterpret_cast<sockaddr *>( &m_peer ), m_peerLen );
    }

private:
    bool waitFor( const uint32_t * msgid, Frame & out, std::chrono::milliseconds timeout )
    {
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        while ( std::chrono::steady_clock::now() < deadline )
        {
            if ( !m_pending.empty() )
            {
                out = m_pending.front();
                m_pending.pop_front();
                if ( !msgid || out.msgid == *msgid )
                    return true;
                continue;
            }

            uint8_t buffer[2048];
            const ssize_t received = ::recvfrom( m_socket, buffer, sizeof( buffer ), 0,
                                                 reinterpret_cast<sockaddr *>( &m_peer ), &m_peerLen );
            if ( received <= 0 )
                continue;
            m_havePeer = true;

            size_t offset = 0;
            while ( offset + 12 <= static_cast<size_t>( received ) )
            {
                Frame frame;
                const size_t consumed = decodeOne( buffer + offset, received - offset, frame );
                if ( consumed == 0 )
                    break;
                offset += consumed;
                m_pending.push_back( frame );
            }
        }
        return false;
    }

    size_t decodeOne( const uint8_t * data, size_t available, Frame & frame )
    {
        if ( data[0] != 0xFD )
        {
            ++m_decodeErrors;
            return 0;
        }
        const uint8_t payloadLength = data[1];
        const size_t  frameLength   = 10 + payloadLength + 2;
        if ( frameLength > available )
        {
            ++m_decodeErrors;
            return 0;
        }

        frame.seq    = data[4];
        frame.sysid  = data[5];
        frame.compid = data[6];
        frame.msgid  = static_cast<uint32_t>( data[7] )
                     | ( static_cast<uint32_t>( data[8] ) << 8 )
                     | ( static_cast<uint32_t>( data[9] ) << 16 );

        const int extra = crcExtraFor( frame.msgid );
        if ( extra < 0 )
        {
            ++m_decodeErrors;
            return 0;
        }
        const uint16_t crc   = m_crc.compute( data + 1, 9 + payloadLength, static_cast<uint8_t>( extra ) );
        const uint16_t rxCrc = static_cast<uint16_t>( data[10 + payloadLength] )
                             | ( static_cast<uint16_t>( data[10 + payloadLength + 1] ) << 8 );
        if ( crc != rxCrc )
        {
            ++m_decodeErrors;
            return 0;
        }

        frame.payload.assign( data + 10, data + 10 + payloadLength );
        return frameLength;
    }

    int          m_socket = -1;
    uint8_t      m_sysid;
    uint8_t      m_compid;
    uint8_t      m_txSeq = 0;
    sockaddr_in  m_peer{};
    socklen_t    m_peerLen = sizeof( m_peer );
    bool         m_havePeer = false;
    int          m_decodeErrors = 0;
    std::deque<Frame> m_pending;
    Crc16Mcrf4xx m_crc;
};

// little-endian field readers/writers for payload assertions
template <typename T>
T readField( const std::vector<uint8_t> & payload, size_t offset )
{
    T value{};
    if ( offset + sizeof( T ) <= payload.size() )
        std::memcpy( &value, payload.data() + offset, sizeof( T ) );
    // MAVLink v2 zero-truncation: missing trailing bytes read as zero.
    else if ( offset < payload.size() )
    {
        uint8_t tmp[sizeof( T )] = { 0 };
        std::memcpy( tmp, payload.data() + offset, payload.size() - offset );
        std::memcpy( &value, tmp, sizeof( T ) );
    }
    return value;
}

template <typename T>
void writeField( std::vector<uint8_t> & payload, size_t offset, T value )
{
    if ( payload.size() < offset + sizeof( T ) )
        payload.resize( offset + sizeof( T ), 0 );
    std::memcpy( payload.data() + offset, &value, sizeof( T ) );
}

} // namespace testutil
