/**
 * Test stub for MMSTypes/GeoPoint.h (MPMS SDK).
 *
 * Provides the minimal MMS::GeoPoint API used by the skydio_me_node sources
 * so the unit tests can compile without the proprietary MPMS SDK. The real
 * SDK header supersedes this stub in production builds.
 */
#pragma once

#include <cstdio>
#include <cstring>
#include <string>

namespace MMS
{

class GeoPoint
{
public:
    GeoPoint() = default;
    GeoPoint( double lat, double lon ) : m_lat( lat ), m_lon( lon ) {}

    double getLat() const { return m_lat; }
    double getLon() const { return m_lon; }
    void   setLat( double lat ) { m_lat = lat; }
    void   setLon( double lon ) { m_lon = lon; }

    /// Parses a WKT "POINT (lon lat)" string.
    bool setFromStr( const std::string & s )
    {
        double lon = 0.0, lat = 0.0;
        const char * p = std::strchr( s.c_str(), '(' );
        if ( !p || std::sscanf( p, "(%lf %lf", &lon, &lat ) != 2 )
            return false;
        m_lon = lon;
        m_lat = lat;
        return true;
    }

private:
    double m_lat = 0.0;
    double m_lon = 0.0;
};

} // namespace MMS
